#include "QrSegmentParser.h"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace
{
constexpr std::uint32_t TerminatorMode = 0x0;
constexpr std::uint32_t NumericMode = 0x1;
constexpr std::uint32_t AlphanumericMode = 0x2;
constexpr std::uint32_t ByteMode = 0x4;
constexpr char AlphanumericCharacters[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:";

class BitReader
{
public:
    explicit BitReader(const std::vector<std::uint8_t>& bytes)
        : bytes_(bytes)
    {
    }

    std::size_t position() const
    {
        return position_;
    }

    std::size_t remaining() const
    {
        return bytes_.size() * 8 - position_;
    }

    bool read(int count, std::uint32_t& value)
    {
        if (count < 0 || count > 32 ||
            static_cast<std::size_t>(count) > remaining())
        {
            return false;
        }

        value = 0;
        for (int index = 0; index < count; ++index)
        {
            const std::size_t byteIndex = position_ / 8;
            const int bitInByte = 7 - static_cast<int>(position_ % 8);
            value = (value << 1) |
                    ((bytes_[byteIndex] >> bitInByte) & 1U);
            ++position_;
        }
        return true;
    }

    bool remainingBitsAreZero() const
    {
        for (std::size_t bit = position_; bit < bytes_.size() * 8; ++bit)
        {
            const std::size_t byteIndex = bit / 8;
            const int bitInByte = 7 - static_cast<int>(bit % 8);
            if (((bytes_[byteIndex] >> bitInByte) & 1U) != 0)
            {
                return false;
            }
        }
        return true;
    }

private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t position_ = 0;
};

int characterCountBitWidth(QrSegmentMode mode, int version)
{
    const int versionGroup = version <= 9 ? 0 : (version <= 26 ? 1 : 2);
    switch (mode)
    {
    case QrSegmentMode::Numeric:
    {
        constexpr int Widths[] = {10, 12, 14};
        return Widths[versionGroup];
    }
    case QrSegmentMode::Alphanumeric:
    {
        constexpr int Widths[] = {9, 11, 13};
        return Widths[versionGroup];
    }
    case QrSegmentMode::Byte:
    {
        constexpr int Widths[] = {8, 16, 16};
        return Widths[versionGroup];
    }
    }
    return 0;
}

std::size_t numericPayloadBits(std::size_t count)
{
    const std::size_t remainder = count % 3;
    return (count / 3) * 10 + (remainder == 2 ? 7 : remainder == 1 ? 4 : 0);
}

std::size_t alphanumericPayloadBits(std::size_t count)
{
    return (count / 2) * 11 + (count % 2) * 6;
}

void appendFixedWidthDecimal(
    std::string& output,
    std::uint32_t value,
    int width)
{
    std::string digits(static_cast<std::size_t>(width), '0');
    for (int index = width - 1; index >= 0; --index)
    {
        digits[static_cast<std::size_t>(index)] =
            static_cast<char>('0' + value % 10);
        value /= 10;
    }
    output += digits;
}

bool parseNumeric(
    BitReader& reader,
    std::size_t count,
    QrParsedSegment& segment,
    std::string& failureReason)
{
    if (reader.remaining() < numericPayloadBits(count))
    {
        failureReason = "truncated_numeric_payload";
        return false;
    }

    while (count >= 3)
    {
        std::uint32_t value = 0;
        reader.read(10, value);
        if (value >= 1000)
        {
            failureReason = "numeric_group_out_of_range";
            return false;
        }
        appendFixedWidthDecimal(segment.text, value, 3);
        count -= 3;
    }

    if (count == 2)
    {
        std::uint32_t value = 0;
        reader.read(7, value);
        if (value >= 100)
        {
            failureReason = "numeric_pair_out_of_range";
            return false;
        }
        appendFixedWidthDecimal(segment.text, value, 2);
    }
    else if (count == 1)
    {
        std::uint32_t value = 0;
        reader.read(4, value);
        if (value >= 10)
        {
            failureReason = "numeric_digit_out_of_range";
            return false;
        }
        appendFixedWidthDecimal(segment.text, value, 1);
    }
    return true;
}

bool parseAlphanumeric(
    BitReader& reader,
    std::size_t count,
    QrParsedSegment& segment,
    std::string& failureReason)
{
    if (reader.remaining() < alphanumericPayloadBits(count))
    {
        failureReason = "truncated_alphanumeric_payload";
        return false;
    }

    while (count >= 2)
    {
        std::uint32_t value = 0;
        reader.read(11, value);
        // Each pair is encoded as 45 * first + second.
        if (value >= 45U * 45U)
        {
            failureReason = "alphanumeric_pair_out_of_range";
            return false;
        }
        segment.text.push_back(AlphanumericCharacters[value / 45]);
        segment.text.push_back(AlphanumericCharacters[value % 45]);
        count -= 2;
    }

    if (count == 1)
    {
        std::uint32_t value = 0;
        reader.read(6, value);
        if (value >= 45)
        {
            failureReason = "alphanumeric_character_out_of_range";
            return false;
        }
        segment.text.push_back(AlphanumericCharacters[value]);
    }
    return true;
}

bool parseBytes(
    BitReader& reader,
    std::size_t count,
    QrParsedSegment& segment,
    std::string& failureReason)
{
    if (count > reader.remaining() / 8)
    {
        failureReason = "truncated_byte_payload";
        return false;
    }

    segment.byteData.reserve(count);
    segment.text.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
    {
        std::uint32_t value = 0;
        reader.read(8, value);
        segment.byteData.push_back(static_cast<std::uint8_t>(value));
        // Without ECI, Stage 23 preserves byte values exactly. The string is
        // a byte-for-byte view; Stage 24 will decide how to present it.
        segment.text.push_back(static_cast<char>(value));
    }
    return true;
}

std::string escapeText(const std::string& text)
{
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setfill('0');
    for (unsigned char character : text)
    {
        if (character >= 0x20 && character <= 0x7E &&
            character != '\\')
        {
            output << static_cast<char>(character);
        }
        else if (character == '\\')
        {
            output << "\\\\";
        }
        else
        {
            output << "\\x" << std::setw(2)
                   << static_cast<int>(character);
        }
    }
    return output.str();
}

std::string bytesAsHex(const std::vector<std::uint8_t>& bytes)
{
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t index = 0; index < bytes.size(); ++index)
    {
        if (index != 0)
        {
            output << ' ';
        }
        output << std::setw(2) << static_cast<int>(bytes[index]);
    }
    return output.str();
}
}

const char* qrSegmentModeName(QrSegmentMode mode)
{
    switch (mode)
    {
    case QrSegmentMode::Numeric:
        return "numeric";
    case QrSegmentMode::Alphanumeric:
        return "alphanumeric";
    case QrSegmentMode::Byte:
        return "byte";
    }
    return "unknown";
}

QrSegmentParsingResult parseQrSegments(
    const CorrectedDataStreamResult& stream)
{
    QrSegmentParsingResult result;
    result.version = stream.version;
    if (!stream.valid)
    {
        result.failureReason = "corrected_data_stream_invalid";
        return result;
    }
    if (stream.version < 1 || stream.version > 40)
    {
        result.failureReason = "version_out_of_range";
        return result;
    }
    if (stream.bitCount != stream.bytes.size() * 8)
    {
        result.failureReason = "bit_count_mismatch";
        return result;
    }

    BitReader reader(stream.bytes);
    while (reader.remaining() != 0)
    {
        if (reader.remaining() < 4)
        {
            // A QR terminator may be shortened when fewer than four capacity
            // bits remain. Those final bits must all be zero.
            if (!reader.remainingBitsAreZero())
            {
                result.failureReason = "nonzero_short_terminator";
                result.bitsConsumed = reader.position();
                result.bitsRemaining = reader.remaining();
                return result;
            }
            std::uint32_t ignored = 0;
            reader.read(static_cast<int>(reader.remaining()), ignored);
            result.terminatorFound = true;
            break;
        }

        const std::size_t segmentStart = reader.position();
        std::uint32_t modeIndicator = 0;
        reader.read(4, modeIndicator);
        if (modeIndicator == TerminatorMode)
        {
            result.terminatorFound = true;
            break;
        }

        QrSegmentMode mode;
        if (modeIndicator == NumericMode)
        {
            mode = QrSegmentMode::Numeric;
        }
        else if (modeIndicator == AlphanumericMode)
        {
            mode = QrSegmentMode::Alphanumeric;
        }
        else if (modeIndicator == ByteMode)
        {
            mode = QrSegmentMode::Byte;
        }
        else
        {
            result.failureReason =
                "unsupported_mode_" + std::to_string(modeIndicator);
            result.bitsConsumed = reader.position();
            result.bitsRemaining = reader.remaining();
            return result;
        }

        const int countWidth = characterCountBitWidth(mode, stream.version);
        std::uint32_t characterCount = 0;
        if (!reader.read(countWidth, characterCount))
        {
            result.failureReason = "truncated_character_count";
            result.bitsConsumed = reader.position();
            result.bitsRemaining = reader.remaining();
            return result;
        }

        QrParsedSegment segment;
        segment.mode = mode;
        segment.startBit = segmentStart;
        segment.characterCount = characterCount;
        bool parsed = false;
        if (mode == QrSegmentMode::Numeric)
        {
            parsed = parseNumeric(
                reader, characterCount, segment, result.failureReason);
        }
        else if (mode == QrSegmentMode::Alphanumeric)
        {
            parsed = parseAlphanumeric(
                reader, characterCount, segment, result.failureReason);
        }
        else
        {
            parsed = parseBytes(
                reader, characterCount, segment, result.failureReason);
        }

        if (!parsed)
        {
            result.bitsConsumed = reader.position();
            result.bitsRemaining = reader.remaining();
            return result;
        }
        segment.endBit = reader.position();
        result.segments.push_back(segment);
    }

    result.bitsConsumed = reader.position();
    result.bitsRemaining = reader.remaining();
    result.valid = true;
    return result;
}

void saveQrSegmentParsingReport(
    const QrSegmentParsingResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::filesystem::create_directories(outputDirectory);
    const std::filesystem::path path =
        std::filesystem::path(outputDirectory) / "21_parsed_segments.txt";
    std::ofstream output(path);
    if (!output)
    {
        return;
    }

    output << "Stage 23: Numeric, alphanumeric, and byte-mode parsing\n";
    output << "input=" << inputFilename << '\n';
    output << "result=" << (result.valid ? "ACCEPT" : "REJECT") << '\n';
    output << "version=" << result.version << '\n';
    output << "bits_consumed=" << result.bitsConsumed << '\n';
    output << "bits_remaining=" << result.bitsRemaining << '\n';
    output << "terminator_found="
           << (result.terminatorFound ? "yes" : "no") << '\n';
    if (!result.valid)
    {
        output << "reason=" << result.failureReason << '\n';
        return;
    }

    output << "segment_count=" << result.segments.size() << "\n\n";
    for (std::size_t index = 0; index < result.segments.size(); ++index)
    {
        const QrParsedSegment& segment = result.segments[index];
        output << "segment[" << index << "]\n";
        output << "mode=" << qrSegmentModeName(segment.mode) << '\n';
        output << "bit_range=[" << segment.startBit << ','
               << segment.endBit << ")\n";
        output << "character_count=" << segment.characterCount << '\n';
        output << "text=" << escapeText(segment.text) << '\n';
        if (segment.mode == QrSegmentMode::Byte)
        {
            output << "raw_bytes=" << bytesAsHex(segment.byteData) << '\n';
        }
        output << '\n';
    }
    output << "Stage 23 reports segments separately; final-result assembly is deferred.\n";
}
