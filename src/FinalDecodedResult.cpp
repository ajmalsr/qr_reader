#include "FinalDecodedResult.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace
{
constexpr char AlphanumericCharacters[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:";

bool isNumericText(const std::string& text)
{
    for (char character : text)
    {
        if (character < '0' || character > '9')
        {
            return false;
        }
    }
    return true;
}

bool isAlphanumericText(const std::string& text)
{
    const std::string alphabet = AlphanumericCharacters;
    for (char character : text)
    {
        if (alphabet.find(character) == std::string::npos)
        {
            return false;
        }
    }
    return true;
}

bool byteTextMatchesRawData(const QrParsedSegment& segment)
{
    if (segment.text.size() != segment.byteData.size())
    {
        return false;
    }
    for (std::size_t index = 0; index < segment.byteData.size(); ++index)
    {
        if (static_cast<std::uint8_t>(
                static_cast<unsigned char>(segment.text[index])) !=
            segment.byteData[index])
        {
            return false;
        }
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

std::string bytesAsHex(const std::string& bytes)
{
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t index = 0; index < bytes.size(); ++index)
    {
        if (index != 0)
        {
            output << ' ';
        }
        output << std::setw(2)
               << static_cast<int>(
                      static_cast<unsigned char>(bytes[index]));
    }
    return output.str();
}

std::string rawBytesAsHex(const std::vector<std::uint8_t>& bytes)
{
    std::string values;
    values.reserve(bytes.size());
    for (std::uint8_t byte : bytes)
    {
        values.push_back(static_cast<char>(byte));
    }
    return bytesAsHex(values);
}
}

FinalDecodedResult assembleFinalDecodedResult(
    const CorrectedDataStreamResult& stream,
    const QrSegmentParsingResult& parsed)
{
    FinalDecodedResult result;
    result.version = stream.version;
    result.errorCorrectionLevel = stream.errorCorrectionLevel;

    // A final result is all-or-nothing. Earlier failures are propagated before
    // any segment text is copied, so callers never receive partial payloads.
    if (!stream.valid)
    {
        result.failureReason = "corrected_data_stream_invalid";
        return result;
    }
    if (!parsed.valid)
    {
        result.failureReason = "segment_parsing_failed:" + parsed.failureReason;
        return result;
    }
    if (stream.version != parsed.version)
    {
        result.failureReason = "version_mismatch";
        return result;
    }
    if (stream.errorCorrectionLevel == QrErrorCorrectionLevel::Unknown)
    {
        result.failureReason = "error_correction_level_unknown";
        return result;
    }
    if (stream.bitCount != stream.bytes.size() * 8 ||
        parsed.bitsConsumed > stream.bitCount ||
        parsed.bitsConsumed + parsed.bitsRemaining != stream.bitCount)
    {
        result.failureReason = "bit_count_mismatch";
        return result;
    }

    std::string assembledText;
    std::size_t byteModeByteCount = 0;
    std::size_t expectedStart = 0;
    for (const QrParsedSegment& segment : parsed.segments)
    {
        if (segment.startBit != expectedStart ||
            segment.endBit <= segment.startBit ||
            segment.endBit > parsed.bitsConsumed)
        {
            result.failureReason = "segment_bit_range_invalid";
            return result;
        }
        if (segment.text.size() != segment.characterCount)
        {
            result.failureReason = "segment_character_count_mismatch";
            return result;
        }

        if (segment.mode == QrSegmentMode::Numeric)
        {
            if (!segment.byteData.empty() || !isNumericText(segment.text))
            {
                result.failureReason = "numeric_segment_invalid";
                return result;
            }
        }
        else if (segment.mode == QrSegmentMode::Alphanumeric)
        {
            if (!segment.byteData.empty() ||
                !isAlphanumericText(segment.text))
            {
                result.failureReason = "alphanumeric_segment_invalid";
                return result;
            }
        }
        else
        {
            if (segment.byteData.size() != segment.characterCount ||
                !byteTextMatchesRawData(segment))
            {
                result.failureReason = "byte_segment_invalid";
                return result;
            }
            byteModeByteCount += segment.byteData.size();
        }

        assembledText += segment.text;
        expectedStart = segment.endBit;
    }

    result.text = assembledText;
    result.segments = parsed.segments;
    result.outputByteCount = result.text.size();
    result.byteModeByteCount = byteModeByteCount;
    result.terminatorFound = parsed.terminatorFound;
    result.bitsConsumed = parsed.bitsConsumed;
    result.bitsRemaining = parsed.bitsRemaining;
    result.valid = true;
    return result;
}

void saveFinalDecodedResultReport(
    const FinalDecodedResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::filesystem::create_directories(outputDirectory);
    const std::filesystem::path path =
        std::filesystem::path(outputDirectory) /
        "22_final_decoded_result.txt";
    std::ofstream output(path);
    if (!output)
    {
        return;
    }

    output << "Stage 24: Final decoded result\n";
    output << "input=" << inputFilename << '\n';
    output << "result=" << (result.valid ? "ACCEPT" : "REJECT") << '\n';
    output << "version=" << result.version << '\n';
    output << "error_correction_level="
           << errorCorrectionLevelName(result.errorCorrectionLevel) << '\n';
    if (!result.valid)
    {
        output << "reason=" << result.failureReason << '\n';
        output << "final_text_not_produced=yes\n";
        return;
    }

    output << "segment_count=" << result.segments.size() << '\n';
    output << "output_byte_count=" << result.outputByteCount << '\n';
    output << "byte_mode_byte_count=" << result.byteModeByteCount << '\n';
    output << "terminator_found="
           << (result.terminatorFound ? "yes" : "no") << '\n';
    output << "bits_consumed=" << result.bitsConsumed << '\n';
    output << "bits_remaining=" << result.bitsRemaining << '\n';
    output << "final_text=" << escapeText(result.text) << '\n';
    output << "final_bytes=" << bytesAsHex(result.text) << "\n\n";

    output << "segments:\n";
    std::size_t outputOffset = 0;
    for (std::size_t index = 0; index < result.segments.size(); ++index)
    {
        const QrParsedSegment& segment = result.segments[index];
        output << "segment[" << index << "] mode="
               << qrSegmentModeName(segment.mode)
               << " characters=" << segment.characterCount
               << " source_bits=[" << segment.startBit << ','
               << segment.endBit << ")"
               << " output_bytes=[" << outputOffset << ','
               << outputOffset + segment.text.size() << ")"
               << " text=" << escapeText(segment.text) << '\n';
        if (segment.mode == QrSegmentMode::Byte)
        {
            output << "  raw_byte_values="
                   << rawBytesAsHex(segment.byteData) << '\n';
        }
        outputOffset += segment.text.size();
    }

    output << "\nByte-mode values are preserved without ECI transcoding.\n";
}
