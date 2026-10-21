#include "FormatInformation.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace
{
int bitLength(unsigned value)
{
    int length = 0;
    while (value)
    {
        ++length;
        value >>= 1;
    }
    return length;
}

int hammingDistance(unsigned a, unsigned b)
{
    unsigned difference = a ^ b;
    int count = 0;
    while (difference)
    {
        count += static_cast<int>(difference & 1U);
        difference >>= 1;
    }
    return count;
}

std::uint16_t makeUnmaskedCodeword(int data)
{
    using namespace FormatInformationConstants;
    unsigned value = static_cast<unsigned>(data) << ErrorCorrectionBitCount;
    unsigned remainder = value;

    // Polynomial long division by G(x)=x^10+x^8+x^5+x^4+x^2+x+1
    // (0x537) produces the ten BCH parity bits appended to the five data bits.
    while (bitLength(remainder) >= bitLength(BchGenerator))
        remainder ^= static_cast<unsigned>(BchGenerator) <<
                     (bitLength(remainder) - bitLength(BchGenerator));
    return static_cast<std::uint16_t>(value | remainder);
}

std::uint16_t readBits(
    const BitMatrix& matrix,
    const std::vector<std::pair<int, int>>& coordinates)
{
    std::uint16_t value = 0;
    for (const auto [x, y] : coordinates)
    {
        value = static_cast<std::uint16_t>(value << 1);
        if (matrix.bits[static_cast<std::size_t>(y) * matrix.width + x] != 0)
            value = static_cast<std::uint16_t>(value | 1U);
    }
    return value;
}

FormatInformationCopy decodeCopy(
    const BitMatrix& matrix,
    std::vector<std::pair<int, int>> coordinates)
{
    using namespace FormatInformationConstants;
    FormatInformationCopy result;
    result.coordinates = std::move(coordinates);
    result.maskedBits = readBits(matrix, result.coordinates);
    result.unmaskedBits = static_cast<std::uint16_t>(result.maskedBits ^ FixedMask);

    for (int data = 0; data < (1 << DataBitCount); ++data)
    {
        const int distance = hammingDistance(
            result.unmaskedBits, makeUnmaskedCodeword(data));
        if (distance < result.hammingDistance)
        {
            result.hammingDistance = distance;
            result.decodedData = data;
            result.ambiguous = false;
        }
        else if (distance == result.hammingDistance && data != result.decodedData)
        {
            result.ambiguous = true;
        }
    }

    result.valid = !result.ambiguous &&
                   result.hammingDistance <= MaximumCorrectableBitErrors;
    return result;
}

QrErrorCorrectionLevel levelFromBits(int bits)
{
    // ISO/IEC 18004 format mapping: 01=L, 00=M, 11=Q, 10=H.
    switch (bits)
    {
    case 0b01: return QrErrorCorrectionLevel::Low;
    case 0b00: return QrErrorCorrectionLevel::Medium;
    case 0b11: return QrErrorCorrectionLevel::Quartile;
    case 0b10: return QrErrorCorrectionLevel::High;
    default: return QrErrorCorrectionLevel::Unknown;
    }
}

std::string bits15(std::uint16_t value)
{
    std::string text;
    text.reserve(FormatInformationConstants::EncodedBitCount);
    for (int bit = FormatInformationConstants::EncodedBitCount - 1; bit >= 0; --bit)
        text.push_back((value & (1U << bit)) ? '1' : '0');
    return text;
}

void writeCopy(std::ofstream& out, const char* name, const FormatInformationCopy& copy)
{
    out << name
        << " masked_bits=" << bits15(copy.maskedBits)
        << " masked_hex=0x" << std::hex << std::uppercase
        << std::setw(4) << std::setfill('0') << copy.maskedBits
        << std::dec << std::nouppercase << std::setfill(' ')
        << " unmasked_bits=" << bits15(copy.unmaskedBits)
        << " decoded_data=" << copy.decodedData
        << " hamming_distance=" << copy.hammingDistance
        << " ambiguous=" << copy.ambiguous
        << " valid=" << copy.valid << '\n'
        << name << " coordinates=";
    for (std::size_t i = 0; i < copy.coordinates.size(); ++i)
        out << (i ? " " : "") << '(' << copy.coordinates[i].first
            << ',' << copy.coordinates[i].second << ')';
    out << '\n';
}
}

FormatInformationResult decodeFormatInformation(
    const NormalizedQrMatrixResult& normalized,
    const FunctionPatternMapResult& functionMap)
{
    FormatInformationResult result;
    result.dimension = normalized.dimension;
    result.version = functionMap.version;
    if (!normalized.valid)
    {
        result.failureReason = "no_valid_normalized_qr_matrix";
        return result;
    }
    if (!functionMap.valid)
    {
        result.failureReason = "no_valid_function_pattern_map";
        return result;
    }
    if (normalized.dimension != functionMap.dimension ||
        normalized.matrix.width != static_cast<unsigned>(normalized.dimension) ||
        normalized.matrix.height != static_cast<unsigned>(normalized.dimension) ||
        normalized.matrix.bits.size() !=
            static_cast<std::size_t>(normalized.dimension) * normalized.dimension)
    {
        result.failureReason = "normalized_matrix_and_function_map_disagree";
        return result;
    }

    // Coordinates are ordered most-significant bit first, matching the QR
    // specification and ZXing-C++'s append-bit reading order.
    std::vector<std::pair<int, int>> firstCoordinates;
    for (int x = 0; x < 6; ++x) firstCoordinates.emplace_back(x, 8);
    firstCoordinates.emplace_back(7, 8);
    firstCoordinates.emplace_back(8, 8);
    firstCoordinates.emplace_back(8, 7);
    for (int y = 5; y >= 0; --y) firstCoordinates.emplace_back(8, y);

    std::vector<std::pair<int, int>> secondCoordinates;
    for (int y = normalized.dimension - 1; y >= normalized.dimension - 7; --y)
        secondCoordinates.emplace_back(8, y);
    for (int x = normalized.dimension - 8; x < normalized.dimension; ++x)
        secondCoordinates.emplace_back(x, 8);

    result.firstCopy = decodeCopy(normalized.matrix, std::move(firstCoordinates));
    result.secondCopy = decodeCopy(normalized.matrix, std::move(secondCoordinates));

    if (!result.firstCopy.valid && !result.secondCopy.valid)
    {
        result.failureReason = "both_format_information_copies_are_uncorrectable";
        return result;
    }

    if (result.firstCopy.valid && result.secondCopy.valid)
    {
        if (result.firstCopy.decodedData == result.secondCopy.decodedData)
        {
            result.selectedCopy = 0;
            result.formatData = result.firstCopy.decodedData;
        }
        else if (result.firstCopy.hammingDistance < result.secondCopy.hammingDistance)
        {
            result.selectedCopy = 1;
            result.formatData = result.firstCopy.decodedData;
        }
        else if (result.secondCopy.hammingDistance < result.firstCopy.hammingDistance)
        {
            result.selectedCopy = 2;
            result.formatData = result.secondCopy.decodedData;
        }
        else
        {
            result.failureReason = "equally_reliable_format_copies_disagree";
            return result;
        }
    }
    else if (result.firstCopy.valid)
    {
        result.selectedCopy = 1;
        result.formatData = result.firstCopy.decodedData;
    }
    else
    {
        result.selectedCopy = 2;
        result.formatData = result.secondCopy.decodedData;
    }

    result.errorCorrectionLevel = levelFromBits((result.formatData >> 3) & 0b11);
    result.dataMask = result.formatData & 0b111;
    if (result.errorCorrectionLevel == QrErrorCorrectionLevel::Unknown ||
        result.dataMask < 0 || result.dataMask > 7)
    {
        result.failureReason = "decoded_format_fields_are_invalid";
        return result;
    }

    result.valid = true;
    return result;
}

const char* errorCorrectionLevelName(QrErrorCorrectionLevel level)
{
    switch (level)
    {
    case QrErrorCorrectionLevel::Low: return "L";
    case QrErrorCorrectionLevel::Medium: return "M";
    case QrErrorCorrectionLevel::Quartile: return "Q";
    case QrErrorCorrectionLevel::High: return "H";
    default: return "unknown";
    }
}

void saveFormatInformationReport(
    const FormatInformationResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::ofstream out(outputDirectory + "/15_format_information.txt");
    if (!out) throw std::runtime_error("Cannot open format-information report");
    out << "Stage 17: QR format-information BCH decoding\n"
        << "input_filename=" << inputFilename << '\n'
        << "dimension=" << result.dimension << " version=" << result.version << '\n'
        << "fixed_xor_mask=0x5412 bch_generator=0x537 maximum_correctable_errors=3\n";
    writeCopy(out, "copy_1", result.firstCopy);
    writeCopy(out, "copy_2", result.secondCopy);
    if (!result.valid)
    {
        out << "result=REJECT reason=" << result.failureReason << '\n';
    }
    else
    {
        out << "selected_copy=" << (result.selectedCopy == 0 ? "both_agree" :
                    (result.selectedCopy == 1 ? "copy_1" : "copy_2")) << '\n'
            << "format_data=" << result.formatData
            << " error_correction_level="
            << errorCorrectionLevelName(result.errorCorrectionLevel)
            << " data_mask=" << result.dataMask << '\n'
            << "result=VALID\n";
    }
    out.close();
    if (!out) throw std::runtime_error("Cannot write format-information report");
}
