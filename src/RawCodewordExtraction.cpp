#include "RawCodewordExtraction.h"

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace
{
RawCodewordExtractionResult rejected(const std::string& reason)
{
    RawCodewordExtractionResult result;
    result.failureReason = reason;
    return result;
}

bool hasExpectedShape(const BitMatrix& matrix, int dimension)
{
    return dimension > 0 && matrix.width == static_cast<unsigned>(dimension) &&
           matrix.height == static_cast<unsigned>(dimension) &&
           matrix.bits.size() == static_cast<std::size_t>(dimension) * dimension;
}

bool containsOnlyBits(const BitMatrix& matrix)
{
    for (const std::uint8_t value : matrix.bits)
    {
        if (value > 1)
            return false;
    }
    return true;
}

void writeByteBits(std::ostream& output, std::uint8_t value)
{
    for (int shift = 7; shift >= 0; --shift)
        output << ((value >> shift) & 1U);
}
}

RawCodewordExtractionResult extractRawCodewords(
    const DataMaskRemovalResult& dataMaskRemoval,
    const FunctionPatternMapResult& functionPatternMap)
{
    if (!dataMaskRemoval.valid)
        return rejected("no_valid_unmasked_qr_matrix");
    if (!functionPatternMap.valid)
        return rejected("no_valid_function_pattern_map");
    if (dataMaskRemoval.dimension != functionPatternMap.dimension)
        return rejected("stage_dimension_mismatch");
    if (dataMaskRemoval.version != functionPatternMap.version)
        return rejected("stage_version_mismatch");
    if (!hasExpectedShape(dataMaskRemoval.unmaskedMatrix, dataMaskRemoval.dimension))
        return rejected("invalid_unmasked_matrix_shape");
    if (!hasExpectedShape(functionPatternMap.functionModules, dataMaskRemoval.dimension))
        return rejected("invalid_function_map_shape");
    if (!containsOnlyBits(dataMaskRemoval.unmaskedMatrix))
        return rejected("unmasked_matrix_contains_non_binary_value");
    if (!containsOnlyBits(functionPatternMap.functionModules))
        return rejected("function_map_contains_non_binary_value");

    RawCodewordExtractionResult result;
    result.dimension = dataMaskRemoval.dimension;
    result.version = dataMaskRemoval.version;
    result.errorCorrectionLevel = dataMaskRemoval.errorCorrectionLevel;
    result.dataMask = dataMaskRemoval.dataMask;
    result.functionModulesExcluded = functionPatternMap.functionModuleCount;

    // Start at the bottom-right and process adjacent columns from right to
    // left. Each pair reverses vertical direction to form the QR zigzag.
    bool readingUp = true;
    for (int rightColumn = result.dimension - 1; rightColumn > 0; rightColumn -= 2)
    {
        // Column 6 is the vertical timing pattern. Moving the pair one column
        // left keeps it out of the payload traversal.
        if (rightColumn == 6)
            --rightColumn;

        for (int rowStep = 0; rowStep < result.dimension; ++rowStep)
        {
            const int row = readingUp ? result.dimension - 1 - rowStep : rowStep;
            for (int pairOffset = 0; pairOffset < 2; ++pairOffset)
            {
                const int column = rightColumn - pairOffset;
                const std::size_t index =
                    static_cast<std::size_t>(row) * result.dimension + column;
                if (functionPatternMap.functionModules.bits[index] != 0)
                    continue;

                RawCodewordModule module;
                module.column = column;
                module.row = row;
                module.bit = dataMaskRemoval.unmaskedMatrix.bits[index];
                module.traversalIndex = result.traversedModules.size();
                result.traversedModules.push_back(module);
            }
        }
        readingUp = !readingUp;
    }

    result.dataOrRemainderModulesVisited = result.traversedModules.size();
    if (result.dataOrRemainderModulesVisited !=
        functionPatternMap.dataOrRemainderModuleCount)
        return rejected("traversal_did_not_cover_every_data_or_remainder_module");

    // Complete groups of eight are raw codewords, read most-significant bit
    // first. Any final 1-7 modules are QR remainder bits, not another byte.
    result.codewordBits = (result.traversedModules.size() / 8) * 8;
    result.rawCodewords.reserve(result.codewordBits / 8);
    for (std::size_t offset = 0; offset < result.codewordBits; offset += 8)
    {
        std::uint8_t value = 0;
        for (std::size_t bit = 0; bit < 8; ++bit)
            value = static_cast<std::uint8_t>(
                (value << 1) | result.traversedModules[offset + bit].bit);
        result.rawCodewords.push_back(value);
    }

    for (std::size_t index = result.codewordBits;
         index < result.traversedModules.size(); ++index)
    {
        result.traversedModules[index].remainder = true;
        const std::uint8_t bit = result.traversedModules[index].bit;
        result.remainderBits.push_back(bit);
        if (bit != 0)
            result.remainderBitsAllZero = false;
    }

    result.valid = true;
    return result;
}

void saveRawCodewordExtractionReport(
    const RawCodewordExtractionResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::filesystem::create_directories(outputDirectory);
    std::ofstream report(
        std::filesystem::path(outputDirectory) / "17_raw_codewords.txt");

    report << "Stage 19: raw QR codeword extraction\n";
    report << "input=" << inputFilename << '\n';
    if (!result.valid)
    {
        report << "result=REJECT\nreason=" << result.failureReason << '\n';
        return;
    }

    report << "result=ACCEPT\n";
    report << "dimension=" << result.dimension << '\n';
    report << "version=" << result.version << '\n';
    report << "error_correction_level="
           << errorCorrectionLevelName(result.errorCorrectionLevel) << '\n';
    report << "data_mask=" << result.dataMask << '\n';
    report << "traversal_start=(" << result.dimension - 1 << ','
           << result.dimension - 1 << ")\n";
    report << "initial_direction=up\n";
    report << "pair_order=right_column_then_left_column\n";
    report << "vertical_timing_column_skipped=6\n";
    report << "function_modules_excluded=" << result.functionModulesExcluded << '\n';
    report << "data_or_remainder_modules_visited="
           << result.dataOrRemainderModulesVisited << '\n';
    report << "codeword_bits=" << result.codewordBits << '\n';
    report << "raw_codeword_count=" << result.rawCodewords.size() << '\n';
    report << "remainder_bit_count=" << result.remainderBits.size() << '\n';
    report << "remainder_bits=";
    if (result.remainderBits.empty())
        report << "none";
    else
        for (const std::uint8_t bit : result.remainderBits)
            report << static_cast<int>(bit);
    report << '\n';
    report << "remainder_bits_all_zero="
           << (result.remainderBitsAllZero ? "yes" : "no") << '\n';

    report << "raw_codewords (index hex binary coordinates)\n";
    for (std::size_t codewordIndex = 0;
         codewordIndex < result.rawCodewords.size(); ++codewordIndex)
    {
        const std::uint8_t value = result.rawCodewords[codewordIndex];
        report << std::dec << std::setw(3) << std::setfill('0') << codewordIndex
               << " 0x" << std::uppercase << std::hex
               << std::setw(2) << std::setfill('0') << static_cast<int>(value)
               << std::nouppercase << std::dec << ' ';
        writeByteBits(report, value);
        report << ' ';
        const std::size_t firstBit = codewordIndex * 8;
        for (std::size_t bit = 0; bit < 8; ++bit)
        {
            const RawCodewordModule& module =
                result.traversedModules[firstBit + bit];
            if (bit != 0)
                report << ' ';
            report << '(' << module.column << ',' << module.row << ')';
        }
        report << '\n';
    }

    report << "remainder_modules\n";
    if (result.remainderBits.empty())
    {
        report << "none\n";
    }
    else
    {
        for (std::size_t index = result.codewordBits;
             index < result.traversedModules.size(); ++index)
        {
            const RawCodewordModule& module = result.traversedModules[index];
            report << "index=" << module.traversalIndex
                   << " coordinate=(" << module.column << ',' << module.row << ')'
                   << " bit=" << static_cast<int>(module.bit) << '\n';
        }
    }
}