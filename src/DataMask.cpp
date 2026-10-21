#include "DataMask.h"

#include "lodepng.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace
{
DataMaskRemovalResult rejected(const std::string& reason)
{
    DataMaskRemovalResult result;
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

const char* maskFormula(int dataMask)
{
    switch (dataMask)
    {
    case 0: return "(row + column) % 2 == 0";
    case 1: return "row % 2 == 0";
    case 2: return "column % 3 == 0";
    case 3: return "(row + column) % 3 == 0";
    case 4: return "((row / 2) + (column / 3)) % 2 == 0";
    case 5: return "(row * column) % 6 == 0";
    case 6: return "(row * column) % 6 < 3";
    case 7: return "(row + column + (row * column) % 3) % 2 == 0";
    default: return "invalid";
    }
}
}

bool qrDataMaskApplies(int dataMask, int column, int row)
{
    if (dataMask < 0 || dataMask >= DataMaskConstants::MaskCount)
        throw std::invalid_argument("QR data-mask index must be in the range 0..7");
    if (column < 0 || row < 0)
        throw std::invalid_argument("QR data-mask coordinates must be non-negative");

    // QR mask equations use integer module coordinates with the origin at the
    // top-left of the normalized matrix. Division is integer division.
    const long long product = static_cast<long long>(row) * column;
    switch (dataMask)
    {
    case 0: return (row + column) % 2 == 0;
    case 1: return row % 2 == 0;
    case 2: return column % 3 == 0;
    case 3: return (row + column) % 3 == 0;
    case 4: return ((row / 2) + (column / 3)) % 2 == 0;
    case 5: return product % 6 == 0;
    case 6: return product % 6 < 3;
    case 7: return (row + column + product % 3) % 2 == 0;
    }
    return false;
}

DataMaskRemovalResult removeQrDataMask(
    const NormalizedQrMatrixResult& normalized,
    const FunctionPatternMapResult& functionPatternMap,
    const FormatInformationResult& formatInformation)
{
    if (!normalized.valid)
        return rejected("no_valid_normalized_qr_matrix");
    if (!functionPatternMap.valid)
        return rejected("no_valid_function_pattern_map");
    if (!formatInformation.valid)
        return rejected("no_valid_format_information");
    if (normalized.dimension != functionPatternMap.dimension ||
        normalized.dimension != formatInformation.dimension)
        return rejected("stage_dimension_mismatch");
    if (functionPatternMap.version != formatInformation.version)
        return rejected("stage_version_mismatch");
    if (!hasExpectedShape(normalized.matrix, normalized.dimension))
        return rejected("invalid_normalized_matrix_shape");
    if (!hasExpectedShape(functionPatternMap.functionModules, normalized.dimension))
        return rejected("invalid_function_map_shape");
    if (!containsOnlyBits(normalized.matrix))
        return rejected("normalized_matrix_contains_non_binary_value");
    if (!containsOnlyBits(functionPatternMap.functionModules))
        return rejected("function_map_contains_non_binary_value");
    if (formatInformation.dataMask < 0 ||
        formatInformation.dataMask >= DataMaskConstants::MaskCount)
        return rejected("invalid_data_mask_index");

    DataMaskRemovalResult result;
    result.dimension = normalized.dimension;
    result.version = functionPatternMap.version;
    result.errorCorrectionLevel = formatInformation.errorCorrectionLevel;
    result.dataMask = formatInformation.dataMask;

    // Work on a copy: Stage 15 remains the observed, still-masked matrix for
    // diagnostics, while this result becomes the input to codeword traversal.
    result.unmaskedMatrix = normalized.matrix;
    for (int row = 0; row < result.dimension; ++row)
    {
        for (int column = 0; column < result.dimension; ++column)
        {
            const std::size_t index = static_cast<std::size_t>(row) * result.dimension + column;
            if (functionPatternMap.functionModules.bits[index] != 0)
            {
                ++result.functionModulesPreserved;
                continue;
            }

            ++result.dataOrRemainderModulesVisited;
            if (qrDataMaskApplies(result.dataMask, column, row))
            {
                // Removing a QR mask is XOR: applying the same predicate a
                // second time would restore the original masked module.
                result.unmaskedMatrix.bits[index] ^= 1;
                ++result.modulesFlipped;
            }
        }
    }

    const std::size_t moduleCount = static_cast<std::size_t>(result.dimension) * result.dimension;
    if (result.functionModulesPreserved != functionPatternMap.functionModuleCount ||
        result.dataOrRemainderModulesVisited != functionPatternMap.dataOrRemainderModuleCount ||
        result.functionModulesPreserved + result.dataOrRemainderModulesVisited != moduleCount)
        return rejected("function_map_summary_mismatch");

    for (const std::uint8_t bit : result.unmaskedMatrix.bits)
        bit != 0 ? ++result.blackModules : ++result.whiteModules;

    result.valid = true;
    return result;
}

void saveDataMaskRemovalDebug(
    const DataMaskRemovalResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::filesystem::create_directories(outputDirectory);
    const std::filesystem::path directory(outputDirectory);

    std::ofstream report(directory / "16_unmasked_qr_matrix.txt");
    report << "Stage 18: QR data-mask removal\n";
    report << "input=" << inputFilename << '\n';
    if (!result.valid)
    {
        report << "result=REJECT\nreason=" << result.failureReason << '\n';
        return;
    }

    report << "result=ACCEPT\n";
    report << "dimension=" << result.dimension << '\n';
    report << "version=" << result.version << '\n';
    report << "error_correction_level=" << errorCorrectionLevelName(result.errorCorrectionLevel) << '\n';
    report << "data_mask=" << result.dataMask << '\n';
    report << "mask_formula=" << maskFormula(result.dataMask) << '\n';
    report << "function_modules_preserved=" << result.functionModulesPreserved << '\n';
    report << "data_or_remainder_modules_visited=" << result.dataOrRemainderModulesVisited << '\n';
    report << "modules_flipped=" << result.modulesFlipped << '\n';
    report << "black_modules_after_unmasking=" << result.blackModules << '\n';
    report << "white_modules_after_unmasking=" << result.whiteModules << '\n';
    report << "matrix_rows (#=black .=white)\n";
    for (int row = 0; row < result.dimension; ++row)
    {
        for (int column = 0; column < result.dimension; ++column)
        {
            const std::size_t index = static_cast<std::size_t>(row) * result.dimension + column;
            report << (result.unmaskedMatrix.bits[index] != 0 ? '#' : '.');
        }
        report << '\n';
    }

    const unsigned scale = DataMaskConstants::PreviewPixelsPerModule;
    const unsigned imageDimension = static_cast<unsigned>(result.dimension) * scale;
    std::vector<unsigned char> rgba(static_cast<std::size_t>(imageDimension) * imageDimension * 4, 255);
    for (int row = 0; row < result.dimension; ++row)
    {
        for (int column = 0; column < result.dimension; ++column)
        {
            const std::size_t moduleIndex = static_cast<std::size_t>(row) * result.dimension + column;
            const unsigned char shade = result.unmaskedMatrix.bits[moduleIndex] != 0 ? 0 : 255;
            for (unsigned pixelY = static_cast<unsigned>(row) * scale;
                 pixelY < static_cast<unsigned>(row + 1) * scale; ++pixelY)
            {
                for (unsigned pixelX = static_cast<unsigned>(column) * scale;
                     pixelX < static_cast<unsigned>(column + 1) * scale; ++pixelX)
                {
                    const std::size_t pixelIndex =
                        (static_cast<std::size_t>(pixelY) * imageDimension + pixelX) * 4;
                    rgba[pixelIndex] = shade;
                    rgba[pixelIndex + 1] = shade;
                    rgba[pixelIndex + 2] = shade;
                }
            }
        }
    }

    const unsigned error = lodepng::encode(
        (directory / "16_unmasked_qr_matrix.png").string(), rgba, imageDimension, imageDimension);
    if (error != 0)
        throw std::runtime_error(std::string("Could not save data-mask preview: ") + lodepng_error_text(error));
}