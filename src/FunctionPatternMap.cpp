#include "FunctionPatternMap.h"

#include "lodepng.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace
{
FunctionPatternMapResult failed(const char* reason)
{
    FunctionPatternMapResult result;
    result.failureReason = reason;
    return result;
}

void markRegion(BitMatrix& mask, int left, int top, int width, int height)
{
    if (left < 0 || top < 0 || width < 0 || height < 0 ||
        left + width > static_cast<int>(mask.width) ||
        top + height > static_cast<int>(mask.height))
        throw std::logic_error("Function-pattern region is outside the QR matrix");

    for (int y = top; y < top + height; ++y)
        for (int x = left; x < left + width; ++x)
            mask.bits[static_cast<std::size_t>(y) * mask.width + x] = 1;
}

std::vector<int> alignmentCenters(int version, int dimension)
{
    using namespace FunctionPatternMapConstants;
    if (version == 1) return {};

    // Model 2 adds one alignment-center coordinate after each group of seven
    // versions. Centers start at 6; the remaining centers are evenly stepped
    // backward from dimension-7. Version 32 has the standard's special step 26.
    const int count = version / 7 + 2;
    const int denominator = 2 * count - 2;
    const int step = version == 32 ? 26 :
        ((4 * version + 2 * count + 1) / denominator) * 2;

    std::vector<int> centers(static_cast<std::size_t>(count));
    centers[0] = FinderCenterCoordinate;
    for (int i = 1; i < count; ++i)
        centers[static_cast<std::size_t>(i)] =
            dimension - 7 - (count - 1 - i) * step;
    return centers;
}
}

FunctionPatternMapResult buildFunctionPatternMap(
    const NormalizedQrMatrixResult& normalized)
{
    using namespace FunctionPatternMapConstants;
    if (!normalized.valid) return failed("no_valid_normalized_qr_matrix");

    FunctionPatternMapResult result;
    result.dimension = normalized.dimension;
    if (result.dimension < 21 || result.dimension > 177 || result.dimension % 4 != 1)
    {
        result.failureReason = "invalid_model_2_dimension";
        return result;
    }
    if (normalized.matrix.width != static_cast<unsigned>(result.dimension) ||
        normalized.matrix.height != static_cast<unsigned>(result.dimension) ||
        normalized.matrix.bits.size() !=
            static_cast<std::size_t>(result.dimension) * result.dimension)
    {
        result.failureReason = "normalized_matrix_shape_disagrees_with_dimension";
        return result;
    }

    result.version = (result.dimension - 17) / 4;
    if (result.version < MinimumVersion || result.version > MaximumVersion)
    {
        result.failureReason = "version_outside_model_2_range";
        return result;
    }

    result.functionModules.width = static_cast<unsigned>(result.dimension);
    result.functionModules.height = static_cast<unsigned>(result.dimension);
    result.functionModules.bits.assign(
        static_cast<std::size_t>(result.dimension) * result.dimension, 0);

    // These three corner regions include each 7x7 finder, its white separator,
    // the adjacent format-information modules, and the fixed dark module beside
    // the bottom-left finder. Their asymmetric sizes match the QR matrix edges.
    markRegion(result.functionModules, 0, 0, 9, 9);
    markRegion(result.functionModules, result.dimension - 8, 0, 8, 9);
    markRegion(result.functionModules, 0, result.dimension - 8, 9, 8);

    result.alignmentPatternCenters = alignmentCenters(result.version, result.dimension);
    const int farCenter = result.dimension - 7;
    for (const int centerY : result.alignmentPatternCenters)
        for (const int centerX : result.alignmentPatternCenters)
        {
            // Three nominal center combinations overlap finder patterns and are
            // omitted. Every other combination reserves a complete 5x5 target.
            const bool overlapsFinder =
                (centerX == 6 && (centerY == 6 || centerY == farCenter)) ||
                (centerX == farCenter && centerY == 6);
            if (!overlapsFinder)
                markRegion(result.functionModules, centerX - 2, centerY - 2, 5, 5);
        }

    // The corner regions already cover the first timing modules at coordinate 8;
    // these spans reserve the remaining alternating modules between the finders.
    markRegion(result.functionModules, 6, 9, 1, result.dimension - 17);
    markRegion(result.functionModules, 9, 6, result.dimension - 17, 1);

    if (result.version >= 7)
    {
        // Versions 7-40 store two 3x6 copies of their BCH-protected version bits.
        markRegion(result.functionModules, result.dimension - 11, 0, 3, 6);
        markRegion(result.functionModules, 0, result.dimension - 11, 6, 3);
    }

    result.functionModuleCount = static_cast<std::size_t>(std::count(
        result.functionModules.bits.begin(), result.functionModules.bits.end(), 1));
    result.dataOrRemainderModuleCount = result.functionModules.bits.size() -
                                        result.functionModuleCount;
    result.valid = true;
    return result;
}

void saveFunctionPatternMap(
    const FunctionPatternMapResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    if (result.valid)
    {
        const unsigned scale = FunctionPatternMapConstants::PreviewPixelsPerModule;
        const unsigned side = static_cast<unsigned>(result.dimension) * scale;
        std::vector<unsigned char> pixels(static_cast<std::size_t>(side) * side, 255);
        for (int row = 0; row < result.dimension; ++row)
            for (int column = 0; column < result.dimension; ++column)
            {
                const bool function = result.functionModules.bits[
                    static_cast<std::size_t>(row) * result.dimension + column] != 0;
                for (unsigned py = 0; py < scale; ++py)
                    for (unsigned px = 0; px < scale; ++px)
                        pixels[(static_cast<std::size_t>(row) * scale + py) * side +
                               static_cast<std::size_t>(column) * scale + px] =
                            function ? 0 : 255;
            }
        const unsigned error = lodepng::encode(
            outputDirectory + "/14_function_pattern_map.png", pixels,
            side, side, LCT_GREY, 8);
        if (error)
            throw std::runtime_error("Function-pattern map PNG writing failed: " +
                                     std::string(lodepng_error_text(error)));
    }

    std::ofstream out(outputDirectory + "/14_function_pattern_map.txt");
    if (!out) throw std::runtime_error("Cannot open function-pattern map report");
    out << "Stage 16: QR function-pattern map\n"
        << "input_filename=" << inputFilename << '\n'
        << "dimension=" << result.dimension << " version=" << result.version << '\n';
    if (!result.valid)
    {
        out << "result=REJECT reason=" << result.failureReason << '\n';
    }
    else
    {
        out << "alignment_pattern_centers=";
        if (result.alignmentPatternCenters.empty()) out << "none";
        for (std::size_t i = 0; i < result.alignmentPatternCenters.size(); ++i)
            out << (i ? "," : "") << result.alignmentPatternCenters[i];
        out << '\n'
            << "function_modules=" << result.functionModuleCount
            << " data_or_remainder_modules=" << result.dataOrRemainderModuleCount
            << " total_modules=" << result.functionModules.bits.size() << '\n'
            << "map_rows (F=function/reserved .=data-or-remainder):\n";
        for (int row = 0; row < result.dimension; ++row)
        {
            for (int column = 0; column < result.dimension; ++column)
                out << (result.functionModules.bits[
                    static_cast<std::size_t>(row) * result.dimension + column] ? 'F' : '.');
            out << '\n';
        }
        out << "result=VALID\n";
    }
    out.close();
    if (!out) throw std::runtime_error("Cannot write function-pattern map report");
}
