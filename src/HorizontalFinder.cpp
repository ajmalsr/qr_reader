#include "HorizontalFinder.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
void validateMatrix(const BitMatrix& matrix)
{
    if (matrix.width == 0 || matrix.height == 0 ||
        matrix.width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        matrix.height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        static_cast<std::size_t>(matrix.width) >
            std::numeric_limits<std::size_t>::max() / matrix.height ||
        matrix.bits.size() != static_cast<std::size_t>(matrix.width) * matrix.height)
        throw std::invalid_argument("Invalid BitMatrix for horizontal finder search");
    for (auto bit : matrix.bits)
        if (bit > BinarizationConstants::BlackBit)
            throw std::invalid_argument("BitMatrix contains a non-binary value");
}
// Caller has validated the matrix. No artificial zero-length leading run.
std::vector<BinaryRowRun> rowRuns(const BitMatrix& matrix, int row)
{
    std::vector<BinaryRowRun> runs;
    const std::size_t offset = static_cast<std::size_t>(row) * matrix.width;
    bool black = matrix.bits[offset] == BinarizationConstants::BlackBit;
    int start = 0;
    const int width = static_cast<int>(matrix.width);
    for (int x = 1; x < width; ++x)
    {
        const bool nextBlack = matrix.bits[offset + x] == BinarizationConstants::BlackBit;
        if (nextBlack != black)
        {
            runs.push_back({black, start, x - start});
            start = x;
            black = nextBlack;
        }
    }
    runs.push_back({black, start, width - start}); // Include run at right edge.
    return runs;
}
}
std::vector<BinaryRowRun> horizontalRowRuns(const BitMatrix& matrix, int row)
{
    validateMatrix(matrix);
    if (row < 0 || static_cast<unsigned>(row) >= matrix.height)
        throw std::out_of_range("Horizontal finder row out of range");
    return rowRuns(matrix, row);
}
// Runs are indexed [0]=left black, [1]=left white, [2]=center black, [3]=right white,
// [4]=right black (the 1:1:3:1:1 pattern). This is a fast, division-free rejection filter
// applied to every black-starting run window before the stricter ratio check runs.
bool finderFastPlausibility(const std::array<int, 5>& runs)
{
    using namespace HorizontalFinderConstants;
    return runs[2] >= MinimumCenterPixels &&
        static_cast<long long>(runs[2]) >=
            static_cast<long long>(CenterOuterMultiplier) * std::max(runs[0], runs[4]) &&
        runs[2] >= std::max(runs[1], runs[3]);
}
// Estimates one module size from the total run width (total / 7 modules), then checks each
// of the five runs against its expected size (ExpectedRuns[i] * moduleSize). A window is
// accepted only if every run's relative error stays strictly below RelativeTolerance (50%).
// This tolerates modest blur/quantization noise while still rejecting non-finder patterns.
FinderRatioResult checkFinderRatio(const std::array<int, 5>& runs)
{
    using namespace HorizontalFinderConstants;
    FinderRatioResult result;
    long long total = 0;
    for (int length : runs)
    {
        if (length <= 0) return result;
        total += length;
    }
    if (total < TotalModules || total > std::numeric_limits<int>::max()) return result;
    result.totalWidth = static_cast<int>(total);
    result.moduleSize = static_cast<double>(total) / TotalModules;
    for (std::size_t i = 0; i < runs.size(); ++i)
    {
        const double expected = ExpectedRuns[i] * result.moduleSize;
        const double error = std::abs(runs[i] - expected) / expected;
        result.maximumRelativeError = std::max(result.maximumRelativeError, error);
    }
    result.accepted = result.maximumRelativeError < RelativeTolerance;
    return result;
}
// Center of the middle (3rd) black run, in pixel-edge coordinates: windowStart is the
// left edge of run[0], so the center run starts at windowStart + run[0] + run[1].
double horizontalFinderCenter(int windowStart, const std::array<int, 5>& runs)
{
    return static_cast<double>(windowStart) + runs[0] + runs[1] + runs[2] / 2.0;
}
HorizontalFinderResult searchHorizontalFinders(const BitMatrix& matrix)
{
    validateMatrix(matrix);
    HorizontalFinderResult result;
    // Wider loop type prevents overflow when adding the final row skip.
    for (long long y = HorizontalFinderConstants::FirstRow; y < matrix.height;
         y += HorizontalFinderConstants::RowSkip)
    {
        ++result.rowsScanned;
        const auto runs = rowRuns(matrix, static_cast<int>(y));
        // Slide a 5-run window across the row's run-length encoding. Only windows that
        // start on a black run can match the black-white-black-white-black finder pattern.
        for (std::size_t i = 0; i + 5 <= runs.size(); ++i)
        {
            if (!runs[i].black) continue;
            ++result.windowsTested;
            std::array<int, 5> lengths{};
            for (std::size_t j = 0; j < lengths.size(); ++j) lengths[j] = runs[i + j].length;
            if (!finderFastPlausibility(lengths)) continue;
            ++result.fastPasses;
            const auto ratio = checkFinderRatio(lengths);
            if (!ratio.accepted) continue;
            ++result.ratioPasses;
            result.hits.push_back({static_cast<int>(y), horizontalFinderCenter(runs[i].startX, lengths),
                y + 0.5, lengths, ratio.moduleSize, ratio.totalWidth, ratio.maximumRelativeError});
        }
    }
    return result;
}
