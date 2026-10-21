#include "AlignmentPattern.h"
#include "lodepng.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace
{
void validateMatrix(const BitMatrix& matrix)
{
    if (!matrix.width || !matrix.height ||
        matrix.width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        matrix.height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        static_cast<std::size_t>(matrix.width) >
            std::numeric_limits<std::size_t>::max() / matrix.height ||
        matrix.bits.size() != static_cast<std::size_t>(matrix.width) * matrix.height)
        throw std::invalid_argument("Invalid BitMatrix for alignment-pattern search");
    for (const auto bit : matrix.bits)
        if (bit > 1) throw std::invalid_argument("Non-binary alignment search matrix");
}

// Convert one binary row into maximal alternating runs. Candidate windows later
// take five consecutive runs beginning with black.
std::vector<BinaryRowRun> rowRuns(const BitMatrix& matrix, int row)
{
    std::vector<BinaryRowRun> runs;
    const std::size_t offset = static_cast<std::size_t>(row) * matrix.width;
    bool black = matrix.bits[offset] == 1;
    int start = 0;
    for (int x = 1; x < static_cast<int>(matrix.width); ++x)
    {
        const bool nextBlack = matrix.bits[offset + static_cast<std::size_t>(x)] == 1;
        if (nextBlack != black)
        {
            runs.push_back({black, start, x - start});
            start = x;
            black = nextBlack;
        }
    }
    runs.push_back({black, start, static_cast<int>(matrix.width) - start});
    return runs;
}

double windowCenter(int start, const std::array<int, 5>& runs)
{
    return static_cast<double>(start) + runs[0] + runs[1] + runs[2] / 2.0;
}

// Starting on a black seed, walk in both directions through center-black,
// white, and outer-black regions. Combining both legs reconstructs the full
// BLACK-WHITE-BLACK-WHITE-BLACK cross-section and refines its center.
AlignmentDirectionalScan scanDirection(
    const BitMatrix& matrix, int x, int y, int dx, int dy, int legLimit)
{
    AlignmentDirectionalScan result;
    const auto inside = [&](long long px, long long py) {
        return px >= 0 && py >= 0 && px < matrix.width && py < matrix.height;
    };
    const auto black = [&](long long px, long long py) {
        return matrix.bits[static_cast<std::size_t>(py) * matrix.width +
                           static_cast<std::size_t>(px)] == 1;
    };
    if (!inside(x, y) || !black(x, y))
    {
        result.failureReason = "seed_not_black_or_outside";
        return result;
    }

    result.runs[2] = 1; // Seed is counted once; each leg starts at its next pixel.
    int centralNegative = 0, centralPositive = 0;
    for (const int sign : {-1, 1})
    {
        long long px = static_cast<long long>(x) + sign * dx;
        long long py = static_cast<long long>(y) + sign * dy;
        int steps = 0;
        const int whiteIndex = sign < 0 ? 1 : 3;
        const int outerIndex = sign < 0 ? 0 : 4;
        const std::array<int, 3> indices{2, whiteIndex, outerIndex};
        for (int stage = 0; stage < 3; ++stage)
        {
            const bool expectedBlack = stage != 1;
            const int before = result.runs[indices[stage]];
            while (inside(px, py) && black(px, py) == expectedBlack)
            {
                if (steps >= legLimit)
                {
                    result.failureReason = "scan_leg_limit";
                    return result;
                }
                ++result.runs[indices[stage]];
                ++steps;
                if (stage == 0) (sign < 0 ? centralNegative : centralPositive)++;
                px += sign * dx;
                py += sign * dy;
            }
            if (stage != 0 && result.runs[indices[stage]] == before)
            {
                result.failureReason = "missing_white_or_outer_black_run";
                return result;
            }
            if (!inside(px, py))
            {
                result.failureReason = "image_boundary_before_run_completion";
                return result;
            }
        }
    }

    const double shift = (centralPositive - centralNegative) / 2.0;
    result.centerX = x + 0.5 + shift * dx;
    result.centerY = y + 0.5 + shift * dy;
    result.complete = true;
    return result;
}

AlignmentPatternSearchResult failed(const char* reason)
{
    AlignmentPatternSearchResult result;
    result.failureReason = reason;
    return result;
}

void blendPixel(Image& image, int x, int y, unsigned char r, unsigned char g,
                unsigned char b, unsigned char alpha)
{
    if (x < 0 || y < 0 || x >= static_cast<int>(image.width) || y >= static_cast<int>(image.height)) return;
    const std::size_t index = (static_cast<std::size_t>(y) * image.width +
                               static_cast<std::size_t>(x)) * 4;
    const unsigned inverse = 255U - alpha;
    image.rgba[index] = static_cast<unsigned char>((image.rgba[index] * inverse + r * alpha) / 255U);
    image.rgba[index + 1] = static_cast<unsigned char>((image.rgba[index + 1] * inverse + g * alpha) / 255U);
    image.rgba[index + 2] = static_cast<unsigned char>((image.rgba[index + 2] * inverse + b * alpha) / 255U);
    image.rgba[index + 3] = 255;
}

void cross(Image& image, double x, double y, int radius,
           unsigned char r, unsigned char g, unsigned char b)
{
    const int cx = static_cast<int>(std::lround(x));
    const int cy = static_cast<int>(std::lround(y));
    for (int d = -radius; d <= radius; ++d)
    {
        blendPixel(image, cx + d, cy, r, g, b, 255);
        blendPixel(image, cx, cy + d, r, g, b, 255);
    }
}

void rectangle(Image& image, int minX, int minY, int maxX, int maxY,
               unsigned char r, unsigned char g, unsigned char b)
{
    for (int x = minX; x <= maxX; ++x)
    {
        blendPixel(image, x, minY, r, g, b, 190);
        blendPixel(image, x, maxY, r, g, b, 190);
    }
    for (int y = minY; y <= maxY; ++y)
    {
        blendPixel(image, minX, y, r, g, b, 190);
        blendPixel(image, maxX, y, r, g, b, 190);
    }
}

void writeRuns(std::ofstream& out, const std::array<int, 5>& runs)
{
    out << '[';
    for (std::size_t i = 0; i < runs.size(); ++i)
        out << (i ? "," : "") << runs[i];
    out << ']';
}
}

AlignmentRatioResult checkAlignmentPatternRatio(
    const std::array<int, 5>& runs, double expectedModuleSize)
{
    using namespace AlignmentPatternConstants;
    AlignmentRatioResult result;
    if (!std::isfinite(expectedModuleSize) || expectedModuleSize <= 0.0) return result;

    long long total = 0;
    for (const int run : runs)
    {
        if (run <= 0) return result;
        total += run;
    }
    if (total < 5 || total > std::numeric_limits<int>::max()) return result;

    // All five alignment runs represent one module. Check both their equality
    // and agreement with the module size measured from the finder patterns.
    result.totalWidth = static_cast<int>(total);
    result.measuredModuleSize = static_cast<double>(total) / 5.0;
    for (const int run : runs)
        result.maximumRunRelativeError = std::max(result.maximumRunRelativeError,
            std::abs(run - result.measuredModuleSize) / result.measuredModuleSize);
    result.expectedModuleRelativeError =
        std::abs(result.measuredModuleSize - expectedModuleSize) / expectedModuleSize;
    result.accepted = result.maximumRunRelativeError <= MaximumRunRelativeError &&
                      result.expectedModuleRelativeError <= MaximumExpectedModuleRelativeError;
    return result;
}

AlignmentPatternSearchResult searchAlignmentPattern(
    const BitMatrix& matrix,
    const std::vector<PhysicalFinderCluster>& clusters,
    const ModuleSizeEstimationResult& moduleSize,
    const QrDimensionEstimationResult& dimension)
{
    using namespace AlignmentPatternConstants;
    validateMatrix(matrix);

    if (!dimension.valid) return failed("no_valid_qr_dimension");

    AlignmentPatternSearchResult result;
    result.dimension = dimension.dimension;
    result.version = dimension.version;
    // Version 1 contains no alignment pattern, so a successful skip is the
    // correct Stage-12 result rather than a failed search.
    if (dimension.version == 1)
    {
        result.valid = true;
        result.failureReason = "version_1_has_no_alignment_pattern";
        return result;
    }

    result.required = true;
    if (!moduleSize.valid || !moduleSize.inputsAvailable)
    {
        result.failureReason = "no_valid_module_size_estimate";
        return result;
    }
    if (dimension.clusterIndices != moduleSize.clusterIndices)
    {
        result.failureReason = "dimension_and_module_size_finder_ids_differ";
        return result;
    }

    result.clusterIndices = dimension.clusterIndices;
    result.moduleSize = moduleSize.estimatedModuleSize;
    if (!std::isfinite(result.moduleSize) || result.moduleSize <= 0.0)
    {
        result.failureReason = "module_size_is_not_finite_and_positive";
        return result;
    }
    const auto ids = result.clusterIndices;
    if (ids[0] >= clusters.size() || ids[1] >= clusters.size() || ids[2] >= clusters.size())
    {
        result.failureReason = "finder_index_out_of_range";
        return result;
    }

    const auto& tl = clusters[ids[0]];
    const auto& tr = clusters[ids[1]];
    const auto& bl = clusters[ids[2]];
    for (const auto* finder : {&tl, &tr, &bl})
        if (!std::isfinite(finder->centerX) || !std::isfinite(finder->centerY))
        {
            result.failureReason = "finder_center_is_not_finite";
            return result;
        }

    const int modulesBetweenFinderCenters = dimension.dimension - 7;
    if (modulesBetweenFinderCenters <= 0)
    {
        result.failureReason = "invalid_dimension_for_alignment_prediction";
        return result;
    }
    // The bottom-right alignment center is three modules inward from the virtual
    // bottom-right finder center. Apply that correction along both finder basis vectors.
    const double correction = 1.0 - 3.0 / modulesBetweenFinderCenters;
    result.predictedCenterX = tl.centerX + correction *
        ((tr.centerX - tl.centerX) + (bl.centerX - tl.centerX));
    result.predictedCenterY = tl.centerY + correction *
        ((tr.centerY - tl.centerY) + (bl.centerY - tl.centerY));
    if (!std::isfinite(result.predictedCenterX) || !std::isfinite(result.predictedCenterY) ||
        result.predictedCenterX < 0.0 || result.predictedCenterY < 0.0 ||
        result.predictedCenterX >= matrix.width || result.predictedCenterY >= matrix.height)
    {
        result.failureReason = "predicted_alignment_center_outside_image";
        return result;
    }

    // Search only a module-scaled square around the geometric prediction. This
    // limits false positives and keeps the work independent of image size.
    const int radius = std::max(1, static_cast<int>(std::ceil(SearchRadiusModules * result.moduleSize)));
    result.searchMinX = std::max(0, static_cast<int>(std::floor(result.predictedCenterX)) - radius);
    result.searchMaxX = std::min(static_cast<int>(matrix.width) - 1,
                                 static_cast<int>(std::floor(result.predictedCenterX)) + radius);
    result.searchMinY = std::max(0, static_cast<int>(std::floor(result.predictedCenterY)) - radius);
    result.searchMaxY = std::min(static_cast<int>(matrix.height) - 1,
                                 static_cast<int>(std::floor(result.predictedCenterY)) + radius);
    const int legLimit = std::max(3, static_cast<int>(std::ceil(ScanLegLimitModules * result.moduleSize)));

    // Each horizontal 1:1:1:1:1 window must survive a vertical cross-check and
    // a refined horizontal re-check before becoming a 2D candidate.
    double bestScore = std::numeric_limits<double>::infinity();
    for (int y = result.searchMinY; y <= result.searchMaxY; ++y)
    {
        ++result.rowsScanned;
        const auto runs = rowRuns(matrix, y);
        for (std::size_t i = 0; i + 5 <= runs.size(); ++i)
        {
            if (!runs[i].black) continue;
            std::array<int, 5> lengths{};
            for (std::size_t j = 0; j < lengths.size(); ++j) lengths[j] = runs[i + j].length;
            const double initialCenterX = windowCenter(runs[i].startX, lengths);
            if (initialCenterX < result.searchMinX || initialCenterX > result.searchMaxX) continue;
            ++result.windowsTested;

            const auto horizontalRatio = checkAlignmentPatternRatio(lengths, result.moduleSize);
            if (!horizontalRatio.accepted) continue;
            ++result.horizontalRatioPasses;

            ++result.verticalScansAttempted;
            const auto vertical = scanDirection(matrix,
                static_cast<int>(std::floor(initialCenterX)), y, 0, 1, legLimit);
            if (!vertical.complete) continue;
            const auto verticalRatio = checkAlignmentPatternRatio(vertical.runs, result.moduleSize);
            if (!verticalRatio.accepted) continue;
            ++result.verticalRatioPasses;

            ++result.horizontalRechecksAttempted;
            const auto refinedHorizontal = scanDirection(matrix,
                static_cast<int>(std::floor(initialCenterX)),
                static_cast<int>(std::floor(vertical.centerY)), 1, 0, legLimit);
            if (!refinedHorizontal.complete) continue;
            const auto refinedRatio = checkAlignmentPatternRatio(refinedHorizontal.runs, result.moduleSize);
            if (!refinedRatio.accepted) continue;

            const double axisDifference = std::hypot(
                refinedHorizontal.centerX - initialCenterX,
                vertical.centerY - (y + 0.5)) / result.moduleSize;
            if (axisDifference > MaximumAxisCenterDifferenceModules)
            {
                ++result.axisCenterFailures;
                continue;
            }

            AlignmentPatternCandidate candidate;
            candidate.centerX = refinedHorizontal.centerX;
            candidate.centerY = vertical.centerY;
            candidate.horizontalRuns = refinedHorizontal.runs;
            candidate.verticalRuns = vertical.runs;
            candidate.horizontalModuleSize = refinedRatio.measuredModuleSize;
            candidate.verticalModuleSize = verticalRatio.measuredModuleSize;
            candidate.horizontalRatioError = refinedRatio.maximumRunRelativeError;
            candidate.verticalRatioError = verticalRatio.maximumRunRelativeError;
            candidate.predictionDistanceModules = std::hypot(
                candidate.centerX - result.predictedCenterX,
                candidate.centerY - result.predictedCenterY) / result.moduleSize;
            candidate.axisCenterDifferenceModules = axisDifference;
            // Lower scores favor equal runs, finder-consistent module size,
            // intersecting axis centers, and proximity to the prediction.
            candidate.score = candidate.horizontalRatioError + candidate.verticalRatioError +
                refinedRatio.expectedModuleRelativeError + verticalRatio.expectedModuleRelativeError +
                candidate.axisCenterDifferenceModules +
                candidate.predictionDistanceModules / SearchRadiusModules;
            result.candidates.push_back(candidate);
            if (candidate.score < bestScore)
            {
                bestScore = candidate.score;
                result.selected = candidate;
            }
        }
    }

    if (result.candidates.empty())
    {
        result.failureReason = "no_concentric_alignment_candidate_passed_cross_checks";
        return result;
    }

    result.found = true;
    result.valid = true;
    return result;
}

void saveAlignmentPatternDebug(
    const Image& image,
    const AlignmentPatternSearchResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    if (!image.width || !image.height || image.rgba.size() != image.pixelCount() * Image::channels)
        throw std::invalid_argument("Invalid image for alignment-pattern debug");

    Image debug = image;
    if (result.required && result.searchMaxX >= result.searchMinX && result.searchMaxY >= result.searchMinY)
    {
        rectangle(debug, result.searchMinX, result.searchMinY,
                  result.searchMaxX, result.searchMaxY, 0, 128, 255);
        cross(debug, result.predictedCenterX, result.predictedCenterY,
              AlignmentPatternConstants::PredictedMarkerRadius, 0, 128, 255);
        for (const auto& candidate : result.candidates)
            cross(debug, candidate.centerX, candidate.centerY, 3, 255, 215, 0);
        if (result.found)
            cross(debug, result.selected.centerX, result.selected.centerY,
                  AlignmentPatternConstants::FoundMarkerRadius, 0, 255, 0);
        else
            cross(debug, result.predictedCenterX, result.predictedCenterY,
                  AlignmentPatternConstants::FoundMarkerRadius, 255, 0, 0);
    }

    const std::string pngFilename = outputDirectory + "/10_alignment_pattern_search.png";
    const unsigned pngError = lodepng::encode(
        pngFilename, debug.rgba, debug.width, debug.height, LCT_RGBA, 8);
    if (pngError)
        throw std::runtime_error("Alignment debug PNG writing failed: " +
                                 std::string(lodepng_error_text(pngError)));

    std::ofstream out(outputDirectory + "/10_alignment_pattern_search.txt");
    if (!out) throw std::runtime_error("Cannot open alignment-pattern report");
    using namespace AlignmentPatternConstants;
    out << "Stage 12: QR alignment-pattern search\n"
        << "input_filename=" << inputFilename << '\n'
        << "radial_pattern=1:1:1 diameter_pattern=1:1:1:1:1\n"
        << "search_radius_modules=" << SearchRadiusModules
        << " scan_leg_limit_modules=" << ScanLegLimitModules
        << " maximum_run_relative_error=" << MaximumRunRelativeError
        << " maximum_expected_module_relative_error=" << MaximumExpectedModuleRelativeError
        << " maximum_axis_center_difference_modules=" << MaximumAxisCenterDifferenceModules << '\n'
        << "dimension=" << result.dimension << " version=" << result.version
        << " required=" << result.required << '\n'
        << std::fixed << std::setprecision(6);

    if (result.required)
        out << "module_size=" << result.moduleSize
            << " selected_cluster_ids_TL_TR_BL=" << result.clusterIndices[0] << ','
            << result.clusterIndices[1] << ',' << result.clusterIndices[2] << '\n'
            << "predicted_center=" << result.predictedCenterX << ',' << result.predictedCenterY << '\n'
            << "search_bounds=" << result.searchMinX << ',' << result.searchMinY << " to "
            << result.searchMaxX << ',' << result.searchMaxY << '\n'
            << "rows_scanned=" << result.rowsScanned
            << " windows_tested=" << result.windowsTested
            << " horizontal_ratio_passes=" << result.horizontalRatioPasses
            << " vertical_scans_attempted=" << result.verticalScansAttempted
            << " vertical_ratio_passes=" << result.verticalRatioPasses
            << " horizontal_rechecks_attempted=" << result.horizontalRechecksAttempted
            << " axis_center_failures=" << result.axisCenterFailures
            << " accepted_candidates=" << result.candidates.size() << '\n';
    else
        out << "module_size=N/A predicted_center=N/A search_bounds=N/A\n";

    for (std::size_t i = 0; i < result.candidates.size(); ++i)
    {
        const auto& candidate = result.candidates[i];
        out << "candidate_" << i << ": center=" << candidate.centerX << ',' << candidate.centerY
            << " horizontal_runs=";
        writeRuns(out, candidate.horizontalRuns);
        out << " vertical_runs=";
        writeRuns(out, candidate.verticalRuns);
        out << " horizontal_module_size=" << candidate.horizontalModuleSize
            << " vertical_module_size=" << candidate.verticalModuleSize
            << " horizontal_ratio_error=" << candidate.horizontalRatioError
            << " vertical_ratio_error=" << candidate.verticalRatioError
            << " prediction_distance_modules=" << candidate.predictionDistanceModules
            << " axis_center_difference_modules=" << candidate.axisCenterDifferenceModules
            << " score=" << candidate.score << '\n';
    }

    if (result.valid && !result.required)
        out << "result=NOT_REQUIRED reason=" << result.failureReason << '\n';
    else if (result.valid && result.found)
        out << "result=FOUND center=" << result.selected.centerX << ',' << result.selected.centerY
            << " score=" << result.selected.score << '\n';
    else
        out << "result=REJECT reason=" << result.failureReason << '\n';
    out << "PNG: blue=prediction/search bounds, yellow=accepted candidates, "
           "green=selected alignment center, red=required but not found\n";

    out.close();
    if (!out) throw std::runtime_error("Cannot write alignment-pattern report");
}
