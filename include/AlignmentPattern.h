#ifndef QR_LEARNING_ALIGNMENT_PATTERN_H
#define QR_LEARNING_ALIGNMENT_PATTERN_H

#include "QrDimensionEstimation.h"
#include "Binarizer.h"
#include "Image.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// An alignment pattern is a 5x5-module black/white/black target. A diameter through
// its center is BLACK-WHITE-BLACK-WHITE-BLACK with equal 1-module runs; from the
// center outward, the concentric radial signature is therefore 1:1:1.
namespace AlignmentPatternConstants
{
constexpr double SearchRadiusModules = 4.0;
constexpr double ScanLegLimitModules = 5.0;
constexpr double MaximumRunRelativeError = 0.75;
constexpr double MaximumExpectedModuleRelativeError = 0.50;
constexpr double MaximumAxisCenterDifferenceModules = 1.0;
constexpr int PredictedMarkerRadius = 5;
constexpr int FoundMarkerRadius = 6;
}

struct AlignmentRatioResult
{
    bool accepted = false;
    int totalWidth = 0;
    double measuredModuleSize = 0.0;
    double maximumRunRelativeError = 0.0;
    double expectedModuleRelativeError = 0.0;
};

struct AlignmentDirectionalScan
{
    bool complete = false;
    std::array<int, 5> runs{};
    double centerX = 0.0;
    double centerY = 0.0;
    std::string failureReason;
};

struct AlignmentPatternCandidate
{
    double centerX = 0.0;
    double centerY = 0.0;
    std::array<int, 5> horizontalRuns{};
    std::array<int, 5> verticalRuns{};
    double horizontalModuleSize = 0.0;
    double verticalModuleSize = 0.0;
    double horizontalRatioError = 0.0;
    double verticalRatioError = 0.0;
    double predictionDistanceModules = 0.0;
    double axisCenterDifferenceModules = 0.0;
    double score = 0.0;
};

struct AlignmentPatternSearchResult
{
    bool valid = false;       // Stage completed successfully, including version-1 skip.
    bool required = false;    // False only for version 1 or invalid upstream input.
    bool found = false;
    std::string failureReason;

    int dimension = 0;
    int version = 0;
    double moduleSize = 0.0;
    std::array<std::size_t, 3> clusterIndices{};
    double predictedCenterX = 0.0;
    double predictedCenterY = 0.0;
    int searchMinX = 0, searchMaxX = -1, searchMinY = 0, searchMaxY = -1;

    std::size_t rowsScanned = 0;
    std::size_t windowsTested = 0;
    std::size_t horizontalRatioPasses = 0;
    std::size_t verticalScansAttempted = 0;
    std::size_t verticalRatioPasses = 0;
    std::size_t horizontalRechecksAttempted = 0;
    std::size_t axisCenterFailures = 0;
    std::vector<AlignmentPatternCandidate> candidates;
    AlignmentPatternCandidate selected;
};

// Public for focused unit tests and for documenting the exact tolerance calculation.
AlignmentRatioResult checkAlignmentPatternRatio(
    const std::array<int, 5>& runs, double expectedModuleSize);

AlignmentPatternSearchResult searchAlignmentPattern(
    const BitMatrix& matrix,
    const std::vector<PhysicalFinderCluster>& clusters,
    const ModuleSizeEstimationResult& moduleSize,
    const QrDimensionEstimationResult& dimension);

// Writes 10_alignment_pattern_search.png and .txt. Blue marks the prediction/search
// region, yellow marks accepted 2D candidates, green marks the chosen center, and red
// marks a required alignment prediction when no candidate survives.
void saveAlignmentPatternDebug(
    const Image& image,
    const AlignmentPatternSearchResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
