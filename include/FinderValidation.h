#ifndef QR_LEARNING_FINDER_VALIDATION_H
#define QR_LEARNING_FINDER_VALIDATION_H
#include "HorizontalFinder.h"
#include <limits>
// Milestone 4 only confirms the 1:1:3:1:1 pattern along a single scanned row, which is
// prone to false positives (long horizontal strokes of unrelated content can coincidentally
// match). Milestone 5 confirms each raw horizontal hit is a genuine 2D finder pattern by
// re-scanning it vertically and horizontally through its own refined center, and optionally
// diagonally, checking that all axes agree on the same 1:1:3:1:1 ratio and total size.
namespace FinderValidationConstants
{
constexpr double SizeRelativeTolerance = 0.35;   // Max allowed relative difference between
                                                  // the horizontal and vertical total widths.
constexpr double VerticalCenterModules = 1.5;    // Max allowed vertical center displacement
                                                  // between the raw hit and the vertical scan,
                                                  // expressed in modules (estimated module sizes).
constexpr double HorizontalCenterModules = 1.0;  // Same, but for the horizontal re-check center.
constexpr double ScanLegWidthMultiplier = 1.5;   // Each scan direction searches at most this
                                                  // many times the raw hit's total width, so a
                                                  // scan can't run away across the whole image.
constexpr double DiagonalRelativeTolerance = 0.75; // Looser tolerance for the diagonal sanity
                                                    // check (diagonal pixel spacing is uneven).
constexpr bool RequireDiagonalByDefault = false;   // Diagonal check is diagnostic-only by default.
constexpr int MarkerRadius = 3;
}
// Result of scanning outward from a seed pixel in one direction (dx,dy) through the
// 5-run finder pattern (black,white,black,white,black), symmetric about the seed.
struct DirectionalFinderScan
{
    bool complete = false;              // True only if all 5 runs terminated within limit pixels.
    std::array<int,5> runs{};
    double centerX = std::numeric_limits<double>::quiet_NaN(); // Refined center along this axis.
    double centerY = std::numeric_limits<double>::quiet_NaN();
    int minimumX = 0, minimumY = 0, maximumX = 0, maximumY = 0; // Visited pixel bounds (debug).
    std::string reason; // Why the scan is incomplete, e.g. "scan_leg_limit".
};
// A finder candidate that has passed vertical cross-check, horizontal re-check, size
// consistency and center consistency (Milestone 5's full validation chain).
struct ValidatedFinderCandidate
{
    double centerX, centerY;
    double horizontalModuleSize, verticalModuleSize, estimatedModuleSize;
    std::array<int,5> horizontalRuns, verticalRuns;
    double horizontalRatioError, verticalRatioError;
    bool diagonalPassed;
};
// Full diagnostic trail for one raw Milestone-4 hit, whether accepted or rejected, so
// debug output can explain exactly where and why validation stopped.
struct FinderValidationRecord
{
    HorizontalFinderHit original;
    DirectionalFinderScan vertical, horizontal;
    FinderRatioResult verticalRatio, horizontalRatio;
    bool verticalPassed = false, accepted = false;
    bool diagonalAttempted = false, diagonalPassed = false;
    double centerDisplacementY = std::numeric_limits<double>::quiet_NaN();
    double centerDisplacementX = std::numeric_limits<double>::quiet_NaN();
    std::string rejectionReason;
};
struct FinderValidationResult
{
    std::vector<FinderValidationRecord> records;
    std::vector<ValidatedFinderCandidate> candidates;
    std::size_t verticalScansAttempted = 0, incompleteVerticalScans = 0;
    std::size_t verticalRatioFailures = 0, sizeConsistencyFailures = 0;
    std::size_t centerConsistencyFailures = 0, horizontalRecheckFailures = 0;
    std::size_t verticalPasses = 0, axisPasses = 0;
    std::size_t diagonalAttempts = 0, diagonalFailures = 0, diagonalPasses = 0;
    bool diagonalRequired = FinderValidationConstants::RequireDiagonalByDefault;
};
// Directions allowed: vertical, horizontal and the two diagonals.
DirectionalFinderScan scanFinderDirection(const BitMatrix& matrix, int x, int y,
                                         int dx, int dy, int maxLegPixels);
// True if at least one of the two diagonals through (x,y) shows a plausible 1:1:3:1:1
// pattern. Used only as a diagnostic; not required for acceptance unless requireDiagonal is set.
bool finderDiagonalSanity(const BitMatrix& matrix, double x, double y, int maxLegPixels);
// Runs the full Milestone 5 validation chain over every raw Milestone-4 hit:
// vertical scan -> vertical ratio/size/center checks -> horizontal re-check at the
// vertically-refined row -> horizontal ratio/size/center checks -> optional diagonal check.
// Multiple raw hits from adjacent scanned rows of the same physical finder each produce
// their own (nearly identical) ValidatedFinderCandidate; merging them is Milestone 6.
FinderValidationResult validateFinderHits(const BitMatrix& matrix,
    const std::vector<HorizontalFinderHit>& hits,
    bool requireDiagonal = FinderValidationConstants::RequireDiagonalByDefault);
// outputDirectory is selected by main: "output" normally or "output/clahe"
// for the isolated experiment. It changes filenames only, not validation.
void saveFinderValidationDebug(const Image& image, const FinderValidationResult& result,
                               const std::string& outputDirectory = "output");
#endif


