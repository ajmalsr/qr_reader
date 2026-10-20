#ifndef QR_LEARNING_HORIZONTAL_FINDER_H
#define QR_LEARNING_HORIZONTAL_FINDER_H
#include "Binarizer.h"
#include <array>

// A QR finder pattern (the three squares in a QR code's corners) is a concentric
// black-white-black-white-black square. Any horizontal (or vertical) line crossing its
// center sees five runs of pixel lengths in the ratio 1:1:3:1:1 (black:white:black:white:black),
// i.e. 7 modules wide in total. This file searches binarized image rows for that signature.
namespace HorizontalFinderConstants
{
constexpr int RowSkip = 3;       // Only every 3rd row is scanned; a finder is >=7 modules tall,
                                  // so no finder can be missed entirely between skipped rows.
constexpr int FirstRow = RowSkip - 1;
constexpr int MinimumCenterPixels = 3;   // Center (3rd) run must be at least this many pixels.
constexpr int CenterOuterMultiplier = 2; // Center run must be >= 2x the larger outer black run
                                          // (cheap pre-filter before the stricter ratio check).
constexpr int TotalModules = 7;          // 1+1+3+1+1 modules across a finder pattern.
constexpr double RelativeTolerance = 0.5; // Each run may differ from its expected size (as a
                                           // fraction of estimated module size) by up to this much.
constexpr std::array<int, 5> ExpectedRuns{1, 1, 3, 1, 1}; // The canonical finder ratio.
constexpr int MarkerRadius = 3;
}
struct BinaryRowRun
{
    bool black;
    int startX;
    int length;
};
struct FinderRatioResult
{
    bool accepted = false;
    int totalWidth = 0;
    double moduleSize = 0;
    double maximumRelativeError = 0;
};
struct HorizontalFinderHit
{
    int row;
    double centerX;
    double centerY;
    std::array<int, 5> runLengths;
    double estimatedModuleSize;
    int totalWidth;
    double maximumRelativeError;
};
struct HorizontalFinderResult
{
    std::vector<HorizontalFinderHit> hits;
    std::size_t rowsScanned = 0;
    std::size_t windowsTested = 0; // Black-starting five-run windows only.
    std::size_t fastPasses = 0;
    std::size_t ratioPasses = 0;
};
// Run-length-encodes one row of the binary matrix into alternating black/white runs.
std::vector<BinaryRowRun> horizontalRowRuns(const BitMatrix& matrix, int row);
// Cheap early-out check before the more expensive ratio computation: rejects most
// non-finder run sequences using only integer comparisons (no division).
bool finderFastPlausibility(const std::array<int, 5>& runs);
// All runs positive, total >= 7, and each relative error STRICTLY below 0.5.
FinderRatioResult checkFinderRatio(const std::array<int, 5>& runs);
// Pixel-edge coordinates: pixel x occupies [x, x+1).
double horizontalFinderCenter(int windowStart, const std::array<int, 5>& runs);
// Scans every third row of the binary matrix for 1:1:3:1:1 run windows, applying the
// fast-plausibility filter and then the strict ratio check. Produces one HorizontalFinderHit
// per accepted window; a single physical finder typically yields several hits from
// consecutive scanned rows. No vertical/geometric confirmation happens here (see Milestone 5).
HorizontalFinderResult searchHorizontalFinders(const BitMatrix& matrix);
void saveHorizontalFinderDebug(const Image& original, const HorizontalFinderResult& result,
                               const std::string& pngFilename, const std::string& textFilename);
#endif
