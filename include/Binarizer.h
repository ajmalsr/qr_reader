#ifndef QR_LEARNING_BINARIZER_H
#define QR_LEARNING_BINARIZER_H
#include "GrayImage.h"

namespace BinarizationConstants
{
constexpr unsigned BlockSize = 8;              // Local threshold is computed per 8x8 pixel block.
constexpr unsigned SmoothingSize = 5;          // Black points are averaged over a 5x5 block window...
constexpr unsigned SmoothingRadius = 2;        // ...i.e. 2 blocks on each side of the center block.
constexpr unsigned MinimumDimension = BlockSize * SmoothingSize; // Need a full 5x5 block window to smooth.
constexpr unsigned MinimumDynamicRange = 24;   // Blocks with max-min <= this are considered "low contrast"
                                                // (e.g. blank background) and cannot reliably self-threshold.
constexpr unsigned LowContrastDivisor = 2;     // Low-contrast fallback black point: half of the block minimum.
// Low-contrast blocks borrow a corrected black point from already-processed neighbors
// (above, left, and upper-left), weighted more toward the left neighbor. This mirrors
// ZXing-C++'s HybridBinarizer behavior for uniform regions like the quiet zone.
constexpr unsigned UpperWeight = 1;
constexpr unsigned LeftWeight = 2;
constexpr unsigned DiagonalWeight = 1;
constexpr unsigned NeighborDivisor = UpperWeight + LeftWeight + DiagonalWeight;
constexpr unsigned BlackBit = 1;
constexpr unsigned WhiteBit = 0;
constexpr unsigned BlackPng = 0;
constexpr unsigned WhitePng = 255;
}

// Educational byte storage: one logical bit per pixel, 1=black, 0=white.
struct BitMatrix
{
    unsigned width = 0;
    unsigned height = 0;
    std::vector<std::uint8_t> bits;
};
struct BlockStatistics
{
    unsigned x = 0, y = 0; // Pixel origin, including edge shifts.
    unsigned minimum = 0, maximum = 0, dynamicRange = 0, mean = 0;
    unsigned blackPoint = 0, threshold = 0; // blackPoint = pass-1 estimate; threshold = pass-2 smoothed value.
    bool lowContrast = false, usedNeighbor = false;
};
struct BinarizationResult
{
    BitMatrix matrix;
    unsigned blocksWide = 0, blocksHigh = 0;
    std::vector<BlockStatistics> blocks; // Row-major block grid.
};
// Two-pass local (adaptive) binarization: pass 1 estimates a black point per 8x8 block
// from its own min/max/mean (with neighbor correction for low-contrast blocks); pass 2
// smooths black points over a 5x5 block neighborhood and classifies pixels against that
// smoothed threshold. This makes the binarizer tolerant to uneven lighting across the image,
// which is essential for detecting QR finder patterns under real-world photographs.
BinarizationResult binarizeLocal(const GrayImage& gray);
// Renders a BitMatrix (1=black, 0=white) as a viewable grayscale image (0=black, 255=white).
GrayImage binaryForPng(const BitMatrix& matrix);
void saveThresholdStatistics(const BinarizationResult& result, const std::string& filename);
#endif
