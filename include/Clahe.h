#ifndef QR_LEARNING_CLAHE_H
#define QR_LEARNING_CLAHE_H
#include "GrayImage.h"
#include <string>
// Experiment defaults. The tile grid controls how locally contrast is measured;
// ClipLimit limits histogram peaks so a small noisy range is not amplified without bound.
namespace ClaheConstants
{
constexpr unsigned TileColumns = 8;
constexpr unsigned TileRows = 8;
constexpr unsigned HistogramBins = 256;
constexpr double ClipLimit = 2.0;
}
// Kept as a settings object so future experiments can change parameters without
// changing the luminance, binarization, or finder-search interfaces.
struct ClaheSettings
{
    unsigned tileColumns = ClaheConstants::TileColumns;
    unsigned tileRows = ClaheConstants::TileRows;
    double clipLimit = ClaheConstants::ClipLimit;
};
// Input: the original 8-bit luminance image. Output: a separate enhanced image
// with identical dimensions; the input is never modified.
GrayImage applyClahe(const GrayImage& input, const ClaheSettings& settings = {});
// Records the exact experiment settings and before/after luminance summaries.
void saveClaheStatistics(const GrayImage& before, const GrayImage& after,
                         const ClaheSettings& settings, const std::string& filename);
#endif


