#ifndef QR_LEARNING_OTSU_H
#define QR_LEARNING_OTSU_H
#include "Binarizer.h"
#include <array>

namespace OtsuConstants
{
constexpr unsigned HistogramBins = 256;
constexpr unsigned FirstThreshold = 0;
constexpr unsigned LastThreshold = HistogramBins - 2;
constexpr unsigned UniformThreshold = 0;
}
struct OtsuResult
{
    BitMatrix matrix;
    unsigned threshold = OtsuConstants::UniformThreshold;
    bool hasValidSplit = false;
    long double betweenClassVariance = 0;
    std::array<std::size_t, OtsuConstants::HistogramBins> histogram{};
};
OtsuResult binarizeOtsu(const GrayImage& gray);
// Writes counts, threshold diagnostics and all histogram bins. The map is
// white where the methods disagree and black where they agree.
GrayImage saveBinarizationComparison(const BinarizationResult& local,
                                    const OtsuResult& otsu, const std::string& filename);
#endif
