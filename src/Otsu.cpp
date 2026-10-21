#include "Otsu.h"
#include <limits>
#include <stdexcept>

// Global (single-threshold) binarization, used only as a diagnostic comparison against
// the local/adaptive binarizer above. Otsu's method picks the threshold that maximizes
// the between-class variance of a two-class (dark/light) split of the luminance histogram -
// equivalently, the split that best separates the image into two distinct luminance groups.
// It works well for evenly-lit images but, unlike the local binarizer, cannot compensate for
// uneven lighting across a photograph.
OtsuResult binarizeOtsu(const GrayImage& gray)
{
    if (gray.width == 0 || gray.height == 0 ||
        static_cast<std::size_t>(gray.width) >
            std::numeric_limits<std::size_t>::max() / gray.height ||
        gray.pixels.size() != gray.pixelCount())
        throw std::invalid_argument("Invalid luminance image for Otsu thresholding");

    OtsuResult result;
    for (auto value : gray.pixels) ++result.histogram[value];
    long double totalSum = 0;
    for (unsigned value = 0; value < OtsuConstants::HistogramBins; ++value)
        totalSum += static_cast<long double>(value) * result.histogram[value];

    const std::size_t totalCount = gray.pixels.size();
    std::size_t darkCount = 0;
    long double darkSum = 0;
    // Cumulative counts/sums make each candidate split constant time.
    for (unsigned t = OtsuConstants::FirstThreshold; t <= OtsuConstants::LastThreshold; ++t)
    {
        darkCount += result.histogram[t];
        darkSum += static_cast<long double>(t) * result.histogram[t];
        const std::size_t lightCount = totalCount - darkCount;
        if (darkCount == 0 || lightCount == 0) continue;
        const long double darkMean = darkSum / darkCount;
        const long double lightMean = (totalSum - darkSum) / lightCount;
        const long double difference = darkMean - lightMean;
        const long double darkWeight = static_cast<long double>(darkCount) / totalCount;
        const long double lightWeight = static_cast<long double>(lightCount) / totalCount;
        // Between-class variance: weight(dark) * weight(light) * (mean(dark) - mean(light))^2.
        // Maximizing this over all candidate thresholds t finds the split with the most
        // separated, most evenly-weighted dark/light classes.
        const long double variance = darkWeight * lightWeight * difference * difference;
        // Strict improvement retains the lowest threshold on equal scores.
        if (!result.hasValidSplit || variance > result.betweenClassVariance)
        {
            result.hasValidSplit = true;
            result.threshold = t;
            result.betweenClassVariance = variance;
        }
    }
    // If uniform, retain the documented fallback threshold 0.
    result.matrix.width = gray.width;
    result.matrix.height = gray.height;
    result.matrix.bits.resize(totalCount);
    for (std::size_t i = 0; i < totalCount; ++i)
        result.matrix.bits[i] = gray.pixels[i] <= result.threshold
            ? BinarizationConstants::BlackBit : BinarizationConstants::WhiteBit;
    return result;
}
