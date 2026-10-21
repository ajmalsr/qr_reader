#include "Otsu.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <stdexcept>

GrayImage saveBinarizationComparison(const BinarizationResult& local,
                                    const OtsuResult& otsu, const std::string& filename)
{
    using namespace BinarizationConstants;
    // Reuse validation and the established black/white PNG mapping.
    GrayImage difference = binaryForPng(local.matrix);
    const GrayImage otsuPng = binaryForPng(otsu.matrix);
    if (difference.width != otsuPng.width || difference.height != otsuPng.height || local.blocks.empty())
        throw std::invalid_argument("Cannot compare mismatched or empty binarization results");
    std::size_t bothBlack = 0, bothWhite = 0, localOnlyBlack = 0, otsuOnlyBlack = 0;
    for (std::size_t i = 0; i < difference.pixels.size(); ++i)
    {
        const bool l = local.matrix.bits[i] == BlackBit;
        const bool o = otsu.matrix.bits[i] == BlackBit;
        bothBlack += l && o;
        bothWhite += !l && !o;
        localOnlyBlack += l && !o;
        otsuOnlyBlack += !l && o;
        difference.pixels[i] = l != o ? WhitePng : BlackPng;
    }
    unsigned minimum = WhitePng, maximum = BlackPng;
    long double thresholdSum = 0;
    for (const auto& b : local.blocks)
    {
        minimum = std::min(minimum, b.threshold);
        maximum = std::max(maximum, b.threshold);
        thresholdSum += b.threshold;
    }
    const std::size_t total = difference.pixels.size();
    const std::size_t disagree = localOnlyBlack + otsuOnlyBlack;
    std::ofstream out(filename);
    if (!out) throw std::runtime_error("Cannot open comparison statistics: " + filename);
    out << std::fixed << std::setprecision(6)
        << "Local Hybrid-inspired vs global Otsu; same input luminance\n"
        << "classification: Y <= threshold is black for both methods\n"
        << "otsu_histogram_bins=" << OtsuConstants::HistogramBins
        << " candidate_thresholds=" << OtsuConstants::FirstThreshold << ".."
        << OtsuConstants::LastThreshold << " tie_policy=lowest uniform_fallback="
        << OtsuConstants::UniformThreshold << '\n'
        << "width=" << difference.width << " height=" << difference.height << " pixels=" << total << '\n'
        << "otsu_threshold=" << otsu.threshold << " otsu_valid_split=" << otsu.hasValidSplit
        << " otsu_between_class_variance=" << otsu.betweenClassVariance << '\n'
        << "local_threshold_min=" << minimum << " local_threshold_max=" << maximum
        << " local_threshold_mean=" << thresholdSum / local.blocks.size() << '\n'
        << "local_black=" << bothBlack + localOnlyBlack << " local_white=" << bothWhite + otsuOnlyBlack << '\n'
        << "otsu_black=" << bothBlack + otsuOnlyBlack << " otsu_white=" << bothWhite + localOnlyBlack << '\n'
        << "both_black=" << bothBlack << " both_white=" << bothWhite << '\n'
        << "local_black_otsu_white=" << localOnlyBlack << '\n'
        << "local_white_otsu_black=" << otsuOnlyBlack << '\n'
        << "disagreement_pixels=" << disagree
        << " disagreement_percent=" << 100.0L * disagree / total << '\n'
        << "agreement_percent=" << 100.0L * (total - disagree) / total << '\n'
        << "difference_png: white=disagreement, black=agreement\n"
        << "Agreement is not accuracy; no ground-truth labels or QR detection evaluated.\n"
        << "luminance histogram_count\n";
    for (unsigned v = 0; v < OtsuConstants::HistogramBins; ++v)
        out << v << ' ' << otsu.histogram[v] << '\n';
    out.close();
    if (!out) throw std::runtime_error("Cannot write comparison statistics: " + filename);
    return difference;
}
