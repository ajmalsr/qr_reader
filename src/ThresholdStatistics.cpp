#include "Binarizer.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <stdexcept>

void saveThresholdStatistics(const BinarizationResult& result, const std::string& filename)
{
    using namespace BinarizationConstants;
    if (result.blocks.empty())
        throw std::invalid_argument("No threshold statistics to save");
    std::ofstream out(filename);
    if (!out) throw std::runtime_error("Cannot open threshold statistics: " + filename);
    // Reduce the block measurements to summary values before writing the full table.
    unsigned minimum = WhitePng, maximum = BlackPng;
    std::size_t low = 0, corrected = 0;
    double thresholdSum = 0;
    for (const auto& block : result.blocks)
    {
        minimum = std::min(minimum, block.threshold);
        maximum = std::max(maximum, block.threshold);
        thresholdSum += block.threshold;
        low += block.lowContrast;
        corrected += block.usedNeighbor;
    }
    const auto black = std::count(result.matrix.bits.begin(), result.matrix.bits.end(), BlackBit);

    // Store the active constants with the results so the report remains self-describing.
    out << "Local binarization inspired by ZXing-C++ v2.2.1 HybridBinarizer\n"
        << "block_size=" << BlockSize << " smoothing_size=" << SmoothingSize
        << " smoothing_radius=" << SmoothingRadius << " minimum_dimension=" << MinimumDimension
        << " dynamic_range_cutoff=" << MinimumDynamicRange << '\n'
        << "low_contrast_divisor=" << LowContrastDivisor << " neighbor_weights="
        << UpperWeight << ',' << LeftWeight << ',' << DiagonalWeight
        << " neighbor_divisor=" << NeighborDivisor << '\n'
        << "classification: Y <= threshold is black; matrix 1=black, 0=white\n"
        << "Edges: shifted full blocks, later blocks overwrite overlaps\n"
        << "Full min/max measured; no global fallback for small images\n"
        << "width=" << result.matrix.width << " height=" << result.matrix.height << '\n'
        << "blocks_wide=" << result.blocksWide << " blocks_high=" << result.blocksHigh
        << " total_blocks=" << result.blocks.size() << '\n'
        << "low_contrast_blocks=" << low << " neighbor_corrected_blocks=" << corrected << '\n'
        << std::fixed << std::setprecision(3)
        << "threshold_min=" << minimum << " threshold_max=" << maximum
        << " threshold_mean=" << thresholdSum / result.blocks.size() << '\n'
        << "black_pixels=" << black << " white_pixels=" << result.matrix.bits.size() - black << '\n'
        << "block_x block_y pixel_x pixel_y min max range mean black_point threshold low_contrast neighbor_used\n";
    // Preserve the binarizer's row-major block order in the diagnostic output.
    for (unsigned y = 0; y < result.blocksHigh; ++y)
        for (unsigned x = 0; x < result.blocksWide; ++x)
        {
            const auto& b = result.blocks[static_cast<std::size_t>(y) * result.blocksWide + x];
            out << x << ' ' << y << ' ' << b.x << ' ' << b.y << ' '
                << b.minimum << ' ' << b.maximum << ' ' << b.dynamicRange << ' '
                << b.mean << ' ' << b.blackPoint << ' ' << b.threshold << ' '
                << b.lowContrast << ' ' << b.usedNeighbor << '\n';
        }
    out.close();
    if (!out) throw std::runtime_error("Cannot write threshold statistics: " + filename);
}
