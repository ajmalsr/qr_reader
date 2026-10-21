#include "HorizontalFinder.h"
#include "lodepng.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

// Writes a debug PNG with red crosses at each raw finder hit's center (overlaid on the
// original RGBA image, not the binary matrix) plus a text log of every hit's runs, module
// size and ratio error. This is Milestone 4 output: no validation has happened yet, so
// false positives are expected and normal at this stage.
void saveHorizontalFinderDebug(const Image& original, const HorizontalFinderResult& result,
                               const std::string& pngFilename, const std::string& textFilename)
{
    if (original.width == 0 || original.height == 0 ||
        static_cast<std::size_t>(original.width) >
            std::numeric_limits<std::size_t>::max() / original.height ||
        original.pixelCount() > std::numeric_limits<std::size_t>::max() / Image::channels ||
        original.rgba.size() != original.pixelCount() * Image::channels)
        throw std::invalid_argument("Invalid RGBA image for finder markers");
    Image marked = original; // Debug overlay never changes source pixels or binary matrix.
    for (const auto& hit : result.hits)
    {
        const long long cx = static_cast<long long>(std::floor(hit.centerX));
        const long long cy = static_cast<long long>(std::floor(hit.centerY));
        for (int d = -HorizontalFinderConstants::MarkerRadius;
             d <= HorizontalFinderConstants::MarkerRadius; ++d)
            for (int axis = 0; axis < 2; ++axis)
            {
                const long long x = cx + (axis == 0 ? d : 0);
                const long long y = cy + (axis == 1 ? d : 0);
                if (x < 0 || y < 0 || x >= marked.width || y >= marked.height) continue;
                const std::size_t p = (static_cast<std::size_t>(y) * marked.width + x) * Image::channels;
                marked.rgba[p] = 255; marked.rgba[p + 1] = 0;
                marked.rgba[p + 2] = 0; marked.rgba[p + 3] = 255; // Red cross.
            }
    }
    const unsigned error = lodepng::encode(pngFilename, marked.rgba, marked.width, marked.height, LCT_RGBA, 8);
    if (error) throw std::runtime_error("Finder debug PNG writing failed: " +
                                      std::string(lodepng_error_text(error)));
    std::ofstream out(textFilename);
    if (!out) throw std::runtime_error("Cannot open horizontal finder hit log");
    out << "Raw horizontal candidates only; no validation or merging\n"
        << "input_matrix=local_binarization black=1 white=0\n"
        << "row_skip=" << HorizontalFinderConstants::RowSkip
        << " first_row=" << HorizontalFinderConstants::FirstRow << '\n'
        << "fast: center>=3, center>=2*max(outer_black), center>=max(neighbor_white)\n"
        << "ratio: positive runs; total>=7; max(abs(run-expected)/expected)<"
        << HorizontalFinderConstants::RelativeTolerance << " expected=(total/7)*[1,1,3,1,1]\n"
        << "coordinates: pixel edges; centerY=row+0.5; markers red crosses radius="
        << HorizontalFinderConstants::MarkerRadius << '\n'
        << "rows_scanned=" << result.rowsScanned << " windows_tested=" << result.windowsTested
        << " (black-starting windows only)\n"
        << "fast_plausibility_passes=" << result.fastPasses << " final_ratio_passes=" << result.ratioPasses
        << " horizontal_hits=" << result.hits.size() << '\n'
        << "row centerX centerY run0 run1 run2 run3 run4 totalWidth moduleSize maxRelativeError\n"
        << std::fixed << std::setprecision(6);
    for (const auto& h : result.hits)
    {
        out << h.row << ' ' << h.centerX << ' ' << h.centerY;
        for (int run : h.runLengths) out << ' ' << run;
        out << ' ' << h.totalWidth << ' ' << h.estimatedModuleSize << ' ' << h.maximumRelativeError << '\n';
    }
    out.close();
    if (!out) throw std::runtime_error("Cannot write horizontal finder hit log");
}
