#include "FinderFailureAnalysis.h"
#include "lodepng.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
struct Color { unsigned char r, g, b; };

// Maps every outcome category actually produced by FinderValidation.cpp to a distinct color
// used consistently across every Milestone 7 debug image.
Color categoryColor(const std::string& category)
{
    if (category == "ACCEPTED") return {0, 170, 0};
    if (category == "INCOMPLETE_VERTICAL") return {255, 140, 0};
    if (category == "VERTICAL_RATIO_FAILURE") return {220, 0, 0};
    if (category == "SIZE_CONSISTENCY_FAILURE") return {160, 0, 200};
    if (category == "CENTER_CONSISTENCY_FAILURE") return {0, 90, 220};
    if (category == "HORIZONTAL_RECHECK_FAILURE") return {0, 170, 170};
    return {120, 120, 120}; // OTHER
}

void validateDebugImage(const Image& image)
{
    if (!image.width || !image.height ||
        static_cast<std::size_t>(image.width) > std::numeric_limits<std::size_t>::max() / image.height ||
        image.pixelCount() > std::numeric_limits<std::size_t>::max() / 4 || image.rgba.size() != image.pixelCount() * 4)
        throw std::invalid_argument("Invalid image for finder failure analysis debug");
}

void setPixel(Image& image, long long x, long long y, Color c)
{
    if (x < 0 || y < 0 || x >= image.width || y >= image.height) return;
    const auto i = (static_cast<std::size_t>(y) * image.width + x) * 4;
    image.rgba[i] = c.r; image.rgba[i + 1] = c.g; image.rgba[i + 2] = c.b; image.rgba[i + 3] = 255;
}

void fillSquare(Image& image, long long cx, long long cy, int halfSize, Color c)
{
    for (long long y = cy - halfSize; y <= cy + halfSize; ++y)
        for (long long x = cx - halfSize; x <= cx + halfSize; ++x)
            setPixel(image, x, y, c);
}

void drawLine(Image& image, long long x0, long long y0, long long x1, long long y1, Color c)
{
    long long dx = std::llabs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    long long dy = -std::llabs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    long long err = dx + dy;
    while (true)
    {
        setPixel(image, x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        const long long e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Minimal embedded 5x7 bitmap font covering only the characters used by candidate-ID digits
// and the outcome-category legend text, so labels can be drawn on debug PNGs without any
// external font or text-rendering library (consistent with this project's "no OpenCV, no
// third-party heavy dependencies" constraint - lodepng is the only external piece already used).
using Glyph = std::array<std::string, 7>;
const std::unordered_map<char, Glyph>& fontTable()
{
    static const std::unordered_map<char, Glyph> table = {
        {'0', {"01110","10001","10011","10101","11001","10001","01110"}},
        {'1', {"00100","01100","00100","00100","00100","00100","01110"}},
        {'2', {"01110","10001","00001","00010","00100","01000","11111"}},
        {'3', {"11111","00010","00100","00010","00001","10001","01110"}},
        {'4', {"00010","00110","01010","10010","11111","00010","00010"}},
        {'5', {"11111","10000","11110","00001","00001","10001","01110"}},
        {'6', {"00110","01000","10000","11110","10001","10001","01110"}},
        {'7', {"11111","00001","00010","00100","01000","01000","01000"}},
        {'8', {"01110","10001","10001","01110","10001","10001","01110"}},
        {'9', {"01110","10001","10001","01111","00001","00010","01100"}},
        {'A', {"01110","10001","10001","11111","10001","10001","10001"}},
        {'C', {"01111","10000","10000","10000","10000","10000","01111"}},
        {'D', {"11110","10001","10001","10001","10001","10001","11110"}},
        {'E', {"11111","10000","10000","11110","10000","10000","11111"}},
        {'F', {"11111","10000","10000","11110","10000","10000","10000"}},
        {'H', {"10001","10001","10001","11111","10001","10001","10001"}},
        {'I', {"01110","00100","00100","00100","00100","00100","01110"}},
        {'K', {"10001","10010","10100","11000","10100","10010","10001"}},
        {'L', {"10000","10000","10000","10000","10000","10000","11111"}},
        {'M', {"10001","11011","10101","10101","10001","10001","10001"}},
        {'N', {"10001","11001","10101","10011","10001","10001","10001"}},
        {'O', {"01110","10001","10001","10001","10001","10001","01110"}},
        {'P', {"11110","10001","10001","11110","10000","10000","10000"}},
        {'R', {"11110","10001","10001","11110","10100","10010","10001"}},
        {'S', {"01111","10000","10000","01110","00001","00001","11110"}},
        {'T', {"11111","00100","00100","00100","00100","00100","00100"}},
        {'U', {"10001","10001","10001","10001","10001","10001","01110"}},
        {'V', {"10001","10001","10001","10001","10001","01010","00100"}},
        {'Y', {"10001","10001","01010","00100","00100","00100","00100"}},
        {'Z', {"11111","00001","00010","00100","01000","10000","11111"}},
        {'_', {"00000","00000","00000","00000","00000","00000","11111"}},
        {' ', {"00000","00000","00000","00000","00000","00000","00000"}},
    };
    return table;
}

int drawChar(Image& image, long long x, long long y, char ch, Color c, int scale)
{
    const auto& table = fontTable();
    const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    auto it = table.find(upper);
    if (it == table.end()) it = table.find(' ');
    const auto& glyph = it->second;
    for (int row = 0; row < 7; ++row)
        for (int col = 0; col < 5; ++col)
            if (glyph[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] == '1')
                for (int sy = 0; sy < scale; ++sy)
                    for (int sx = 0; sx < scale; ++sx)
                        setPixel(image, x + col * scale + sx, y + row * scale + sy, c);
    return (5 + 1) * scale; // Glyph width plus one column of spacing, both scaled.
}

int drawText(Image& image, long long x, long long y, const std::string& text, Color c, int scale)
{
    long long cursor = x;
    for (char ch : text) cursor += drawChar(image, cursor, y, ch, c, scale);
    return static_cast<int>(cursor - x);
}

int textWidth(const std::string& text, int scale)
{
    return static_cast<int>(text.size()) * (5 + 1) * scale;
}

void savePng(const Image& image, const std::string& filename)
{
    const unsigned error = lodepng::encode(filename, image.rgba, image.width, image.height, LCT_RGBA, 8);
    if (error) throw std::runtime_error("Finder failure analysis PNG writing failed: " +
                                         std::string(lodepng_error_text(error)));
}
} // namespace

std::string classifyFinderOutcome(const FinderValidationRecord& record)
{
    if (record.accepted) return "ACCEPTED";
    const auto& reason = record.rejectionReason;
    if (reason.rfind("vertical_incomplete", 0) == 0) return "INCOMPLETE_VERTICAL";
    if (reason == "vertical_ratio_invalid") return "VERTICAL_RATIO_FAILURE";
    if (reason == "vertical_horizontal_size_mismatch" || reason == "horizontal_recheck_size_mismatch")
        return "SIZE_CONSISTENCY_FAILURE";
    if (reason == "vertical_center_displacement" || reason == "horizontal_center_displacement")
        return "CENTER_CONSISTENCY_FAILURE";
    if (reason.rfind("horizontal_incomplete", 0) == 0 || reason == "horizontal_recheck_ratio_invalid")
        return "HORIZONTAL_RECHECK_FAILURE";
    return "OTHER"; // invalid_raw_hit, diagonal_sanity_failed (diagnostic-only), or unforeseen reasons.
}

void saveFinderFailureAnalysisText(const FinderValidationResult& result, const std::string& filename)
{
    std::ofstream out(filename);
    if (!out) throw std::runtime_error("Cannot open finder failure analysis log: " + filename);
    out << "Milestone 7: per-candidate failure analysis over Milestone 5's validateFinderHits() output\n"
        << "No detection or validation logic changed here; this only reports and classifies\n"
        << "measurements that validateFinderHits() already computed.\n"
        << "candidate_count=" << result.records.size() << '\n'
        << "categories: ACCEPTED, INCOMPLETE_VERTICAL, VERTICAL_RATIO_FAILURE, SIZE_CONSISTENCY_FAILURE,\n"
        << "            CENTER_CONSISTENCY_FAILURE, HORIZONTAL_RECHECK_FAILURE, OTHER\n"
        << "nan-like 0 values below mean that stage's scan never ran (rejected at an earlier stage)\n\n";
    out << std::fixed << std::setprecision(6);
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < result.records.size(); ++i)
    {
        const auto& r = result.records[i];
        const auto category = classifyFinderOutcome(r);
        out << "candidate " << i << "\n"
            << "  row=" << r.original.row << " centerX=" << r.original.centerX
            << " centerY=" << r.original.centerY << "\n"
            << "  horizontal_runs=[";
        for (int v : r.original.runLengths) out << v << ' ';
        out << "] horizontal_total_width=" << r.original.totalWidth
            << " horizontal_module_size=" << r.original.estimatedModuleSize
            << " horizontal_ratio_error=" << r.original.maximumRelativeError << "\n";

        out << "  vertical_scan_complete=" << (r.vertical.complete ? "true" : "false");
        if (!r.vertical.reason.empty()) out << " reason=" << r.vertical.reason;
        out << " visited_y=[" << r.vertical.minimumY << "," << r.vertical.maximumY << "]\n"
            << "  vertical_runs=[";
        for (int v : r.vertical.runs) out << v << ' ';
        out << "] vertical_total_width=" << r.verticalRatio.totalWidth
            << " vertical_module_size=" << r.verticalRatio.moduleSize
            << " vertical_ratio_error=" << (r.verticalRatio.totalWidth ? r.verticalRatio.maximumRelativeError : nan) << "\n";

        // Size difference: compare whichever pair of axis measurements was actually taken -
        // vertical vs refined-horizontal if the horizontal re-check ran, else vertical vs the
        // original (unrefined) horizontal hit, else unavailable.
        double sizeDifference = nan;
        if (r.horizontalRatio.totalWidth && r.verticalRatio.totalWidth)
            sizeDifference = std::abs(r.horizontalRatio.totalWidth - r.verticalRatio.totalWidth);
        else if (r.verticalRatio.totalWidth)
            sizeDifference = std::abs(r.verticalRatio.totalWidth - r.original.totalWidth);
        out << "  horizontal_vertical_size_difference=" << sizeDifference
            << " center_displacement_x=" << r.centerDisplacementX
            << " center_displacement_y=" << r.centerDisplacementY << "\n";

        out << "  horizontal_recheck_complete=" << (r.horizontal.complete ? "true" : "false");
        if (!r.horizontal.reason.empty()) out << " reason=" << r.horizontal.reason;
        out << " refined_runs=[";
        for (int v : r.horizontal.runs) out << v << ' ';
        out << "] refined_total_width=" << r.horizontalRatio.totalWidth
            << " refined_module_size=" << r.horizontalRatio.moduleSize
            << " refined_ratio_error=" << (r.horizontalRatio.totalWidth ? r.horizontalRatio.maximumRelativeError : nan) << "\n";

        out << "  diagonal_diagnostic=" << (r.diagonalAttempted ? (r.diagonalPassed ? "PASS" : "FAIL") : "NOT_RUN") << "\n"
            << "  final_result=" << (r.accepted ? "ACCEPT" : "REJECT")
            << " rejection_reason=" << (r.accepted ? "none" : r.rejectionReason)
            << " category=" << category << "\n\n";
    }
    out.close();
    if (!out) throw std::runtime_error("Cannot write finder failure analysis log: " + filename);
}

void saveHorizontalCandidatesMap(const Image& image, const FinderValidationResult& result, const std::string& filename)
{
    validateDebugImage(image);
    using namespace FinderFailureAnalysisConstants;
    Image marked = image;
    const Color color{0, 120, 255};
    for (std::size_t i = 0; i < result.records.size(); ++i)
    {
        const auto& hit = result.records[i].original;
        if (!std::isfinite(hit.centerX) || !std::isfinite(hit.centerY)) continue;
        const auto cx = static_cast<long long>(std::floor(hit.centerX));
        const auto cy = static_cast<long long>(std::floor(hit.centerY));
        fillSquare(marked, cx, cy, MarkerHalfSize, color);
        drawText(marked, cx + MarkerHalfSize + 2, cy - 8, std::to_string(i), color, LabelFontScale);
    }
    savePng(marked, filename);
}

void saveVerticalFailuresMap(const Image& image, const FinderValidationResult& result, const std::string& filename)
{
    validateDebugImage(image);
    using namespace FinderFailureAnalysisConstants;
    Image marked = image;
    for (const auto& r : result.records)
    {
        // Only draw candidates that actually reached and were evaluated by the vertical
        // stage: accepted ones (context, in green) and ones rejected specifically during the
        // vertical stage (reason starts with "vertical_"). Candidates rejected before any
        // vertical scan (invalid_raw_hit) or after it (horizontal_* reasons) are omitted here.
        const bool verticalStageFailure = r.rejectionReason.rfind("vertical_", 0) == 0;
        if (!r.accepted && !verticalStageFailure) continue;
        if (!std::isfinite(r.original.centerX)) continue;
        const auto x = static_cast<long long>(std::floor(r.original.centerX));
        const Color c = r.accepted ? Color{0, 170, 0} : categoryColor(classifyFinderOutcome(r));
        for (long long y = r.vertical.minimumY; y <= r.vertical.maximumY; ++y) setPixel(marked, x, y, c);
        fillSquare(marked, x, static_cast<long long>(std::floor(r.original.centerY)), 1, c);
    }
    savePng(marked, filename);
}

void saveFailureReasonsMap(const Image& image, const FinderValidationResult& result, const std::string& filename)
{
    validateDebugImage(image);
    using namespace FinderFailureAnalysisConstants;
    Image marked = image;
    for (const auto& r : result.records)
    {
        if (!std::isfinite(r.original.centerX) || !std::isfinite(r.original.centerY)) continue;
        const Color c = categoryColor(classifyFinderOutcome(r));
        const auto cx = static_cast<long long>(std::floor(r.original.centerX));
        const auto cy = static_cast<long long>(std::floor(r.original.centerY));
        fillSquare(marked, cx, cy, MarkerHalfSize, c);
    }
    savePng(marked, filename);
}

void saveAllCandidatesRejectionMap(const Image& image, const FinderValidationResult& result, const std::string& filename)
{
    validateDebugImage(image);
    using namespace FinderFailureAnalysisConstants;

    // Fixed, readable priority order; only categories that actually occur in this result end
    // up in the legend (per the "use only categories that actually occur" instruction).
    static const std::array<std::string, 7> priority = {
        "ACCEPTED", "INCOMPLETE_VERTICAL", "VERTICAL_RATIO_FAILURE", "SIZE_CONSISTENCY_FAILURE",
        "CENTER_CONSISTENCY_FAILURE", "HORIZONTAL_RECHECK_FAILURE", "OTHER"};
    std::vector<std::string> present;
    for (const auto& name : priority)
        for (const auto& r : result.records)
            if (classifyFinderOutcome(r) == name) { present.push_back(name); break; }

    const int legendRowHeight = std::max(LegendSwatchSize, 7 * LegendFontScale) + 4;
    const int legendHeight = present.empty() ? 0 : LegendPadding * 2 + static_cast<int>(present.size()) * legendRowHeight;
    int longestTextWidth = 0;
    for (const auto& name : present) longestTextWidth = std::max(longestTextWidth, textWidth(name, LegendFontScale));
    const int legendWidth = LegendPadding * 3 + LegendSwatchSize + longestTextWidth;

    const unsigned canvasWidth = std::max<unsigned>(image.width, static_cast<unsigned>(std::max(0, legendWidth)));
    const unsigned canvasHeight = image.height + static_cast<unsigned>(std::max(0, legendHeight));

    Image canvas;
    canvas.width = canvasWidth;
    canvas.height = canvasHeight;
    canvas.rgba.assign(static_cast<std::size_t>(canvasWidth) * canvasHeight * 4, 255); // White background everywhere.
    for (unsigned y = 0; y < image.height; ++y)
        for (unsigned x = 0; x < image.width; ++x)
        {
            const auto src = (static_cast<std::size_t>(y) * image.width + x) * 4;
            const auto dst = (static_cast<std::size_t>(y) * canvas.width + x) * 4;
            for (int ch = 0; ch < 4; ++ch) canvas.rgba[dst + ch] = image.rgba[src + ch];
        }

    // Every candidate: marker, short leader line, index label. Labels cycle through four
    // offset directions (upper-right, lower-right, upper-left, lower-left) so nearby
    // candidates (common near the same physical finder) don't all stack their labels in the
    // same spot.
    static const std::array<std::pair<int, int>, 4> labelOffsets{{
        {LabelOffsetMagnitude - 1, -11}, {LabelOffsetMagnitude - 1, 9},
        {-LabelOffsetMagnitude - 14, -11}, {-LabelOffsetMagnitude - 14, 9}}};
    for (std::size_t i = 0; i < result.records.size(); ++i)
    {
        const auto& r = result.records[i];
        if (!std::isfinite(r.original.centerX) || !std::isfinite(r.original.centerY)) continue;
        const Color c = categoryColor(classifyFinderOutcome(r));
        const auto cx = static_cast<long long>(std::floor(r.original.centerX));
        const auto cy = static_cast<long long>(std::floor(r.original.centerY));
        fillSquare(canvas, cx, cy, MarkerHalfSize, c);
        const auto [dx, dy] = labelOffsets[i % labelOffsets.size()];
        const long long lx = cx + dx, ly = cy + dy;
        drawLine(canvas, cx, cy, lx, ly, c);
        drawText(canvas, lx, ly, std::to_string(i), c, LabelFontScale);
    }

    // Legend band: one row per category actually present, color swatch plus name.
    if (!present.empty())
    {
        int y = static_cast<int>(image.height) + LegendPadding;
        for (const auto& name : present)
        {
            const Color c = categoryColor(name);
            for (int dy = 0; dy < LegendSwatchSize; ++dy)
                for (int dx = 0; dx < LegendSwatchSize; ++dx)
                    setPixel(canvas, LegendPadding + dx, y + dy, c);
            drawText(canvas, LegendPadding * 2 + LegendSwatchSize, y, name, {0, 0, 0}, LegendFontScale);
            y += legendRowHeight;
        }
    }

    savePng(canvas, filename);
}
