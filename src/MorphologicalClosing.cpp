#include "MorphologicalClosing.h"
#include <fstream>
#include <limits>
#include <stdexcept>

namespace
{
void validateMatrix(const BitMatrix& m)
{
    if (!m.width || !m.height ||
        static_cast<std::size_t>(m.width) > std::numeric_limits<std::size_t>::max()/m.height ||
        m.bits.size() != static_cast<std::size_t>(m.width)*m.height)
        throw std::invalid_argument("Invalid BitMatrix for morphological closing");
    for (auto b : m.bits)
        if (b > BinarizationConstants::BlackBit)
            throw std::invalid_argument("BitMatrix contains a non-binary value");
}
// Pixels outside the image are treated as white (background) for both dilation and
// erosion. This is a simple, consistent convention: it means dilation cannot invent
// foreground beyond the border, and erosion treats the border as touching background,
// so black regions touching the image edge can erode there.
bool blackAt(const BitMatrix& m, long long x, long long y)
{
    if (x < 0 || y < 0 || x >= m.width || y >= m.height) return false;
    return m.bits[static_cast<std::size_t>(y)*m.width + static_cast<std::size_t>(x)] ==
        BinarizationConstants::BlackBit;
}
}

BitMatrix dilateBitMatrix3x3(const BitMatrix& matrix)
{
    validateMatrix(matrix);
    using namespace MorphologicalClosingConstants;
    constexpr int radius = static_cast<int>(StructuringElementRadius);
    BitMatrix result;
    result.width = matrix.width;
    result.height = matrix.height;
    result.bits.resize(matrix.bits.size());
    for (unsigned y = 0; y < matrix.height; ++y)
        for (unsigned x = 0; x < matrix.width; ++x)
        {
            bool black = false;
            for (int dy = -radius; dy <= radius && !black; ++dy)
                for (int dx = -radius; dx <= radius && !black; ++dx)
                    black = blackAt(matrix, static_cast<long long>(x) + dx, static_cast<long long>(y) + dy);
            result.bits[static_cast<std::size_t>(y)*matrix.width + x] =
                black ? BinarizationConstants::BlackBit : BinarizationConstants::WhiteBit;
        }
    return result;
}

BitMatrix erodeBitMatrix3x3(const BitMatrix& matrix)
{
    validateMatrix(matrix);
    using namespace MorphologicalClosingConstants;
    constexpr int radius = static_cast<int>(StructuringElementRadius);
    BitMatrix result;
    result.width = matrix.width;
    result.height = matrix.height;
    result.bits.resize(matrix.bits.size());
    for (unsigned y = 0; y < matrix.height; ++y)
        for (unsigned x = 0; x < matrix.width; ++x)
        {
            bool allBlack = true;
            for (int dy = -radius; dy <= radius && allBlack; ++dy)
                for (int dx = -radius; dx <= radius && allBlack; ++dx)
                    allBlack = blackAt(matrix, static_cast<long long>(x) + dx, static_cast<long long>(y) + dy);
            result.bits[static_cast<std::size_t>(y)*matrix.width + x] =
                allBlack ? BinarizationConstants::BlackBit : BinarizationConstants::WhiteBit;
        }
    return result;
}

BitMatrix closeBitMatrix3x3(const BitMatrix& matrix)
{
    // Closing = erosion of the dilation. Applying erosion with the same structuring
    // element used for dilation is what makes closing extensive (fill, never remove).
    return erodeBitMatrix3x3(dilateBitMatrix3x3(matrix));
}

void saveMorphologicalClosingDebug(const BitMatrix& before, const BitMatrix& after,
                                    const std::string& filename)
{
    if (before.width != after.width || before.height != after.height ||
        before.bits.size() != after.bits.size())
        throw std::invalid_argument("Mismatched matrices for morphological closing debug");
    std::size_t filled = 0, removed = 0, unchangedBlack = 0, unchangedWhite = 0;
    for (std::size_t i = 0; i < before.bits.size(); ++i)
    {
        const bool wasBlack = before.bits[i] == BinarizationConstants::BlackBit;
        const bool isBlack = after.bits[i] == BinarizationConstants::BlackBit;
        if (!wasBlack && isBlack) ++filled;
        else if (wasBlack && !isBlack) ++removed;
        else if (wasBlack) ++unchangedBlack;
        else ++unchangedWhite;
    }
    std::ofstream out(filename);
    if (!out) throw std::runtime_error("Cannot open morphological closing log: " + filename);
    out << "Morphological closing = dilate(3x3) then erode(3x3), full square structuring element\n"
        << "border_convention=outside_image_is_white\n"
        << "width=" << before.width << " height=" << before.height << '\n'
        << "pixels_filled_white_to_black=" << filled << '\n'
        << "pixels_removed_black_to_white=" << removed
        << " (expected 0; closing should be extensive)\n"
        << "unchanged_black=" << unchangedBlack << " unchanged_white=" << unchangedWhite << '\n';
    out.close();
    if (!out) throw std::runtime_error("Cannot write morphological closing log: " + filename);
}
