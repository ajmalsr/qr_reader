#include "Image.h"
#include "lodepng.h"

#include <limits>
#include <stdexcept>

Image loadPng(const std::string& filename)
{
    Image image;
    // Request a consistent output layout regardless of the PNG's source format.
    constexpr unsigned bitsPerChannel = 8;
    const unsigned error = lodepng::decode(
        image.rgba, image.width, image.height, filename, LCT_RGBA, bitsPerChannel);

    if (error != 0)
    {
        throw std::runtime_error("PNG decoding failed (LodePNG error " +
            std::to_string(error) + "): " + lodepng_error_text(error));
    }

    // Check multiplication before computing the expected buffer size.
    const std::size_t maxSize = std::numeric_limits<std::size_t>::max();
    if (image.width == 0 || image.height == 0 ||
        static_cast<std::size_t>(image.width) > maxSize / image.height)
    {
        throw std::runtime_error("Invalid or oversized PNG dimensions");
    }
    const std::size_t pixels = image.pixelCount();
    if (pixels > maxSize / Image::channels ||
        image.rgba.size() != pixels * Image::channels)
    {
        throw std::runtime_error("PNG RGBA buffer size does not match dimensions");
    }
    return image;
}
