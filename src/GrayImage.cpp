#include "GrayImage.h"

#include <limits>
#include <stdexcept>

GrayImage toLuminance(const Image& image)
{
    const std::size_t maxSize = std::numeric_limits<std::size_t>::max();
    if (image.width == 0 || image.height == 0 ||
        static_cast<std::size_t>(image.width) > maxSize / image.height)
    {
        throw std::invalid_argument("Invalid RGBA image dimensions");
    }
    const std::size_t count = image.pixelCount();
    if (count > maxSize / Image::channels || image.rgba.size() != count * Image::channels)
    {
        throw std::invalid_argument("RGBA buffer size does not match dimensions");
    }

    GrayImage gray;
    gray.width = image.width;
    gray.height = image.height;
    gray.pixels.resize(count);

    for (std::size_t pixel = 0; pixel < count; ++pixel)
    {
        const std::size_t offset = pixel * Image::channels;
        const unsigned r = image.rgba[offset];
        const unsigned g = image.rgba[offset + 1];
        const unsigned b = image.rgba[offset + 2];

        // Integer approximation of 0.299 R + 0.587 G + 0.114 B.
        // Weights sum to 1024; +512 rounds before dividing by 1024.
        // Same arithmetic as ZXing-C++ RGBToLum, implemented locally.
        gray.pixels[pixel] = static_cast<std::uint8_t>(
            (306 * r + 601 * g + 117 * b + 512) >> 10);
    }
    return gray;
}
