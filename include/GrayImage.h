#ifndef QR_LEARNING_GRAY_IMAGE_H
#define QR_LEARNING_GRAY_IMAGE_H

#include "Image.h"
#include <cstdint>

// One 8-bit luminance byte per pixel, row-major, with no padding.
struct GrayImage
{
    unsigned width = 0;
    unsigned height = 0;
    static constexpr std::size_t channels = 1;
    std::vector<std::uint8_t> pixels;

    std::size_t pixelCount() const
    {
        return static_cast<std::size_t>(width) * height;
    }
};

// Our conversion algorithm; alpha is ignored (no compositing).
GrayImage toLuminance(const Image& image);

// PNG output is separate from the luminance algorithm.
void saveGrayPng(const GrayImage& image, const std::string& filename);

#endif
