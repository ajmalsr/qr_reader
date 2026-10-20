#ifndef QR_LEARNING_IMAGE_H
#define QR_LEARNING_IMAGE_H

#include <cstddef>
#include <string>
#include <vector>

// Pixels are stored row by row, from top to bottom, in R, G, B, A order.
// Each channel is one byte. Image has no dependency on the PNG decoder.
struct Image
{
    unsigned width = 0;
    unsigned height = 0;
    static constexpr std::size_t channels = 4;
    std::vector<unsigned char> rgba;

    std::size_t pixelCount() const
    {
        return static_cast<std::size_t>(width) * height;
    }
};

// Decode a PNG into an RGBA image. Throws std::runtime_error on failure.
Image loadPng(const std::string& filename);

#endif
