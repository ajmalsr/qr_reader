#include "GrayImage.h"
#include "lodepng.h"

#include <limits>
#include <stdexcept>

void saveGrayPng(const GrayImage& image, const std::string& filename)
{
    if (image.width == 0 || image.height == 0 ||
        static_cast<std::size_t>(image.width) >
            std::numeric_limits<std::size_t>::max() / image.height ||
        image.pixels.size() != image.pixelCount())
    {
        throw std::invalid_argument("Invalid grayscale image dimensions or buffer size");
    }

    // Keep the actual PNG grayscale, 8 bits per pixel, even for black/white data.
    lodepng::State state;
    state.info_raw.colortype = LCT_GREY;
    state.info_raw.bitdepth = 8;
    state.info_png.color.colortype = LCT_GREY;
    state.info_png.color.bitdepth = 8;
    state.encoder.auto_convert = 0;

    std::vector<unsigned char> png;
    unsigned error = lodepng::encode(
        png, image.pixels.data(), image.width, image.height, state);
    if (error == 0)
    {
        error = lodepng::save_file(png, filename);
    }
    if (error != 0)
    {
        throw std::runtime_error("Grayscale PNG writing failed (LodePNG error " +
            std::to_string(error) + "): " + lodepng_error_text(error));
    }
}
