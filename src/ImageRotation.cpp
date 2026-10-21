#include "ImageRotation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double TrigSnapTolerance = 1e-12;

double snapNearZero(double value)
{
    return std::abs(value) < TrigSnapTolerance ? 0.0 : value;
}
}

Image rotateImage(const Image& source, double clockwiseDegrees)
{
    if (source.width == 0 || source.height == 0 ||
        source.rgba.size() != source.pixelCount() * Image::channels)
    {
        throw std::invalid_argument("Cannot rotate an invalid RGBA image");
    }
    if (!std::isfinite(clockwiseDegrees))
        throw std::invalid_argument("Image rotation angle must be finite");

    // Reduce large angles before evaluating sine and cosine. main.cpp bypasses
    // this function completely for its normal zero-degree setting.
    const double normalizedDegrees = std::fmod(clockwiseDegrees, 360.0);
    if (normalizedDegrees == 0.0)
        return source;

    const double radians = normalizedDegrees * Pi / 180.0;
    const double cosine = snapNearZero(std::cos(radians));
    const double sine = snapNearZero(std::sin(radians));

    // Rotating the source boundaries gives these axis-aligned canvas extents.
    // Snapping trigonometric round-off keeps quarter turns at exact dimensions.
    const double widthExtent =
        std::abs(static_cast<double>(source.width) * cosine) +
        std::abs(static_cast<double>(source.height) * sine);
    const double heightExtent =
        std::abs(static_cast<double>(source.width) * sine) +
        std::abs(static_cast<double>(source.height) * cosine);
    const double maximumDimension =
        static_cast<double>(std::numeric_limits<unsigned>::max());
    if (widthExtent > maximumDimension || heightExtent > maximumDimension)
        throw std::overflow_error("Rotated image dimensions are too large");

    Image rotated;
    rotated.width = static_cast<unsigned>(
        std::ceil(widthExtent - TrigSnapTolerance));
    rotated.height = static_cast<unsigned>(
        std::ceil(heightExtent - TrigSnapTolerance));
    if (rotated.width == 0 || rotated.height == 0 ||
        rotated.pixelCount() >
            std::numeric_limits<std::size_t>::max() / Image::channels)
    {
        throw std::overflow_error("Rotated RGBA buffer is too large");
    }
    rotated.rgba.assign(rotated.pixelCount() * Image::channels, 255);

    const double sourceCenterX =
        (static_cast<double>(source.width) - 1.0) / 2.0;
    const double sourceCenterY =
        (static_cast<double>(source.height) - 1.0) / 2.0;
    const double targetCenterX =
        (static_cast<double>(rotated.width) - 1.0) / 2.0;
    const double targetCenterY =
        (static_cast<double>(rotated.height) - 1.0) / 2.0;

    for (unsigned targetY = 0; targetY < rotated.height; ++targetY)
    {
        for (unsigned targetX = 0; targetX < rotated.width; ++targetX)
        {
            const double relativeX =
                static_cast<double>(targetX) - targetCenterX;
            const double relativeY =
                static_cast<double>(targetY) - targetCenterY;

            // Inverse mapping asks which source position produced this target
            // pixel. This fills every target pixel and avoids forward-map holes.
            double sourceX =
                cosine * relativeX + sine * relativeY + sourceCenterX;
            double sourceY =
                -sine * relativeX + cosine * relativeY + sourceCenterY;
            if (sourceX < -0.5 || sourceY < -0.5 ||
                sourceX > static_cast<double>(source.width) - 0.5 ||
                sourceY > static_cast<double>(source.height) - 0.5)
            {
                continue; // Retain the opaque-white background outside the source.
            }

            // Bilinear interpolation avoids the jagged edges produced by nearest
            // neighbor rotation. Half-pixel boundary positions clamp to the
            // nearest source pixel rather than introducing a dark border.
            sourceX = std::clamp(
                sourceX, 0.0, static_cast<double>(source.width - 1));
            sourceY = std::clamp(
                sourceY, 0.0, static_cast<double>(source.height - 1));
            const unsigned x0 = static_cast<unsigned>(std::floor(sourceX));
            const unsigned y0 = static_cast<unsigned>(std::floor(sourceY));
            const unsigned x1 = std::min(x0 + 1, source.width - 1);
            const unsigned y1 = std::min(y0 + 1, source.height - 1);
            const double xFraction = sourceX - x0;
            const double yFraction = sourceY - y0;
            const std::size_t targetOffset =
                (static_cast<std::size_t>(targetY) * rotated.width + targetX) *
                Image::channels;

            for (std::size_t channel = 0; channel < Image::channels; ++channel)
            {
                const auto sample = [&](unsigned x, unsigned y) {
                    return static_cast<double>(source.rgba[
                        (static_cast<std::size_t>(y) * source.width + x) *
                        Image::channels + channel]);
                };
                const double top =
                    sample(x0, y0) * (1.0 - xFraction) +
                    sample(x1, y0) * xFraction;
                const double bottom =
                    sample(x0, y1) * (1.0 - xFraction) +
                    sample(x1, y1) * xFraction;
                rotated.rgba[targetOffset + channel] =
                    static_cast<unsigned char>(std::lround(
                        top * (1.0 - yFraction) + bottom * yFraction));
            }
        }
    }
    return rotated;
}
