#include "Binarizer.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

BinarizationResult binarizeLocal(const GrayImage& gray)
{
    using namespace BinarizationConstants;
    if (gray.width < MinimumDimension || gray.height < MinimumDimension)
        throw std::invalid_argument("Local binarization requires width and height >= " +
                                    std::to_string(MinimumDimension));
    if (static_cast<std::size_t>(gray.width) >
            std::numeric_limits<std::size_t>::max() / gray.height ||
        gray.pixels.size() != gray.pixelCount())
        throw std::invalid_argument("Invalid luminance buffer size");

    BinarizationResult result;
    // Overflow-safe ceiling division.
    result.blocksWide = gray.width / BlockSize + (gray.width % BlockSize != 0);
    result.blocksHigh = gray.height / BlockSize + (gray.height % BlockSize != 0);
    result.blocks.resize(static_cast<std::size_t>(result.blocksWide) * result.blocksHigh);
    result.matrix.width = gray.width;
    result.matrix.height = gray.height;
    result.matrix.bits.resize(gray.pixelCount(), WhiteBit);
    const auto blockIndex = [&](unsigned x, unsigned y) {
        return static_cast<std::size_t>(y) * result.blocksWide + x;
    };

    // Pass 1: measure all pixels and estimate one black point per block.
    // For each 8x8 block we compute min/max/mean luminance. Normally the block's own
    // mean is used as its black point. But a block with very low dynamic range (near-uniform
    // luminance, e.g. blank paper or a large filled area) cannot reliably self-threshold, so
    // instead we fall back to half its minimum, optionally raised toward already-processed
    // neighboring blocks' black points if those neighbors suggest a brighter local background.
    for (unsigned by = 0; by < result.blocksHigh; ++by)
    {
        for (unsigned bx = 0; bx < result.blocksWide; ++bx)
        {
            auto& block = result.blocks[blockIndex(bx, by)];
            block.x = std::min(bx * BlockSize, gray.width - BlockSize);
            block.y = std::min(by * BlockSize, gray.height - BlockSize);
            block.minimum = WhitePng;
            block.maximum = BlackPng;
            unsigned sum = 0;
            for (unsigned dy = 0; dy < BlockSize; ++dy)
                for (unsigned dx = 0; dx < BlockSize; ++dx)
                {
                    const unsigned value = gray.pixels[
                        static_cast<std::size_t>(block.y + dy) * gray.width + block.x + dx];
                    sum += value;
                    block.minimum = std::min(block.minimum, value);
                    block.maximum = std::max(block.maximum, value);
                }
            block.mean = sum / (BlockSize * BlockSize);
            block.dynamicRange = block.maximum - block.minimum;
            block.lowContrast = block.dynamicRange <= MinimumDynamicRange;
            block.blackPoint = block.mean;
            if (block.lowContrast)
            {
                block.blackPoint = block.minimum / LowContrastDivisor;
                if (bx > 0 && by > 0)
                {
                    const unsigned neighbor = (
                        UpperWeight * result.blocks[blockIndex(bx, by - 1)].blackPoint +
                        LeftWeight * result.blocks[blockIndex(bx - 1, by)].blackPoint +
                        DiagonalWeight * result.blocks[blockIndex(bx - 1, by - 1)].blackPoint
                    ) / NeighborDivisor;
                    if (block.minimum < neighbor)
                    {
                        block.blackPoint = neighbor;
                        block.usedNeighbor = true;
                    }
                }
            }
        }
    }

    // Pass 2: smooth black points, then classify. Edge overlaps are overwritten.
    // Averaging each block's black point with its 5x5 neighborhood (clamped at the image
    // edges) removes block-to-block threshold jitter, producing a smoothly varying threshold
    // surface across the image. A pixel is classified black when its luminance is <= this
    // smoothed threshold for the block it belongs to.
    for (unsigned by = 0; by < result.blocksHigh; ++by)
        for (unsigned bx = 0; bx < result.blocksWide; ++bx)
        {
            auto& block = result.blocks[blockIndex(bx, by)];
            const unsigned centerX = std::clamp(bx, SmoothingRadius,
                                               result.blocksWide - SmoothingRadius - 1);
            const unsigned centerY = std::clamp(by, SmoothingRadius,
                                               result.blocksHigh - SmoothingRadius - 1);
            unsigned sum = 0;
            for (unsigned wy = 0; wy < SmoothingSize; ++wy)
                for (unsigned wx = 0; wx < SmoothingSize; ++wx)
                    sum += result.blocks[blockIndex(centerX - SmoothingRadius + wx,
                                                     centerY - SmoothingRadius + wy)].blackPoint;
            block.threshold = sum / (SmoothingSize * SmoothingSize);
            for (unsigned dy = 0; dy < BlockSize; ++dy)
                for (unsigned dx = 0; dx < BlockSize; ++dx)
                {
                    const std::size_t index =
                        static_cast<std::size_t>(block.y + dy) * gray.width + block.x + dx;
                    result.matrix.bits[index] = gray.pixels[index] <= block.threshold
                        ? BlackBit : WhiteBit;
                }
        }
    return result;
}

GrayImage binaryForPng(const BitMatrix& matrix)
{
    if (matrix.width == 0 || matrix.height == 0 ||
        static_cast<std::size_t>(matrix.width) >
            std::numeric_limits<std::size_t>::max() / matrix.height ||
        matrix.bits.size() != static_cast<std::size_t>(matrix.width) * matrix.height)
        throw std::invalid_argument("Invalid binary matrix dimensions or buffer");
    GrayImage image;
    image.width = matrix.width;
    image.height = matrix.height;
    image.pixels.resize(matrix.bits.size());
    for (std::size_t i = 0; i < matrix.bits.size(); ++i)
    {
        if (matrix.bits[i] > BinarizationConstants::BlackBit)
            throw std::invalid_argument("Binary matrix contains a non-binary value");
        image.pixels[i] = matrix.bits[i] == BinarizationConstants::BlackBit
            ? BinarizationConstants::BlackPng : BinarizationConstants::WhitePng;
    }
    return image;
}
