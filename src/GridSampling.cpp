#include "GridSampling.h"

#include "lodepng.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace
{
void validateMatrix(const BitMatrix& matrix)
{
    const std::size_t expected = static_cast<std::size_t>(matrix.width) * matrix.height;
    if (!matrix.width || !matrix.height || matrix.bits.size() != expected)
        throw std::invalid_argument("Invalid BitMatrix for grid sampling");
}

void setPixel(Image& image, int x, int y,
              unsigned char red, unsigned char green, unsigned char blue)
{
    if (x < 0 || y < 0 || x >= static_cast<int>(image.width) ||
        y >= static_cast<int>(image.height)) return;
    const std::size_t index = 4 * (static_cast<std::size_t>(y) * image.width + x);
    image.rgba[index] = red;
    image.rgba[index + 1] = green;
    image.rgba[index + 2] = blue;
    image.rgba[index + 3] = 255;
}

void fillPreviewModule(Image& preview, int column, int row,
                       unsigned char red, unsigned char green, unsigned char blue)
{
    const int scale = GridSamplingConstants::PreviewPixelsPerModule;
    for (int y = row * scale; y < (row + 1) * scale; ++y)
        for (int x = column * scale; x < (column + 1) * scale; ++x)
            setPixel(preview, x, y, red, green, blue);
}
}

const char* gridSampleFailureText(GridSampleFailure failure)
{
    switch (failure)
    {
    case GridSampleFailure::None: return "none";
    case GridSampleFailure::ProjectionFailed: return "homogeneous_projection_failed";
    case GridSampleFailure::OutsideSourceImage: return "projected_center_outside_source_image";
    }
    return "unknown";
}

GridSamplingResult sampleQrGrid(
    const BitMatrix& source,
    const QrPerspectiveTransformResult& perspective)
{
    validateMatrix(source);
    GridSamplingResult result;
    result.dimension = perspective.dimension;
    if (!perspective.valid)
    {
        result.failureReason = "no_valid_perspective_transform";
        return result;
    }
    if (result.dimension < 21 || result.dimension > 177 || result.dimension % 4 != 1)
    {
        result.failureReason = "invalid_model_2_dimension";
        return result;
    }

    // Visit every logical module exactly once. The +0.5 offset selects the
    // module center, avoiding boundaries where binarization is least stable.
    result.records.reserve(static_cast<std::size_t>(result.dimension) * result.dimension);
    for (int row = 0; row < result.dimension; ++row)
    {
        for (int column = 0; column < result.dimension; ++column)
        {
            GridSampleRecord record;
            record.column = column;
            record.row = row;
            record.idealCenter = {
                column + GridSamplingConstants::ModuleCenterOffset,
                row + GridSamplingConstants::ModuleCenterOffset};
            ++result.samplesAttempted;

            // Keep a record even when projection fails so diagnostics preserve
            // the failed logical coordinate instead of silently shortening the grid.
            if (!projectPoint(perspective.idealToImage, record.idealCenter, record.imagePoint))
            {
                record.failure = GridSampleFailure::ProjectionFailed;
                ++result.projectionFailures;
                result.records.push_back(record);
                continue;
            }

            // Positive image coordinates use floor so a continuous coordinate in
            // [x,x+1) selects discrete source pixel x, matching ZXing-style truncation.
            record.sourceX = static_cast<int>(std::floor(record.imagePoint.x));
            record.sourceY = static_cast<int>(std::floor(record.imagePoint.y));
            if (record.imagePoint.x < 0.0 || record.imagePoint.y < 0.0 ||
                record.imagePoint.x >= source.width || record.imagePoint.y >= source.height)
            {
                record.failure = GridSampleFailure::OutsideSourceImage;
                ++result.outsideImageFailures;
                result.records.push_back(record);
                continue;
            }

            // Read one value from the already binarized source. This stage does
            // no interpolation, neighborhood voting, or fixed-pattern repair.
            const std::size_t sourceIndex = static_cast<std::size_t>(record.sourceY) *
                                            source.width + record.sourceX;
            record.black = source.bits[sourceIndex] != 0;
            ++result.samplesRead;
            if (record.black) ++result.blackSamples;
            else ++result.whiteSamples;
            result.records.push_back(record);
        }
    }

    // A normalized QR grid must be complete, so one unsampled module invalidates
    // the stage while all per-module records remain available for debugging.
    if (result.projectionFailures || result.outsideImageFailures)
    {
        result.failureReason = "one_or_more_module_centers_could_not_be_sampled";
        return result;
    }
    result.valid = true;
    return result;
}

void saveGridSamplingDebug(
    const Image& image,
    const GridSamplingResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    if (!image.width || !image.height || image.rgba.size() != image.pixelCount() * Image::channels)
        throw std::invalid_argument("Invalid image for grid-sampling debug");

    Image overlay = image;
    for (const auto& record : result.records)
    {
        if (record.failure != GridSampleFailure::None)
        {
            if (std::isfinite(record.imagePoint.x) && std::isfinite(record.imagePoint.y))
                setPixel(overlay, static_cast<int>(std::lround(record.imagePoint.x)),
                         static_cast<int>(std::lround(record.imagePoint.y)), 255, 0, 0);
        }
        else if (record.black)
            setPixel(overlay, record.sourceX, record.sourceY, 0, 255, 0);
        else
            setPixel(overlay, record.sourceX, record.sourceY, 0, 128, 255);
    }

    const std::string overlayFilename = outputDirectory + "/12_grid_sample_points.png";
    unsigned pngError = lodepng::encode(
        overlayFilename, overlay.rgba, overlay.width, overlay.height, LCT_RGBA, 8);
    if (pngError)
        throw std::runtime_error("Grid-sampling overlay writing failed: " +
                                 std::string(lodepng_error_text(pngError)));

    if (result.dimension > 0)
    {
        const unsigned previewSide = static_cast<unsigned>(
            result.dimension * GridSamplingConstants::PreviewPixelsPerModule);
        Image preview;
        preview.width = previewSide;
        preview.height = previewSide;
        preview.rgba.assign(preview.pixelCount() * Image::channels, 255);
        for (std::size_t i = 3; i < preview.rgba.size(); i += 4) preview.rgba[i] = 255;
        for (const auto& record : result.records)
        {
            if (record.failure != GridSampleFailure::None)
                fillPreviewModule(preview, record.column, record.row, 255, 0, 0);
            else if (record.black)
                fillPreviewModule(preview, record.column, record.row, 0, 0, 0);
            else
                fillPreviewModule(preview, record.column, record.row, 255, 255, 255);
        }
        pngError = lodepng::encode(outputDirectory + "/12_sampled_grid_preview.png",
                                   preview.rgba, preview.width, preview.height, LCT_RGBA, 8);
        if (pngError)
            throw std::runtime_error("Sampled-grid preview writing failed: " +
                                     std::string(lodepng_error_text(pngError)));
    }

    std::ofstream out(outputDirectory + "/12_grid_sampling.txt");
    if (!out) throw std::runtime_error("Cannot open grid-sampling report");
    out << "Stage 14: QR grid sampling\n"
        << "input_filename=" << inputFilename << '\n'
        << "sampling_coordinate=(column+" << GridSamplingConstants::ModuleCenterOffset
        << ",row+" << GridSamplingConstants::ModuleCenterOffset << ")"
        << " discrete_pixel_rule=floor preview_pixels_per_module="
        << GridSamplingConstants::PreviewPixelsPerModule << '\n'
        << "dimension=" << result.dimension
        << " samples_attempted=" << result.samplesAttempted
        << " samples_read=" << result.samplesRead
        << " black_samples=" << result.blackSamples
        << " white_samples=" << result.whiteSamples
        << " projection_failures=" << result.projectionFailures
        << " outside_image_failures=" << result.outsideImageFailures << '\n';

    for (const auto& record : result.records)
        if (record.failure != GridSampleFailure::None)
            out << "failure module=" << record.column << ',' << record.row
                << " ideal=" << record.idealCenter.x << ',' << record.idealCenter.y
                << " image=" << record.imagePoint.x << ',' << record.imagePoint.y
                << " reason=" << gridSampleFailureText(record.failure) << '\n';

    if (result.valid)
    {
        out << "diagnostic_sampled_rows (#=black .=white):\n";
        for (int row = 0; row < result.dimension; ++row)
        {
            for (int column = 0; column < result.dimension; ++column)
            {
                const auto& record = result.records[
                    static_cast<std::size_t>(row) * result.dimension + column];
                out << (record.black ? '#' : '.');
            }
            out << '\n';
        }
        out << "result=VALID\n";
    }
    else
        out << "result=REJECT reason=" << result.failureReason << '\n';
    out << "Overlay: green=black sample, blue=white sample, red=failed sample\n"
           "Preview is diagnostic sampled data; Stage 15 normalized BitMatrix is not created.\n";
    out.close();
    if (!out) throw std::runtime_error("Cannot write grid-sampling report");

}
