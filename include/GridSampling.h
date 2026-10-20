#ifndef QR_LEARNING_GRID_SAMPLING_H
#define QR_LEARNING_GRID_SAMPLING_H

#include "Binarizer.h"
#include "Image.h"
#include "PerspectiveTransform.h"

#include <cstddef>
#include <string>
#include <vector>

namespace GridSamplingConstants
{
constexpr double ModuleCenterOffset = 0.5;
constexpr int PreviewPixelsPerModule = 8;
}

enum class GridSampleFailure
{
    None,
    ProjectionFailed,
    OutsideSourceImage
};

const char* gridSampleFailureText(GridSampleFailure failure);

// This is an educational record of one sampling operation. Stage 14 deliberately
// keeps these records instead of publishing the final normalized BitMatrix.
struct GridSampleRecord
{
    int column = 0;
    int row = 0;
    Point2D idealCenter;
    Point2D imagePoint;
    int sourceX = -1;
    int sourceY = -1;
    bool black = false;
    GridSampleFailure failure = GridSampleFailure::None;
};

struct GridSamplingResult
{
    bool valid = false;
    std::string failureReason;
    int dimension = 0;
    std::size_t samplesAttempted = 0;
    std::size_t samplesRead = 0;
    std::size_t blackSamples = 0;
    std::size_t whiteSamples = 0;
    std::size_t projectionFailures = 0;
    std::size_t outsideImageFailures = 0;
    std::vector<GridSampleRecord> records;
};

GridSamplingResult sampleQrGrid(
    const BitMatrix& source,
    const QrPerspectiveTransformResult& perspective);

// Writes a full-image sampling overlay, a square diagnostic preview, and a text
// report. The preview visualizes sampled records but is not the Stage-15 BitMatrix.
void saveGridSamplingDebug(
    const Image& image,
    const GridSamplingResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
