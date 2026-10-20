#ifndef QR_LEARNING_NORMALIZED_QR_MATRIX_H
#define QR_LEARNING_NORMALIZED_QR_MATRIX_H

#include "GridSampling.h"

#include <cstddef>
#include <string>

namespace NormalizedQrMatrixConstants
{
constexpr double IdealCenterTolerance = 1e-9;
}

// Stage 15 owns the compact, row-major QR module matrix. Function-pattern mismatch
// counts are diagnostics: sampled noise is preserved for a future decoder/error path.
struct NormalizedQrMatrixResult
{
    bool valid = false;
    std::string failureReason;
    int dimension = 0;
    BitMatrix matrix;

    std::size_t expectedRecords = 0;
    std::size_t recordsReceived = 0;
    std::size_t recordsAccepted = 0;
    std::size_t failedSampleRecords = 0;
    std::size_t outOfRangeRecords = 0;
    std::size_t duplicateRecords = 0;
    std::size_t missingRecords = 0;
    std::size_t inconsistentIdealCenters = 0;
    std::size_t blackModules = 0;
    std::size_t whiteModules = 0;

    std::size_t finderPatternMismatches = 0;
    std::size_t timingPatternMismatches = 0;
    std::size_t alignmentPatternMismatches = 0;
};

NormalizedQrMatrixResult buildNormalizedQrMatrix(const GridSamplingResult& sampling);

void saveNormalizedQrMatrix(
    const NormalizedQrMatrixResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
