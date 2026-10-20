#ifndef CORRECTED_DATA_STREAM_H
#define CORRECTED_DATA_STREAM_H

#include "ReedSolomonCorrection.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct CorrectedDataStreamResult
{
    bool valid = false;
    std::string failureReason;
    int version = 0;
    QrErrorCorrectionLevel errorCorrectionLevel =
        QrErrorCorrectionLevel::Unknown;
    std::vector<std::uint8_t> bytes;
    std::size_t expectedDataCodewords = 0;
    std::size_t blocksAppended = 0;
    std::size_t bitCount = 0;
};

CorrectedDataStreamResult reconstructCorrectedDataStream(
    const ReedSolomonCorrectionResult& corrected,
    const ErrorCorrectionBlockResult& deinterleaved);

void saveCorrectedDataStreamReport(
    const CorrectedDataStreamResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
