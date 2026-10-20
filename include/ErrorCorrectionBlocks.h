#ifndef ERROR_CORRECTION_BLOCKS_H
#define ERROR_CORRECTION_BLOCKS_H

#include "RawCodewordExtraction.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct QrErrorCorrectionBlockGroup
{
    int count = 0;
    int dataCodewordsPerBlock = 0;
};

struct QrErrorCorrectionBlockLayout
{
    bool valid = false;
    int version = 0;
    QrErrorCorrectionLevel errorCorrectionLevel =
        QrErrorCorrectionLevel::Unknown;
    int errorCorrectionCodewordsPerBlock = 0;
    std::vector<QrErrorCorrectionBlockGroup> groups;
    std::size_t totalBlocks = 0;
    std::size_t totalDataCodewords = 0;
    std::size_t totalErrorCorrectionCodewords = 0;
    std::size_t totalCodewords = 0;
};

struct QrCodewordBlock
{
    std::size_t index = 0;
    std::vector<std::uint8_t> dataCodewords;
    std::vector<std::uint8_t> errorCorrectionCodewords;
};

struct ErrorCorrectionBlockResult
{
    bool valid = false;
    std::string failureReason;
    int version = 0;
    QrErrorCorrectionLevel errorCorrectionLevel =
        QrErrorCorrectionLevel::Unknown;
    QrErrorCorrectionBlockLayout layout;
    std::vector<QrCodewordBlock> blocks;
    std::size_t rawCodewordsConsumed = 0;
};

QrErrorCorrectionBlockLayout qrErrorCorrectionBlockLayout(
    int version,
    QrErrorCorrectionLevel errorCorrectionLevel);

ErrorCorrectionBlockResult deinterleaveErrorCorrectionBlocks(
    const RawCodewordExtractionResult& rawCodewords);

void saveErrorCorrectionBlockReport(
    const ErrorCorrectionBlockResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif