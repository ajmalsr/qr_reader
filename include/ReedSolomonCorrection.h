#ifndef REED_SOLOMON_CORRECTION_H
#define REED_SOLOMON_CORRECTION_H

#include "ErrorCorrectionBlocks.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace QrReedSolomonConstants
{
constexpr int PrimitivePolynomial = 0x11D;
constexpr int FieldSize = 256;
constexpr int GeneratorBase = 0;
}

struct ReedSolomonByteCorrection
{
    std::size_t codewordPosition = 0;
    std::uint8_t before = 0;
    std::uint8_t after = 0;
    std::uint8_t magnitude = 0;
};

struct ReedSolomonBlockResult
{
    bool valid = false;
    std::string failureReason;
    std::size_t blockIndex = 0;
    int errorCorrectionCodewordCount = 0;
    int correctionCapacity = 0;
    std::vector<std::uint8_t> originalDataCodewords;
    std::vector<std::uint8_t> originalErrorCorrectionCodewords;
    std::vector<std::uint8_t> correctedDataCodewords;
    std::vector<std::uint8_t> correctedErrorCorrectionCodewords;
    std::vector<std::uint8_t> syndromes;
    std::vector<ReedSolomonByteCorrection> corrections;
};

struct ReedSolomonCorrectionResult
{
    bool valid = false;
    std::string failureReason;
    int version = 0;
    QrErrorCorrectionLevel errorCorrectionLevel =
        QrErrorCorrectionLevel::Unknown;
    std::vector<ReedSolomonBlockResult> blocks;
    std::size_t blocksWithErrors = 0;
    std::size_t totalCorrectedCodewords = 0;
};

ReedSolomonCorrectionResult correctReedSolomonBlocks(
    const ErrorCorrectionBlockResult& deinterleaved);

void saveReedSolomonCorrectionReport(
    const ReedSolomonCorrectionResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif