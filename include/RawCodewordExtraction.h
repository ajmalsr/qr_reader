#ifndef RAW_CODEWORD_EXTRACTION_H
#define RAW_CODEWORD_EXTRACTION_H

#include "DataMask.h"
#include "FunctionPatternMap.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct RawCodewordModule
{
    int column = 0;
    int row = 0;
    std::uint8_t bit = 0;
    std::size_t traversalIndex = 0;
    bool remainder = false;
};

struct RawCodewordExtractionResult
{
    bool valid = false;
    std::string failureReason;
    int dimension = 0;
    int version = 0;
    QrErrorCorrectionLevel errorCorrectionLevel = QrErrorCorrectionLevel::Unknown;
    int dataMask = -1;

    std::vector<RawCodewordModule> traversedModules;
    std::vector<std::uint8_t> rawCodewords;
    std::vector<std::uint8_t> remainderBits;

    std::size_t functionModulesExcluded = 0;
    std::size_t dataOrRemainderModulesVisited = 0;
    std::size_t codewordBits = 0;
    bool remainderBitsAllZero = true;
};

RawCodewordExtractionResult extractRawCodewords(
    const DataMaskRemovalResult& dataMaskRemoval,
    const FunctionPatternMapResult& functionPatternMap);

void saveRawCodewordExtractionReport(
    const RawCodewordExtractionResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif