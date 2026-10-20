#ifndef QR_LEARNING_FUNCTION_PATTERN_MAP_H
#define QR_LEARNING_FUNCTION_PATTERN_MAP_H

#include "NormalizedQrMatrix.h"

#include <cstddef>
#include <string>
#include <vector>

namespace FunctionPatternMapConstants
{
constexpr int MinimumVersion = 1;
constexpr int MaximumVersion = 40;
constexpr int FinderCenterCoordinate = 6;
constexpr int PreviewPixelsPerModule = 8;
}

// Stage 16 marks every QR module reserved for a fixed/function structure.
// A mask bit of 1 means function/reserved; 0 means data or remainder.
struct FunctionPatternMapResult
{
    bool valid = false;
    std::string failureReason;
    int dimension = 0;
    int version = 0;
    BitMatrix functionModules;
    std::vector<int> alignmentPatternCenters;
    std::size_t functionModuleCount = 0;
    std::size_t dataOrRemainderModuleCount = 0;
};

FunctionPatternMapResult buildFunctionPatternMap(
    const NormalizedQrMatrixResult& normalized);

void saveFunctionPatternMap(
    const FunctionPatternMapResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
