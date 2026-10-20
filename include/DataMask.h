#ifndef DATA_MASK_H
#define DATA_MASK_H

#include "FormatInformation.h"
#include "FunctionPatternMap.h"
#include "NormalizedQrMatrix.h"

#include <cstddef>
#include <string>

namespace DataMaskConstants
{
constexpr int MaskCount = 8;
constexpr unsigned PreviewPixelsPerModule = 8;
}

struct DataMaskRemovalResult
{
    bool valid = false;
    std::string failureReason;
    int dimension = 0;
    int version = 0;
    QrErrorCorrectionLevel errorCorrectionLevel = QrErrorCorrectionLevel::Unknown;
    int dataMask = -1;
    BitMatrix unmaskedMatrix;
    std::size_t functionModulesPreserved = 0;
    std::size_t dataOrRemainderModulesVisited = 0;
    std::size_t modulesFlipped = 0;
    std::size_t blackModules = 0;
    std::size_t whiteModules = 0;
};

// Returns whether the selected QR Model 2 data mask applies at (column, row).
bool qrDataMaskApplies(int dataMask, int column, int row);

DataMaskRemovalResult removeQrDataMask(
    const NormalizedQrMatrixResult& normalized,
    const FunctionPatternMapResult& functionPatternMap,
    const FormatInformationResult& formatInformation);

void saveDataMaskRemovalDebug(
    const DataMaskRemovalResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif