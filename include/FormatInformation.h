#ifndef QR_LEARNING_FORMAT_INFORMATION_H
#define QR_LEARNING_FORMAT_INFORMATION_H

#include "FunctionPatternMap.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace FormatInformationConstants
{
constexpr std::uint16_t FixedMask = 0x5412;
constexpr std::uint16_t BchGenerator = 0x0537;
constexpr int EncodedBitCount = 15;
constexpr int DataBitCount = 5;
constexpr int ErrorCorrectionBitCount = EncodedBitCount - DataBitCount;
constexpr int MaximumCorrectableBitErrors = 3;
}

enum class QrErrorCorrectionLevel
{
    Unknown,
    Low,
    Medium,
    Quartile,
    High
};

struct FormatInformationCopy
{
    std::uint16_t maskedBits = 0;
    std::uint16_t unmaskedBits = 0;
    int decodedData = -1;
    int hammingDistance = FormatInformationConstants::EncodedBitCount + 1;
    bool ambiguous = false;
    bool valid = false;
    std::vector<std::pair<int, int>> coordinates;
};

struct FormatInformationResult
{
    bool valid = false;
    std::string failureReason;
    int dimension = 0;
    int version = 0;
    FormatInformationCopy firstCopy;
    FormatInformationCopy secondCopy;
    int selectedCopy = 0; // 0=both agree, 1=first, 2=second.
    int formatData = -1;
    QrErrorCorrectionLevel errorCorrectionLevel = QrErrorCorrectionLevel::Unknown;
    int dataMask = -1;
};

FormatInformationResult decodeFormatInformation(
    const NormalizedQrMatrixResult& normalized,
    const FunctionPatternMapResult& functionMap);

const char* errorCorrectionLevelName(QrErrorCorrectionLevel level);

void saveFormatInformationReport(
    const FormatInformationResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
