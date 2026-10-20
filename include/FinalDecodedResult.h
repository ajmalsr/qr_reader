#ifndef FINAL_DECODED_RESULT_H
#define FINAL_DECODED_RESULT_H

#include "QrSegmentParser.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct FinalDecodedResult
{
    bool valid = false;
    std::string failureReason;
    int version = 0;
    QrErrorCorrectionLevel errorCorrectionLevel =
        QrErrorCorrectionLevel::Unknown;
    std::string text;
    std::vector<QrParsedSegment> segments;
    std::size_t outputByteCount = 0;
    std::size_t byteModeByteCount = 0;
    bool terminatorFound = false;
    std::size_t bitsConsumed = 0;
    std::size_t bitsRemaining = 0;
};

FinalDecodedResult assembleFinalDecodedResult(
    const CorrectedDataStreamResult& stream,
    const QrSegmentParsingResult& parsed);

void saveFinalDecodedResultReport(
    const FinalDecodedResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
