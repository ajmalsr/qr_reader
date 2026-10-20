#ifndef QR_SEGMENT_PARSER_H
#define QR_SEGMENT_PARSER_H

#include "CorrectedDataStream.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

enum class QrSegmentMode
{
    Numeric,
    Alphanumeric,
    Byte
};

struct QrParsedSegment
{
    QrSegmentMode mode = QrSegmentMode::Byte;
    std::size_t startBit = 0;
    std::size_t endBit = 0;
    std::size_t characterCount = 0;
    std::string text;
    std::vector<std::uint8_t> byteData;
};

struct QrSegmentParsingResult
{
    bool valid = false;
    std::string failureReason;
    int version = 0;
    std::vector<QrParsedSegment> segments;
    bool terminatorFound = false;
    std::size_t bitsConsumed = 0;
    std::size_t bitsRemaining = 0;
};

const char* qrSegmentModeName(QrSegmentMode mode);

QrSegmentParsingResult parseQrSegments(
    const CorrectedDataStreamResult& stream);

void saveQrSegmentParsingReport(
    const QrSegmentParsingResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
