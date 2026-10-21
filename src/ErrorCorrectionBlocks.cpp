#include "ErrorCorrectionBlocks.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace
{
// Each record is: EC bytes/block, short-block count, short data bytes,
// long-block count, long data bytes. Rows are ordered L, M, Q, H.
struct LayoutRecord
{
    int errorCorrectionCodewordsPerBlock;
    int firstCount;
    int firstDataCodewords;
    int secondCount;
    int secondDataCodewords;
};

using VersionLayouts = std::array<LayoutRecord, 4>;

constexpr std::array<VersionLayouts, 40> Model2Layouts = {{
    {{{7,1,19,0,0},{10,1,16,0,0},{13,1,13,0,0},{17,1,9,0,0}}},
    {{{10,1,34,0,0},{16,1,28,0,0},{22,1,22,0,0},{28,1,16,0,0}}},
    {{{15,1,55,0,0},{26,1,44,0,0},{18,2,17,0,0},{22,2,13,0,0}}},
    {{{20,1,80,0,0},{18,2,32,0,0},{26,2,24,0,0},{16,4,9,0,0}}},
    {{{26,1,108,0,0},{24,2,43,0,0},{18,2,15,2,16},{22,2,11,2,12}}},
    {{{18,2,68,0,0},{16,4,27,0,0},{24,4,19,0,0},{28,4,15,0,0}}},
    {{{20,2,78,0,0},{18,4,31,0,0},{18,2,14,4,15},{26,4,13,1,14}}},
    {{{24,2,97,0,0},{22,2,38,2,39},{22,4,18,2,19},{26,4,14,2,15}}},
    {{{30,2,116,0,0},{22,3,36,2,37},{20,4,16,4,17},{24,4,12,4,13}}},
    {{{18,2,68,2,69},{26,4,43,1,44},{24,6,19,2,20},{28,6,15,2,16}}},
    {{{20,4,81,0,0},{30,1,50,4,51},{28,4,22,4,23},{24,3,12,8,13}}},
    {{{24,2,92,2,93},{22,6,36,2,37},{26,4,20,6,21},{28,7,14,4,15}}},
    {{{26,4,107,0,0},{22,8,37,1,38},{24,8,20,4,21},{22,12,11,4,12}}},
    {{{30,3,115,1,116},{24,4,40,5,41},{20,11,16,5,17},{24,11,12,5,13}}},
    {{{22,5,87,1,88},{24,5,41,5,42},{30,5,24,7,25},{24,11,12,7,13}}},
    {{{24,5,98,1,99},{28,7,45,3,46},{24,15,19,2,20},{30,3,15,13,16}}},
    {{{28,1,107,5,108},{28,10,46,1,47},{28,1,22,15,23},{28,2,14,17,15}}},
    {{{30,5,120,1,121},{26,9,43,4,44},{28,17,22,1,23},{28,2,14,19,15}}},
    {{{28,3,113,4,114},{26,3,44,11,45},{26,17,21,4,22},{26,9,13,16,14}}},
    {{{28,3,107,5,108},{26,3,41,13,42},{30,15,24,5,25},{28,15,15,10,16}}},
    {{{28,4,116,4,117},{26,17,42,0,0},{28,17,22,6,23},{30,19,16,6,17}}},
    {{{28,2,111,7,112},{28,17,46,0,0},{30,7,24,16,25},{24,34,13,0,0}}},
    {{{30,4,121,5,122},{28,4,47,14,48},{30,11,24,14,25},{30,16,15,14,16}}},
    {{{30,6,117,4,118},{28,6,45,14,46},{30,11,24,16,25},{30,30,16,2,17}}},
    {{{26,8,106,4,107},{28,8,47,13,48},{30,7,24,22,25},{30,22,15,13,16}}},
    {{{28,10,114,2,115},{28,19,46,4,47},{28,28,22,6,23},{30,33,16,4,17}}},
    {{{30,8,122,4,123},{28,22,45,3,46},{30,8,23,26,24},{30,12,15,28,16}}},
    {{{30,3,117,10,118},{28,3,45,23,46},{30,4,24,31,25},{30,11,15,31,16}}},
    {{{30,7,116,7,117},{28,21,45,7,46},{30,1,23,37,24},{30,19,15,26,16}}},
    {{{30,5,115,10,116},{28,19,47,10,48},{30,15,24,25,25},{30,23,15,25,16}}},
    {{{30,13,115,3,116},{28,2,46,29,47},{30,42,24,1,25},{30,23,15,28,16}}},
    {{{30,17,115,0,0},{28,10,46,23,47},{30,10,24,35,25},{30,19,15,35,16}}},
    {{{30,17,115,1,116},{28,14,46,21,47},{30,29,24,19,25},{30,11,15,46,16}}},
    {{{30,13,115,6,116},{28,14,46,23,47},{30,44,24,7,25},{30,59,16,1,17}}},
    {{{30,12,121,7,122},{28,12,47,26,48},{30,39,24,14,25},{30,22,15,41,16}}},
    {{{30,6,121,14,122},{28,6,47,34,48},{30,46,24,10,25},{30,2,15,64,16}}},
    {{{30,17,122,4,123},{28,29,46,14,47},{30,49,24,10,25},{30,24,15,46,16}}},
    {{{30,4,122,18,123},{28,13,46,32,47},{30,48,24,14,25},{30,42,15,32,16}}},
    {{{30,20,117,4,118},{28,40,47,7,48},{30,43,24,22,25},{30,10,15,67,16}}},
    {{{30,19,118,6,119},{28,18,47,31,48},{30,34,24,34,25},{30,20,15,61,16}}}
}};

int levelIndex(QrErrorCorrectionLevel level)
{
    switch (level)
    {
    case QrErrorCorrectionLevel::Low: return 0;
    case QrErrorCorrectionLevel::Medium: return 1;
    case QrErrorCorrectionLevel::Quartile: return 2;
    case QrErrorCorrectionLevel::High: return 3;
    default: return -1;
    }
}

ErrorCorrectionBlockResult rejected(const std::string& reason)
{
    ErrorCorrectionBlockResult result;
    result.failureReason = reason;
    return result;
}

void writeBytes(std::ostream& output, const std::vector<std::uint8_t>& bytes)
{
    if (bytes.empty())
    {
        output << "none";
        return;
    }

    for (std::size_t index = 0; index < bytes.size(); ++index)
    {
        if (index != 0)
            output << ' ';
        output << std::uppercase << std::hex << std::setw(2)
               << std::setfill('0') << static_cast<int>(bytes[index])
               << std::nouppercase << std::dec;
    }
}
}

QrErrorCorrectionBlockLayout qrErrorCorrectionBlockLayout(
    int version,
    QrErrorCorrectionLevel errorCorrectionLevel)
{
    QrErrorCorrectionBlockLayout layout;
    if (version < 1 || version > static_cast<int>(Model2Layouts.size()))
        return layout;

    const int index = levelIndex(errorCorrectionLevel);
    if (index < 0)
        return layout;

    const LayoutRecord& record =
        Model2Layouts[static_cast<std::size_t>(version - 1)]
                     [static_cast<std::size_t>(index)];
    if (record.errorCorrectionCodewordsPerBlock <= 0 ||
        record.firstCount <= 0 || record.firstDataCodewords <= 0)
        return layout;

    layout.version = version;
    layout.errorCorrectionLevel = errorCorrectionLevel;
    layout.errorCorrectionCodewordsPerBlock =
        record.errorCorrectionCodewordsPerBlock;
    layout.groups.push_back(
        {record.firstCount, record.firstDataCodewords});
    if (record.secondCount > 0)
        layout.groups.push_back(
            {record.secondCount, record.secondDataCodewords});

    for (const QrErrorCorrectionBlockGroup& group : layout.groups)
    {
        layout.totalBlocks += static_cast<std::size_t>(group.count);
        layout.totalDataCodewords +=
            static_cast<std::size_t>(group.count) *
            group.dataCodewordsPerBlock;
    }
    layout.totalErrorCorrectionCodewords =
        layout.totalBlocks * layout.errorCorrectionCodewordsPerBlock;
    layout.totalCodewords =
        layout.totalDataCodewords + layout.totalErrorCorrectionCodewords;
    layout.valid = true;
    return layout;
}

ErrorCorrectionBlockResult deinterleaveErrorCorrectionBlocks(
    const RawCodewordExtractionResult& rawCodewords)
{
    if (!rawCodewords.valid)
        return rejected("no_valid_raw_codewords");

    const QrErrorCorrectionBlockLayout layout =
        qrErrorCorrectionBlockLayout(
            rawCodewords.version, rawCodewords.errorCorrectionLevel);
    if (!layout.valid)
        return rejected("unsupported_version_or_error_correction_level");
    if (rawCodewords.rawCodewords.size() != layout.totalCodewords)
        return rejected("raw_codeword_count_does_not_match_block_layout");

    ErrorCorrectionBlockResult result;
    result.version = rawCodewords.version;
    result.errorCorrectionLevel = rawCodewords.errorCorrectionLevel;
    result.layout = layout;
    result.blocks.reserve(layout.totalBlocks);

    for (const QrErrorCorrectionBlockGroup& group : layout.groups)
    {
        for (int occurrence = 0; occurrence < group.count; ++occurrence)
        {
            QrCodewordBlock block;
            block.index = result.blocks.size();
            block.dataCodewords.resize(
                static_cast<std::size_t>(group.dataCodewordsPerBlock));
            block.errorCorrectionCodewords.resize(
                static_cast<std::size_t>(
                    layout.errorCorrectionCodewordsPerBlock));
            result.blocks.push_back(std::move(block));
        }
    }

    const auto dataSize = [](const QrCodewordBlock& block) {
        return block.dataCodewords.size();
    };
    const std::size_t minimumDataCodewords =
        std::min_element(
            result.blocks.begin(), result.blocks.end(),
            [&](const QrCodewordBlock& first, const QrCodewordBlock& second) {
                return dataSize(first) < dataSize(second);
            })->dataCodewords.size();
    const std::size_t maximumDataCodewords =
        std::max_element(
            result.blocks.begin(), result.blocks.end(),
            [&](const QrCodewordBlock& first, const QrCodewordBlock& second) {
                return dataSize(first) < dataSize(second);
            })->dataCodewords.size();
    if (maximumDataCodewords > minimumDataCodewords + 1)
        return rejected("block_layout_has_unsupported_data_size_difference");

    std::size_t rawOffset = 0;

    // QR first interleaves every common data position across all blocks.
    for (std::size_t dataIndex = 0;
         dataIndex < minimumDataCodewords; ++dataIndex)
    {
        for (QrCodewordBlock& block : result.blocks)
            block.dataCodewords[dataIndex] =
                rawCodewords.rawCodewords[rawOffset++];
    }

    // Longer blocks, when present, receive one additional data byte.
    for (QrCodewordBlock& block : result.blocks)
    {
        if (block.dataCodewords.size() > minimumDataCodewords)
            block.dataCodewords[minimumDataCodewords] =
                rawCodewords.rawCodewords[rawOffset++];
    }

    // Error-correction bytes have equal length in every block and are
    // interleaved position-by-position after all data bytes.
    for (int ecIndex = 0;
         ecIndex < layout.errorCorrectionCodewordsPerBlock; ++ecIndex)
    {
        for (QrCodewordBlock& block : result.blocks)
        {
            block.errorCorrectionCodewords[
                static_cast<std::size_t>(ecIndex)] =
                    rawCodewords.rawCodewords[rawOffset++];
        }
    }

    if (rawOffset != rawCodewords.rawCodewords.size())
        return rejected("deinterleaving_did_not_consume_all_raw_codewords");

    result.rawCodewordsConsumed = rawOffset;
    result.valid = true;
    return result;
}

void saveErrorCorrectionBlockReport(
    const ErrorCorrectionBlockResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::filesystem::create_directories(outputDirectory);
    std::ofstream report(
        std::filesystem::path(outputDirectory) /
        "18_error_correction_blocks.txt");

    report << "Stage 20: error-correction block layout and deinterleaving\n";
    report << "input=" << inputFilename << '\n';
    if (!result.valid)
    {
        report << "result=REJECT\nreason=" << result.failureReason << '\n';
        return;
    }

    report << "result=ACCEPT\n";
    report << "version=" << result.version << '\n';
    report << "error_correction_level="
           << errorCorrectionLevelName(result.errorCorrectionLevel) << '\n';
    report << "error_correction_codewords_per_block="
           << result.layout.errorCorrectionCodewordsPerBlock << '\n';
    report << "block_count=" << result.layout.totalBlocks << '\n';
    report << "total_data_codewords="
           << result.layout.totalDataCodewords << '\n';
    report << "total_error_correction_codewords="
           << result.layout.totalErrorCorrectionCodewords << '\n';
    report << "expected_total_codewords="
           << result.layout.totalCodewords << '\n';
    report << "raw_codewords_consumed="
           << result.rawCodewordsConsumed << '\n';
    report << "groups (count data_codewords_per_block)\n";
    for (std::size_t index = 0;
         index < result.layout.groups.size(); ++index)
    {
        report << "group=" << index
               << " count=" << result.layout.groups[index].count
               << " data_codewords_per_block="
               << result.layout.groups[index].dataCodewordsPerBlock
               << '\n';
    }

    report << "blocks\n";
    for (const QrCodewordBlock& block : result.blocks)
    {
        report << "block=" << block.index
               << " data_count=" << block.dataCodewords.size()
               << " ec_count="
               << block.errorCorrectionCodewords.size() << '\n';
        report << "  data=";
        writeBytes(report, block.dataCodewords);
        report << '\n';
        report << "  error_correction=";
        writeBytes(report, block.errorCorrectionCodewords);
        report << '\n';
    }
}