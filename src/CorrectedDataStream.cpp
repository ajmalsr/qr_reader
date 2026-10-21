#include "CorrectedDataStream.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace
{
std::string byteBits(std::uint8_t value)
{
    std::string bits;
    bits.reserve(8);
    for (int bit = 7; bit >= 0; --bit)
    {
        bits.push_back((value & (1U << bit)) != 0 ? '1' : '0');
    }
    return bits;
}

std::string bytesAsHex(const std::vector<std::uint8_t>& bytes)
{
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t index = 0; index < bytes.size(); ++index)
    {
        if (index != 0)
        {
            output << ' ';
        }
        output << std::setw(2) << static_cast<int>(bytes[index]);
    }
    return output.str();
}
}

CorrectedDataStreamResult reconstructCorrectedDataStream(
    const ReedSolomonCorrectionResult& corrected,
    const ErrorCorrectionBlockResult& deinterleaved)
{
    CorrectedDataStreamResult result;
    result.version = corrected.version;
    result.errorCorrectionLevel = corrected.errorCorrectionLevel;

    if (!corrected.valid)
    {
        result.failureReason = "reed_solomon_result_invalid";
        return result;
    }
    if (!deinterleaved.valid || !deinterleaved.layout.valid)
    {
        result.failureReason = "error_correction_layout_invalid";
        return result;
    }
    if (corrected.version != deinterleaved.version ||
        corrected.errorCorrectionLevel != deinterleaved.errorCorrectionLevel)
    {
        result.failureReason = "metadata_mismatch";
        return result;
    }

    const QrErrorCorrectionBlockLayout& layout = deinterleaved.layout;
    result.expectedDataCodewords = layout.totalDataCodewords;
    if (corrected.blocks.size() != layout.totalBlocks ||
        deinterleaved.blocks.size() != layout.totalBlocks)
    {
        result.failureReason = "block_count_mismatch";
        return result;
    }

    result.bytes.reserve(layout.totalDataCodewords);
    for (std::size_t index = 0; index < layout.totalBlocks; ++index)
    {
        const ReedSolomonBlockResult& correctedBlock = corrected.blocks[index];
        const QrCodewordBlock& sourceBlock = deinterleaved.blocks[index];

        // Block order is significant. After QR deinterleaving and independent
        // RS correction, the original data stream is block 0's data followed
        // by block 1's data, and so on. Error-correction bytes are omitted.
        if (!correctedBlock.valid)
        {
            result.failureReason = "corrected_block_invalid";
            return result;
        }
        if (correctedBlock.blockIndex != index || sourceBlock.index != index)
        {
            result.failureReason = "block_index_mismatch";
            return result;
        }
        if (correctedBlock.correctedDataCodewords.size() !=
            sourceBlock.dataCodewords.size())
        {
            result.failureReason = "block_data_size_mismatch";
            return result;
        }

        result.bytes.insert(
            result.bytes.end(),
            correctedBlock.correctedDataCodewords.begin(),
            correctedBlock.correctedDataCodewords.end());
        ++result.blocksAppended;
    }

    if (result.bytes.size() != layout.totalDataCodewords)
    {
        result.failureReason = "total_data_codeword_mismatch";
        return result;
    }

    // The next stage reads fields most-significant-bit first from this exact
    // byte sequence. No mode or payload interpretation belongs in Stage 22.
    result.bitCount = result.bytes.size() * 8;
    result.valid = true;
    return result;
}

void saveCorrectedDataStreamReport(
    const CorrectedDataStreamResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::filesystem::create_directories(outputDirectory);
    const std::filesystem::path path =
        std::filesystem::path(outputDirectory) /
        "20_corrected_data_stream.txt";
    std::ofstream output(path);
    if (!output)
    {
        return;
    }

    output << "Stage 22: Corrected data-stream reconstruction\n";
    output << "input=" << inputFilename << '\n';
    output << "result=" << (result.valid ? "ACCEPT" : "REJECT") << '\n';
    if (!result.valid)
    {
        output << "reason=" << result.failureReason << '\n';
        return;
    }

    output << "version=" << result.version << '\n';
    output << "error_correction_level="
           << errorCorrectionLevelName(result.errorCorrectionLevel) << '\n';
    output << "expected_data_codewords="
           << result.expectedDataCodewords << '\n';
    output << "reconstructed_data_codewords=" << result.bytes.size() << '\n';
    output << "blocks_appended=" << result.blocksAppended << '\n';
    output << "bit_count=" << result.bitCount << '\n';
    output << "corrected_data_hex=" << bytesAsHex(result.bytes) << '\n';
    output << "continuous_bit_stream=";
    for (std::uint8_t byte : result.bytes)
    {
        output << byteBits(byte);
    }
    output << "\n\nbyte_table:\n";
    output << "index hex bits\n";
    for (std::size_t index = 0; index < result.bytes.size(); ++index)
    {
        output << std::dec << index << ' '
               << std::uppercase << std::hex << std::setfill('0')
               << std::setw(2) << static_cast<int>(result.bytes[index])
               << std::dec << ' ' << byteBits(result.bytes[index]) << '\n';
    }
    output << "\nStage 22 preserves bytes only; mode and payload parsing are deferred.\n";
}
