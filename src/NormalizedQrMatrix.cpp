#include "NormalizedQrMatrix.h"

#include "lodepng.h"

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace
{
bool matrixBlack(const BitMatrix& matrix, int x, int y)
{
    return matrix.bits[static_cast<std::size_t>(y) * matrix.width + x] != 0;
}

bool expectedFinderBlack(int localX, int localY)
{
    return localX == 0 || localX == 6 || localY == 0 || localY == 6 ||
           (localX >= 2 && localX <= 4 && localY >= 2 && localY <= 4);
}

// Fixed-pattern checks are diagnostics only. They reveal upstream sampling
// errors but never rewrite the sampled module values.
std::size_t countFinderMismatches(const BitMatrix& matrix, int originX, int originY)
{
    std::size_t mismatches = 0;
    for (int y = 0; y < 7; ++y)
        for (int x = 0; x < 7; ++x)
            if (matrixBlack(matrix, originX + x, originY + y) != expectedFinderBlack(x, y))
                ++mismatches;
    return mismatches;
}

std::size_t countTimingMismatches(const BitMatrix& matrix)
{
    std::size_t mismatches = 0;
    const int dimension = static_cast<int>(matrix.width);
    for (int coordinate = 8; coordinate <= dimension - 9; ++coordinate)
    {
        const bool expectedBlack = coordinate % 2 == 0;
        if (matrixBlack(matrix, coordinate, 6) != expectedBlack) ++mismatches;
        if (matrixBlack(matrix, 6, coordinate) != expectedBlack) ++mismatches;
    }
    return mismatches;
}

std::size_t countBottomRightAlignmentMismatches(const BitMatrix& matrix)
{
    if (matrix.width == 21) return 0; // Version 1 has no alignment pattern.
    const int origin = static_cast<int>(matrix.width) - 9;
    std::size_t mismatches = 0;
    for (int y = 0; y < 5; ++y)
        for (int x = 0; x < 5; ++x)
        {
            const bool expectedBlack = x == 0 || x == 4 || y == 0 || y == 4 ||
                                       (x == 2 && y == 2);
            if (matrixBlack(matrix, origin + x, origin + y) != expectedBlack) ++mismatches;
        }
    return mismatches;
}
}

NormalizedQrMatrixResult buildNormalizedQrMatrix(const GridSamplingResult& sampling)
{
    NormalizedQrMatrixResult result;
    result.dimension = sampling.dimension;
    result.recordsReceived = sampling.records.size();
    if (!sampling.valid)
    {
        result.failureReason = "no_valid_grid_sampling_result";
        return result;
    }
    if (result.dimension < 21 || result.dimension > 177 || result.dimension % 4 != 1)
    {
        result.failureReason = "invalid_model_2_dimension";
        return result;
    }

    // Allocate the final row-major matrix. Input record order is irrelevant;
    // each record is placed using index = row * dimension + column.
    result.expectedRecords = static_cast<std::size_t>(result.dimension) * result.dimension;
    result.matrix.width = static_cast<unsigned>(result.dimension);
    result.matrix.height = static_cast<unsigned>(result.dimension);
    result.matrix.bits.assign(result.expectedRecords, 0);
    std::vector<bool> seen(result.expectedRecords, false);

    for (const GridSampleRecord& record : sampling.records)
    {
        if (record.failure != GridSampleFailure::None)
        {
            ++result.failedSampleRecords;
            continue;
        }
        if (record.column < 0 || record.row < 0 ||
            record.column >= result.dimension || record.row >= result.dimension)
        {
            ++result.outOfRangeRecords;
            continue;
        }
        // Confirm that the recorded ideal point is the center of the declared
        // logical module, guarding against mismatched coordinates and records.
        const double expectedX = record.column + GridSamplingConstants::ModuleCenterOffset;
        const double expectedY = record.row + GridSamplingConstants::ModuleCenterOffset;
        if (!std::isfinite(record.idealCenter.x) || !std::isfinite(record.idealCenter.y) ||
            std::abs(record.idealCenter.x - expectedX) > NormalizedQrMatrixConstants::IdealCenterTolerance ||
            std::abs(record.idealCenter.y - expectedY) > NormalizedQrMatrixConstants::IdealCenterTolerance)
        {
            ++result.inconsistentIdealCenters;
            continue;
        }

        const std::size_t index = static_cast<std::size_t>(record.row) *
                                  result.dimension + record.column;
        // The seen map enforces exactly one successful record per module and
        // allows missing coordinates to be counted after ingestion.
        if (seen[index])
        {
            ++result.duplicateRecords;
            continue;
        }
        seen[index] = true;
        result.matrix.bits[index] = record.black ? 1 : 0;
        ++result.recordsAccepted;
        if (record.black) ++result.blackModules;
        else ++result.whiteModules;
    }

    for (const bool present : seen)
        if (!present) ++result.missingRecords;

    if (result.recordsReceived != result.expectedRecords ||
        result.recordsAccepted != result.expectedRecords ||
        result.failedSampleRecords || result.outOfRangeRecords || result.duplicateRecords ||
        result.missingRecords || result.inconsistentIdealCenters)
    {
        result.failureReason = "grid_sampling_records_are_not_complete_and_unique";
        return result;
    }
    if (sampling.samplesRead != result.expectedRecords ||
        sampling.blackSamples != result.blackModules ||
        sampling.whiteSamples != result.whiteModules)
    {
        result.failureReason = "grid_sampling_summary_disagrees_with_records";
        return result;
    }

    // Audit known QR structures after completeness checks. A valid result can
    // still report mismatches because Stage 15 preserves sampled bits unchanged.
    result.finderPatternMismatches =
        countFinderMismatches(result.matrix, 0, 0) +
        countFinderMismatches(result.matrix, result.dimension - 7, 0) +
        countFinderMismatches(result.matrix, 0, result.dimension - 7);
    result.timingPatternMismatches = countTimingMismatches(result.matrix);
    result.alignmentPatternMismatches = countBottomRightAlignmentMismatches(result.matrix);
    result.valid = true;
    return result;
}

void saveNormalizedQrMatrix(
    const NormalizedQrMatrixResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    if (result.valid)
    {
        std::vector<unsigned char> pixels(result.matrix.bits.size(), 255);
        for (std::size_t i = 0; i < result.matrix.bits.size(); ++i)
            pixels[i] = result.matrix.bits[i] ? 0 : 255;
        const unsigned pngError = lodepng::encode(
            outputDirectory + "/13_normalized_qr_matrix.png", pixels,
            result.matrix.width, result.matrix.height, LCT_GREY, 8);
        if (pngError)
            throw std::runtime_error("Normalized QR matrix PNG writing failed: " +
                                     std::string(lodepng_error_text(pngError)));
    }

    std::ofstream out(outputDirectory + "/13_normalized_qr_matrix.txt");
    if (!out) throw std::runtime_error("Cannot open normalized QR matrix report");
    out << "Stage 15: normalized QR BitMatrix\n"
        << "input_filename=" << inputFilename << '\n'
        << "dimension=" << result.dimension
        << " expected_records=" << result.expectedRecords
        << " records_received=" << result.recordsReceived
        << " records_accepted=" << result.recordsAccepted << '\n'
        << "failed_sample_records=" << result.failedSampleRecords
        << " out_of_range_records=" << result.outOfRangeRecords
        << " duplicate_records=" << result.duplicateRecords
        << " missing_records=" << result.missingRecords
        << " inconsistent_ideal_centers=" << result.inconsistentIdealCenters << '\n';

    if (!result.valid)
    {
        out << "result=REJECT reason=" << result.failureReason << '\n';
    }
    else
    {
        out << "black_modules=" << result.blackModules
            << " white_modules=" << result.whiteModules << '\n'
            << "finder_pattern_mismatches=" << result.finderPatternMismatches
            << " timing_pattern_mismatches=" << result.timingPatternMismatches
            << " bottom_right_alignment_pattern_mismatches="
            << result.alignmentPatternMismatches << '\n'
            << "matrix_rows (#=black .=white):\n";
        for (int row = 0; row < result.dimension; ++row)
        {
            for (int column = 0; column < result.dimension; ++column)
                out << (matrixBlack(result.matrix, column, row) ? '#' : '.');
            out << '\n';
        }
        out << "result=VALID\n";
    }
    out.close();
    if (!out) throw std::runtime_error("Cannot write normalized QR matrix report");
}
