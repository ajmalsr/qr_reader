#include "ReedSolomonCorrection.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
class QrGaloisField
{
public:
    QrGaloisField()
    {
        int value = 1;
        for (int exponent = 0;
             exponent < QrReedSolomonConstants::FieldSize; ++exponent)
        {
            exponentTable_[static_cast<std::size_t>(exponent)] = value;
            value <<= 1;
            if (value >= QrReedSolomonConstants::FieldSize)
            {
                value ^= QrReedSolomonConstants::PrimitivePolynomial;
                value &= QrReedSolomonConstants::FieldSize - 1;
            }
        }

        for (int exponent = 0;
             exponent < QrReedSolomonConstants::FieldSize - 1; ++exponent)
        {
            logarithmTable_[static_cast<std::size_t>(
                exponentTable_[static_cast<std::size_t>(exponent)])] =
                    exponent;
        }
    }

    int exponent(int power) const
    {
        int reduced = power % (QrReedSolomonConstants::FieldSize - 1);
        if (reduced < 0)
            reduced += QrReedSolomonConstants::FieldSize - 1;
        return exponentTable_[static_cast<std::size_t>(reduced)];
    }

    int logarithm(int value) const
    {
        if (value == 0)
            throw std::invalid_argument("GF(256) logarithm of zero");
        return logarithmTable_[static_cast<std::size_t>(value)];
    }

    int inverse(int value) const
    {
        if (value == 0)
            throw std::invalid_argument("GF(256) inverse of zero");
        return exponentTable_[static_cast<std::size_t>(
            (QrReedSolomonConstants::FieldSize - 1) - logarithm(value))];
    }

    int multiply(int first, int second) const
    {
        if (first == 0 || second == 0)
            return 0;
        const int logarithmSum = logarithm(first) + logarithm(second);
        return exponentTable_[static_cast<std::size_t>(
            logarithmSum % (QrReedSolomonConstants::FieldSize - 1))];
    }

private:
    std::array<int, QrReedSolomonConstants::FieldSize> exponentTable_{};
    std::array<int, QrReedSolomonConstants::FieldSize> logarithmTable_{};
};

class GfPolynomial
{
public:
    GfPolynomial(const QrGaloisField& field, std::vector<int> coefficients)
        : field_(field)
    {
        if (coefficients.empty())
            throw std::invalid_argument("GF polynomial needs coefficients");

        std::size_t firstNonZero = 0;
        while (firstNonZero + 1 < coefficients.size() &&
               coefficients[firstNonZero] == 0)
        {
            ++firstNonZero;
        }
        coefficients_.assign(
            coefficients.begin() + static_cast<std::ptrdiff_t>(firstNonZero),
            coefficients.end());
    }

    GfPolynomial(const GfPolynomial& other)
        : field_(other.field_), coefficients_(other.coefficients_)
    {
    }

    GfPolynomial(GfPolynomial&& other)
        : field_(other.field_),
          coefficients_(std::move(other.coefficients_))
    {
    }

    GfPolynomial& operator=(const GfPolynomial& other)
    {
        if (&field_ != &other.field_)
            throw std::invalid_argument("GF polynomial field mismatch");
        coefficients_ = other.coefficients_;
        return *this;
    }

    GfPolynomial& operator=(GfPolynomial&& other)
    {
        if (&field_ != &other.field_)
            throw std::invalid_argument("GF polynomial field mismatch");
        coefficients_ = std::move(other.coefficients_);
        return *this;
    }

    int degree() const
    {
        return static_cast<int>(coefficients_.size()) - 1;
    }

    bool isZero() const
    {
        return coefficients_.size() == 1 && coefficients_[0] == 0;
    }

    int coefficient(int degree) const
    {
        return coefficients_[
            coefficients_.size() - 1 - static_cast<std::size_t>(degree)];
    }

    int evaluateAt(int value) const
    {
        if (value == 0)
            return coefficient(0);
        if (value == 1)
        {
            int result = 0;
            for (const int coefficientValue : coefficients_)
                result ^= coefficientValue;
            return result;
        }

        int result = coefficients_[0];
        for (std::size_t index = 1; index < coefficients_.size(); ++index)
            result = field_.multiply(value, result) ^ coefficients_[index];
        return result;
    }

    GfPolynomial addOrSubtract(const GfPolynomial& other) const
    {
        if (isZero())
            return other;
        if (other.isZero())
            return *this;

        const std::vector<int>* smaller = &coefficients_;
        const std::vector<int>* larger = &other.coefficients_;
        if (smaller->size() > larger->size())
            std::swap(smaller, larger);

        std::vector<int> sum = *larger;
        const std::size_t difference = larger->size() - smaller->size();
        for (std::size_t index = 0; index < smaller->size(); ++index)
            sum[index + difference] ^= (*smaller)[index];
        return GfPolynomial(field_, std::move(sum));
    }

    GfPolynomial multiply(const GfPolynomial& other) const
    {
        if (isZero() || other.isZero())
            return GfPolynomial(field_, {0});

        std::vector<int> product(
            coefficients_.size() + other.coefficients_.size() - 1, 0);
        for (std::size_t first = 0; first < coefficients_.size(); ++first)
        {
            for (std::size_t second = 0;
                 second < other.coefficients_.size(); ++second)
            {
                product[first + second] ^=
                    field_.multiply(
                        coefficients_[first],
                        other.coefficients_[second]);
            }
        }
        return GfPolynomial(field_, std::move(product));
    }

    GfPolynomial multiply(int scalar) const
    {
        if (scalar == 0)
            return GfPolynomial(field_, {0});
        if (scalar == 1)
            return *this;

        std::vector<int> product(coefficients_.size());
        for (std::size_t index = 0; index < coefficients_.size(); ++index)
            product[index] =
                field_.multiply(coefficients_[index], scalar);
        return GfPolynomial(field_, std::move(product));
    }

    GfPolynomial multiplyByMonomial(int degree, int coefficientValue) const
    {
        if (degree < 0)
            throw std::invalid_argument("negative GF monomial degree");
        if (coefficientValue == 0)
            return GfPolynomial(field_, {0});

        std::vector<int> product(
            coefficients_.size() + static_cast<std::size_t>(degree), 0);
        for (std::size_t index = 0; index < coefficients_.size(); ++index)
            product[index] =
                field_.multiply(coefficients_[index], coefficientValue);
        return GfPolynomial(field_, std::move(product));
    }

private:
    const QrGaloisField& field_;
    std::vector<int> coefficients_;
};

GfPolynomial monomial(
    const QrGaloisField& field,
    int degree,
    int coefficientValue)
{
    if (degree < 0)
        throw std::invalid_argument("negative GF monomial degree");
    if (coefficientValue == 0)
        return GfPolynomial(field, {0});

    std::vector<int> coefficients(
        static_cast<std::size_t>(degree) + 1, 0);
    coefficients[0] = coefficientValue;
    return GfPolynomial(field, std::move(coefficients));
}

bool runEuclideanAlgorithm(
    const QrGaloisField& field,
    GfPolynomial first,
    GfPolynomial second,
    int errorCorrectionCodewords,
    GfPolynomial& errorLocator,
    GfPolynomial& errorEvaluator,
    std::string& failureReason)
{
    if (first.degree() < second.degree())
        std::swap(first, second);

    GfPolynomial previousRemainder = first;
    GfPolynomial remainder = second;
    GfPolynomial previousAuxiliary(field, {0});
    GfPolynomial auxiliary(field, {1});

    while (2 * remainder.degree() >= errorCorrectionCodewords)
    {
        const GfPolynomial previousPreviousRemainder = previousRemainder;
        const GfPolynomial previousPreviousAuxiliary = previousAuxiliary;
        previousRemainder = remainder;
        previousAuxiliary = auxiliary;

        if (previousRemainder.isZero())
        {
            failureReason = "euclidean_algorithm_remainder_became_zero";
            return false;
        }

        remainder = previousPreviousRemainder;
        GfPolynomial quotient(field, {0});
        const int denominatorLeadingTerm =
            previousRemainder.coefficient(previousRemainder.degree());
        const int denominatorInverse =
            field.inverse(denominatorLeadingTerm);

        while (remainder.degree() >= previousRemainder.degree() &&
               !remainder.isZero())
        {
            const int degreeDifference =
                remainder.degree() - previousRemainder.degree();
            const int scale = field.multiply(
                remainder.coefficient(remainder.degree()),
                denominatorInverse);
            quotient = quotient.addOrSubtract(
                monomial(field, degreeDifference, scale));
            remainder = remainder.addOrSubtract(
                previousRemainder.multiplyByMonomial(
                    degreeDifference, scale));
        }

        auxiliary =
            quotient.multiply(previousAuxiliary)
                .addOrSubtract(previousPreviousAuxiliary);
        if (remainder.degree() >= previousRemainder.degree())
        {
            failureReason = "euclidean_division_did_not_reduce_degree";
            return false;
        }
    }

    const int locatorAtZero = auxiliary.coefficient(0);
    if (locatorAtZero == 0)
    {
        failureReason = "error_locator_constant_is_zero";
        return false;
    }

    const int normalization = field.inverse(locatorAtZero);
    errorLocator = auxiliary.multiply(normalization);
    errorEvaluator = remainder.multiply(normalization);
    return true;
}

bool findErrorLocations(
    const QrGaloisField& field,
    const GfPolynomial& errorLocator,
    std::vector<int>& locations,
    std::string& failureReason)
{
    const int errorCount = errorLocator.degree();
    if (errorCount <= 0)
    {
        failureReason = "error_locator_has_no_roots";
        return false;
    }
    if (errorCount == 1)
    {
        locations.push_back(errorLocator.coefficient(1));
        return true;
    }

    for (int candidate = 1;
         candidate < QrReedSolomonConstants::FieldSize &&
         static_cast<int>(locations.size()) < errorCount;
         ++candidate)
    {
        if (errorLocator.evaluateAt(candidate) == 0)
            locations.push_back(field.inverse(candidate));
    }

    if (static_cast<int>(locations.size()) != errorCount)
    {
        failureReason =
            "error_locator_degree_does_not_match_root_count";
        return false;
    }
    return true;
}

std::vector<int> findErrorMagnitudes(
    const QrGaloisField& field,
    const GfPolynomial& errorEvaluator,
    const std::vector<int>& errorLocations)
{
    std::vector<int> magnitudes(errorLocations.size());
    for (std::size_t index = 0; index < errorLocations.size(); ++index)
    {
        const int inverseLocation =
            field.inverse(errorLocations[index]);
        int denominator = 1;
        for (std::size_t other = 0;
             other < errorLocations.size(); ++other)
        {
            if (index == other)
                continue;
            const int term =
                field.multiply(errorLocations[other], inverseLocation);
            denominator =
                field.multiply(denominator, 1 ^ term);
        }

        magnitudes[index] = field.multiply(
            errorEvaluator.evaluateAt(inverseLocation),
            field.inverse(denominator));
    }
    return magnitudes;
}

std::vector<std::uint8_t> calculateSyndromes(
    const QrGaloisField& field,
    const std::vector<std::uint8_t>& codewords,
    int errorCorrectionCodewords)
{
    std::vector<int> coefficients;
    coefficients.reserve(codewords.size());
    for (const std::uint8_t codeword : codewords)
        coefficients.push_back(codeword);
    const GfPolynomial received(field, std::move(coefficients));

    std::vector<std::uint8_t> syndromes(
        static_cast<std::size_t>(errorCorrectionCodewords));
    for (int index = 0; index < errorCorrectionCodewords; ++index)
    {
        syndromes[static_cast<std::size_t>(index)] =
            static_cast<std::uint8_t>(
                received.evaluateAt(
                    field.exponent(
                        index +
                        QrReedSolomonConstants::GeneratorBase)));
    }
    return syndromes;
}

bool allZero(const std::vector<std::uint8_t>& values)
{
    return std::all_of(
        values.begin(), values.end(),
        [](std::uint8_t value) { return value == 0; });
}

bool correctBlock(
    std::vector<std::uint8_t>& codewords,
    int errorCorrectionCodewords,
    std::vector<std::uint8_t>& initialSyndromes,
    std::vector<ReedSolomonByteCorrection>& corrections,
    std::string& failureReason)
{
    if (errorCorrectionCodewords <= 0 ||
        errorCorrectionCodewords >= static_cast<int>(codewords.size()))
    {
        failureReason = "invalid_error_correction_codeword_count";
        return false;
    }

    const QrGaloisField field;
    initialSyndromes =
        calculateSyndromes(field, codewords, errorCorrectionCodewords);
    if (allZero(initialSyndromes))
        return true;

    // Polynomial storage is highest-degree first, so reverse S0..S(n-1)
    // before solving the Reed-Solomon key equation.
    std::vector<int> syndromeCoefficients(
        static_cast<std::size_t>(errorCorrectionCodewords));
    for (int index = 0; index < errorCorrectionCodewords; ++index)
    {
        syndromeCoefficients[
            static_cast<std::size_t>(
                errorCorrectionCodewords - 1 - index)] =
                    initialSyndromes[static_cast<std::size_t>(index)];
    }

    GfPolynomial errorLocator(field, {0});
    GfPolynomial errorEvaluator(field, {0});
    if (!runEuclideanAlgorithm(
            field,
            monomial(field, errorCorrectionCodewords, 1),
            GfPolynomial(field, std::move(syndromeCoefficients)),
            errorCorrectionCodewords,
            errorLocator,
            errorEvaluator,
            failureReason))
    {
        return false;
    }

    if (errorLocator.degree() >
        errorCorrectionCodewords / 2)
    {
        failureReason = "error_count_exceeds_correction_capacity";
        return false;
    }

    std::vector<int> errorLocations;
    if (!findErrorLocations(
            field, errorLocator, errorLocations, failureReason))
    {
        return false;
    }
    const std::vector<int> errorMagnitudes =
        findErrorMagnitudes(
            field, errorEvaluator, errorLocations);

    for (std::size_t index = 0;
         index < errorLocations.size(); ++index)
    {
        const int position =
            static_cast<int>(codewords.size()) - 1 -
            field.logarithm(errorLocations[index]);
        if (position < 0 ||
            position >= static_cast<int>(codewords.size()))
        {
            failureReason = "error_location_outside_block";
            return false;
        }

        ReedSolomonByteCorrection correction;
        correction.codewordPosition =
            static_cast<std::size_t>(position);
        correction.before =
            codewords[static_cast<std::size_t>(position)];
        correction.magnitude =
            static_cast<std::uint8_t>(errorMagnitudes[index]);
        correction.after = static_cast<std::uint8_t>(
            correction.before ^ correction.magnitude);
        codewords[static_cast<std::size_t>(position)] =
            correction.after;
        corrections.push_back(correction);
    }

    const std::vector<std::uint8_t> verificationSyndromes =
        calculateSyndromes(
            field, codewords, errorCorrectionCodewords);
    if (!allZero(verificationSyndromes))
    {
        failureReason = "nonzero_syndrome_after_correction";
        return false;
    }

    std::sort(
        corrections.begin(), corrections.end(),
        [](const ReedSolomonByteCorrection& first,
           const ReedSolomonByteCorrection& second) {
            return first.codewordPosition < second.codewordPosition;
        });
    return true;
}

ReedSolomonCorrectionResult rejected(const std::string& reason)
{
    ReedSolomonCorrectionResult result;
    result.failureReason = reason;
    return result;
}

void writeBytes(
    std::ostream& output,
    const std::vector<std::uint8_t>& values)
{
    if (values.empty())
    {
        output << "none";
        return;
    }
    for (std::size_t index = 0; index < values.size(); ++index)
    {
        if (index != 0)
            output << ' ';
        output << std::uppercase << std::hex << std::setw(2)
               << std::setfill('0')
               << static_cast<int>(values[index])
               << std::nouppercase << std::dec;
    }
}
}

ReedSolomonCorrectionResult correctReedSolomonBlocks(
    const ErrorCorrectionBlockResult& deinterleaved)
{
    if (!deinterleaved.valid)
        return rejected("no_valid_deinterleaved_blocks");
    if (!deinterleaved.layout.valid ||
        deinterleaved.blocks.empty())
        return rejected("invalid_error_correction_block_layout");
    if (deinterleaved.blocks.size() !=
        deinterleaved.layout.totalBlocks)
        return rejected("block_count_does_not_match_layout");

    ReedSolomonCorrectionResult result;
    result.version = deinterleaved.version;
    result.errorCorrectionLevel =
        deinterleaved.errorCorrectionLevel;
    result.blocks.reserve(deinterleaved.blocks.size());

    bool everyBlockValid = true;
    for (const QrCodewordBlock& inputBlock :
         deinterleaved.blocks)
    {
        ReedSolomonBlockResult block;
        block.blockIndex = inputBlock.index;
        block.errorCorrectionCodewordCount =
            deinterleaved.layout
                .errorCorrectionCodewordsPerBlock;
        block.correctionCapacity =
            block.errorCorrectionCodewordCount / 2;
        block.originalDataCodewords =
            inputBlock.dataCodewords;
        block.originalErrorCorrectionCodewords =
            inputBlock.errorCorrectionCodewords;

        if (inputBlock.errorCorrectionCodewords.size() !=
            static_cast<std::size_t>(
                block.errorCorrectionCodewordCount))
        {
            block.failureReason =
                "block_ec_count_does_not_match_layout";
            everyBlockValid = false;
            result.blocks.push_back(std::move(block));
            continue;
        }

        std::vector<std::uint8_t> combined =
            inputBlock.dataCodewords;
        combined.insert(
            combined.end(),
            inputBlock.errorCorrectionCodewords.begin(),
            inputBlock.errorCorrectionCodewords.end());

        try
        {
            block.valid = correctBlock(
                combined,
                block.errorCorrectionCodewordCount,
                block.syndromes,
                block.corrections,
                block.failureReason);
        }
        catch (const std::exception&)
        {
            block.valid = false;
            block.failureReason =
                "reed_solomon_arithmetic_failure";
        }

        if (block.valid)
        {
            const std::size_t dataCount =
                inputBlock.dataCodewords.size();
            block.correctedDataCodewords.assign(
                combined.begin(),
                combined.begin() +
                    static_cast<std::ptrdiff_t>(dataCount));
            block.correctedErrorCorrectionCodewords.assign(
                combined.begin() +
                    static_cast<std::ptrdiff_t>(dataCount),
                combined.end());

            if (!block.corrections.empty())
                ++result.blocksWithErrors;
            result.totalCorrectedCodewords +=
                block.corrections.size();
        }
        else
        {
            everyBlockValid = false;
        }

        result.blocks.push_back(std::move(block));
    }

    if (!everyBlockValid)
    {
        result.failureReason =
            "one_or_more_blocks_uncorrectable";
        return result;
    }

    result.valid = true;
    return result;
}

void saveReedSolomonCorrectionReport(
    const ReedSolomonCorrectionResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    std::filesystem::create_directories(outputDirectory);
    std::ofstream report(
        std::filesystem::path(outputDirectory) /
        "19_reed_solomon_correction.txt");

    report << "Stage 21: Reed-Solomon correction in QR GF(256)\n";
    report << "input=" << inputFilename << '\n';
    report << "primitive_polynomial=0x11D\n";
    report << "generator_base=0\n";
    report << "result=" << (result.valid ? "ACCEPT" : "REJECT")
           << '\n';
    if (!result.failureReason.empty())
        report << "reason=" << result.failureReason << '\n';

    report << "version=" << result.version << '\n';
    report << "error_correction_level="
           << errorCorrectionLevelName(
                  result.errorCorrectionLevel)
           << '\n';
    report << "block_count=" << result.blocks.size() << '\n';
    report << "blocks_with_errors="
           << result.blocksWithErrors << '\n';
    report << "total_corrected_codewords="
           << result.totalCorrectedCodewords << '\n';

    for (const ReedSolomonBlockResult& block : result.blocks)
    {
        report << "block=" << block.blockIndex
               << " result="
               << (block.valid ? "ACCEPT" : "REJECT")
               << " ec_codewords="
               << block.errorCorrectionCodewordCount
               << " correction_capacity="
               << block.correctionCapacity
               << " corrected="
               << block.corrections.size() << '\n';
        if (!block.failureReason.empty())
            report << "  reason=" << block.failureReason << '\n';
        report << "  initial_syndromes=";
        writeBytes(report, block.syndromes);
        report << '\n';

        for (const ReedSolomonByteCorrection& correction :
             block.corrections)
        {
            report << "  correction position="
                   << correction.codewordPosition
                   << " before=0x"
                   << std::uppercase << std::hex
                   << std::setw(2) << std::setfill('0')
                   << static_cast<int>(correction.before)
                   << " magnitude=0x"
                   << std::setw(2)
                   << static_cast<int>(correction.magnitude)
                   << " after=0x"
                   << std::setw(2)
                   << static_cast<int>(correction.after)
                   << std::nouppercase << std::dec << '\n';
        }

        report << "  original_data=";
        writeBytes(report, block.originalDataCodewords);
        report << '\n';
        report << "  corrected_data=";
        writeBytes(report, block.correctedDataCodewords);
        report << '\n';
        report << "  original_error_correction=";
        writeBytes(
            report,
            block.originalErrorCorrectionCodewords);
        report << '\n';
        report << "  corrected_error_correction=";
        writeBytes(
            report,
            block.correctedErrorCorrectionCodewords);
        report << '\n';
    }
}