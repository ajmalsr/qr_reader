#include "QrDimensionEstimation.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace
{
QrDimensionEstimationResult failure(const char* reason)
{
    QrDimensionEstimationResult result;
    result.failureReason = reason;
    return result;
}

double centerDistance(const PhysicalFinderCluster& a, const PhysicalFinderCluster& b)
{
    return std::hypot(a.centerX - b.centerX, a.centerY - b.centerY);
}

// Shift an integer estimate to the Model 2 sequence 21,25,...,177. Because every
// valid dimension has remainder 1 modulo 4, correction = 1 - remainder gives:
// remainder 0 -> +1, 1 -> 0, 2 -> -1, and 3 -> -2.
int correctionToModel2Dimension(int dimension)
{
    int remainder = dimension % QrDimensionEstimationConstants::DimensionStep;
    if (remainder < 0) remainder += QrDimensionEstimationConstants::DimensionStep;
    return QrDimensionEstimationConstants::RequiredModulo - remainder;
}
}

QrDimensionEstimationResult estimateQrDimension(
    const std::vector<PhysicalFinderCluster>& clusters,
    const ModuleSizeEstimationResult& moduleSize)
{
    using namespace QrDimensionEstimationConstants;

    if (!moduleSize.valid)
        return failure("no_valid_module_size_estimate");
    if (!moduleSize.inputsAvailable)
        return failure("module_size_inputs_are_unavailable");

    QrDimensionEstimationResult result;
    result.inputsAvailable = true;
    result.clusterIndices = moduleSize.clusterIndices;
    result.moduleSize = moduleSize.estimatedModuleSize;

    const auto ids = result.clusterIndices;
    if (ids[0] == ids[1] || ids[0] == ids[2] || ids[1] == ids[2])
    {
        result.failureReason = "finder_indices_are_not_unique";
        return result;
    }
    if (ids[0] >= clusters.size() || ids[1] >= clusters.size() || ids[2] >= clusters.size())
    {
        result.failureReason = "finder_index_out_of_range";
        return result;
    }
    if (!std::isfinite(result.moduleSize) || result.moduleSize <= 0.0)
    {
        result.failureReason = "module_size_is_not_finite_and_positive";
        return result;
    }

    const auto& topLeft = clusters[ids[0]];
    const auto& topRight = clusters[ids[1]];
    const auto& bottomLeft = clusters[ids[2]];
    for (const auto* finder : {&topLeft, &topRight, &bottomLeft})
        if (!std::isfinite(finder->centerX) || !std::isfinite(finder->centerY))
        {
            result.failureReason = "finder_center_is_not_finite";
            return result;
        }

    // Finder centers are separated by (dimension - 7) modules: each center is
    // 3.5 modules from its adjacent QR boundary. Therefore each side gives
    // dimension = centerDistance / moduleSize + 7.
    result.topDistancePixels = centerDistance(topLeft, topRight);
    result.leftDistancePixels = centerDistance(topLeft, bottomLeft);
    if (!(result.topDistancePixels > 0.0) || !(result.leftDistancePixels > 0.0) ||
        !std::isfinite(result.topDistancePixels) || !std::isfinite(result.leftDistancePixels))
    {
        result.failureReason = "finder_center_distance_is_not_finite_and_positive";
        return result;
    }

    result.topCenterSeparationModules = result.topDistancePixels / result.moduleSize;
    result.leftCenterSeparationModules = result.leftDistancePixels / result.moduleSize;
    result.topContinuousDimension = result.topCenterSeparationModules + FinderCenterSeparationOffset;
    result.leftContinuousDimension = result.leftCenterSeparationModules + FinderCenterSeparationOffset;
    const double maximumRoundable = static_cast<double>(std::numeric_limits<int>::max() - 2);
    if (!std::isfinite(result.topContinuousDimension) ||
        !std::isfinite(result.leftContinuousDimension) ||
        result.topContinuousDimension > maximumRoundable ||
        result.leftContinuousDimension > maximumRoundable)
    {
        result.failureReason = "continuous_dimension_is_not_finite_or_representable";
        return result;
    }

    // Estimate the top and left sides independently. Rounding handles subpixel
    // measurement noise; modulo correction then moves each estimate onto the
    // legal Model 2 sequence 21, 25, 29, ... 177.
    result.geometryAvailable = true;
    result.topRoundedDimension = static_cast<int>(std::lround(result.topContinuousDimension));
    result.leftRoundedDimension = static_cast<int>(std::lround(result.leftContinuousDimension));
    result.topCorrection = correctionToModel2Dimension(result.topRoundedDimension);
    result.leftCorrection = correctionToModel2Dimension(result.leftRoundedDimension);
    result.topAdjustedDimension = result.topRoundedDimension + result.topCorrection;
    result.leftAdjustedDimension = result.leftRoundedDimension + result.leftCorrection;

    if (result.topAdjustedDimension < MinimumDimension ||
        result.topAdjustedDimension > MaximumDimension ||
        result.leftAdjustedDimension < MinimumDimension ||
        result.leftAdjustedDimension > MaximumDimension)
    {
        result.failureReason = "adjusted_dimension_outside_model2_range";
        return result;
    }

    result.adjustedDimensionDifference =
        std::abs(result.topAdjustedDimension - result.leftAdjustedDimension);
    if (result.adjustedDimensionDifference > MaximumSideDimensionDifference)
    {
        result.failureReason = "top_left_dimension_estimates_are_inconsistent";
        return result;
    }

    // If the sides disagree by one version step, prefer the side that needed
    // less modulo correction. The larger dimension breaks an exact tie.
    if (result.topAdjustedDimension == result.leftAdjustedDimension)
    {
        result.dimension = result.topAdjustedDimension;
        result.selectionBasis = "top_and_left_agree";
    }
    else if (std::abs(result.topCorrection) < std::abs(result.leftCorrection))
    {
        result.dimension = result.topAdjustedDimension;
        result.selectionBasis = "top_requires_smaller_modulo_correction";
    }
    else if (std::abs(result.leftCorrection) < std::abs(result.topCorrection))
    {
        result.dimension = result.leftAdjustedDimension;
        result.selectionBasis = "left_requires_smaller_modulo_correction";
    }
    else if (result.topAdjustedDimension > result.leftAdjustedDimension)
    {
        result.dimension = result.topAdjustedDimension;
        result.selectionBasis = "equal_correction_choose_larger_top";
    }
    else
    {
        result.dimension = result.leftAdjustedDimension;
        result.selectionBasis = "equal_correction_choose_larger_left";
    }

    if (result.dimension < MinimumDimension || result.dimension > MaximumDimension ||
        result.dimension % DimensionStep != RequiredModulo)
    {
        result.failureReason = "reconciled_dimension_is_not_valid_model2";
        result.dimension = 0;
        return result;
    }

    // QR Model 2 dimensions obey dimension = 17 + 4 * version.
    result.version = (result.dimension - 17) / DimensionStep;
    result.valid = true;
    return result;
}

void saveQrDimensionEstimationReport(
    const QrDimensionEstimationResult& result,
    const std::string& outputFilename,
    const std::string& inputFilename)
{
    using namespace QrDimensionEstimationConstants;
    std::ofstream out(outputFilename);
    if (!out) throw std::runtime_error("Cannot open QR dimension estimation report");

    out << "Stage 11: QR Code Model 2 dimension estimation\n"
        << "input_filename=" << inputFilename << '\n'
        << "equation=dimension=round(finder_center_distance/module_size)+7, then normalize to 1 mod 4\n"
        << "minimum_dimension=" << MinimumDimension
        << " maximum_dimension=" << MaximumDimension
        << " dimension_step=" << DimensionStep
        << " required_modulo=" << RequiredModulo
        << " maximum_side_dimension_difference=" << MaximumSideDimensionDifference << '\n';

    if (result.inputsAvailable)
        out << "selected_cluster_ids_TL_TR_BL=" << result.clusterIndices[0] << ','
            << result.clusterIndices[1] << ',' << result.clusterIndices[2] << '\n'
            << std::fixed << std::setprecision(6)
            << "module_size=" << result.moduleSize << '\n';
    else
        out << "selected_cluster_ids_TL_TR_BL=N/A\nmodule_size=N/A\n";

    if (result.geometryAvailable)
        out << std::fixed << std::setprecision(6)
            << "top_distance_pixels=" << result.topDistancePixels
            << " left_distance_pixels=" << result.leftDistancePixels << '\n'
            << "top_center_separation_modules=" << result.topCenterSeparationModules
            << " left_center_separation_modules=" << result.leftCenterSeparationModules << '\n'
            << "top_continuous_dimension=" << result.topContinuousDimension
            << " left_continuous_dimension=" << result.leftContinuousDimension << '\n'
            << "top_rounded_dimension=" << result.topRoundedDimension
            << " top_correction=" << result.topCorrection
            << " top_adjusted_dimension=" << result.topAdjustedDimension << '\n'
            << "left_rounded_dimension=" << result.leftRoundedDimension
            << " left_correction=" << result.leftCorrection
            << " left_adjusted_dimension=" << result.leftAdjustedDimension << '\n'
            << "adjusted_dimension_difference=" << result.adjustedDimensionDifference << '\n';
    else
        out << "geometry_measurements=N/A\n";

    if (result.valid)
        out << "result=ACCEPT dimension=" << result.dimension
            << " version=" << result.version
            << " selection_basis=" << result.selectionBasis << '\n';
    else
        out << "result=REJECT reason=" << result.failureReason << '\n';

    out.close();
    if (!out) throw std::runtime_error("Cannot write QR dimension estimation report");
}
