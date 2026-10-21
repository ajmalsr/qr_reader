#include "ModuleSizeEstimation.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace
{
ModuleSizeEstimationResult failure(const char* reason)
{
    ModuleSizeEstimationResult result;
    result.failureReason = reason;
    return result;
}
}

ModuleSizeEstimationResult estimateSelectedTripletModuleSize(
    const std::vector<PhysicalFinderCluster>& clusters,
    const FinderTripletSelectionResult& selection)
{
    using namespace ModuleSizeEstimationConstants;

    if (!selection.hasSelection)
        return failure("no_selected_finder_triplet");
    if (selection.selectedEvaluationIndex >= selection.evaluations.size())
        return failure("selected_evaluation_index_out_of_range");

    const auto& evaluation = selection.evaluations[selection.selectedEvaluationIndex];
    if (!evaluation.accepted)
        return failure("selected_evaluation_is_not_accepted");

    const auto ids = selection.selectedCandidateIndices;
    if (ids != std::array<std::size_t, 3>{evaluation.topLeftIndex,
                                          evaluation.topRightIndex,
                                          evaluation.bottomLeftIndex})
        return failure("selected_candidate_order_does_not_match_evaluation");
    if (ids[0] == ids[1] || ids[0] == ids[2] || ids[1] == ids[2])
        return failure("selected_finder_indices_are_not_unique");
    if (ids[0] >= clusters.size() || ids[1] >= clusters.size() || ids[2] >= clusters.size())
        return failure("selected_finder_index_out_of_range");

    ModuleSizeEstimationResult result;
    result.inputsAvailable = true;
    result.clusterIndices = ids;
    for (std::size_t role = 0; role < ids.size(); ++role)
        result.finderModuleSizes[role] = clusters[ids[role]].estimatedModuleSize;
    for (const double size : result.finderModuleSizes)
        if (!std::isfinite(size) || size <= 0.0)
        {
            result.failureReason = "finder_module_size_is_not_finite_and_positive";
            return result;
        }

    std::array<double, 3> sorted = result.finderModuleSizes;
    std::sort(sorted.begin(), sorted.end());
    result.minimumModuleSize = sorted[0];
    result.medianModuleSize = sorted[1];
    result.maximumModuleSize = sorted[2];
    result.arithmeticMean = (sorted[0] + sorted[1] + sorted[2]) / 3.0;
    result.relativeSpread = (result.maximumModuleSize - result.minimumModuleSize) /
                            result.medianModuleSize;

    double squaredDeviationSum = 0.0;
    for (const double size : result.finderModuleSizes)
    {
        result.maximumRelativeDeviation = std::max(
            result.maximumRelativeDeviation,
            std::abs(size - result.medianModuleSize) / result.medianModuleSize);
        const double difference = size - result.arithmeticMean;
        squaredDeviationSum += difference * difference;
    }
    // Population deviation describes these three selected measurements; they are the
    // complete inputs to this stage rather than a sample of a larger data set.
    result.standardDeviation = std::sqrt(squaredDeviationSum / 3.0);

    if (result.maximumRelativeDeviation > MaximumRelativeDeviationFromMedian)
    {
        result.failureReason = "finder_module_sizes_exceed_median_deviation_tolerance";
        return result;
    }

    // The median protects the consistency test from one extreme measurement. Once all
    // three pass that gate, an equal-weight mean retains information from every finder
    // without giving extra weight to finders that happened to be observed on more rows.
    result.estimatedModuleSize = result.arithmeticMean;
    result.valid = true;
    return result;
}

void saveModuleSizeEstimationReport(
    const ModuleSizeEstimationResult& result,
    const std::string& outputFilename,
    const std::string& inputFilename)
{
    std::ofstream out(outputFilename);
    if (!out) throw std::runtime_error("Cannot open module-size estimation report");

    out << "Stage 10: module-size estimation from the selected finder triplet\n"
        << "input_filename=" << inputFilename << '\n'
        << "unit=pixels_per_module\n"
        << "method=median consistency gate, then equal-weight arithmetic mean\n"
        << "maximum_relative_deviation_from_median="
        << ModuleSizeEstimationConstants::MaximumRelativeDeviationFromMedian << '\n';
    if (result.inputsAvailable)
        out << "selected_cluster_ids_TL_TR_BL=" << result.clusterIndices[0] << ','
            << result.clusterIndices[1] << ',' << result.clusterIndices[2] << '\n'
            << std::fixed << std::setprecision(6)
            << "finder_module_sizes_TL_TR_BL=" << result.finderModuleSizes[0] << ','
            << result.finderModuleSizes[1] << ',' << result.finderModuleSizes[2] << '\n'
            << "minimum=" << result.minimumModuleSize
            << " median=" << result.medianModuleSize
            << " maximum=" << result.maximumModuleSize
            << " arithmetic_mean=" << result.arithmeticMean
            << " standard_deviation=" << result.standardDeviation << '\n'
            << "relative_spread=" << result.relativeSpread
            << " maximum_relative_deviation=" << result.maximumRelativeDeviation << '\n';
    else
        out << "selected_cluster_ids_TL_TR_BL=N/A\n"
            << "finder_module_sizes_TL_TR_BL=N/A\n"
            << "measurements=N/A\n";

    if (result.valid)
        out << "result=ACCEPT estimated_module_size=" << result.estimatedModuleSize << '\n';
    else
        out << "result=REJECT reason=" << result.failureReason << '\n';

    out.close();
    if (!out) throw std::runtime_error("Cannot write module-size estimation report");
}
