#ifndef QR_LEARNING_MODULE_SIZE_ESTIMATION_H
#define QR_LEARNING_MODULE_SIZE_ESTIMATION_H

#include "FinderTripletSelection.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// Stage 10 combines the three local module-size measurements that belong to the
// selected top-left, top-right, and bottom-left finder clusters. It deliberately
// produces only a pixel-per-module estimate; QR dimension estimation is Stage 11.
namespace ModuleSizeEstimationConstants
{
// The median is resistant to one unusually large or small finder measurement.
// Every finder must remain within 35% of that median before all three are averaged.
constexpr double MaximumRelativeDeviationFromMedian = 0.35;
}

struct ModuleSizeEstimationResult
{
    bool valid = false;
    bool inputsAvailable = false; // True after TL/TR/BL references have been validated.
    std::string failureReason;

    std::array<std::size_t, 3> clusterIndices{}; // TL, TR, BL.
    std::array<double, 3> finderModuleSizes{};   // Pixels per module, TL/TR/BL.

    double minimumModuleSize = 0.0;
    double maximumModuleSize = 0.0;
    double medianModuleSize = 0.0;
    double arithmeticMean = 0.0;
    double standardDeviation = 0.0;
    double relativeSpread = 0.0;                // (max-min)/median.
    double maximumRelativeDeviation = 0.0;      // max(|size-median|/median).
    double estimatedModuleSize = 0.0;            // Final pixels-per-module result.
};

// Validates the selected triplet and combines its three cluster measurements.
// Failure is returned as data so the pipeline can still write a useful report.
ModuleSizeEstimationResult estimateSelectedTripletModuleSize(
    const std::vector<PhysicalFinderCluster>& clusters,
    const FinderTripletSelectionResult& selection);

// Writes the compact Stage-10 report. Recording the input filename prevents the
// provenance ambiguity present in some earlier diagnostic text files.
void saveModuleSizeEstimationReport(
    const ModuleSizeEstimationResult& result,
    const std::string& outputFilename,
    const std::string& inputFilename);

#endif
