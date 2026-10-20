#ifndef QR_LEARNING_QR_DIMENSION_ESTIMATION_H
#define QR_LEARNING_QR_DIMENSION_ESTIMATION_H

#include "ModuleSizeEstimation.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// A Model 2 QR symbol starts at 21 modules and grows by four modules per version.
// Finder centers are 3.5 modules from their outer edges, so their center-to-center
// separation is dimension - 7 modules along both the top and left sides.
namespace QrDimensionEstimationConstants
{
constexpr int FinderCenterSeparationOffset = 7;
constexpr int MinimumDimension = 21;
constexpr int MaximumDimension = 177;
constexpr int DimensionStep = 4;
constexpr int RequiredModulo = 1;
constexpr int MaximumSideDimensionDifference = 4; // At most one QR-version step.
}

struct QrDimensionEstimationResult
{
    bool valid = false;
    bool inputsAvailable = false;
    bool geometryAvailable = false;
    std::string failureReason;
    std::string selectionBasis;

    std::array<std::size_t, 3> clusterIndices{}; // TL, TR, BL.
    double moduleSize = 0.0;
    double topDistancePixels = 0.0;
    double leftDistancePixels = 0.0;
    double topCenterSeparationModules = 0.0;
    double leftCenterSeparationModules = 0.0;
    double topContinuousDimension = 0.0;
    double leftContinuousDimension = 0.0;
    int topRoundedDimension = 0;
    int leftRoundedDimension = 0;
    int topCorrection = 0;
    int leftCorrection = 0;
    int topAdjustedDimension = 0;
    int leftAdjustedDimension = 0;
    int adjustedDimensionDifference = 0;
    int dimension = 0;
    int version = 0;
};

// Uses the Stage-10 estimate and its ordered finder IDs to independently estimate
// the top and left dimensions, reconcile them, and return one valid Model 2 size.
QrDimensionEstimationResult estimateQrDimension(
    const std::vector<PhysicalFinderCluster>& clusters,
    const ModuleSizeEstimationResult& moduleSize);

void saveQrDimensionEstimationReport(
    const QrDimensionEstimationResult& result,
    const std::string& outputFilename,
    const std::string& inputFilename);

#endif
