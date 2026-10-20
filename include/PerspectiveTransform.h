#ifndef QR_LEARNING_PERSPECTIVE_TRANSFORM_H
#define QR_LEARNING_PERSPECTIVE_TRANSFORM_H

#include "AlignmentPattern.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace PerspectiveTransformConstants
{
constexpr double FinderCenterCoordinate = 3.5;
constexpr double AlignmentCenterInset = 6.5;
constexpr double MinimumPivotMagnitude = 1e-12;
constexpr double MinimumDenominatorMagnitude = 1e-12;
constexpr double MaximumControlReprojectionErrorPixels = 1e-6;
constexpr int ControlMarkerRadius = 5;
}

struct Point2D
{
    double x = 0.0;
    double y = 0.0;
};

// Homogeneous 3x3 matrix in row-major order. For input (x,y), divide the first
// two rows by the third row to obtain the projected Cartesian point.
struct PerspectiveTransform
{
    std::array<double, 9> coefficients{};
};

struct PerspectiveSolveResult
{
    bool valid = false;
    std::string failureReason;
    PerspectiveTransform transform;
    std::array<double, 4> reprojectionErrors{};
    double maximumReprojectionError = 0.0;
};

struct QrPerspectiveTransformResult
{
    bool valid = false;
    std::string failureReason;
    int dimension = 0;
    int version = 0;
    bool usedAlignmentPattern = false;
    std::array<std::size_t, 3> clusterIndices{};

    // Ordered TL, TR, fourth control point, BL. The fourth point is the alignment
    // center for version 2+, or the estimated bottom-right finder center for version 1.
    std::array<Point2D, 4> idealControlPoints{};
    std::array<Point2D, 4> imageControlPoints{};
    PerspectiveTransform idealToImage;
    PerspectiveTransform imageToIdeal;
    std::array<double, 4> forwardReprojectionErrors{};
    std::array<double, 4> inverseReprojectionErrors{};
    double maximumForwardReprojectionError = 0.0;
    double maximumInverseReprojectionError = 0.0;
    std::array<Point2D, 4> projectedBoundary{}; // Ideal (0,0), (D,0), (D,D), (0,D).
};

// Applies homogeneous division. False means the denominator is zero or a value
// is non-finite, so the point cannot be represented in ordinary image coordinates.
bool projectPoint(const PerspectiveTransform& transform,
                  const Point2D& input,
                  Point2D& output);

// Solves the eight linear equations for a homography from four point pairs.
PerspectiveSolveResult solvePerspectiveTransform(
    const std::array<Point2D, 4>& source,
    const std::array<Point2D, 4>& destination);

QrPerspectiveTransformResult buildQrPerspectiveTransform(
    const std::vector<PhysicalFinderCluster>& clusters,
    const QrDimensionEstimationResult& dimension,
    const AlignmentPatternSearchResult& alignment);

// Writes 11_perspective_transform.png and .txt. The PNG shows the projected QR
// boundary and all four control points; it does not sample or normalize modules.
void savePerspectiveTransformDebug(
    const Image& image,
    const QrPerspectiveTransformResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename);

#endif
