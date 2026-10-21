#include "PerspectiveTransform.h"

#include "lodepng.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace
{
using EquationRow = std::array<double, 9>; // Eight coefficients plus right-hand side.

bool finitePoint(const Point2D& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

double distance(const Point2D& a, const Point2D& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

PerspectiveSolveResult failedSolve(const char* reason)
{
    PerspectiveSolveResult result;
    result.failureReason = reason;
    return result;
}

QrPerspectiveTransformResult failedBuild(const char* reason)
{
    QrPerspectiveTransformResult result;
    result.failureReason = reason;
    return result;
}

void setPixel(Image& image, int x, int y,
              unsigned char red, unsigned char green, unsigned char blue)
{
    if (x < 0 || y < 0 || x >= static_cast<int>(image.width) ||
        y >= static_cast<int>(image.height)) return;
    const std::size_t index = 4 * (static_cast<std::size_t>(y) * image.width + x);
    image.rgba[index] = red;
    image.rgba[index + 1] = green;
    image.rgba[index + 2] = blue;
    image.rgba[index + 3] = 255;
}

void drawLine(Image& image, Point2D start, Point2D end,
              unsigned char red, unsigned char green, unsigned char blue)
{
    int x0 = static_cast<int>(std::lround(start.x));
    int y0 = static_cast<int>(std::lround(start.y));
    const int x1 = static_cast<int>(std::lround(end.x));
    const int y1 = static_cast<int>(std::lround(end.y));
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true)
    {
        setPixel(image, x0, y0, red, green, blue);
        if (x0 == x1 && y0 == y1) break;
        const int twiceError = 2 * error;
        if (twiceError >= dy) { error += dy; x0 += sx; }
        if (twiceError <= dx) { error += dx; y0 += sy; }
    }
}

void drawCross(Image& image, const Point2D& center, int radius,
               unsigned char red, unsigned char green, unsigned char blue)
{
    const int x = static_cast<int>(std::lround(center.x));
    const int y = static_cast<int>(std::lround(center.y));
    for (int offset = -radius; offset <= radius; ++offset)
    {
        setPixel(image, x + offset, y, red, green, blue);
        setPixel(image, x, y + offset, red, green, blue);
    }
}

void writePoint(std::ostream& out, const Point2D& point)
{
    out << point.x << ',' << point.y;
}
}

bool projectPoint(const PerspectiveTransform& transform,
                  const Point2D& input,
                  Point2D& output)
{
    using namespace PerspectiveTransformConstants;
    if (!finitePoint(input)) return false;
    // Homogeneous projection applies x' = (h00*x+h01*y+h02)/w and
    // y' = (h10*x+h11*y+h12)/w, where w = h20*x+h21*y+h22.
    const auto& h = transform.coefficients;
    const double denominator = h[6] * input.x + h[7] * input.y + h[8];
    if (!std::isfinite(denominator) || std::abs(denominator) < MinimumDenominatorMagnitude)
        return false;
    output.x = (h[0] * input.x + h[1] * input.y + h[2]) / denominator;
    output.y = (h[3] * input.x + h[4] * input.y + h[5]) / denominator;
    return finitePoint(output);
}

PerspectiveSolveResult solvePerspectiveTransform(
    const std::array<Point2D, 4>& source,
    const std::array<Point2D, 4>& destination)
{
    using namespace PerspectiveTransformConstants;
    for (std::size_t i = 0; i < 4; ++i)
        if (!finitePoint(source[i]) || !finitePoint(destination[i]))
            return failedSolve("control_point_is_not_finite");

    // Four point pairs provide eight scalar equations for the eight unknown
    // homography coefficients; h22 is fixed to 1 to remove arbitrary scale.
    std::array<EquationRow, 8> equations{};
    for (std::size_t i = 0; i < 4; ++i)
    {
        const double u = source[i].x;
        const double v = source[i].y;
        const double x = destination[i].x;
        const double y = destination[i].y;
        equations[2 * i] = {u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v, x};
        equations[2 * i + 1] = {0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v, y};
    }

    // Gauss-Jordan elimination with partial pivoting keeps the educational solver
    // compact while rejecting degenerate or almost-degenerate point arrangements.
    for (std::size_t column = 0; column < 8; ++column)
    {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < 8; ++row)
            if (std::abs(equations[row][column]) > std::abs(equations[pivot][column]))
                pivot = row;
        if (std::abs(equations[pivot][column]) < MinimumPivotMagnitude)
            return failedSolve("degenerate_control_points");
        if (pivot != column) std::swap(equations[pivot], equations[column]);

        const double divisor = equations[column][column];
        for (std::size_t entry = column; entry < 9; ++entry)
            equations[column][entry] /= divisor;
        for (std::size_t row = 0; row < 8; ++row)
        {
            if (row == column) continue;
            const double factor = equations[row][column];
            for (std::size_t entry = column; entry < 9; ++entry)
                equations[row][entry] -= factor * equations[column][entry];
        }
    }

    PerspectiveSolveResult result;
    for (std::size_t i = 0; i < 8; ++i)
        result.transform.coefficients[i] = equations[i][8];
    result.transform.coefficients[8] = 1.0;

    for (std::size_t i = 0; i < 4; ++i)
    {
        Point2D projected;
        if (!projectPoint(result.transform, source[i], projected))
            return failedSolve("control_point_projection_failed");
        result.reprojectionErrors[i] = distance(projected, destination[i]);
        result.maximumReprojectionError = std::max(
            result.maximumReprojectionError, result.reprojectionErrors[i]);
    }
    if (result.maximumReprojectionError > MaximumControlReprojectionErrorPixels)
        return failedSolve("control_reprojection_error_too_large");
    result.valid = true;
    return result;
}

QrPerspectiveTransformResult buildQrPerspectiveTransform(
    const std::vector<PhysicalFinderCluster>& clusters,
    const QrDimensionEstimationResult& dimension,
    const AlignmentPatternSearchResult& alignment)
{
    using namespace PerspectiveTransformConstants;
    if (!dimension.valid) return failedBuild("no_valid_qr_dimension");
    if (dimension.dimension < 21 || dimension.dimension > 177 ||
        dimension.dimension % 4 != 1 || dimension.version < 1 || dimension.version > 40)
        return failedBuild("invalid_model_2_dimension_or_version");
    if (dimension.version != (dimension.dimension - 17) / 4)
        return failedBuild("dimension_and_version_are_inconsistent");

    QrPerspectiveTransformResult result;
    result.dimension = dimension.dimension;
    result.version = dimension.version;
    result.clusterIndices = dimension.clusterIndices;
    for (const std::size_t id : result.clusterIndices)
        if (id >= clusters.size()) return failedBuild("finder_index_out_of_range");

    const auto& tl = clusters[result.clusterIndices[0]];
    const auto& tr = clusters[result.clusterIndices[1]];
    const auto& bl = clusters[result.clusterIndices[2]];
    const Point2D imageTl{tl.centerX, tl.centerY};
    const Point2D imageTr{tr.centerX, tr.centerY};
    const Point2D imageBl{bl.centerX, bl.centerY};
    if (!finitePoint(imageTl) || !finitePoint(imageTr) || !finitePoint(imageBl))
        return failedBuild("finder_center_is_not_finite");

    // QR-space uses module coordinates. Finder centers lie at 3.5 modules from
    // their adjacent boundaries, so the far coordinate is dimension - 3.5.
    const double farFinder = result.dimension - FinderCenterCoordinate;
    result.idealControlPoints[0] = {FinderCenterCoordinate, FinderCenterCoordinate};
    result.idealControlPoints[1] = {farFinder, FinderCenterCoordinate};
    result.idealControlPoints[3] = {FinderCenterCoordinate, farFinder};
    result.imageControlPoints[0] = imageTl;
    result.imageControlPoints[1] = imageTr;
    result.imageControlPoints[3] = imageBl;

    if (result.version == 1)
    {
        // Version 1 has no alignment target. The fourth correspondence is the
        // parallelogram completion of the three measured finder centers.
        result.idealControlPoints[2] = {farFinder, farFinder};
        result.imageControlPoints[2] = {
            imageTr.x + imageBl.x - imageTl.x,
            imageTr.y + imageBl.y - imageTl.y};
    }
    else
    {
        if (!alignment.valid || !alignment.required || !alignment.found)
            return failedBuild("required_alignment_pattern_is_unavailable");
        if (alignment.dimension != result.dimension || alignment.version != result.version)
            return failedBuild("alignment_dimension_or_version_differs");
        if (alignment.clusterIndices != result.clusterIndices)
            return failedBuild("alignment_and_dimension_finder_ids_differ");
        result.usedAlignmentPattern = true;
        const double alignmentCoordinate = result.dimension - AlignmentCenterInset;
        result.idealControlPoints[2] = {alignmentCoordinate, alignmentCoordinate};
        result.imageControlPoints[2] = {alignment.selected.centerX, alignment.selected.centerY};
        if (!finitePoint(result.imageControlPoints[2]))
            return failedBuild("alignment_center_is_not_finite");
    }

    // Solve both directions independently. The forward map samples the image;
    // the inverse map is retained for diagnostics and later geometric queries.
    const PerspectiveSolveResult forward = solvePerspectiveTransform(
        result.idealControlPoints, result.imageControlPoints);
    if (!forward.valid)
    {
        result.failureReason = "forward_transform_" + forward.failureReason;
        return result;
    }
    const PerspectiveSolveResult inverse = solvePerspectiveTransform(
        result.imageControlPoints, result.idealControlPoints);
    if (!inverse.valid)
    {
        result.failureReason = "inverse_transform_" + inverse.failureReason;
        return result;
    }
    result.idealToImage = forward.transform;
    result.imageToIdeal = inverse.transform;
    result.forwardReprojectionErrors = forward.reprojectionErrors;
    result.inverseReprojectionErrors = inverse.reprojectionErrors;
    result.maximumForwardReprojectionError = forward.maximumReprojectionError;
    result.maximumInverseReprojectionError = inverse.maximumReprojectionError;

    // Project the ideal boundary corners to visualize the entire QR quadrilateral.
    // These corners are boundaries, while later sampling uses module centers.
    const double d = static_cast<double>(result.dimension);
    const std::array<Point2D, 4> idealBoundary{{{0.0,0.0},{d,0.0},{d,d},{0.0,d}}};
    for (std::size_t i = 0; i < 4; ++i)
        if (!projectPoint(result.idealToImage, idealBoundary[i], result.projectedBoundary[i]))
        {
            result.failureReason = "projected_qr_boundary_is_unrepresentable";
            return result;
        }
    result.valid = true;
    return result;
}

void savePerspectiveTransformDebug(
    const Image& image,
    const QrPerspectiveTransformResult& result,
    const std::string& outputDirectory,
    const std::string& inputFilename)
{
    if (!image.width || !image.height || image.rgba.size() != image.pixelCount() * Image::channels)
        throw std::invalid_argument("Invalid image for perspective-transform debug");

    Image debug = image;
    if (result.valid)
    {
        for (std::size_t i = 0; i < 4; ++i)
            drawLine(debug, result.projectedBoundary[i], result.projectedBoundary[(i + 1) % 4],
                     0, 255, 255);
        drawCross(debug, result.imageControlPoints[0], PerspectiveTransformConstants::ControlMarkerRadius,
                  0, 255, 0);
        drawCross(debug, result.imageControlPoints[1], PerspectiveTransformConstants::ControlMarkerRadius,
                  255, 215, 0);
        drawCross(debug, result.imageControlPoints[3], PerspectiveTransformConstants::ControlMarkerRadius,
                  255, 215, 0);
        drawCross(debug, result.imageControlPoints[2], PerspectiveTransformConstants::ControlMarkerRadius,
                  255, 0, 255);
    }

    const std::string pngFilename = outputDirectory + "/11_perspective_transform.png";
    const unsigned pngError = lodepng::encode(
        pngFilename, debug.rgba, debug.width, debug.height, LCT_RGBA, 8);
    if (pngError)
        throw std::runtime_error("Perspective-transform debug PNG writing failed: " +
                                 std::string(lodepng_error_text(pngError)));

    std::ofstream out(outputDirectory + "/11_perspective_transform.txt");
    if (!out) throw std::runtime_error("Cannot open perspective-transform report");
    out << "Stage 13: QR perspective transform\n"
        << "input_filename=" << inputFilename << '\n'
        << "finder_center_coordinate=" << PerspectiveTransformConstants::FinderCenterCoordinate
        << " alignment_center_inset=" << PerspectiveTransformConstants::AlignmentCenterInset
        << " minimum_pivot_magnitude=" << PerspectiveTransformConstants::MinimumPivotMagnitude
        << " minimum_denominator_magnitude=" << PerspectiveTransformConstants::MinimumDenominatorMagnitude
        << " maximum_control_reprojection_error_pixels="
        << PerspectiveTransformConstants::MaximumControlReprojectionErrorPixels << '\n'
        << "dimension=" << result.dimension << " version=" << result.version
        << " used_alignment_pattern=" << result.usedAlignmentPattern << '\n'
        << std::fixed << std::setprecision(9);

    if (!result.valid)
    {
        out << "result=REJECT reason=" << result.failureReason << '\n';
    }
    else
    {
        out << "selected_cluster_ids_TL_TR_BL=" << result.clusterIndices[0] << ','
            << result.clusterIndices[1] << ',' << result.clusterIndices[2] << '\n';
        const char* labels[4] = {"TL", "TR", result.usedAlignmentPattern ? "ALIGNMENT" : "ESTIMATED_BR", "BL"};
        for (std::size_t i = 0; i < 4; ++i)
        {
            out << "control_" << labels[i] << " ideal=";
            writePoint(out, result.idealControlPoints[i]);
            out << " image=";
            writePoint(out, result.imageControlPoints[i]);
            out << " forward_error_pixels=" << result.forwardReprojectionErrors[i]
                << " inverse_error_modules=" << result.inverseReprojectionErrors[i] << '\n';
        }
        out << "ideal_to_image_coefficients=";
        for (std::size_t i = 0; i < 9; ++i) out << (i ? "," : "") << result.idealToImage.coefficients[i];
        out << "\nimage_to_ideal_coefficients=";
        for (std::size_t i = 0; i < 9; ++i) out << (i ? "," : "") << result.imageToIdeal.coefficients[i];
        out << "\nmaximum_forward_reprojection_error_pixels=" << result.maximumForwardReprojectionError
            << " maximum_inverse_reprojection_error_modules=" << result.maximumInverseReprojectionError << '\n';
        const char* cornerLabels[4] = {"TOP_LEFT", "TOP_RIGHT", "BOTTOM_RIGHT", "BOTTOM_LEFT"};
        for (std::size_t i = 0; i < 4; ++i)
        {
            out << "projected_boundary_" << cornerLabels[i] << '=';
            writePoint(out, result.projectedBoundary[i]);
            out << '\n';
        }
        out << "result=VALID\n";
    }
    out << "PNG: cyan=projected QR boundary, green=TL, yellow=TR/BL, "
           "magenta=alignment or estimated BR control point\n";
    out.close();
    if (!out) throw std::runtime_error("Cannot write perspective-transform report");
}
