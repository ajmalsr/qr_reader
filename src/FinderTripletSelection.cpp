#include "FinderTripletSelection.h"
#include "lodepng.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>

const char* tripletReasonCode(TripletRejectionReason reason)
{
    switch (reason)
    {
    case TripletRejectionReason::None: return "OK";
    case TripletRejectionReason::InvalidGeometry: return "IG";
    case TripletRejectionReason::ModuleSizeMismatch: return "MS";
    case TripletRejectionReason::CandidatesTooClose: return "DC";
    case TripletRejectionReason::NearlyCollinear: return "CL";
    case TripletRejectionReason::PoorRightAngle: return "RA";
    case TripletRejectionReason::SideLengthMismatch: return "SL";
    case TripletRejectionReason::PythagoreanMismatch: return "PY";
    }
    return "IG";
}

const char* tripletReasonText(TripletRejectionReason reason)
{
    switch (reason)
    {
    case TripletRejectionReason::None: return "accepted geometry";
    case TripletRejectionReason::InvalidGeometry: return "non-finite or degenerate geometry";
    case TripletRejectionReason::ModuleSizeMismatch: return "finder module sizes are inconsistent";
    case TripletRejectionReason::CandidatesTooClose: return "finder centers are too close";
    case TripletRejectionReason::NearlyCollinear: return "finder centers are nearly collinear";
    case TripletRejectionReason::PoorRightAngle: return "largest-angle corner is not close enough to 90 degrees";
    case TripletRejectionReason::SideLengthMismatch: return "the two QR side lengths differ excessively";
    case TripletRejectionReason::PythagoreanMismatch: return "diagonal is inconsistent with the two QR sides";
    }
    return "unknown geometry failure";
}

namespace
{
double distance(const PhysicalFinderCluster& a, const PhysicalFinderCluster& b)
{
    return std::hypot(a.centerX - b.centerX, a.centerY - b.centerY);
}

bool finiteCluster(const PhysicalFinderCluster& c)
{
    return std::isfinite(c.centerX) && std::isfinite(c.centerY) &&
           std::isfinite(c.estimatedModuleSize) && c.estimatedModuleSize > 0.0;
}

FinderTripletEvaluation evaluateTriplet(
    const std::vector<PhysicalFinderCluster>& clusters,
    std::size_t i, std::size_t j, std::size_t k)
{
    using namespace FinderTripletConstants;
    FinderTripletEvaluation e;
    e.candidateIndices = {i, j, k};
    const auto& c0 = clusters[i];
    const auto& c1 = clusters[j];
    const auto& c2 = clusters[k];
    if (!finiteCluster(c0) || !finiteCluster(c1) || !finiteCluster(c2))
    {
        e.rejectionReason = TripletRejectionReason::InvalidGeometry;
        return e;
    }

    const double d01 = distance(c0, c1), d02 = distance(c0, c2), d12 = distance(c1, c2);
    if (!(d01 > 0.0 && d02 > 0.0 && d12 > 0.0) ||
        !std::isfinite(d01) || !std::isfinite(d02) || !std::isfinite(d12))
    {
        e.rejectionReason = TripletRejectionReason::InvalidGeometry;
        return e;
    }

    // In a QR finder triangle, the longest pair is top-right to bottom-left. The
    // remaining point is therefore the right-angle (top-left) vertex.
    std::size_t armA = 0, armB = 0;
    if (d12 >= d01 && d12 >= d02)
    {
        e.topLeftIndex = i; armA = j; armB = k; e.diagonalLength = d12;
    }
    else if (d02 >= d01 && d02 >= d12)
    {
        e.topLeftIndex = j; armA = i; armB = k; e.diagonalLength = d02;
    }
    else
    {
        e.topLeftIndex = k; armA = i; armB = j; e.diagonalLength = d01;
    }

    const auto& tl = clusters[e.topLeftIndex];
    const auto& a = clusters[armA];
    const auto& b = clusters[armB];
    const double ax = a.centerX - tl.centerX, ay = a.centerY - tl.centerY;
    const double bx = b.centerX - tl.centerX, by = b.centerY - tl.centerY;
    const double cross = ax * by - ay * bx;

    // Image Y grows downward. Choosing a positive cross product gives a stable TL,TR,BL
    // ordering for any rotation; it does not assume the QR is axis-aligned.
    if (cross >= 0.0)
    {
        e.topRightIndex = armA; e.bottomLeftIndex = armB;
    }
    else
    {
        e.topRightIndex = armB; e.bottomLeftIndex = armA;
    }

    e.topSideLength = distance(tl, clusters[e.topRightIndex]);
    e.leftSideLength = distance(tl, clusters[e.bottomLeftIndex]);
    const double legProduct = e.topSideLength * e.leftSideLength;
    const double dot = ax * bx + ay * by;
    const double largestModule = std::max({c0.estimatedModuleSize, c1.estimatedModuleSize,
                                           c2.estimatedModuleSize});
    const double smallestModule = std::min({c0.estimatedModuleSize, c1.estimatedModuleSize,
                                            c2.estimatedModuleSize});
    e.averageModuleSize = (c0.estimatedModuleSize + c1.estimatedModuleSize +
                           c2.estimatedModuleSize) / 3.0;
    e.moduleSizeError = (largestModule - smallestModule) / largestModule;
    e.minimumCenterDistanceModules = std::min({d01, d02, d12}) / e.averageModuleSize;
    e.collinearitySine = std::abs(cross) / legProduct;
    e.rightAngleError = std::abs(dot) / legProduct;
    e.sideLengthError = std::abs(e.topSideLength - e.leftSideLength) /
                        std::max(e.topSideLength, e.leftSideLength);
    const double expectedDiagonal = std::hypot(e.topSideLength, e.leftSideLength);
    e.pythagoreanError = std::abs(e.diagonalLength - expectedDiagonal) /
                         std::max(e.diagonalLength, expectedDiagonal);

    if (!std::isfinite(e.collinearitySine) || !std::isfinite(e.rightAngleError) ||
        !std::isfinite(e.sideLengthError) || !std::isfinite(e.pythagoreanError))
        e.rejectionReason = TripletRejectionReason::InvalidGeometry;
    else if (e.moduleSizeError > ModuleSizeRelativeTolerance)
        e.rejectionReason = TripletRejectionReason::ModuleSizeMismatch;
    else if (e.minimumCenterDistanceModules < MinimumCenterDistanceModules)
        e.rejectionReason = TripletRejectionReason::CandidatesTooClose;
    else if (e.collinearitySine < CollinearitySineTolerance)
        e.rejectionReason = TripletRejectionReason::NearlyCollinear;
    else if (e.rightAngleError > RightAngleMaximumCosine)
        e.rejectionReason = TripletRejectionReason::PoorRightAngle;
    else if (e.sideLengthError > SideLengthRelativeTolerance)
        e.rejectionReason = TripletRejectionReason::SideLengthMismatch;
    else if (e.pythagoreanError > PythagoreanRelativeTolerance)
        e.rejectionReason = TripletRejectionReason::PythagoreanMismatch;
    else
    {
        e.accepted = true;
        e.rejectionReason = TripletRejectionReason::None;
        // Normalize each error by its allowed tolerance so different units contribute
        // comparably. A lower score means more consistent scale and QR geometry.
        e.score = e.moduleSizeError / ModuleSizeRelativeTolerance +
                  e.rightAngleError / RightAngleMaximumCosine +
                  e.sideLengthError / SideLengthRelativeTolerance +
                  e.pythagoreanError / PythagoreanRelativeTolerance;
    }
    return e;
}

void blendPixel(Image& image, int x, int y, unsigned char r, unsigned char g,
                unsigned char b, unsigned char alpha)
{
    if (x < 0 || y < 0 || x >= static_cast<int>(image.width) || y >= static_cast<int>(image.height)) return;
    const std::size_t p = (static_cast<std::size_t>(y) * image.width + static_cast<unsigned>(x)) * 4;
    const unsigned inverse = 255U - alpha;
    image.rgba[p] = static_cast<unsigned char>((image.rgba[p] * inverse + r * alpha) / 255U);
    image.rgba[p + 1] = static_cast<unsigned char>((image.rgba[p + 1] * inverse + g * alpha) / 255U);
    image.rgba[p + 2] = static_cast<unsigned char>((image.rgba[p + 2] * inverse + b * alpha) / 255U);
    image.rgba[p + 3] = 255;
}

void line(Image& image, double x0d, double y0d, double x1d, double y1d,
          unsigned char r, unsigned char g, unsigned char b, unsigned char alpha, int thickness)
{
    int x0 = static_cast<int>(std::lround(x0d)), y0 = static_cast<int>(std::lround(y0d));
    const int x1 = static_cast<int>(std::lround(x1d)), y1 = static_cast<int>(std::lround(y1d));
    const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;)
    {
        for (int oy = -thickness / 2; oy <= thickness / 2; ++oy)
            for (int ox = -thickness / 2; ox <= thickness / 2; ++ox)
                blendPixel(image, x0 + ox, y0 + oy, r, g, b, alpha);
        if (x0 == x1 && y0 == y1) break;
        const int twice = 2 * error;
        if (twice >= dy) { error += dy; x0 += sx; }
        if (twice <= dx) { error += dx; y0 += sy; }
    }
}

void triangle(Image& image, const std::vector<PhysicalFinderCluster>& clusters,
              const FinderTripletEvaluation& e, unsigned char r, unsigned char g,
              unsigned char b, unsigned char alpha, int thickness)
{
    const auto& a = clusters[e.candidateIndices[0]];
    const auto& c = clusters[e.candidateIndices[1]];
    const auto& d = clusters[e.candidateIndices[2]];
    line(image, a.centerX, a.centerY, c.centerX, c.centerY, r, g, b, alpha, thickness);
    line(image, c.centerX, c.centerY, d.centerX, d.centerY, r, g, b, alpha, thickness);
    line(image, d.centerX, d.centerY, a.centerX, a.centerY, r, g, b, alpha, thickness);
}

// A compact 3x5 font is sufficient for cluster numbers and two-letter rejection codes.
// Each row stores three on/off bits, most-significant bit on the left.
std::array<unsigned char, 5> glyph(char c)
{
    switch (c)
    {
    case '0': return {7,5,5,5,7}; case '1': return {2,6,2,2,7};
    case '2': return {7,1,7,4,7}; case '3': return {7,1,7,1,7};
    case '4': return {5,5,7,1,1}; case '5': return {7,4,7,1,7};
    case '6': return {7,4,7,5,7}; case '7': return {7,1,1,1,1};
    case '8': return {7,5,7,5,7}; case '9': return {7,5,7,1,7};
    case 'A': return {2,5,7,5,5}; case 'B': return {6,5,6,5,6};
    case 'C': return {7,4,4,4,7};
    case 'D': return {6,5,5,5,6}; case 'G': return {7,4,5,5,7};
    case 'F': return {7,4,6,4,4}; case 'I': return {7,2,2,2,7};
    case 'L': return {4,4,4,4,7};
    case 'M': return {5,7,7,5,5}; case 'O': return {7,5,5,5,7};
    case 'P': return {7,5,7,4,4}; case 'R': return {6,5,6,5,5};
    case 'S': return {7,4,7,1,7}; case 'T': return {7,2,2,2,2};
    case 'Y': return {5,5,2,2,2};
    default: return {0,0,0,0,0};
    }
}

void text(Image& image, int x, int y, const std::string& value,
          unsigned char r, unsigned char g, unsigned char b)
{
    constexpr int scale = 2;
    for (char c : value)
    {
        const auto rows = glyph(c);
        for (int row = 0; row < 5; ++row)
            for (int column = 0; column < 3; ++column)
                if (rows[row] & (1U << (2 - column)))
                    for (int sy = 0; sy < scale; ++sy)
                        for (int sx = 0; sx < scale; ++sx)
                            blendPixel(image, x + column * scale + sx, y + row * scale + sy,
                                       r, g, b, 255);
        x += 4 * scale;
    }
}

void candidateMarker(Image& image, const PhysicalFinderCluster& c, std::size_t index)
{
    const int x = static_cast<int>(std::lround(c.centerX));
    const int y = static_cast<int>(std::lround(c.centerY));
    const int radius = FinderTripletConstants::CandidateMarkerRadius;
    for (int d = -radius; d <= radius; ++d)
    {
        blendPixel(image, x + d, y - radius, 0, 255, 255, 255);
        blendPixel(image, x + d, y + radius, 0, 255, 255, 255);
        blendPixel(image, x - radius, y + d, 0, 255, 255, 255);
        blendPixel(image, x + radius, y + d, 0, 255, 255, 255);
    }
    text(image, x + radius + 2, y - radius, std::to_string(index), 0, 255, 255);
}
}

FinderTripletSelectionResult selectFinderTriplet(
    const std::vector<PhysicalFinderCluster>& clusters)
{
    FinderTripletSelectionResult result;
    if (clusters.size() < 3)
    {
        result.overallFailureReason = "fewer than three physical finder clusters";
        return result;
    }

    double bestScore = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 2 < clusters.size(); ++i)
        for (std::size_t j = i + 1; j + 1 < clusters.size(); ++j)
            for (std::size_t k = j + 1; k < clusters.size(); ++k)
            {
                result.evaluations.push_back(evaluateTriplet(clusters, i, j, k));
                const std::size_t index = result.evaluations.size() - 1;
                const auto& e = result.evaluations.back();
                if (e.accepted && e.score < bestScore)
                {
                    bestScore = e.score;
                    result.hasSelection = true;
                    result.selectedEvaluationIndex = index;
                    result.selectedCandidateIndices = {e.topLeftIndex, e.topRightIndex, e.bottomLeftIndex};
                }
            }
    if (!result.hasSelection)
        result.overallFailureReason = "all finder triplet combinations failed geometry checks";
    return result;
}

void saveFinderTripletSelectionDebug(
    const Image& image,
    const std::vector<PhysicalFinderCluster>& clusters,
    const FinderTripletSelectionResult& result,
    const std::string& outputDirectory)
{
    if (!image.width || !image.height || image.rgba.size() != image.pixelCount() * Image::channels)
        throw std::invalid_argument("Invalid image for finder triplet debug");

    Image debug = image;
    // Rejections go down first and remain translucent. Valid alternatives and the chosen
    // triplet are layered above them so the final decision stays readable.
    for (std::size_t n = 0; n < result.evaluations.size(); ++n)
    {
        const auto& e = result.evaluations[n];
        if (e.accepted) continue;
        triangle(debug, clusters, e, 255, 0, 0, 105, 1);
        const auto& a = clusters[e.candidateIndices[0]];
        const auto& b = clusters[e.candidateIndices[1]];
        const auto& c = clusters[e.candidateIndices[2]];
        const int x = static_cast<int>(std::lround((a.centerX + b.centerX + c.centerX) / 3.0));
        const int y = static_cast<int>(std::lround((a.centerY + b.centerY + c.centerY) / 3.0));
        text(debug, x + static_cast<int>(n % 3) * 9, y + static_cast<int>(n % 4) * 11,
             std::to_string(n) + tripletReasonCode(e.rejectionReason), 255, 40, 40);
    }
    for (std::size_t n = 0; n < result.evaluations.size(); ++n)
        if (result.evaluations[n].accepted && n != result.selectedEvaluationIndex)
            triangle(debug, clusters, result.evaluations[n], 255, 215, 0, 190, 2);
    if (result.hasSelection)
        triangle(debug, clusters, result.evaluations[result.selectedEvaluationIndex], 0, 255, 0, 255, 3);
    for (std::size_t i = 0; i < clusters.size(); ++i) candidateMarker(debug, clusters[i], i);
    if (result.hasSelection)
    {
        // Role labels make the ordering visible without consulting the text report.
        const auto labelRole = [&](std::size_t index, const char* role) {
            const auto& c = clusters[index];
            text(debug, static_cast<int>(std::lround(c.centerX)) - 7,
                 static_cast<int>(std::lround(c.centerY)) + FinderTripletConstants::CandidateMarkerRadius + 3,
                 role, 0, 255, 0);
        };
        labelRole(result.selectedCandidateIndices[0], "TL");
        labelRole(result.selectedCandidateIndices[1], "TR");
        labelRole(result.selectedCandidateIndices[2], "BL");
    }
    else if (clusters.size() < 3)
    {
        // There is no triangle to annotate in this case, so place the overall failure
        // code at the mean candidate position (or image center when there are none).
        double x = image.width / 2.0, y = image.height / 2.0;
        if (!clusters.empty())
        {
            x = 0.0; y = 0.0;
            for (const auto& c : clusters) { x += c.centerX; y += c.centerY; }
            x /= clusters.size(); y /= clusters.size();
        }
        text(debug, static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)),
             "F3", 255, 40, 40);
    }

    const std::string pngName = outputDirectory + "/07_finder_triplet_selection.png";
    const unsigned pngError = lodepng::encode(pngName, debug.rgba, debug.width, debug.height, LCT_RGBA, 8);
    if (pngError) throw std::runtime_error("Triplet PNG writing failed: " + std::string(lodepng_error_text(pngError)));

    std::ofstream out(outputDirectory + "/07_finder_triplet_selection.txt");
    if (!out) throw std::runtime_error("Cannot open finder triplet selection log");
    using namespace FinderTripletConstants;
    out << "Finder triplet selection (detection stage 9)\n"
        << "Input: merged physical finder clusters; output: at most one ordered TL/TR/BL triplet\n"
        << "Reason codes: F3=fewer than three clusters, IG=invalid geometry, MS=module-size mismatch, DC=centers too close, "
           "CL=nearly collinear, RA=poor right angle, SL=side-length mismatch, PY=Pythagorean mismatch\n"
        << "PNG: cyan boxes/IDs=candidates, red triangle+triplet-ID/reason-code=rejected, yellow=valid alternative, green=selected with TL/TR/BL labels\n"
        << "module_size_relative_tolerance=" << ModuleSizeRelativeTolerance
        << " minimum_center_distance_modules=" << MinimumCenterDistanceModules
        << " collinearity_sine_tolerance=" << CollinearitySineTolerance
        << " right_angle_maximum_cosine=" << RightAngleMaximumCosine
        << " side_length_relative_tolerance=" << SideLengthRelativeTolerance
        << " pythagorean_relative_tolerance=" << PythagoreanRelativeTolerance << '\n'
        << "physical_finder_clusters=" << clusters.size()
        << " combinations_tested=" << result.evaluations.size() << '\n'
        << std::fixed << std::setprecision(6);

    std::map<TripletRejectionReason, std::size_t> rejectionCounts;
    std::size_t acceptedCount = 0;
    for (std::size_t n = 0; n < result.evaluations.size(); ++n)
    {
        const auto& e = result.evaluations[n];
        if (e.accepted) ++acceptedCount; else ++rejectionCounts[e.rejectionReason];
        out << "triplet_" << n << ": candidate_ids=" << e.candidateIndices[0] << ','
            << e.candidateIndices[1] << ',' << e.candidateIndices[2]
            << " ordered_TL_TR_BL=" << e.topLeftIndex << ',' << e.topRightIndex << ',' << e.bottomLeftIndex
            << " average_module_size=" << e.averageModuleSize
            << " module_size_error=" << e.moduleSizeError
            << " minimum_center_distance_modules=" << e.minimumCenterDistanceModules
            << " top_side=" << e.topSideLength << " left_side=" << e.leftSideLength
            << " diagonal=" << e.diagonalLength
            << " collinearity_sine=" << e.collinearitySine
            << " right_angle_error=" << e.rightAngleError
            << " side_length_error=" << e.sideLengthError
            << " pythagorean_error=" << e.pythagoreanError;
        if (e.accepted)
            out << " score=" << e.score << " result=ACCEPT"
                << (n == result.selectedEvaluationIndex ? " SELECTED" : " VALID_NOT_SELECTED") << '\n';
        else
            out << " result=REJECT reason_code=" << tripletReasonCode(e.rejectionReason)
                << " reason=\"" << tripletReasonText(e.rejectionReason) << "\"\n";
    }

    out << "summary:\n"
        << "  physical_finder_clusters=" << clusters.size() << '\n'
        << "  triplet_combinations_tested=" << result.evaluations.size() << '\n'
        << "  valid_triplets=" << acceptedCount << '\n'
        << "  rejected_triplets=" << result.evaluations.size() - acceptedCount << '\n';
    for (const auto reason : {TripletRejectionReason::InvalidGeometry,
                              TripletRejectionReason::ModuleSizeMismatch,
                              TripletRejectionReason::CandidatesTooClose,
                              TripletRejectionReason::NearlyCollinear,
                              TripletRejectionReason::PoorRightAngle,
                              TripletRejectionReason::SideLengthMismatch,
                              TripletRejectionReason::PythagoreanMismatch})
        out << "  rejected_" << tripletReasonCode(reason) << '=' << rejectionCounts[reason]
            << " (" << tripletReasonText(reason) << ")\n";
    if (result.hasSelection)
        out << "selected_triplet_evaluation=" << result.selectedEvaluationIndex
            << " selected_TL_TR_BL=" << result.selectedCandidateIndices[0] << ','
            << result.selectedCandidateIndices[1] << ',' << result.selectedCandidateIndices[2] << '\n';
    else
        out << "selected_triplet=NONE overall_failure_reason=\"" << result.overallFailureReason << "\"\n";
    out.close();
    if (!out) throw std::runtime_error("Cannot write finder triplet selection log");
}
