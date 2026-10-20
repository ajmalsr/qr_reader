#ifndef QR_LEARNING_FINDER_TRIPLET_SELECTION_H
#define QR_LEARNING_FINDER_TRIPLET_SELECTION_H

#include "FinderMerging.h"

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

// Finder triplet selection receives merged physical finder candidates. It does not
// estimate the QR dimension or sample modules; it only asks whether three centers and
// their measured module sizes form plausible top-left/top-right/bottom-left finders.
namespace FinderTripletConstants
{
constexpr double ModuleSizeRelativeTolerance = 0.50; // (largest-smallest)/largest.
constexpr double MinimumCenterDistanceModules = 7.0; // Finder centers must be at least
                                                      // one 7-module finder width apart.
constexpr double CollinearitySineTolerance = 0.15;   // |cross|/(leg1*leg2); below this
                                                      // means the three points are near a line.
constexpr double RightAngleMaximumCosine = 0.35;     // |dot|/(leg1*leg2), about 69.5-110.5 deg.
constexpr double SideLengthRelativeTolerance = 0.50; // |leg1-leg2|/max(leg1,leg2).
constexpr double PythagoreanRelativeTolerance = 0.25;// |diagonal-hypot(legs)| / larger value.
constexpr int CandidateMarkerRadius = 5;
}

enum class TripletRejectionReason
{
    None,
    InvalidGeometry,
    ModuleSizeMismatch,
    CandidatesTooClose,
    NearlyCollinear,
    PoorRightAngle,
    SideLengthMismatch,
    PythagoreanMismatch
};

const char* tripletReasonCode(TripletRejectionReason reason);
const char* tripletReasonText(TripletRejectionReason reason);

// Complete measurements for one n-choose-3 combination. The input IDs preserve the
// cluster order; the ordered IDs identify the right-angle vertex and its two arms.
struct FinderTripletEvaluation
{
    std::array<std::size_t, 3> candidateIndices{};
    std::size_t topLeftIndex = 0;
    std::size_t topRightIndex = 0;
    std::size_t bottomLeftIndex = 0;

    double averageModuleSize = 0.0;
    double moduleSizeError = std::numeric_limits<double>::infinity();
    double minimumCenterDistanceModules = 0.0;
    double topSideLength = 0.0;
    double leftSideLength = 0.0;
    double diagonalLength = 0.0;
    double collinearitySine = 0.0;
    double rightAngleError = std::numeric_limits<double>::infinity();
    double sideLengthError = std::numeric_limits<double>::infinity();
    double pythagoreanError = std::numeric_limits<double>::infinity();
    double score = std::numeric_limits<double>::infinity();

    bool accepted = false;
    TripletRejectionReason rejectionReason = TripletRejectionReason::InvalidGeometry;
};

struct FinderTripletSelectionResult
{
    std::vector<FinderTripletEvaluation> evaluations;
    bool hasSelection = false;
    std::size_t selectedEvaluationIndex = std::numeric_limits<std::size_t>::max();
    std::array<std::size_t, 3> selectedCandidateIndices{}; // TL, TR, BL.
    std::string overallFailureReason;
};

// Tests every cluster combination, records an explicit reason for each rejection, and
// chooses the accepted combination with the lowest normalized error score.
FinderTripletSelectionResult selectFinderTriplet(
    const std::vector<PhysicalFinderCluster>& clusters);

// Writes one image containing candidate IDs and every tested triangle:
// red + reason code = rejected, yellow = valid alternative, green = selected.
// The TXT file expands every code and records the measurements and summary counts.
void saveFinderTripletSelectionDebug(
    const Image& image,
    const std::vector<PhysicalFinderCluster>& clusters,
    const FinderTripletSelectionResult& result,
    const std::string& outputDirectory = "output");

#endif
