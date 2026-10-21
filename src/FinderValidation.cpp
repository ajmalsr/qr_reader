#include "FinderValidation.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
void validateMatrix(const BitMatrix& m)
{
    if (!m.width || !m.height || m.width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        m.height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        static_cast<std::size_t>(m.width) > std::numeric_limits<std::size_t>::max()/m.height ||
        m.bits.size() != static_cast<std::size_t>(m.width)*m.height)
        throw std::invalid_argument("Invalid matrix for finder validation");
    for (auto b : m.bits) if (b > 1) throw std::invalid_argument("Non-binary matrix value");
}
// Walks outward from seed pixel (x,y) in both directions along (dx,dy), counting the
// black center run, then the white run, then the outer black run on each side, to
// reconstruct the full 5-run 1:1:3:1:1 profile centered near the seed. This is how the
// vertical cross-check, horizontal re-check and diagonal sanity check are all implemented -
// only the (dx,dy) direction differs.
DirectionalFinderScan scan(const BitMatrix& m, int x, int y, int dx, int dy, int limit)
{
    DirectionalFinderScan r;
    r.minimumX = r.maximumX = x; r.minimumY = r.maximumY = y;
    const auto inside = [&](long long px, long long py) {
        return px >= 0 && py >= 0 && px < m.width && py < m.height;
    };
    const auto black = [&](long long px, long long py) {
        return m.bits[static_cast<std::size_t>(py)*m.width + static_cast<std::size_t>(px)] == 1;
    };
    if (!inside(x,y) || !black(x,y)) { r.reason = "seed_not_black_or_outside"; return r; }
    r.runs[2] = 1; // Count seed once; both legs start at the next pixel.
    int centralNegative = 0, centralPositive = 0;
    // Walk both directions (negative then positive along dx,dy) from the seed. Each leg
    // passes through 3 stages: extend the center black run, then the white run, then the
    // outer black run, recording their lengths into runs[2]/runs[1 or 3]/runs[0 or 4].
    for (int sign : {-1,1})
    {
        long long px = static_cast<long long>(x)+sign*dx;
        long long py = static_cast<long long>(y)+sign*dy;
        int steps = 0;
        const int whiteIndex = sign < 0 ? 1 : 3;
        const int outerIndex = sign < 0 ? 0 : 4;
        const std::array<int,3> indices{2,whiteIndex,outerIndex};
        for (int stage = 0; stage < 3; ++stage)
        {
            const bool expectedBlack = stage != 1;
            const int before = r.runs[indices[stage]];
            while (inside(px,py) && black(px,py) == expectedBlack)
            {
                if (steps >= limit) { r.reason = "scan_leg_limit"; return r; }
                ++r.runs[indices[stage]]; ++steps;
                if (stage == 0) (sign < 0 ? centralNegative : centralPositive)++;
                r.minimumX = std::min(r.minimumX,static_cast<int>(px));
                r.maximumX = std::max(r.maximumX,static_cast<int>(px));
                r.minimumY = std::min(r.minimumY,static_cast<int>(py));
                r.maximumY = std::max(r.maximumY,static_cast<int>(py));
                px += sign*dx; py += sign*dy;
            }
            // Central leg may be empty when seed lies at its edge; other runs cannot.
            if (stage != 0 && r.runs[indices[stage]] == before)
            { r.reason = "missing_white_or_outer_black_run"; return r; }
            if (!inside(px,py)) { r.reason = "image_boundary_before_run_completion"; return r; }
        }
    }
    // If the seed wasn't exactly centered in the black center run, the two legs of the
    // center run will have different lengths; shift the reported center by half that
    // difference (in whole pixels, along the scan direction) to refine it.
    const double shift = (centralPositive-centralNegative)/2.0;
    r.centerX = x+0.5+shift*dx; r.centerY = y+0.5+shift*dy;
    r.complete = true;
    return r;
}
bool diagonal(const BitMatrix& m, double x, double y, int limit)
{
    // A loose sanity check: at least one diagonal has all five runs and reasonable proportions.
    for (int dy : {-1,1})
    {
        const auto s = scan(m,static_cast<int>(std::floor(x)),static_cast<int>(std::floor(y)),1,dy,limit);
        if (!s.complete) continue;
        const auto ratio = checkFinderRatio(s.runs);
        if (ratio.totalWidth >= HorizontalFinderConstants::TotalModules && s.runs[2] >= 3 &&
            ratio.maximumRelativeError < FinderValidationConstants::DiagonalRelativeTolerance) return true;
    }
    return false;
}
// Two total-widths (in pixels) are consistent if they differ by no more than
// SizeRelativeTolerance of the reference width; used to confirm the horizontal and
// vertical scans through a candidate agree on how large the finder pattern actually is.
bool sizesAgree(int measured, int reference)
{
    return std::abs(static_cast<double>(measured)-reference) <=
        FinderValidationConstants::SizeRelativeTolerance*reference;
}
}
DirectionalFinderScan scanFinderDirection(const BitMatrix& m, int x, int y, int dx, int dy, int limit)
{
    validateMatrix(m);
    if ((dx != 0 && dx != 1) || (dy < -1 || dy > 1) || (dx == 0 && dy != 1) || limit <= 0)
        throw std::invalid_argument("Invalid scan direction or limit");
    return scan(m,x,y,dx,dy,limit);
}
bool finderDiagonalSanity(const BitMatrix& m, double x, double y, int limit)
{
    validateMatrix(m);
    if (!std::isfinite(x) || !std::isfinite(y) || x<0 || y<0 || x>=m.width || y>=m.height || limit<=0)
        return false;
    return diagonal(m,x,y,limit);
}
FinderValidationResult validateFinderHits(const BitMatrix& m,
    const std::vector<HorizontalFinderHit>& hits, bool requireDiagonal)
{
    validateMatrix(m);
    FinderValidationResult result; result.diagonalRequired = requireDiagonal;
    for (const auto& hit : hits)
    {
        FinderValidationRecord r; r.original = hit;
        // Public API expects genuine Milestone 4 hits; reject malformed inputs explicitly.
        const auto originalRatio = checkFinderRatio(hit.runLengths);
        if (!std::isfinite(hit.centerX) || !std::isfinite(hit.centerY) || hit.centerX<0 ||
            hit.centerX>=m.width || hit.row<0 || static_cast<unsigned>(hit.row)>=m.height ||
            !originalRatio.accepted || hit.totalWidth != originalRatio.totalWidth ||
            !std::isfinite(hit.estimatedModuleSize) || hit.estimatedModuleSize<=0 ||
            hit.centerY != hit.row+0.5)
        { r.rejectionReason="invalid_raw_hit"; result.records.push_back(r); continue; }
        const int limit = static_cast<int>(std::min<double>(std::numeric_limits<int>::max(),
            std::ceil(hit.totalWidth*FinderValidationConstants::ScanLegWidthMultiplier)));
        ++result.verticalScansAttempted;
        // Step 1: vertical cross-check. Scan straight up/down through the raw hit's
        // (unrefined) center column to confirm a genuine 1:1:3:1:1 pattern exists there too,
        // not just along the originally scanned row.
        r.vertical = scan(m,static_cast<int>(std::floor(hit.centerX)),hit.row,0,1,limit);
        if (!r.vertical.complete)
        { ++result.incompleteVerticalScans; r.rejectionReason="vertical_incomplete:"+r.vertical.reason; }
        else
        {
            r.verticalRatio = checkFinderRatio(r.vertical.runs);
            r.centerDisplacementY = std::abs(r.vertical.centerY-hit.centerY);
            if (!r.verticalRatio.accepted || !finderFastPlausibility(r.vertical.runs))
            { ++result.verticalRatioFailures; r.rejectionReason="vertical_ratio_invalid"; }
            else if (!sizesAgree(r.verticalRatio.totalWidth,hit.totalWidth))
            { ++result.sizeConsistencyFailures; r.rejectionReason="vertical_horizontal_size_mismatch"; }
            else if (r.centerDisplacementY > FinderValidationConstants::VerticalCenterModules*hit.estimatedModuleSize)
            { ++result.centerConsistencyFailures; r.rejectionReason="vertical_center_displacement"; }
            else
            {
                r.verticalPassed=true; ++result.verticalPasses;
                // Step 2: horizontal re-check. Re-scan horizontally, but now through the
                // vertically-refined row, to get a horizontal center/ratio consistent with
                // the true (refined) row rather than the originally scanned one.
                r.horizontal = scan(m,static_cast<int>(std::floor(hit.centerX)),
                    static_cast<int>(std::floor(r.vertical.centerY)),1,0,limit);
                if (!r.horizontal.complete)
                { ++result.horizontalRecheckFailures; r.rejectionReason="horizontal_incomplete:"+r.horizontal.reason; }
                else
                {
                    r.horizontalRatio = checkFinderRatio(r.horizontal.runs);
                    r.centerDisplacementX=std::abs(r.horizontal.centerX-hit.centerX);
                    if (!r.horizontalRatio.accepted || !finderFastPlausibility(r.horizontal.runs))
                    { ++result.horizontalRecheckFailures; r.rejectionReason="horizontal_recheck_ratio_invalid"; }
                    else if (!sizesAgree(r.horizontalRatio.totalWidth,hit.totalWidth) ||
                             !sizesAgree(r.verticalRatio.totalWidth,r.horizontalRatio.totalWidth))
                    { ++result.horizontalRecheckFailures; r.rejectionReason="horizontal_recheck_size_mismatch"; }
                    else if (r.centerDisplacementX > FinderValidationConstants::HorizontalCenterModules*hit.estimatedModuleSize)
                    { ++result.centerConsistencyFailures; r.rejectionReason="horizontal_center_displacement"; }
                    else
                    {
                        // Step 3 (diagnostic): diagonal sanity check. Not required for
                        // acceptance unless requireDiagonal was requested by the caller.
                        ++result.axisPasses; ++result.diagonalAttempts; r.diagonalAttempted=true;
                        r.diagonalPassed=diagonal(m,r.horizontal.centerX,r.vertical.centerY,limit);
                        if (r.diagonalPassed) ++result.diagonalPasses; else ++result.diagonalFailures;
                        if (requireDiagonal && !r.diagonalPassed) r.rejectionReason="diagonal_sanity_failed";
                        else
                        {
                            r.accepted=true;
                            result.candidates.push_back({r.horizontal.centerX,r.vertical.centerY,
                                r.horizontalRatio.moduleSize,r.verticalRatio.moduleSize,
                                (r.horizontalRatio.moduleSize+r.verticalRatio.moduleSize)/2.0,
                                r.horizontal.runs,r.vertical.runs,r.horizontalRatio.maximumRelativeError,
                                r.verticalRatio.maximumRelativeError,r.diagonalPassed});
                        }
                    }
                }
            }
        }
        result.records.push_back(r);
    }
    return result;
}
