#include "FinderValidation.h"
#include "lodepng.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace
{
void pixel(Image& image, long long x, long long y, bool passed)
{
    if (x<0 || y<0 || x>=image.width || y>=image.height) return;
    const auto i=(static_cast<std::size_t>(y)*image.width+x)*4;
    image.rgba[i]=passed?0:255; image.rgba[i+1]=passed?255:0;
    image.rgba[i+2]=0; image.rgba[i+3]=255;
}
void marker(Image& image, double x, double y, bool passed)
{
    if (!std::isfinite(x) || !std::isfinite(y)) return;
    const auto cx=static_cast<long long>(std::floor(x)), cy=static_cast<long long>(std::floor(y));
    for (int d=-FinderValidationConstants::MarkerRadius; d<=FinderValidationConstants::MarkerRadius; ++d)
    { pixel(image,cx+d,cy,passed); pixel(image,cx,cy+d,passed); }
}
void save(const Image& image, const char* filename)
{
    const unsigned error=lodepng::encode(filename,image.rgba,image.width,image.height,LCT_RGBA,8);
    if (error) throw std::runtime_error("Validation PNG writing failed: "+std::string(lodepng_error_text(error)));
}
void runs(std::ostream& out, const std::array<int,5>& values)
{ for (auto value:values) out<<' '<<value; }
}
// Writes two debug PNGs (vertical cross-check visualization, and final accepted candidates)
// plus a detailed per-hit text log covering every stage of the Milestone 5 validation chain,
// so a rejected or accepted hit's full reasoning trail can be inspected.
// All three files use the caller-selected directory so CLAHE mode never writes
// into the baseline output directory.
void saveFinderValidationDebug(const Image& image, const FinderValidationResult& result,
                               const std::string& outputDirectory)
{
    if (!image.width || !image.height ||
        static_cast<std::size_t>(image.width)>std::numeric_limits<std::size_t>::max()/image.height ||
        image.pixelCount()>std::numeric_limits<std::size_t>::max()/4 || image.rgba.size()!=image.pixelCount()*4)
        throw std::invalid_argument("Invalid image for finder validation debug");
    Image vertical=image, final=image;
    for (const auto& r:result.records)
    {
        // Show actual visited vertical pixels, including partially completed scans.
        if (std::isfinite(r.original.centerX))
        {
            const auto x=static_cast<long long>(std::floor(r.original.centerX));
            for (long long y=r.vertical.minimumY; y<=r.vertical.maximumY; ++y)
                pixel(vertical,x,y,r.verticalPassed);
        }
        marker(vertical,r.original.centerX,r.original.centerY,r.verticalPassed);
    }
    for (const auto& c:result.candidates) marker(final,c.centerX,c.centerY,true);
    save(vertical,(outputDirectory+"/04_vertical_crosscheck.png").c_str());
    save(final,(outputDirectory+"/04_validated_finder_hits.png").c_str());
    std::ofstream out(outputDirectory+"/04_finder_validation.txt");
    if (!out) throw std::runtime_error("Cannot open finder validation log");
    using namespace FinderValidationConstants;
    out<<"Unmerged candidates; no geometry or decoding\n"
       <<"ratio_tolerance_strict="<<HorizontalFinderConstants::RelativeTolerance
       <<" size_difference_inclusive="<<SizeRelativeTolerance<<" (relative to reference width)\n"
       <<"max_vertical_center_modules="<<VerticalCenterModules<<" max_horizontal_center_modules="<<HorizontalCenterModules
       <<" scan_leg_width_multiplier="<<ScanLegWidthMultiplier<<'\n'
       <<"outer black runs require terminating white pixels; nan=measurement unavailable\n"
       <<"diagonal_tolerance_strict="<<DiagonalRelativeTolerance<<" at_least_one_of_two_diagonals; required="
       <<result.diagonalRequired<<'\n'
       <<"estimated module size is mean of the two axes of THIS candidate only\n"
       <<"vertical PNG: green=vertical accepted, red=vertical rejected; lines=visited pixels, crosses=raw hits\n"
       <<"raw_horizontal_hits="<<result.records.size()<<" vertical_scans_attempted="<<result.verticalScansAttempted<<'\n'
       <<"incomplete_vertical_scans="<<result.incompleteVerticalScans<<" vertical_ratio_failures="<<result.verticalRatioFailures<<'\n'
       <<"size_consistency_failures="<<result.sizeConsistencyFailures<<" center_consistency_failures="<<result.centerConsistencyFailures<<'\n'
       <<"horizontal_recheck_failures="<<result.horizontalRecheckFailures<<" diagonal_failures="<<result.diagonalFailures<<'\n'
       <<"vertical_passes="<<result.verticalPasses<<" axis_passes="<<result.axisPasses
       <<" diagonal_attempts="<<result.diagonalAttempts<<" diagonal_passes="<<result.diagonalPasses<<'\n'
       <<"final_validated_candidates="<<result.candidates.size()<<'\n'
       <<"row originalX originalY original_runs[5] vertical_runs[5] refined_horizontal_runs[5] refinedX refinedY horizontalModule verticalModule displacementX displacementY originalRatioError horizontalRatioError verticalRatioError diagonal status reason\n"
       <<std::fixed<<std::setprecision(6);
    const auto nan=std::numeric_limits<double>::quiet_NaN();
    for (const auto& r:result.records)
    {
        out<<r.original.row<<' '<<r.original.centerX<<' '<<r.original.centerY;
        runs(out,r.original.runLengths); runs(out,r.vertical.runs); runs(out,r.horizontal.runs);
        out<<' '<<r.horizontal.centerX<<' '<<r.vertical.centerY
           <<' '<<(r.horizontalRatio.totalWidth?r.horizontalRatio.moduleSize:nan)
           <<' '<<(r.verticalRatio.totalWidth?r.verticalRatio.moduleSize:nan)
           <<' '<<r.centerDisplacementX<<' '<<r.centerDisplacementY<<' '<<r.original.maximumRelativeError
           <<' '<<(r.horizontalRatio.totalWidth?r.horizontalRatio.maximumRelativeError:nan)
           <<' '<<(r.verticalRatio.totalWidth?r.verticalRatio.maximumRelativeError:nan)
           <<' '<<(r.diagonalAttempted?(r.diagonalPassed?"PASS":"FAIL"):"NOT_RUN")
           <<' '<<(r.accepted?"ACCEPT":"REJECT")<<' '<<(r.accepted?"none":r.rejectionReason)<<'\n';
    }
    out.close(); if (!out) throw std::runtime_error("Cannot write finder validation log");
}


