#include "Clahe.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace {
// One histogram and one intensity lookup table are built for every tile.
using Hist=std::array<std::uint64_t,ClaheConstants::HistogramBins>;
using Lut=std::array<unsigned char,ClaheConstants::HistogramBins>;
// Reject malformed images before indexing their row-major pixel storage.
void validate(const GrayImage& im)
{
    if(!im.width||!im.height||static_cast<std::size_t>(im.width)>std::numeric_limits<std::size_t>::max()/im.height||im.pixels.size()!=im.pixelCount())
        throw std::invalid_argument("Invalid grayscale image for CLAHE");
}
// Find the two tile centers surrounding a pixel coordinate. At an image edge,
// both indices become the nearest tile, which naturally disables interpolation.
std::pair<unsigned,unsigned> neighbors(unsigned p,const std::vector<double>& c)
{
    auto it=std::upper_bound(c.begin(),c.end(),p+0.5);
    if(it==c.begin())return {0,0};
    if(it==c.end()){unsigned n=static_cast<unsigned>(c.size()-1);return {n,n};}
    unsigned hi=static_cast<unsigned>(it-c.begin());return {hi-1,hi};
}
// Fractional distance between neighboring tile centers, clamped at the borders.
double weight(unsigned p,const std::vector<double>& c,unsigned lo,unsigned hi)
{return lo==hi?0.0:std::clamp(((p+0.5)-c[lo])/(c[hi]-c[lo]),0.0,1.0);}
}

GrayImage applyClahe(const GrayImage& input,const ClaheSettings& settings)
{
    validate(input);
    if(!settings.tileColumns||!settings.tileRows||!std::isfinite(settings.clipLimit)||settings.clipLimit<=0)
        throw std::invalid_argument("Invalid CLAHE settings");
    // Images smaller than the requested grid use at most one tile per pixel.
    const unsigned nx=std::min(settings.tileColumns,input.width),ny=std::min(settings.tileRows,input.height);
    // Integer edges cover every source pixel once; centers are used later for
    // smooth interpolation between the independently equalized tile mappings.
    std::vector<unsigned> xe(nx+1),ye(ny+1);std::vector<double> xc(nx),yc(ny);
    for(unsigned i=0;i<=nx;++i)xe[i]=static_cast<unsigned>(static_cast<std::uint64_t>(i)*input.width/nx);
    for(unsigned i=0;i<=ny;++i)ye[i]=static_cast<unsigned>(static_cast<std::uint64_t>(i)*input.height/ny);
    for(unsigned i=0;i<nx;++i)xc[i]=(xe[i]+xe[i+1])/2.0;
    for(unsigned i=0;i<ny;++i)yc[i]=(ye[i]+ye[i+1])/2.0;
    // First pass: create a contrast-limited equalization LUT for every tile.
    std::vector<Lut> luts(static_cast<std::size_t>(nx)*ny);
    for(unsigned ty=0;ty<ny;++ty)for(unsigned tx=0;tx<nx;++tx){
        // Count the tile's 8-bit luminance values into 256 bins.
        Hist h{};
        for(unsigned y=ye[ty];y<ye[ty+1];++y)for(unsigned x=xe[tx];x<xe[tx+1];++x)++h[input.pixels[static_cast<std::size_t>(y)*input.width+x]];
        const std::uint64_t pixels=static_cast<std::uint64_t>(xe[tx+1]-xe[tx])*(ye[ty+1]-ye[ty]);
        // Convert the normalized clip limit to an absolute per-bin count.
        // A value of 1.0 equals the average bin population for this tile.
        const std::uint64_t limit=std::max<std::uint64_t>(1,static_cast<std::uint64_t>(std::floor(settings.clipLimit*pixels/ClaheConstants::HistogramBins)));
        // Clip tall bins and remember all removed samples so the histogram's
        // total population can be restored without losing pixels.
        std::uint64_t excess=0;for(auto& n:h)if(n>limit){excess+=n-limit;n=limit;}
        // Redistribute the excess uniformly. Remainder samples are spread across
        // the full 0..255 range instead of being concentrated in dark bins.
        const auto each=excess/ClaheConstants::HistogramBins;const unsigned remainder=static_cast<unsigned>(excess%ClaheConstants::HistogramBins);
        for(auto& n:h)n+=each;
        for(unsigned i=0;i<remainder;++i)++h[static_cast<unsigned>(static_cast<std::uint64_t>(i)*ClaheConstants::HistogramBins/remainder)];
        // The cumulative distribution maps each input value to an enhanced value.
        std::uint64_t sum=0;auto& lut=luts[static_cast<std::size_t>(ty)*nx+tx];
        for(unsigned v=0;v<ClaheConstants::HistogramBins;++v){sum+=h[v];lut[v]=static_cast<unsigned char>(std::min<std::uint64_t>(255,(sum*255+pixels/2)/pixels));}
    }
    // Second pass: interpolate the four neighboring tile LUT results for every
    // pixel. This removes the hard seams produced by independent tile mapping.
    GrayImage out;out.width=input.width;out.height=input.height;out.pixels.resize(input.pixels.size());
    for(unsigned y=0;y<input.height;++y){auto [y0,y1]=neighbors(y,yc);double wy=weight(y,yc,y0,y1);
        for(unsigned x=0;x<input.width;++x){auto [x0,x1]=neighbors(x,xc);double wx=weight(x,xc,x0,x1);unsigned v=input.pixels[static_cast<std::size_t>(y)*input.width+x];
            auto at=[&](unsigned tx,unsigned ty){return double(luts[static_cast<std::size_t>(ty)*nx+tx][v]);};
            double top=at(x0,y0)*(1-wx)+at(x1,y0)*wx,bottom=at(x0,y1)*(1-wx)+at(x1,y1)*wx;
            out.pixels[static_cast<std::size_t>(y)*input.width+x]=static_cast<unsigned char>(std::clamp(std::lround(top*(1-wy)+bottom*wy),0L,255L));}}
    return out;
}

void saveClaheStatistics(const GrayImage& before,const GrayImage& after,const ClaheSettings& settings,const std::string& filename)
{
    validate(before);validate(after);if(before.width!=after.width||before.height!=after.height)throw std::invalid_argument("Mismatched CLAHE images");
    // Compact summaries make different CLAHE experiments easy to compare without
    // storing another full histogram in the report.
    auto summary=[](const GrayImage& im){long double sum=0;unsigned lo=255,hi=0;for(auto v:im.pixels){lo=std::min(lo,unsigned(v));hi=std::max(hi,unsigned(v));sum+=v;}return std::array<long double,3>{static_cast<long double>(lo),static_cast<long double>(hi),sum/im.pixels.size()};};
    auto a=summary(before),b=summary(after);std::ofstream out(filename);if(!out)throw std::runtime_error("Cannot open CLAHE statistics: "+filename);
    out<<"CLAHE implemented locally; no OpenCV\n"<<"tile_columns="<<std::min(settings.tileColumns,before.width)<<" tile_rows="<<std::min(settings.tileRows,before.height)<<" histogram_bins="<<ClaheConstants::HistogramBins<<" normalized_clip_limit="<<settings.clipLimit<<'\n'
       <<"clip_count=max(1,floor(clip_limit*tile_pixels/256))\nclipped counts redistributed uniformly; tile LUTs bilinearly interpolated\n"<<std::fixed<<std::setprecision(3)
       <<"input_min="<<a[0]<<" input_max="<<a[1]<<" input_mean="<<a[2]<<'\n'<<"output_min="<<b[0]<<" output_max="<<b[1]<<" output_mean="<<b[2]<<'\n';
    out.close();if(!out)throw std::runtime_error("Cannot write CLAHE statistics: "+filename);
}


