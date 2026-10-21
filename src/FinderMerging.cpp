#include "FinderMerging.h"
#include "lodepng.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

bool findersShouldMerge(double x1, double y1, double moduleSize1,
                         double x2, double y2, double moduleSize2)
{
    using namespace FinderMergingConstants;
    const double maxModule = std::max(moduleSize1, moduleSize2);
    const double distance = std::hypot(x1 - x2, y1 - y2);
    if (distance > MergeCenterDistanceMultiplier * maxModule) return false;
    const double sizeDifference = std::abs(moduleSize1 - moduleSize2);
    if (sizeDifference > MergeModuleSizeRelativeTolerance * maxModule) return false;
    return true;
}

FinderMergingResult mergeFinderObservations(const std::vector<ValidatedFinderCandidate>& candidates)
{
    FinderMergingResult result;
    // Single greedy pass in input order: each observation joins the closest compatible
    // existing cluster (by current running center/module size), or seeds a new cluster.
    // This never re-examines earlier decisions, so the outcome only depends on input order,
    // which itself comes deterministically from FinderValidationResult::candidates.
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
        const auto& observation = candidates[i];
        std::size_t bestCluster = 0;
        double bestDistance = std::numeric_limits<double>::infinity();
        bool found = false;
        for (std::size_t k = 0; k < result.clusters.size(); ++k)
        {
            const auto& cluster = result.clusters[k];
            if (!findersShouldMerge(observation.centerX, observation.centerY, observation.estimatedModuleSize,
                                     cluster.centerX, cluster.centerY, cluster.estimatedModuleSize)) continue;
            const double distance = std::hypot(observation.centerX - cluster.centerX,
                                                observation.centerY - cluster.centerY);
            if (distance < bestDistance) { bestDistance = distance; bestCluster = k; found = true; }
        }
        if (found)
        {
            auto& cluster = result.clusters[bestCluster];
            const int n = cluster.observationCount;
            // Weighted average: existing running value counts n times, the new observation once.
            cluster.centerX = (cluster.centerX * n + observation.centerX) / (n + 1);
            cluster.centerY = (cluster.centerY * n + observation.centerY) / (n + 1);
            cluster.estimatedModuleSize = (cluster.estimatedModuleSize * n + observation.estimatedModuleSize) / (n + 1);
            cluster.observationCount = n + 1;
            cluster.minX = std::min(cluster.minX, observation.centerX);
            cluster.maxX = std::max(cluster.maxX, observation.centerX);
            cluster.minY = std::min(cluster.minY, observation.centerY);
            cluster.maxY = std::max(cluster.maxY, observation.centerY);
            cluster.observationIndices.push_back(i);
        }
        else
        {
            PhysicalFinderCluster cluster;
            cluster.centerX = observation.centerX;
            cluster.centerY = observation.centerY;
            cluster.estimatedModuleSize = observation.estimatedModuleSize;
            cluster.observationCount = 1;
            cluster.minX = cluster.maxX = observation.centerX;
            cluster.minY = cluster.maxY = observation.centerY;
            cluster.observationIndices.push_back(i);
            result.clusters.push_back(cluster);
        }
    }
    // Second pass: now that each cluster's final weighted center/module size is fixed,
    // compute how far its member observations actually spread from that final value.
    for (auto& cluster : result.clusters)
    {
        double maxCenterDistance = 0.0, maxModuleSizeRelativeDifference = 0.0;
        for (const auto index : cluster.observationIndices)
        {
            const auto& observation = candidates[index];
            maxCenterDistance = std::max(maxCenterDistance,
                std::hypot(observation.centerX - cluster.centerX, observation.centerY - cluster.centerY));
            maxModuleSizeRelativeDifference = std::max(maxModuleSizeRelativeDifference,
                std::abs(observation.estimatedModuleSize - cluster.estimatedModuleSize) / cluster.estimatedModuleSize);
        }
        cluster.centerSpread = maxCenterDistance;
        cluster.moduleSizeSpread = maxModuleSizeRelativeDifference;
    }
    return result;
}

namespace
{
void pixel(Image& image, long long x, long long y, unsigned char r, unsigned char g, unsigned char b)
{
    if (x < 0 || y < 0 || x >= image.width || y >= image.height) return;
    const auto i = (static_cast<std::size_t>(y) * image.width + x) * 4;
    image.rgba[i] = r; image.rgba[i + 1] = g; image.rgba[i + 2] = b; image.rgba[i + 3] = 255;
}
// Small cross: marks a raw validated observation (same style as Milestone 5's debug PNGs).
void cross(Image& image, double x, double y, unsigned char r, unsigned char g, unsigned char b)
{
    if (!std::isfinite(x) || !std::isfinite(y)) return;
    const auto cx = static_cast<long long>(std::floor(x)), cy = static_cast<long long>(std::floor(y));
    for (int d = -FinderValidationConstants::MarkerRadius; d <= FinderValidationConstants::MarkerRadius; ++d)
    { pixel(image, cx + d, cy, r, g, b); pixel(image, cx, cy + d, r, g, b); }
}
// Square outline: marks a merged physical finder cluster, visually distinct from the
// small crosses used for the raw per-row observations that were merged into it.
void box(Image& image, double x, double y, unsigned char r, unsigned char g, unsigned char b)
{
    if (!std::isfinite(x) || !std::isfinite(y)) return;
    const auto cx = static_cast<long long>(std::floor(x)), cy = static_cast<long long>(std::floor(y));
    const int radius = FinderMergingConstants::MarkerRadius;
    for (int d = -radius; d <= radius; ++d)
    {
        pixel(image, cx + d, cy - radius, r, g, b); pixel(image, cx + d, cy + radius, r, g, b);
        pixel(image, cx - radius, cy + d, r, g, b); pixel(image, cx + radius, cy + d, r, g, b);
    }
}
void save(const Image& image, const char* filename)
{
    const unsigned error = lodepng::encode(filename, image.rgba, image.width, image.height, LCT_RGBA, 8);
    if (error) throw std::runtime_error("Merging PNG writing failed: " + std::string(lodepng_error_text(error)));
}
}

// Routing is the only mode-dependent behavior here; the merge result is unchanged.
void saveFinderMergingDebug(const Image& image, const std::vector<ValidatedFinderCandidate>& candidates,
                             const FinderMergingResult& merging, const std::string& outputDirectory)
{
    if (!image.width || !image.height ||
        static_cast<std::size_t>(image.width) > std::numeric_limits<std::size_t>::max() / image.height ||
        image.pixelCount() > std::numeric_limits<std::size_t>::max() / 4 || image.rgba.size() != image.pixelCount() * 4)
        throw std::invalid_argument("Invalid image for finder merging debug");

    Image debug = image;
    // Draw raw observations first (small blue crosses), then cluster centers on top
    // (larger magenta boxes), so both are visible even where they nearly coincide.
    for (const auto& observation : candidates) cross(debug, observation.centerX, observation.centerY, 0, 128, 255);
    for (const auto& cluster : merging.clusters) box(debug, cluster.centerX, cluster.centerY, 255, 0, 255);
    save(debug, (outputDirectory + "/05_merged_finders.png").c_str());

    std::ofstream out(outputDirectory + "/05_finder_clusters.txt");
    if (!out) throw std::runtime_error("Cannot open finder clusters log");
    using namespace FinderMergingConstants;
    out << "Milestone 6: merge repeated Milestone-5 observations into physical finder candidates\n"
        << "merge_center_distance_multiplier=" << MergeCenterDistanceMultiplier
        << " merge_module_size_relative_tolerance=" << MergeModuleSizeRelativeTolerance << '\n'
        << "PNG: blue crosses=raw validated observations, magenta boxes=merged physical finder clusters\n"
        << "validated_observations=" << candidates.size() << " physical_finder_clusters=" << merging.clusters.size() << '\n'
        << std::fixed << std::setprecision(6);
    for (std::size_t k = 0; k < merging.clusters.size(); ++k)
    {
        const auto& cluster = merging.clusters[k];
        out << "cluster_" << k << ": centerX=" << cluster.centerX << " centerY=" << cluster.centerY
            << " moduleSize=" << cluster.estimatedModuleSize << " observationCount=" << cluster.observationCount
            << " minX=" << cluster.minX << " maxX=" << cluster.maxX
            << " minY=" << cluster.minY << " maxY=" << cluster.maxY
            << " centerSpread=" << cluster.centerSpread << " moduleSizeSpread=" << cluster.moduleSizeSpread << '\n';
        for (const auto index : cluster.observationIndices)
        {
            const auto& observation = candidates[index];
            out << "  observation_" << index << ": centerX=" << observation.centerX
                << " centerY=" << observation.centerY << " moduleSize=" << observation.estimatedModuleSize << '\n';
        }
    }
    out.close();
    if (!out) throw std::runtime_error("Cannot write finder clusters log");
}



