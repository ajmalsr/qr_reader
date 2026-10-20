#ifndef QR_LEARNING_FINDER_MERGING_H
#define QR_LEARNING_FINDER_MERGING_H
#include "FinderValidation.h"

// Milestone 5 validates each raw Milestone-4 hit independently, so a single physical
// finder pattern (which spans several scanned rows) typically produces multiple
// ValidatedFinderCandidate observations that are all nearly the same point. Milestone 6
// groups those repeated observations into one PhysicalFinderCluster per physical finder,
// without touching anything from Milestones 1-5.
namespace FinderMergingConstants
{
constexpr double MergeCenterDistanceMultiplier = 1.5; // Two observations merge only if their
                                                       // center distance is <= this many times
                                                       // the larger of their two module sizes.
constexpr double MergeModuleSizeRelativeTolerance = 0.5; // ...and their module sizes differ by
                                                          // no more than this fraction of the
                                                          // larger module size.
constexpr int MarkerRadius = 5; // Debug PNG: cluster marker size (bigger than the raw-hit marker).
}

// One physical finder candidate: the merged result of one or more ValidatedFinderCandidate
// observations believed to be repeated measurements of the same physical finder pattern.
struct PhysicalFinderCluster
{
    double centerX = 0.0, centerY = 0.0;       // Weighted-average center of all observations.
    double estimatedModuleSize = 0.0;          // Weighted-average module size of all observations.
    int observationCount = 0;                  // Number of observations merged into this cluster.
    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0; // Bounding box of observation centers.
    double centerSpread = 0.0;                 // Max distance from final center to any observation center.
    double moduleSizeSpread = 0.0;             // Max relative module-size difference from the final estimate.
    std::vector<std::size_t> observationIndices; // Indices into the candidates vector that was merged.
};

struct FinderMergingResult
{
    std::vector<PhysicalFinderCluster> clusters;
};

// True if two observations (given by center and module size) are close enough in both
// position and scale to be considered repeated measurements of the same physical finder.
bool findersShouldMerge(double x1, double y1, double moduleSize1,
                         double x2, double y2, double moduleSize2);

// Greedily clusters validated candidates into physical finders. Processes candidates in
// input order; each candidate joins the closest existing cluster it is compatible with
// (by findersShouldMerge against the cluster's current running center/module size), or
// starts a new cluster if none match. Deterministic: given the same input order, the same
// clustering results every time. Does not reorder or otherwise use anything beyond what is
// in FinderValidationResult::candidates.
FinderMergingResult mergeFinderObservations(const std::vector<ValidatedFinderCandidate>& candidates);

// Writes output/05_merged_finders.png (raw validated observations as small crosses, merged
// physical finder clusters as larger circles) and output/05_finder_clusters.txt (full
// per-cluster breakdown: member observations, final center/module size, spreads).
// The directory parameter keeps CLAHE debug files separate; clustering is identical
// in both modes. The default preserves existing callers and tests.
void saveFinderMergingDebug(const Image& image, const std::vector<ValidatedFinderCandidate>& candidates,
                             const FinderMergingResult& merging,
                             const std::string& outputDirectory = "output");
#endif


