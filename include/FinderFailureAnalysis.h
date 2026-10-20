#ifndef QR_LEARNING_FINDER_FAILURE_ANALYSIS_H
#define QR_LEARNING_FINDER_FAILURE_ANALYSIS_H
#include "FinderValidation.h"

// Milestone 7: pure diagnostic/reporting layer built on top of Milestone 5's
// FinderValidationResult. This file adds no new detection or validation logic - it only
// classifies each existing FinderValidationRecord::rejectionReason string (and the accepted
// case) into a small set of human-readable outcome categories, and renders/reports the full
// per-candidate measurement trail that validateFinderHits() already computed. The goal is to
// see, for every raw Milestone-4 hit, exactly which pipeline stage it reached and why it was
// accepted or rejected there.
namespace FinderFailureAnalysisConstants
{
constexpr int MarkerHalfSize = 2;         // Candidate marker: filled square, this many pixels each side of center.
constexpr int LabelFontScale = 1;         // Font scale for the small candidate-ID digits drawn on the image itself.
constexpr int LegendFontScale = 2;        // Font scale for the larger legend text in the appended legend band.
constexpr int LegendPadding = 6;          // Pixels of margin around legend text/swatches.
constexpr int LegendSwatchSize = 10;      // Legend color-swatch square size, in pixels.
constexpr int LabelOffsetMagnitude = 10;  // Base horizontal distance candidate ID labels are placed from their marker.
}

// Classifies one validation record into one of a small fixed set of human-readable outcome
// buckets: "ACCEPTED", "INCOMPLETE_VERTICAL", "VERTICAL_RATIO_FAILURE",
// "SIZE_CONSISTENCY_FAILURE", "CENTER_CONSISTENCY_FAILURE", "HORIZONTAL_RECHECK_FAILURE", or
// "OTHER" (covers invalid_raw_hit and the diagnostic-only diagonal_sanity_failed). The mapping
// is derived directly from the exact rejectionReason strings produced by
// FinderValidation.cpp; only categories that actually occur should be shown in a legend.
std::string classifyFinderOutcome(const FinderValidationRecord& record);

// output/06_finder_failure_analysis.txt: one detailed, multi-line record per raw candidate,
// covering every measurement captured by Milestone 5 (horizontal hit, vertical scan,
// horizontal re-check, diagonal diagnostic) plus the exact rejection reason and its category.
void saveFinderFailureAnalysisText(const FinderValidationResult& result, const std::string& filename);

// output/06_horizontal_candidates.png: every raw Milestone-4 hit location (before any
// vertical/horizontal validation ran), each labeled with its candidate index.
void saveHorizontalCandidatesMap(const Image& image, const FinderValidationResult& result,
                                  const std::string& filename);

// output/06_vertical_failures.png: the vertical cross-check scan extent (visited pixels, even
// if incomplete) for every candidate whose vertical stage failed (incomplete scan, ratio
// failure, vertical/horizontal size mismatch, or vertical center displacement), colored by its
// exact failure reason. Candidates that passed the vertical stage are drawn as thin green
// reference lines for spatial context; candidates rejected at a later stage are omitted here.
void saveVerticalFailuresMap(const Image& image, const FinderValidationResult& result,
                              const std::string& filename);

// output/06_failure_reasons.png: every candidate's location, colored solely by outcome
// category with no labels - a quick spatial overview of where each failure type clusters.
void saveFailureReasonsMap(const Image& image, const FinderValidationResult& result,
                            const std::string& filename);

// output/06_all_finder_candidates_rejection_map.png (REQUIRED): every raw candidate, labeled
// with its index and a short leader line to reduce overlap, colored by outcome category, with
// a legend of every category that actually occurs appended as a white band below the image.
void saveAllCandidatesRejectionMap(const Image& image, const FinderValidationResult& result,
                                    const std::string& filename);
#endif
