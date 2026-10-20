#ifndef QR_LEARNING_MORPHOLOGICAL_CLOSING_H
#define QR_LEARNING_MORPHOLOGICAL_CLOSING_H
#include "Binarizer.h"

namespace MorphologicalClosingConstants
{
constexpr unsigned StructuringElementSize = 3;   // 3x3 full square structuring element.
constexpr unsigned StructuringElementRadius = 1; // 1 pixel on each side of the center.
}

// Experimental, optional pre-processing step: morphological closing (dilation followed
// by erosion) with a 3x3 full square structuring element, applied to a binary matrix
// (1=black/foreground, 0=white/background).
//
// Dilation grows black regions by one pixel in every direction (a pixel becomes black if
// any pixel in its 3x3 neighborhood is black). Erosion then shrinks black regions back by
// one pixel (a pixel stays black only if every pixel in its 3x3 neighborhood is black).
// Doing dilation then erosion (closing) fills small white gaps/holes inside or between
// black regions and bridges narrow one-pixel breaks, without growing the overall shape
// back out to where it started. On an image where the boundary is treated as background
// for both passes, closing is "extensive": it can only add black pixels relative to the
// input, never remove any that were already black.
//
// Motivation: local binarization can fragment a finder pattern's rings with stray white
// pixels (noise, compression artifacts, uneven lighting at the pattern edges), which
// breaks the horizontal/vertical run-length scans used by HorizontalFinder and
// FinderValidation. Closing may repair such small fragmentation before finder search runs.
//
// This is kept in its own translation unit, independent of every earlier milestone file,
// so it can be dropped from the pipeline (and this .h/.cpp pair deleted) without touching
// Binarizer, HorizontalFinder or FinderValidation at all.
BitMatrix dilateBitMatrix3x3(const BitMatrix& matrix);
BitMatrix erodeBitMatrix3x3(const BitMatrix& matrix);
BitMatrix closeBitMatrix3x3(const BitMatrix& matrix);

// Writes a text log comparing before/after pixel counts: pixels filled (white->black),
// pixels removed (black->white; should be zero, see the extensivity note above), and
// unchanged counts. A non-zero "removed" count would indicate a bug in the implementation.
void saveMorphologicalClosingDebug(const BitMatrix& before, const BitMatrix& after,
                                    const std::string& filename);
#endif
