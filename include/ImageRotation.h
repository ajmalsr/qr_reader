#ifndef QR_LEARNING_IMAGE_ROTATION_H
#define QR_LEARNING_IMAGE_ROTATION_H

#include "Image.h"

// Rotate an RGBA image around its center. Positive angles are clockwise in
// image coordinates; negative angles are anticlockwise. The returned image
// uses an expanded, opaque-white canvas so the source is not cropped.
Image rotateImage(const Image& source, double clockwiseDegrees);

#endif
