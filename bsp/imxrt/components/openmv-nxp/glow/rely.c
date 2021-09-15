/* This file is part of the OpenMV project.
 * Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io> & Kwabena W. Agyeman <kwagyeman@openmv.io>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 */
#include "math.h"
#include "stdint.h"
// The Glow bundle when using the Softmax layer has the "expf" function (symbol)
// unresolved. Since not all the toolchains provide an implementation we define
// the function here and use the standard "exp" implementation.
float expf(float f) {
  return exp(f);
}