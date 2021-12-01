#include "math.h"
#include "stdint.h"
// The Glow bundle when using the Softmax layer has the "expf" function (symbol)
// unresolved. Since not all the toolchains provide an implementation we define
// the function here and use the standard "exp" implementation.
float expf(float f) {
  return exp(f);
}