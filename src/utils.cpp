#include "utils.h"

float mapRange(float refNumber, float refStart, float refEnd, float targetStart, float targetEnd, int decimalPrecision) {
  float deltaRef = refEnd - refStart;
  float deltaTarget = targetEnd - targetStart;

  float pct = fmax(0.0f, fmin(1.0f, abs((refNumber - refStart) / deltaRef)));
  float finalNumber = targetStart + pct * deltaTarget;

  int calcScale = (int) pow(10, decimalPrecision);
  return (float) round(finalNumber * calcScale) / calcScale;
}

float mapIntRange(int refNumber, int refStart, int refEnd, float targetStart, float targetEnd, int decimalPrecision) {
  double deltaRef = (double) refEnd - refStart;
  double normNumber = (double) refNumber - refStart;
  float deltaTarget = targetEnd - targetStart;

  double pct = fmax(0.0, fmin(1.0, normNumber > 0 && deltaRef > 0 ? normNumber / deltaRef : 0.0));
  float finalNumber = targetStart + pct * deltaTarget;

  int calcScale = (int) pow(10, decimalPrecision);
  return (float) round(finalNumber * calcScale) / calcScale;
}