#ifndef GAGGIA_UTILS_H
#define GAGGIA_UTILS_H

#include "Arduino.h"
float mapRange(float sourceNumber, float fromA, float fromB, float toA, float toB, int decimalPrecision);
float mapIntRange(int refNumber, int refStart, int refEnd, float targetStart, float targetEnd, int decimalPrecision);
#endif
