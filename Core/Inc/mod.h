#ifndef MOD_H
#define MOD_H

#include "math.h"

inline int cmodi(int a, int b) {
  return ((a % b) + b) % b;
}

inline float cmodf(float a, float b) {
  return fmod((fmod(a, b) + b), b);
}

inline long long cmodl(long long a, long long b) {
  return ((a % b) + b) % b;
}

inline double cmod(double a, double b) {
  return fmod((fmod(a, b) + b), b);
}

#endif  // MOD_H