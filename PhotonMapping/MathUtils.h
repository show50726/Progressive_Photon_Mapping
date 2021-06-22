#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include "algebra3.h"

#define PI ((double)3.14159265358979)
#define ALPHA ((double)0.7)

using namespace std;

// Halton sequence with reverse permutation
const int primes[61] = {
	2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,61,67,71,73,79,
	83,89,97,101,103,107,109,113,127,131,137,139,149,151,157,163,167,173,179,181,
	191,193,197,199,211,223,227,229,233,239,241,251,257,263,269,271,277,281,283
};


const double REFRACTIVE_INDEX_IN_VACUUM = 1.0;
const double REFRACTIVE_INDEX_IN_GLASS = 1.5;


inline int rev(const int& i, const int& p) {
	return i == 0 ? i : p - i;
}

inline bool IsTotalInternalReflection(double cos2t) {
	return cos2t < 0;
}

int ToInt(double x);
unsigned int CalcHash(const int& x, const int& y, const int& z, int numHash);
double Hal(const int& b, int j);
double CalculateCos2t(bool isIntoGlass, vec3& rayDirection, vec3& normal);
vec3 Reflect(vec3 inVector, vec3 normal);
vec3 Refract(bool isIntoGlass, vec3& rayDirection, vec3& normal, vec3& hitNormal);
