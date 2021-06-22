#include "MathUtils.h"

int ToInt(double x) {
	return max(min(int(pow(1 - exp(-x), 1 / 2.2) * 255 + 0.5), 255), 0);
}

unsigned int CalcHash(const int& x, const int& y, const int& z, int numHash) {
	return (unsigned int)((x * 73856093) ^ (y * 19349663) ^ (z * 83492791)) % numHash;
}

double Hal(const int& b, int j) {
	const int p = primes[b];
	double h = 0.0, f = 1.0 / (double)p, fct = f;
	while (j > 0) {
		h += rev(j%p, p)*fct;
		j /= p;
		fct *= f;
	}
	return h;
}


vec3 Reflect(const vec3 inVector, const vec3 normal)
{
	vec3 v = inVector * normal * normal;
	vec3 h = inVector - v;
	v *= -1;

	return (v + h).normalize();
}

double CalculateCos2t(bool isIntoGlass, vec3& rayDirection, vec3& normal) {
	double nnt = isIntoGlass ? REFRACTIVE_INDEX_IN_VACUUM / REFRACTIVE_INDEX_IN_GLASS : REFRACTIVE_INDEX_IN_GLASS / REFRACTIVE_INDEX_IN_VACUUM;
	double ddn = rayDirection * normal;
	double cos2t = 1 - nnt * nnt*(1 - ddn * ddn);

	return cos2t;
}

vec3 Refract(bool isIntoGlass, vec3& rayDirection, vec3& normal, vec3& hitNormal) {
	double nnt = isIntoGlass ? REFRACTIVE_INDEX_IN_VACUUM / REFRACTIVE_INDEX_IN_GLASS : REFRACTIVE_INDEX_IN_GLASS / REFRACTIVE_INDEX_IN_VACUUM;
	double ddn = rayDirection * normal;
	double cos2t = 1 - nnt * nnt*(1 - ddn * ddn);

	vec3 refractDir = (rayDirection*nnt - hitNormal * ((isIntoGlass ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).normalize();
	return refractDir;
}