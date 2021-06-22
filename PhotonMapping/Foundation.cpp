#include "Foundation.h"
#include <cstdio>
#include <iostream>
#include <cmath>

using namespace std;

const double REFRACTIVE_INDEX_IN_VACUUM = 1.0;
const double REFRACTIVE_INDEX_IN_GLASS = 1.5;

// TODO: Move math utils to another .c
int ToInt(double x) {
	return max(min(int(pow(1 - exp(-x), 1 / 2.2) * 255 + 0.5), 255), 0);
}

unsigned int CalcHash(const int& x, const int& y, const int& z, int numHash) {
	return (unsigned int)((x * 73856093) ^ (y * 19349663) ^ (z * 83492791)) % numHash;
}

BoundingBox BuildHashGridForPhoton(const int width, const int height, double& hashS, double& numHash, vector<HitInfo>& hitPoints, vector<HitInfoNode*>& hashGrid) {
	BoundingBox bbox1 = BoundingBox();
	BoundingBox bbox2 = BoundingBox();

	for (auto& hp : hitPoints) {
		bbox1.Fit(hp.hitPos);
	}

	vec3 ssize = bbox1.GetSize();
	double irad = ((ssize[0] + ssize[1] + ssize[2]) / 3.0) / ((width + height) / 2.0)*2.0;
	cout << "irad: " << irad << endl;

	int vphoton = 0;
	
	for (auto& hp : hitPoints) {
		hp.r2 = irad * irad;
		vphoton++;
		bbox2.Fit(hp.hitPos - irad);
		bbox2.Fit(hp.hitPos + irad);
	}

	cout << vphoton << endl;

	hashS = 1.0 / (irad*2.0);
	numHash = vphoton;

	hashGrid.resize(numHash);
	for (unsigned int i = 0; i < numHash; i++) {
		hashGrid[i] = NULL;
	}

	for (auto& hp : hitPoints) {
		vec3 bMin = ((hp.hitPos - irad) - vec3(bbox2.xMin, bbox2.yMin, bbox2.zMin))*hashS;
		vec3 bMax = ((hp.hitPos + irad) - vec3(bbox2.xMin, bbox2.yMin, bbox2.zMin))*hashS;

		// NEED REFACTORING
		// MAKE HASH GRID A CLASS
		for (int iz = abs(int(bMin[2])); iz <= abs(int(bMax[2])); iz++) {
			for (int iy = abs(int(bMin[1])); iy <= abs(int(bMax[1])); iy++) {
				for (int ix = abs(int(bMin[0])); ix <= abs(int(bMax[0])); ix++) {
					int hv = CalcHash(ix, iy, iz, numHash);
					HitInfoNode* node = new HitInfoNode();
					node->hitInfo = &hp;
					if (hashGrid[hv] == NULL) {
						hashGrid[hv] = node;
					}
					else {
						auto curNode = hashGrid[hv];
						while (curNode->next != NULL) {
							curNode = curNode->next;
						}
						curNode->next = node;
					}
				}
			}
		}
		//
	}
	

	return bbox2;
}

inline int rev(const int& i, const int& p) {
	return i == 0 ? i : p - i;
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

HitInfo Ray::BroadPhaseDetection(vector<Shape*>& shapes)
{
	if (_broadPhase == NULL) {
		return NSquareBroadPhase().BroadPhaseDetection(*this, shapes);
	}
	return _broadPhase->BroadPhaseDetection(*this, shapes);
}


Ray::Ray(vec3 p, vec3 d, BroadPhase* broadPhase)
{
	startPoint = p;
	direction = d.normalize();

	_broadPhase = broadPhase;
}

void Ray::GeneratePhotonRay(vec3& f, int i) {
	f = vec3(2500, 2500, 2500)*(PI*4.0);
	double p = 2.0*PI*Hal(0, i), t = 2.0*acos(sqrt(1.0 - Hal(1, i)));
	double st = sin(t);
	startPoint = vec3(50, 60, 85);
	direction = vec3(cos(p)*st, cos(t), sin(p)*st);
}

double CalculateCos2t(bool isIntoGlass, vec3& rayDirection, vec3& normal) {
	double nnt = isIntoGlass ? REFRACTIVE_INDEX_IN_VACUUM / REFRACTIVE_INDEX_IN_GLASS : REFRACTIVE_INDEX_IN_GLASS / REFRACTIVE_INDEX_IN_VACUUM;
	double ddn = rayDirection * normal;
	double cos2t = 1 - nnt * nnt*(1 - ddn * ddn);

	return cos2t;
}

bool IsTotalInternalReflection(double cos2t) {
	return cos2t < 0;
}

vec3 Refract(bool isIntoGlass, vec3& rayDirection, vec3& normal, vec3& hitNormal) {
	double nnt = isIntoGlass ? REFRACTIVE_INDEX_IN_VACUUM / REFRACTIVE_INDEX_IN_GLASS : REFRACTIVE_INDEX_IN_GLASS / REFRACTIVE_INDEX_IN_VACUUM;
	double ddn = rayDirection * normal;
	double cos2t = 1 - nnt * nnt*(1 - ddn * ddn);

	vec3 refractDir = (rayDirection*nnt - hitNormal * ((isIntoGlass ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).normalize();
	return refractDir;
}

void Ray::CastEyeRay(vector<Shape*>& shapes, int dpt, vec3 fl, vec3 adj, int i, int pixelIndex, vector<HitInfo>& hitInfoList) {
	dpt++;
	int d3 = dpt * 3;
	if (dpt >= 20)
		return;

	HitInfo hitInfo = BroadPhaseDetection(shapes);
	Shape* shape = hitInfo.hitObj;
	vec3 hitPos = hitInfo.hitPos;

	if (shape == NULL)
		return;

	vec3 hitNormal = shape->GetNormal(hitPos);
	vec3 color = shape->material.color;
	vec3 normal = hitNormal * this->direction < 0 ? hitNormal : hitNormal * -1.0;
	double maxC = max(color[0], max(color[1], color[2]));

	switch (shape->material.rType) {
	case DIFFUSE:
	{
		hitInfo.f = color % adj;
		hitInfo.pix = pixelIndex;
		hitInfo.norm = hitNormal;
		hitInfoList.push_back(hitInfo);
		break;
	}
	case SPECULAR:
	{
		vec3 reflectDir = Reflect(direction, hitNormal);
		Ray newRay = Ray(hitPos, reflectDir);
		newRay.CastEyeRay(shapes, dpt, color%fl, color%adj, i, pixelIndex, hitInfoList);

		break;
	}
	case REFRACTION:
	{
		vec3 reflectDir = Reflect(direction, hitNormal);
		Ray reflectRay = Ray(hitPos, reflectDir);
		bool into = (hitNormal * normal > 0.0);
		double ddn = direction * normal;
		
		double cos2t = CalculateCos2t(into, direction, normal);

		if (IsTotalInternalReflection(cos2t) < 0) {
			return reflectRay.CastEyeRay(shapes, dpt, fl, adj, i, pixelIndex, hitInfoList);
		}

		vec3 refractDir = Refract(into, direction, normal, hitNormal);
		double a = REFRACTIVE_INDEX_IN_GLASS - REFRACTIVE_INDEX_IN_VACUUM;
		double b = REFRACTIVE_INDEX_IN_GLASS + REFRACTIVE_INDEX_IN_VACUUM;
		double c = 1 - (into ? -ddn : (refractDir*hitNormal));
		double R0 = (a*a) / (b*b);
		double Re = R0 + (1 - R0)*c*c*c*c*c;
		double P = Re;
		vec3 fa = color % adj;
		Ray refractRay = Ray(hitPos, refractDir);

		reflectRay.CastEyeRay(shapes, dpt, fl, fa*Re, i, pixelIndex, hitInfoList);
		refractRay.CastEyeRay(shapes, dpt, fl, fa*(1.0 - Re), i, pixelIndex, hitInfoList);

		break;
	}
	}
}

void Ray::CastPhotonRay(vector<Shape*>& shapes, int dpt, vec3 fl, vec3 adj, int i, BoundingBox* bbox, double numHash, double hashS, vector<HitInfoNode*>& hashGrid) {
	dpt++;
	int d3 = dpt * 3;
	if (dpt >= 20)
		return;

	HitInfo hitInfo = BroadPhaseDetection(shapes);
	Shape* shape = hitInfo.hitObj;
	vec3 hitPos = hitInfo.hitPos;

	if (shape == NULL)
		return;

	vec3 hitNormal = shape->GetNormal(hitPos);
	vec3 color = shape->material.color;
	vec3 normal = hitNormal * this->direction < 0 ? hitNormal : hitNormal * -1.0;
	double maxC = max(color[0], max(color[1], color[2]));

	switch (shape->material.rType) {
	case DIFFUSE:
	{
		double r1 = 2.0*PI*Hal(d3 - 1, i);
		double r2 = Hal(d3 + 0, i);
		double r2s = sqrt(r2);

		vec3 hh = (hitPos - vec3(bbox->xMin, bbox->yMin, bbox->zMin))*hashS;
		int ix = abs(int(hh[0]));
		int iy = abs(int(hh[1]));
		int iz = abs(int(hh[2]));

		// parallel?
		{
			HitInfoNode* hp = hashGrid[CalcHash(ix, iy, iz, numHash)];
			while (hp != NULL) {
				HitInfo* info = hp->hitInfo;
				hp = hp->next;
				vec3 v = info->hitPos - hitPos;

				if ((info->norm*hitNormal > 1e-3) && (v*v <= info->r2)) {
					double g = (info->n*ALPHA+ALPHA) / (info->n*ALPHA + 1.0);
					info->r2 = info->r2*g;
					info->n++;
					info->flux = (info->flux + info->f%fl*(1.0 / PI))*g;
				}
			}
		}

		// QMC
		vec3 w = normal;
		vec3 u = ((fabs(w[0]) > 0.1 ? vec3(0, 1, 0) : vec3(1, 0, 0)) ^ w).normalize();
		vec3 v = w ^ u;
		vec3 d = (u*cos(r1)*r2s + v * sin(r1)*r2s + w * sqrt(1 - r2)).normalize();
		//

		if (Hal(d3 + 1, i) < maxC) {
			Ray newRay = Ray(hitPos, d);
			newRay.CastPhotonRay(shapes, dpt, (color%fl)*(1.0 / maxC), adj, i, bbox, numHash, hashS, hashGrid);
		}
		break;
	}
	case SPECULAR:
	{
		vec3 reflectDir = Reflect(direction, hitNormal);
		Ray newRay = Ray(hitPos, reflectDir);
		newRay.CastPhotonRay(shapes, dpt, color%fl, color%adj, i, bbox, numHash, hashS, hashGrid);

		break;
	}
	case REFRACTION:
	{
		vec3 reflectDir = Reflect(direction, hitNormal);
		Ray reflectRay = Ray(hitPos, reflectDir);
		bool into = (hitNormal * normal > 0.0);
		double ddn = direction * normal;

		double cos2t = CalculateCos2t(into, direction, normal);

		if (IsTotalInternalReflection(cos2t)) {
			return reflectRay.CastPhotonRay(shapes, dpt, fl, adj, i, bbox, numHash, hashS, hashGrid);
		}

		vec3 refractDir = Refract(into, direction, normal, hitNormal);
		double a = REFRACTIVE_INDEX_IN_GLASS - REFRACTIVE_INDEX_IN_VACUUM;
		double b = REFRACTIVE_INDEX_IN_GLASS + REFRACTIVE_INDEX_IN_VACUUM;
		double c = 1 - (into ? -ddn : (refractDir*hitNormal));
		double R0 = (a*a) / (b*b);
		double Re = R0 + (1 - R0)*c*c*c*c*c;
		double P = Re;
		vec3 fa = color % adj;
		Ray refractRay = Ray(hitPos, refractDir);

		(Hal(d3 - 1, i) < P) ? reflectRay.CastPhotonRay(shapes, dpt, fl, fa, i, bbox, numHash, hashS, hashGrid) :
			refractRay.CastPhotonRay(shapes, dpt, fl, fa, i, bbox, numHash, hashS, hashGrid);
		break;
	}
	}
}
