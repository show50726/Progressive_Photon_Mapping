#pragma once
#include <vector>
#include <thread>
#include "algebra3.h"
#include "Materials.h"
#include "Shape.h"
#include "BroadPhase.h"

#define PI ((double)3.14159265358979)
#define ALPHA ((double)0.7)

using namespace std;

// Halton sequence with reverse permutation
const int primes[61] = {
	2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,61,67,71,73,79,
	83,89,97,101,103,107,109,113,127,131,137,139,149,151,157,163,167,173,179,181,
	191,193,197,199,211,223,227,229,233,239,241,251,257,263,269,271,277,281,283
};


class Shape;
class BroadPhase;
class BoundingBox;

vec3 Reflect(const vec3 inVector, const vec3 normal);
double Hal(const int& b, int j);
unsigned int CalcHash(const int& x, const int& y, const int& z, int numHash);
int ToInt(double x);

struct HitInfo {
	Shape * hitObj;
	vec3 hitPos, f, norm, flux;
	double r2;
	unsigned int n;
	int pix;

	HitInfo(Shape* obj, vec3 pos)
	{
		hitObj = obj;
		hitPos = pos;
	}
};

struct HitInfoNode {
	HitInfo* hitInfo;
	HitInfoNode* next;
};

class Ray {
public:
	vec3 startPoint;
	vec3 direction;

	Ray(vec3 p, vec3 d, BroadPhase* broadPhase = NULL);

	HitInfo BroadPhaseDetection(vector<Shape*>& shapes);
	void GeneratePhotonRay(vec3& f, int i);
	void CastEyeRay(vector<Shape*>& shapes, int dpt, vec3 fl, vec3 adj, int i, int pixelIndex, vector<HitInfo>& hitInfoList);
	void CastPhotonRay(vector<Shape*>& shapes, int dpt, vec3 fl, vec3 adj, int i, BoundingBox* bbox, double numHash, double hashS, vector<HitInfoNode*>& hashGrid);
private:
	BroadPhase * _broadPhase;
};

struct ViewInfo {
public:
	vec3 eyePos;
	vec3 direction;
	vec3 upVector;
	float fieldOfView;

	vec3 CalcRightVector()
	{
		vec3 v = upVector ^ direction;
		return v.normalize();
	}
};


BoundingBox BuildHashGridForPhoton(const int width, const int height, double& hashS, double& numHash, vector<HitInfo>& hitPoints, vector<HitInfoNode*>& hashGrid);


