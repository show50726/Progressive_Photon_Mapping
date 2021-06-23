#pragma once
#include <vector>
#include <thread>
#include "algebra3.h"
#include "Materials.h"
#include "Shape.h"
#include "BroadPhase.h"
#include "MathUtils.h"
using namespace std;

class Shape;
class BroadPhase;
class BoundingBox;

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
		r2 = 0;
		flux = vec3();
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

class HashGrid {
public:
	HashGrid(const int width, const int height, vector<HitInfo>& hitPoints);
	~HashGrid() { delete boundingBox; }
	
	vector<HitInfoNode*> grid;
	BoundingBox* boundingBox;
	double hashS;
	double numHash;

private:
	void _ResetGrid();
	void _InsertNode(HitInfo* hitInfo, int ix, int iy, int iz);
	int _FitBoundingBoxWithHitPointsAndIRad(vector<HitInfo>& hitPoints, double irad);
	double _CalculateIRad(const int width, const int height, vector<HitInfo>& hitPoints);
};


