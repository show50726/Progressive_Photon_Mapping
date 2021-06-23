#include "Foundation.h"
#include <cstdio>
#include <iostream>
#include <cmath>

using namespace std;


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

HashGrid::HashGrid(const int width, const int height, vector<HitInfo>& hitPoints) {
	boundingBox = new BoundingBox();
	double irad = _CalculateIRad(width, height, hitPoints);
	
	cout << "irad: " << irad << endl;

	numHash = _FitBoundingBoxWithHitPointsAndIRad(hitPoints, irad);
	hashS = 1.0 / (irad*2.0);

	_ResetGrid();

	for (auto& hp : hitPoints) {
		vec3 bMin = ((hp.hitPos - irad) - vec3(boundingBox->xMin, boundingBox->yMin, boundingBox->zMin))*hashS;
		vec3 bMax = ((hp.hitPos + irad) - vec3(boundingBox->xMin, boundingBox->yMin, boundingBox->zMin))*hashS;

		for (int iz = abs(int(bMin[2])); iz <= abs(int(bMax[2])); iz++) {
			for (int iy = abs(int(bMin[1])); iy <= abs(int(bMax[1])); iy++) {
				for (int ix = abs(int(bMin[0])); ix <= abs(int(bMax[0])); ix++) {
					_InsertNode(&hp, ix, iy, iz);
				}
			}
		}
	}
}

double HashGrid::_CalculateIRad(const int width, const int height, vector<HitInfo>& hitPoints) {
	BoundingBox bbox1 = BoundingBox();

	for (auto& hp : hitPoints) {
		bbox1.Fit(hp.hitPos);
	}

	vec3 ssize = bbox1.GetSize();
	double irad = ((ssize[0] + ssize[1] + ssize[2]) / 3.0) / ((width + height) / 2.0)*2.0;

	return irad;
}

void HashGrid::_ResetGrid() {
	grid.resize(numHash);
	for (unsigned int i = 0; i < numHash; i++) {
		grid[i] = NULL;
	}
}

int HashGrid::_FitBoundingBoxWithHitPointsAndIRad(vector<HitInfo>& hitPoints, double irad) {
	int vphoton = 0;

	for (auto& hp : hitPoints) {
		hp.r2 = irad * irad;
		vphoton++;
		boundingBox->Fit(hp.hitPos - irad);
		boundingBox->Fit(hp.hitPos + irad);
	}

	return vphoton;
}

void HashGrid::_InsertNode(HitInfo* hitInfo, int ix, int iy, int iz) {
	int hv = CalcHash(ix, iy, iz, numHash);
	HitInfoNode* node = new HitInfoNode();
	node->hitInfo = hitInfo;

	if (grid[hv] == NULL) {
		grid[hv] = node;
	}
	else {
		auto curNode = grid[hv];
		while (curNode->next != NULL) {
			curNode = curNode->next;
		}
		curNode->next = node;
	}
}
