#include "Foundation.h"
#include <cstdio>
#include <iostream>
#include <cmath>

using namespace std;

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
		hp.n = 0;
		hp.flux = vec3();
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

bool IsInShadow(vec3 testPos, vec3 lightPos, vector<Shape*>& shapes, vector<Shape*> exclude)
{
	Ray ray = Ray(testPos, lightPos - testPos);
	vector<Shape*> temp = GetExcludeVector(shapes, exclude);
	HitInfo hitInfo = ray.BroadPhaseDetection(temp);

	return hitInfo.hitObj != NULL;
}

vec3 Reflect(const vec3 inVector, const vec3 normal)
{
	vec3 v = inVector * normal * normal;
	vec3 h = inVector - v;
	v *= -1;

	return (v + h).normalize();
}

vector<Shape*> GetExcludeVector(vector<Shape*>& shapes, vector<Shape*> exclude)
{
	vector<Shape*> temp;

	for (auto shape : shapes)
	{
		if (find(exclude.begin(), exclude.end(), shape) != exclude.end())
			continue;

		temp.push_back(shape);
	}
	return temp;
}

HitInfo Ray::BroadPhaseDetection(vector<Shape*>& shapes)
{
	if (_broadPhase == NULL) {
		return NSquareBroadPhase().BroadPhaseDetection(*this, shapes);
	}
	return _broadPhase->BroadPhaseDetection(*this, shapes);
}

vec3 Ray::CastRay(vector<Shape*>& shapes, vec3 lightPos, vec3 eyePos, float weight)
{
	HitInfo hitInfo = BroadPhaseDetection(shapes);
	Shape* shape = hitInfo.hitObj;
	vec3 hitPos = hitInfo.hitPos;
	
	if (shape == NULL)
	{
		return vec3(0.0, 0.0, 0.0);
	}

	vec3 hitNormal = shape->GetNormal(hitPos);
	
	if (IsInShadow(hitPos, lightPos, shapes, {shape}))
	{
		return shape->material.color * shape->material.Ka;
	}
	
	vec3 lightDir = (lightPos - hitPos).normalize();
	vec3 viewDir = (hitPos - eyePos).normalize();
	vec3 color = shape->material.CalcColor(hitNormal, lightDir, viewDir);

	vec3 subRayColor = vec3(0.0f, 0.0f, 0.0f);
	if (shape->material.reflectionRadio > 0.0f)
	{
		vec3 reflectDir = Reflect(direction, hitNormal);
		Ray subRay = Ray(hitPos, reflectDir);
		float newWeight = weight * shape->material.reflectionRadio;
		subRayColor += weight * shape->material.reflectionRadio * subRay.CastRay(shapes, lightPos, viewDir, newWeight);
	}

	return color + subRayColor;
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
	double p = max(color[0], max(color[1], color[2]));

	switch (shape->material.rType) {
	case DIFFUSE:
	{
		//cout << "Diffuse" << endl;
		double r1 = 2.0*PI*Hal(d3 - 1, i);
		double r2 = Hal(d3 + 0, i);
		double r2s = sqrt(r2);

		// QMC
		vec3 w = normal;
		vec3 u = ((fabs(w[0]) > 0.1 ? vec3(0, 1, 0) : vec3(1, 0, 0)) ^ w).normalize();
		vec3 v = w ^ u;
		vec3 d = (u*cos(r1)*r2s + v * sin(r1)*r2s + w * sqrt(1 - r2)).normalize();
		//
		hitInfo.f = color % adj;
		//cout << hitInfo.f[0] << " " << hitInfo.f[1] <<" "<< hitInfo.f[2] << endl;
		hitInfo.pix = pixelIndex;
		hitInfo.norm = hitNormal;
		hitInfoList.push_back(hitInfo);
		break;
	}
	case SPECULAR:
	{
		/*cout << "Specular" << endl;
		vec3 newD = direction - hitNormal * 2.0*(hitNormal*direction);
		cout << hitPos[0] << " " << hitPos[1] << " " << hitPos[2];
		cout << newD[0] << " " << newD[1] << " " << newD[2];*/

		Ray newRay = Ray(hitPos, direction - hitNormal * 2.0*(hitNormal*direction));
		newRay.CastEyeRay(shapes, dpt, color%fl, color%adj, i, pixelIndex, hitInfoList);

		break;
	}
	case REFRACTION:
	{
		//cout << "Refraction" << endl;

		Ray lr = Ray(hitPos, direction - hitNormal * 2.0*(hitNormal*direction));
		bool into = (hitNormal * normal > 0.0);
		double nc = 1.0, nt = 1.5;
		double nnt = into ? nc / nt : nt / nc;
		double ddn = direction * normal;
		double cos2t = 1 - nnt * nnt*(1 - ddn * ddn);

		if (cos2t < 0) {
			return lr.CastEyeRay(shapes, dpt, fl, adj, i, pixelIndex, hitInfoList);
		}

		vec3 td = (direction*nnt - hitNormal * ((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).normalize();
		double a = nt - nc;
		double b = nt + nc;
		double c = 1 - (into ? -ddn : (td*hitNormal));
		double R0 = (a*a) / (b*b);
		double Re = R0 + (1 - R0)*c*c*c*c*c;
		double P = Re;
		vec3 fa = color % adj;
		Ray rr = Ray(hitPos, td);

		lr.CastEyeRay(shapes, dpt, fl, fa*Re, i, pixelIndex, hitInfoList);
		rr.CastEyeRay(shapes, dpt, fl, fa*(1.0 - Re), i, pixelIndex, hitInfoList);

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
	double p = max(color[0], max(color[1], color[2]));

	switch (shape->material.rType) {
	case DIFFUSE:
	{
		//cout << "Diffuse" << endl;

		double r1 = 2.0*PI*Hal(d3 - 1, i);
		double r2 = Hal(d3 + 0, i);
		double r2s = sqrt(r2);

		// QMC
		vec3 w = normal;
		vec3 u = ((fabs(w[0]) > 0.1 ? vec3(0, 1, 0) : vec3(1, 0, 0)) ^ w).normalize();
		vec3 v = w ^ u;
		vec3 d = (u*cos(r1)*r2s + v * sin(r1)*r2s + w * sqrt(1 - r2)).normalize();
		//

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

				//cout << v[0] << " "<< v[1] <<" "<< v[2] << endl;
				if ((info->norm*hitNormal > 1e-3) && (v*v <= info->r2)) {
					double g = (info->n*ALPHA+ALPHA) / (info->n*ALPHA + 1.0);
					info->r2 = info->r2*g;
					info->n++;
					info->flux = (info->flux + info->f%fl*(1.0 / PI))*g;
					//cout << info->flux[0] << " " << info->flux[1] << " " << info->flux[2] << endl;
				}
			}
		}
		if (Hal(d3 + 1, i) < p) {
			Ray newRay = Ray(hitPos, d);
			newRay.CastPhotonRay(shapes, dpt, (color%fl)*(1.0 / p), adj, i, bbox, numHash, hashS, hashGrid);
		}
		break;
	}
	case SPECULAR:
	{
		//cout << "Specular" << endl;

		Ray newRay = Ray(hitPos, direction - hitNormal * 2.0*(hitNormal*direction));
		newRay.CastPhotonRay(shapes, dpt, color%fl, color%adj, i, bbox, numHash, hashS, hashGrid);

		break;
	}
	case REFRACTION:
	{
		//cout << "Refraction" << endl;

		Ray lr = Ray(hitPos, direction - hitNormal * 2.0*(hitNormal*direction));
		bool into = hitNormal * normal > 0.0;
		double nc = 1.0, nt = 1.5;
		double nnt = into ? nc / nt : nt / nc;
		double ddn = direction * normal;
		double cos2t = 1 - nnt * nnt*(1 - ddn * ddn);

		if (cos2t < 0) {
			return lr.CastPhotonRay(shapes, dpt, fl, adj, i, bbox, numHash, hashS, hashGrid);
		}

		vec3 td = (direction*nnt - hitNormal * ((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).normalize();
		double a = nt - nc;
		double b = nt + nc;
		double c = 1 - (into ? -ddn : (td*hitNormal));
		double R0 = (a*a) / (b*b);
		double Re = R0 + (1 - R0)*c*c*c*c*c;
		double P = Re;
		vec3 fa = color % adj;
		Ray rr = Ray(hitPos, td);

		(Hal(d3 - 1, i) < P) ? lr.CastPhotonRay(shapes, dpt, fl, fa, i, bbox, numHash, hashS, hashGrid) :
			rr.CastPhotonRay(shapes, dpt, fl, fa, i, bbox, numHash, hashS, hashGrid);
		break;
	}
	}
}
