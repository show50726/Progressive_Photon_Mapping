#pragma once
#include <vector>
#include <algorithm>
#include <tuple>
#include <assert.h>
#include "algebra3.h"
#include "Foundation.h"

using namespace std;

class Ray;

struct BoundingBox {
public:
	BoundingBox() {
		xMin = 1e20;
		yMin = 1e20;
		zMin = 1e20;
		xMax = -1e20;
		yMax = -1e20;
		zMax = -1e20;
	}

	BoundingBox(vec3 c, float xs, float ys, float zs) {
		center = c;
		xMin = center[0] - xs / 2;
		xMax = center[0] + xs / 2;
		yMin = center[1] - ys / 2;
		yMax = center[1] + ys / 2;
		zMin = center[2] - zs / 2;
		zMax = center[2] + zs / 2;
	}

	BoundingBox(float xMin_, float xMax_, float yMin_, float yMax_, float zMin_, float zMax_) {
		xMin = xMin_;
		xMax = xMax_;
		yMin = yMin_;
		yMax = yMax_;
		zMin = zMin_;
		zMax = zMax_;
		center = vec3((xMax + xMin) / 2, (yMax + yMin) / 2, (zMax + zMin) / 2);
	}


	tuple<bool, float> HasIntersect(Ray ray);
	bool HasIntersectionWithBox(BoundingBox* box);
	bool IsPointInBox(vec3& point);
	void Fit(vec3 point);
	vec3 GetSize();

	vec3 center;
	float xMin, xMax, yMin, yMax, zMin, zMax;
};

class Shape {
public:
	Material material;
	BoundingBox boundingBox;
	tuple<bool, float> virtual HasIntersect(Ray ray) = 0;
	vec3 virtual GetNormal(vec3 position) = 0;

	Shape() {}

	bool HasIntersectionWithBox(BoundingBox* box) {
		return boundingBox.HasIntersectionWithBox(box);
	}
};

class Sphere : public Shape {
public:
	vec3 center;
	float radius;

	Sphere(vec3 c, float r, Material mat) {
		center = c;
		radius = r;
		material = mat;

		boundingBox = BoundingBox(center, radius * 2, radius * 2, radius * 2);
	}

	tuple<bool, float> HasIntersect(Ray ray) override;

	vec3 GetNormal(vec3 position)
	{
		vec3 dir = position - center;
		return dir.normalize();
	}
};

class Triangle : public Shape {
public:
	vector<vec3> vertices;

	Triangle(vector<vec3>& vert, Material mat, vec3 normal)
	{
		vertices = vert;
		_normal = normal;
		material = mat;
		float xmin = min(vertices[0][0], min(vertices[1][0], vertices[2][0]));
		float ymin = min(vertices[0][1], min(vertices[1][1], vertices[2][1]));
		float zmin = min(vertices[0][2], min(vertices[1][2], vertices[2][2]));

		float xmax = max(vertices[0][0], max(vertices[1][0], vertices[2][0]));
		float ymax = max(vertices[0][1], max(vertices[1][1], vertices[2][1]));
		float zmax = max(vertices[0][2], max(vertices[1][2], vertices[2][2]));

		boundingBox = BoundingBox(xmin, xmax, ymin, ymax, zmin, zmax);
	}

	tuple<bool, float> HasIntersect(Ray ray) override;
	static void ReorderToCounterClockWise(vector<vec3>& vert, vec3 eyePos);

	vec3 GetNormal(vec3 position) override
	{
		return _normal;
	}

private:
	vec3 _normal;
};

