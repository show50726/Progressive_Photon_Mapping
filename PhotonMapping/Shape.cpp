#include "Shape.h"

using namespace std;

tuple<bool, float> Sphere::HasIntersect(Ray ray)
{
	float a = ray.direction.length2();
	float b = 2.0f * (ray.startPoint * ray.direction - center * ray.direction);
	float c = (ray.startPoint - center).length2() - radius * radius;

	float delta = b * b - 4.0f * a * c;

	if (delta < 0)
		return make_tuple(false, 0);

	float sd = sqrt(delta);
	float r1 = (-b + sd) / (2 * a);
	float r2 = (-b - sd) / (2 * a);

	if ((r1 < 0 && r2 < 0) || (r1 < 5e-2 && r2 < 5e-2))
		return make_tuple(false, 0);

	if (r1 < 0 && r2 > 5e-2)
		return make_tuple(true, r2);

	if (r2 < 0 && r1 > 5e-2)
		return make_tuple(true, r1);

	return make_tuple(true, min(r1, r2) > 5e-2 ? min(r1, r2) : max(r1, r2));
}

tuple<bool, float> Triangle::HasIntersect(Ray ray)
{
	vec3 d1 = vertices[1] - vertices[0];
	vec3 d2 = vertices[2] - vertices[0];

	mat3 A;
	A[0][0] = d1[0];
	A[0][1] = d2[0];
	A[0][2] = -ray.direction[0];

	A[1][0] = d1[1];
	A[1][1] = d2[1];
	A[1][2] = -ray.direction[1];

	A[2][0] = d1[2];
	A[2][1] = d2[2];
	A[2][2] = -ray.direction[2];

	vec3 b;
	b[0] = ray.startPoint[0] - vertices[0][0];
	b[1] = ray.startPoint[1] - vertices[0][1];
	b[2] = ray.startPoint[2] - vertices[0][2];

	vec3 x = A.inverse() * b;

	if (x[0] < 0 || x[0] > 1)
		return make_tuple(false, 0);

	if (x[1] < 0 || x[1] > 1)
		return make_tuple(false, 0);

	if (x[2] > 0 && x[1] + x[0] <= 1)
		return make_tuple(true, x[2]);

	return make_tuple(false, 0);
}

void Triangle::ReorderToCounterClockWise(vector<vec3>& vert, vec3 eyePos)
{
	assert(vert.size() == 3);

	vec3 d1 = vert[1] - vert[0];
	vec3 d2 = vert[2] - vert[0];
	vec3 cross = d1 ^ d2;
	vec3 eyeDir = eyePos - vert[0];

	if (cross * eyeDir < 0)
	{
		swap(vert[1], vert[2]);
	}

	return;
}

tuple<bool, float> BoundingBox::HasIntersect(Ray ray) {
	/*
	if (ray.startPoint[0] >= xMin && ray.startPoint[0] <= xMax && ray.startPoint[1] >= yMin && ray.startPoint[1] <= yMax && ray.startPoint[2] >= zMin && ray.startPoint[2] <= zMax)
		return make_tuple(true, 0);
	*/

	float tmin = (xMin - ray.startPoint[0]) / ray.direction[0];
	float tmax = (xMax - ray.startPoint[0]) / ray.direction[0];

	if (tmin > tmax) 
		swap(tmin, tmax);

	if (tmax < 0)
		tmax = 10000010;

	if (tmin < 0)
		tmin = -10000010;

	float tymin = (yMin - ray.startPoint[1]) / ray.direction[1];
	float tymax = (yMax - ray.startPoint[1]) / ray.direction[1];

	if (tymin > tymax) 
		swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax)) {
		return make_tuple(false, 0);
	}

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax && tymax > 0)
		tmax = tymax;

	float tzmin = (zMin - ray.startPoint[2]) / ray.direction[2];
	float tzmax = (zMax - ray.startPoint[2]) / ray.direction[2];

	if (tzmin > tzmax)
		swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax)) {
		return make_tuple(false, 0);
	}

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax && tzmax > 0)
		tmax = tzmax;

	if (tmax < 0 && tmin < 0)
		return make_tuple(false, 0);

	return make_tuple(true, tmin > 0 ? tmin : tmax);
}

bool BoundingBox::HasIntersectionWithBox(BoundingBox* box) {
	if (xMin > box->xMax)
		return false;

	if (xMax < box->xMin)
		return false;

	if (yMin > box->yMax)
		return false;

	if (yMax < box->yMin)
		return false;

	if (zMin > box->zMax)
		return false;

	if (zMax < box->zMin)
		return false;

	return true;
}

bool BoundingBox::IsPointInBox(vec3& point) {
	float x = point[0];
	float y = point[1];
	float z = point[2];

	if (x < xMin || x > xMax)
		return false;

	if (y < yMin || y > yMax)
		return false;

	if (z < zMin || z > zMax)
		return false;

	return true;
}

void BoundingBox::Fit(vec3 point) {
	xMin = min(xMin, point[0]);
	yMin = min(yMin, point[1]);
	zMin = min(zMin, point[2]);

	xMax = max(xMax, point[0]);
	yMax = max(yMax, point[1]);
	zMax = max(zMax, point[2]);
}

vec3 BoundingBox::GetSize() {
	return vec3(xMax - xMin, yMax - yMin, zMax - zMin);
}