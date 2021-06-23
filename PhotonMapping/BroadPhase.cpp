#include "BroadPhase.h"


HitInfo NSquareBroadPhase::BroadPhaseDetection(Ray ray, vector<Shape*> shapes) {
	float minT = 100010.0f;
	Shape* candidate = NULL;
	for (auto shape : shapes)
	{
		bool hit;
		float t;
		tie(hit, t) = shape->HasIntersect(ray);

		if (hit && t < minT)
		{
			candidate = shape;
			minT = t;
		}
	}

	if (candidate)
	{
		vec3 hitPos = ray.startPoint + minT * ray.direction;
		return HitInfo(candidate, hitPos);
	}

	return HitInfo(NULL, 0.0f);
}

