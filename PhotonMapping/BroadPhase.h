#pragma once
#include <iostream>
#include <vector>
#include "Shape.h"
#include "Foundation.h"

using namespace std;

class Shape;
class BoundingBox;
class HitInfo;
class Ray;
class HitInfoNode;

class BroadPhase {
public:
	HitInfo virtual BroadPhaseDetection(Ray ray, vector<Shape*> shapes) = 0;
};

class NSquareBroadPhase : public BroadPhase {
public:
	NSquareBroadPhase() {}

	HitInfo BroadPhaseDetection(Ray ray, vector<Shape*> shapes) override;
};
