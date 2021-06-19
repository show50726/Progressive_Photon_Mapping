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

class GridNode {
public:
	GridNode() {}
	GridNode(Shape* cur) {
		shape = cur;
		next = NULL;
	}

	Shape * shape;
	GridNode * next;
};

class GridHash {
public:
	GridHash() {}

	GridHash(int xs, int ys, int zs, vec2 xBound, vec2 yBound, vec2 zBound) {
		_xSize = xs;
		_ySize = ys;
		_zSize = zs;

		_xBound = xBound;
		_yBound = yBound;
		_zBound = zBound;

		_Grid.resize(_xSize, vector<vector<BoundingBox*> >(_ySize, vector<BoundingBox*>(_zSize, NULL)));
		_nodeList.resize(_xSize, vector<vector<GridNode*> >(_ySize, vector<GridNode*>(_zSize, NULL)));
		_InitCells(xBound, yBound, zBound);
	}

	void Insert(Shape* shape);
	bool InRange(int x, int y, int z);
	HitInfo FindClosestIntersectionNode(Ray& ray, vector<Shape*> shapes);
	tuple<int, int, int> FindClosestNode(vec3& pos);

private:
	void _InitCells(vec2 xBound, vec2 yBound, vec2 zBound);

	int _xSize, _ySize, _zSize;
	float _xLength, _yLength, _zLength;
	vec2 _xBound, _yBound, _zBound;
	vector<vector<vector<BoundingBox*>>> _Grid;
	vector<vector<vector<GridNode*>>> _nodeList;
};

class SpatialHashBroadPhase : public BroadPhase {
public:
	SpatialHashBroadPhase(vector<Shape*> shapes, int xs, int ys, int zs, vec2 xBound, vec2 yBound, vec2 zBound) {
		_grid = new GridHash(xs, ys, zs, xBound, yBound, zBound);

		for (int i = 0; i < shapes.size(); i++) {
			_grid->Insert(shapes[i]);
		}
	}

	HitInfo BroadPhaseDetection(Ray ray, vector<Shape*> shapes) override;

private:
	GridHash* _grid;
};

