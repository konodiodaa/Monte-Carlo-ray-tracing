#pragma once
#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.h"
#include "Ray.h"
#include "BoundBox.h"
#include "Intersection.h"
#include "Vector.h"

#endif //RAYTRACING_BVH_H

struct  BVHNode
{
	BoundBox box;
	BVHNode* left;
	BVHNode* right;
	Object* object;
	float area;

public:
	int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;

	BVHNode() {
		box = BoundBox();
		left = nullptr;
		right = nullptr;
		object = nullptr;
	}

};

class BVH {
public:
	const int maxPrimsInNode;
	std::vector<Object*> primitives;

	BVHNode* root;

	BVH(std::vector<Object*> obj, int maxPrimsInNode = 1);
	~BVH();

	Intersection Intersect(const Ray& r) const;
	Intersection getIntersection(BVHNode* node, const Ray& r)const;
	bool IntersectPoint(const Ray& ray) const;


	BVHNode* recursiveBuild(std::vector<Object*> objects);

	void getSample(BVHNode* node, float p, Intersection& pos, float& pdf);
	void Sample(Intersection& pos, float& pdf);
};