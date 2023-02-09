#pragma once

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.h"
#include "Vector.h"
#include "global.h"
#include <limits>
#include <array>

class BoundBox
{
public:
	Vector3f pMin, pMax;
	BoundBox()
	{
		double minNum = doubleMin;
		double maxNum = doubleMax;
		
		// value pMax and pMin with maxDouble and minDouble
		pMax = Vector3f(minNum, minNum, minNum);
		pMin = Vector3f(maxNum, maxNum, maxNum);
	}
	// value pMin and pMax with one point
	BoundBox(const Vector3f p) : pMin(p), pMax(p) {}

	// value pMin and pMax with two points
	BoundBox(const Vector3f p1, const Vector3f p2)
	{
		pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
		pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
	}

	// Diagonal of AABB boundbox
	Vector3f Diagonal() const { return pMax - pMin; }

	// Find longest axis in BoundBox
	int longestAxis() const
	{
		Vector3f diag = Diagonal();
		if (diag.x > diag.y && diag.x > diag.z)
			return 0; // longest axis is x
		else if (diag.y > diag.z)
			return 1; // longest axis is y
		else
			return 2; // longest axis is z
	}

	// area of boundbox 
	double SurfaceArea() const
	{
		Vector3f diag = Diagonal();
		return 2 * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z);
	}

	// get center of boundbox
	Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

	inline const Vector3f& operator[](int i) const
	{
		if (i == 0) return pMin;
		else return pMax;
	}

	inline bool IntersectPoint(const Ray& ray, const Vector3f& invDir,
									const std::array<int, 3>& dirisNeg) const;
};

// invDir incident light
inline bool BoundBox::IntersectPoint(const Ray& ray, const Vector3f& invDir,
	const std::array<int, 3>& dirIsNeg) const
{
	const auto& origin = ray.origin;
	float tEnter = -std::numeric_limits<float>::infinity();
	float tExit = std::numeric_limits<float>::infinity();
	for (int i = 0; i < 3; i++)
	{
		float min = (pMin[i] - origin[i]) * invDir[i];
		float max = (pMax[i] - origin[i]) * invDir[i];
		if (!dirIsNeg[i])
		{
			std::swap(min, max);
		}
		tEnter = std::max(min, tEnter);
		tExit = std::min(max, tExit);
	}
	return tEnter <= tExit&& tExit >= 0;
}

inline BoundBox Union(const BoundBox& b1, const BoundBox& b2)
{
	BoundBox ret;
	ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
	ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
	return ret;
}

inline BoundBox Union(const BoundBox& b, const Vector3f& p)
{
	BoundBox ret;
	ret.pMin = Vector3f::Min(b.pMin, p);
	ret.pMax = Vector3f::Max(b.pMax, p);
	return ret;
}


#endif // RAYTRACING_BOUNDS3_H