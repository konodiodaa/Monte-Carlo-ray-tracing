#ifndef RAYTRACING_SPHERE_H
#define RAYTRACING_SPHERE_H

#include "Object.h"
#include "Vector.h"
#include "BoundBox.h"
#include "Material.h"

class Sphere : public Object {
public:
	Vector3f center;
	float radius, radius_sqa;
	Material* m;
	Sphere(const Vector3f& c, const float& r) : center(c), radius(r), radius_sqa(r* r), m(new Material()) {}
	
    
    bool Intersect(const Ray& r) {
        Vector3f L = r.origin - center;
        float a = dotProduct(r.dir, r.dir);
        float b = 2 * dotProduct(r.dir, L);
        float c = dotProduct(L, L) - radius_sqa;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
	}

    bool Intersect(const Ray& ray, float& tnear, uint32_t& index) const
    {
        // analytic solution
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.dir, ray.dir);
        float b = 2 * dotProduct(ray.dir, L);
        float c = dotProduct(L, L) - radius_sqa;
        float t0, t1;
        if (!
            solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        tnear = t0;

        return true;
    }

    Intersection getIntersection(Ray ray) {
        Intersection inter;
        inter.isMeet = false;
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.dir, ray.dir);
        float b = 2 * dotProduct(ray.dir, L);
        float c = dotProduct(L, L) - radius_sqa;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return inter;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return inter;
        inter.isMeet = true;

        inter.pos = Vector3f(ray.origin + ray.dir * t0);
        inter.normal = normalize(Vector3f(inter.pos - center));
        inter.m = this->m;
        inter.obj = this;
        inter.dist = t0;
        return inter;

    }
    void getNormal(const Vector3f& P, const Vector3f& I, const uint32_t& index, const Vector2f& uv, Vector3f& N, Vector2f& st) const
    {
        N = normalize(P - center);
    }

    Vector3f getDiffuseColor(const Vector2f& st)const {
        return m->getColor();
    }
    
    BoundBox getBounds() {
        return BoundBox(Vector3f(center.x - radius, center.y - radius, center.z - radius),
            Vector3f(center.x + radius, center.y + radius, center.z + radius));
    }

};


#endif //RAYTRACING_SPHERE_H