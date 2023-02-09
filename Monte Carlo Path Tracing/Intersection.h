#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.h"
#include "Material.h"
#include "global.h"
class Object;
class Sphere;

struct Intersection
{
    Intersection() {
        isMeet = false;
        pos = Vector3f();
        normal = Vector3f();
        dist = std::numeric_limits<double>::max();
        obj = nullptr;
        m = nullptr;
    }

    bool isMeet;
    Vector3f pos;
    Vector3f normal;
    Vector3f emit;
    double dist;
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H