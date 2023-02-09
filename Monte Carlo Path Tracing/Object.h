#pragma once
#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H

#include "Vector.h"
#include "global.h"
#include "BoundBox.h"
#include "Ray.h"
#include "Intersection.h"

class Object
{
public:
    Object() {}
    virtual ~Object() {}
    virtual bool Intersect(const Ray& ray) = 0;
    virtual bool Intersect(const Ray& ray, float&, uint32_t&) const = 0;
    virtual Intersection getIntersection(Ray _ray) = 0;
    
    virtual void getNormal(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&, Vector2f&) const = 0;
    virtual Vector3f getDiffuseColor(const Vector2f&) const = 0;
    virtual BoundBox getBounds() = 0;

    virtual float getArea() = 0;
    virtual void Sample(Intersection& pos, float& pdf) = 0;
    virtual bool isLightSource() = 0;
};



#endif //RAYTRACING_OBJECT_H
