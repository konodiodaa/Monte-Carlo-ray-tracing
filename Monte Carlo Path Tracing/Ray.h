
#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H
#include "Vector.h"
#include "global.h"

struct Ray
{
    //Destination = origin + t*direction
    Vector3f origin;
    Vector3f dir, dir_inverse;
    double t;//transportation time,
    double t_min, t_max;

    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0) : origin(ori), dir(dir), t(_t) {
        dir_inverse = Vector3f(1. / dir.x, 1. / dir.y, 1. / dir.z);
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();

    }

    Vector3f operator()(double t) const { return origin + dir * t; }
};

class Light
{
public:
    Light(const Vector3f& p, const Vector3f& i) : position(p), intensity(i) {}
    virtual ~Light() = default;
    Vector3f position;
    Vector3f intensity;
};

#endif //RAYTRACING_RAY_H