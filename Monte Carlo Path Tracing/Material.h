#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.h"

enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION,DIFFUSE };

class Material {
public:
    MaterialType m_type;
    Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float spw;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline MaterialType getType();
    inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();

    // if object is a light source
    inline bool isLightSource();

    inline Vector3f sample(const Vector3f& wi, const Vector3f& N);
    inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);

    // BRDF term detemine how light reflect. Only consider diffuse material for easy-access
    inline Vector3f brdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);


private:
    // convert localRay where N  = (0,0,1) to world space
    Vector3f toWorld(const Vector3f& localRay, const Vector3f& N) {
        Vector3f fac1, fac2;

        // bool to avoid division with zero
        if (std::fabs(N.x) > std::fabs(N.y))
        { // weight in XZ coord and vertical to N
            float normalize_factor = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            fac1 = Vector3f(N.z / normalize_factor, 0.0f, -N.x / normalize_factor);
        }
        else
        { // weight in YZ coord and vertical to N
            float normalize_factor = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            fac1 = Vector3f(0.0f, N.z / normalize_factor, -N.y / normalize_factor);
        }

        fac2 = crossProduct(fac1, N);
        return localRay.x * fac2 + localRay.y * fac1 + localRay.z * N;
    }
};

Material::Material(MaterialType t, Vector3f e) {
    m_type = t;
    m_emission = e;
}

MaterialType Material::getType() { return m_type; }
Vector3f Material::getColor() { return m_color; }
Vector3f Material::getEmission() { return m_emission; }

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}

bool Material::isLightSource() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::sample(const Vector3f& wi, const Vector3f& N) {
    switch (m_type) {
        case DIFFUSE:
        {
            //uniform sampling on hemisphere 
            //getting an direction on hemisphere, converting to worldspace and return.
            float x1 = get_random_float(), x2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x2;
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            return toWorld(localRay, N);

            break;
        }
    }
}

float Material::pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
    switch (m_type) {
    case DIFFUSE:
    {
        // sample on hemisphere with summing solid angle 1/ 2PI
        if (dotProduct(wo, N) > 0.0f)
            return 0.5f / M_PI;
        else
            return 0.0f;
        break;
    }
    }
}

Vector3f Material::brdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
    switch (m_type) {
        case DIFFUSE:
        {
            if (dotProduct(N, wo) > 0.0) { //only consider hemisphere 
                return Kd / M_PI; // why divided by PI? should be 2 * PI ????
            }
            else
                return Vector3f(0.0f);
            break;
        }
    }

    return Vector3f(0.0f);
}

#endif //RAYTRACING_MATERIAL_H

