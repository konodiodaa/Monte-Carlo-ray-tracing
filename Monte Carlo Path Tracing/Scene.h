#pragma once

#include <vector>
#include "Vector.h"
#include "Object.h"
#include "BVH.h"
#include "Ray.h"

class Scene
{
public:
    int width = 1280;
    int height = 960;
    double fov = 40;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 5; // num of bounce of light
	float RussianRoulette = 0.8;

    // objects and lights in the scene
    std::vector<Object* > objects;
    std::vector<std::unique_ptr<Light> > lights;

	//BVH
	BVH* bvhTree;


    Scene(int w, int h) : width(w), height(h)
    {}

	// add light or object to scene
    void Add(Object* object) { objects.push_back(object); }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

	// get lights and objects
	const std::vector<Object*>& getObjects() const { return objects; }
	const std::vector<std::unique_ptr<Light> >& getLights() const { return lights; }

	// intial BVH tree
	void initBVH();

	// if intersect with objects
	Intersection Intersect(const Ray& ray) const;

    Vector3f castRay_Whitted(const Ray& ray, int depth) const;

    Vector3f castRay(const Ray &ray,int depth) const;
	bool Trace(const Ray& ray, const std::vector<Object*>& objects, float& tNear, uint32_t& index, Object** hit_obj);



	void sampleLight(Intersection& inter, float& pdf) const;
	Vector3f shade(Intersection& hit_obj, Vector3f wo) const;


    // reflection func
    Vector3f reflect(const Vector3f& I, const Vector3f& N) const {
        return I - 2 * dotProduct(I, N) * N;
    }

    Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float incident_rate = 1, exit_rate = ior;
        Vector3f n = N;
        if (cosi < 0) {
            cosi = -cosi;
        }
        else
        {
            std::swap(incident_rate, exit_rate);
            n = -N;
        }

        float tmp = incident_rate / exit_rate;
        float k = 1 - tmp * tmp * (1 - cosi * cosi);
        return k < 0 ? 0 : tmp * I + (tmp * cosi - sqrtf(k)) * n;
    }

    float fresnel(const Vector3f& I, const Vector3f& N, const float& ior) const {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float int_rate = 1, ext_rate = ior;
        if (cosi > 0) std::swap(int_rate, ext_rate);

        // calculate sint using Snell`s law
        float sint = (int_rate / ext_rate) * sqrtf(std::max(0.f, 1 - cosi * cosi));

        if (sint >= 1) return 1;
        else
        {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((ext_rate * cosi) - (int_rate * cost)) / ((ext_rate * cosi) + (int_rate * cost));
            float Rp = ((int_rate * cosi) - (ext_rate * cost)) / ((int_rate * cosi) + (ext_rate * cost));
            return (Rs * Rs + Rp * Rp) / 2;
        }
    }
};