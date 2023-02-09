#include "Scene.h"

// initial BVH tree with objects in scene
void Scene::initBVH()
{
	this->bvhTree = new BVH(objects,1);
}

void Scene::sampleLight(Intersection& inter,float& pdf) const
{
	float sum = 0;
	//todo change?
	for (uint32_t i = 0; i < objects.size(); i++) {
		// find light sources in the scene, sum their area.
		if (objects[i]->isLightSource()) {
			sum += objects[i]->getArea();
		}
	}

	// get a random area from range [0, sum_area]
	// traverse all light sources, sum their area again, when sum > random area
	// render object with that light source
	float rad_A = get_random_float() * sum;
	sum = 0;

	//todo change?
	for (uint32_t i = 0; i < objects.size(); i++) {
		if (objects[i]->isLightSource()) {
			sum += objects[i]->getArea();
			if (rad_A <= sum) {
				objects[i]->Sample(inter, pdf);
				break;
			}
		}
	}
}

// traverse BVH find if rayintersect with object
Intersection Scene::Intersect(const Ray& ray) const
{
	return this->bvhTree->Intersect(ray);
}

// trace light 
bool Scene::Trace(const Ray& ray, const std::vector<Object*>& objects, 
	float& tNear, uint32_t& index, Object** hit_obj)
{
	bool isIntersect = false;
	*hit_obj = nullptr;

	for (auto object : objects) {
		float nearest = floatMax;
		uint32_t index_tri;
		Vector2f bbs_tri;

		if (object->Intersect(ray, nearest, index_tri) && nearest < tNear) {
			*hit_obj = object;
			tNear = nearest;
			index = index_tri;
			isIntersect = true;
		}
	} 

	return isIntersect;
}

Vector3f Scene::shade(Intersection& inter, Vector3f wo) const
{
    if (inter.m->isLightSource())
    {
        return inter.m->getEmission(); // if object is light source, get emission term
    }
    const float epsilon = 0.0005f;

    Vector3f light_dir;
    Vector3f light_indir;

    // direct light effect

    float pdf;
    Intersection hit_obj;
    sampleLight(hit_obj,pdf);
    Vector3f obj2Light = hit_obj.pos - inter.pos; // cast a ray from obeject to light
    Vector3f obj2Light_dir = obj2Light.normalized();
    float obj2Light_dist = dotProduct(obj2Light, obj2Light);

    //whether ray is blocked by obejct

    auto shadowInter = Intersect(Ray(inter.pos, obj2Light_dir));
    if (shadowInter.dist - obj2Light.norm() >  - epsilon)
    {
        Vector3f brdfTerm = inter.m->brdf(obj2Light_dir, wo, inter.normal);
        float cosA = std::max(0.0f, dotProduct(inter.normal, obj2Light_dir));
        float cosB = std::max(0.0f, dotProduct(hit_obj.normal, -obj2Light_dir));
        light_dir = hit_obj.emit * brdfTerm * cosA * cosB / obj2Light_dist / pdf;
    }


    // indirect light effect

    if (get_random_float() < RussianRoulette)
    {
        Vector3f nextRay = inter.m->sample(wo, inter.normal).normalized(); // sample on object to get next ray direction
        float pdf2 = inter.m->pdf(wo, nextRay, inter.normal); // get pdf, area of hit object
        if (pdf2 > epsilon) {
            // recursive
            Intersection nextObj = Intersect(Ray(inter.pos, nextRay)); 
            if (nextObj.isMeet && !nextObj.m->isLightSource()) // if intersect with next object and object is not light source
            {
                Vector3f brdfTerm = inter.m->brdf(nextRay, wo, inter.normal);
                float costheta = std::max(.0f, dotProduct(nextRay, inter.normal));
                light_indir = shade(nextObj, -nextRay) * brdfTerm * costheta / pdf2 / RussianRoulette; // recursive at next object
            }
        }
    }

    return light_dir + light_indir;
}

//Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	Intersection inter = Intersect(ray);
	if (!inter.isMeet) return {}; // if intersect with object
	return shade(inter, -ray.dir);
}

// Ray tracing for whitted-style
Vector3f Scene::castRay_Whitted(const Ray& ray, int depth) const
{
    if (depth > this->maxDepth) {
        return Vector3f(0.0, 0.0, 0.0);
    }
    Intersection intersection = Scene::Intersect(ray);
    Material* m = intersection.m;
    Object* hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    //    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;
    if (intersection.isMeet) {

        Vector3f hitPoint = intersection.pos;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        hitObject->getNormal(hitPoint, ray.dir, index, uv, N, st);
        //        Vector3f tmp = hitPoint;
        switch (m->getType()) {
        case REFLECTION_AND_REFRACTION:
        {
            Vector3f reflectionDirection = normalize(reflect(ray.dir, N));
            Vector3f refractionDirection = normalize(refract(ray.dir, N, m->ior));
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                hitPoint - N * EPSILON :
                hitPoint + N * EPSILON;
            Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                hitPoint - N * EPSILON :
                hitPoint + N * EPSILON;
            Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
            Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
            float kr = fresnel(ray.dir, N, m->ior);
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);
            break;
        }
        case REFLECTION:
        {
            float kr = fresnel(ray.dir, N, m->ior);
            Vector3f reflectionDirection = reflect(ray.dir, N);
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                hitPoint + N * EPSILON :
                hitPoint - N * EPSILON;
            hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1) * kr;
            break;
        }
        default:
        {

            Vector3f lightAmt = 0, specularColor = 0;
            Vector3f shadowPointOrig = (dotProduct(ray.dir, N) < 0) ?
                hitPoint + N * EPSILON :
                hitPoint - N * EPSILON;

            for (uint32_t i = 0; i < getLights().size(); ++i)
            {

                    Vector3f lightDir = getLights()[i]->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    lightDir = normalize(lightDir);
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    Object* shadowHitObject = nullptr;
                    float tNearShadow = floatMax;
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    bool inShadow = bvhTree->Intersect(Ray(shadowPointOrig, lightDir)).isMeet;
                    lightAmt += (1 - inShadow) * getLights()[i]->intensity * LdotN;
                    Vector3f reflectionDirection = reflect(-lightDir, N);
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.dir)),
                        m->spw) * getLights()[i]->intensity;
            }
            hitColor = lightAmt * (hitObject->getDiffuseColor(st) * m->Kd + specularColor * m->Ks);
            break;
        }
        }
    }

    return hitColor;
}






