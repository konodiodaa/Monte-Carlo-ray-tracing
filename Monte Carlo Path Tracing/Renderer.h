#pragma once

#include "Scene.h"

class Renderer
{
public:
	void Render(const Scene& scene,int spp);
	void Render_Whitted(const Scene& scene);
};