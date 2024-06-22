#pragma once

#include "objeto.h"

class esfera : public objeto
{
private:
	vec3 center;
	double radius;
public:
	esfera(vec3 center, double radius, color colorNormalizado, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia);
	hitRecord* intersects(const rayo& ray);
};