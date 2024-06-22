#pragma once

#include "objeto.h"

#include <vector>

class cilindro : public objeto
{
private:
	vec3 centro;
	double radio;
	double altura;
	vec3 direccion;

	bool intersectsCaps(const vec3& rayOrigin, const vec3& rayDirection, float& t, vec3& normal) const;
	bool intersectsSide(const vec3& rayOrigin, const vec3& rayDirection, float& t) const;
	vec3 getNormalHitPoint(vec3 hitPoint);
public:
	cilindro(vec3 centro, double radio, double altura, vec3 direccion, color ambient, color especular, color difuso, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia);
	hitRecord* intersects(const rayo& ray);
};
