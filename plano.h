#pragma once

#include "objeto.h"

class plano : public objeto
{
private:
    vec3 normal;
    point3 center;
    vec3 up;
    vec3 side;
    double width;
    double height;
public:
	plano(vec3 normal, point3 center, vec3 up, double width, double height, color ambient, color especular, color difuso, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia);
	hitRecord* intersects(const rayo& ray);
};

