#pragma once

#include "vec3.h"
#include "rayo.h"

using color = vec3;

class objeto;

struct hitRecord {
	vec3 normal;
	vec3 position;
	vec3 hitDir;
	double angle;
	objeto* object;
	double distanceFromOrigin;
};

class objeto
{
public:
	color colorNormalizado;

	double indice_reflexion;
	double indice_refraccion;
	double indice_especular;
	double indice_transparencia;

	virtual hitRecord* intersects(const rayo& rayo) = 0;
};

