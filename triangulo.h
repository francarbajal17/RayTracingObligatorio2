#pragma once

#include "vec3.h"

class triangulo
{
private:
	vec3 vertices[3];
public:
	triangulo(vec3 coordsA, vec3 coordsB, vec3 coordsC); 
	vec3 getNormal();

	vec3 getCoordsA() {
		return vertices[0];
	}
	vec3 getCoordsB() {
		return vertices[1];
	}
	vec3 getCoordsC() {
		return vertices[2];
	}
};

