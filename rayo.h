#pragma once

#include "vec3.h"

class rayo
{
private:
	point3 origen;
	vec3 direccion;
	double medio;

public:
	rayo(vec3 origen, vec3 direccion, double medio);
	vec3 obtenerOrigen() const;
	vec3 obtenerDireccion() const;
	double obtenerMedio() const;
};

