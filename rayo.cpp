#include "rayo.h"

rayo::rayo(vec3 origen, vec3 direccion, double medio) {
	this->origen = origen;
	this->direccion = unit_vector(direccion);
	this->medio = medio;
}

vec3 rayo::obtenerOrigen() const {
	return origen;
}

vec3 rayo::obtenerDireccion() const {
	return direccion;
}

double rayo::obtenerMedio() const {
	return medio;
}