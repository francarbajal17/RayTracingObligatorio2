#include "rayo.h"

rayo::rayo(vec3 origin, vec3 direction, double medium) {
	this->origin = origin;
	this->direction = unit_vector(direction);
	this->medium = medium;
}

vec3 rayo::getOrigin() const {
	return origin;
}

vec3 rayo::getDirection() const {
	return direction;
}

double rayo::getMedium() const {
	return medium;
}