#include "luz.h"

luz::luz(vec3 pos, double intensity) {
	this->position = pos;
	this->intensity = intensity;
}

vec3 luz::getPosition() {
	return position;
}

double luz::getIntensity() {
	return intensity;
}
