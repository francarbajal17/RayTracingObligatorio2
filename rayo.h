#pragma once

#include "vec3.h"

class rayo
{
private:
	point3 origin;
	vec3 direction;
	double medium;
public:
	rayo(vec3 origin, vec3 direction, double medium);
	vec3 getOrigin() const;
	vec3 getDirection() const;
	double getMedium() const;
};

