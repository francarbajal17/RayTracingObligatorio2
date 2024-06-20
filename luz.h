#pragma once

#include "vec3.h"

class luz
{
private:
	vec3 position;
	double intensity;
public:
	luz(vec3 pos, double intensity);
	vec3 getPosition();
	double getIntensity();
};

