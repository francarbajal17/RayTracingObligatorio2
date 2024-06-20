#pragma once

#include "vec3.h"
#include "objeto.h"

struct hitRecord {
    vec3 normal;
    vec3 position;
    vec3 hitDir;
    double angle;
    objeto* object;
    double distanceFromOrigin;
};
