#include "triangulo.h"

triangulo::triangulo(vec3 coordsA, vec3 coordsB, vec3 coordsC) {
    vertices[0] = coordsA;
    vertices[1] = coordsB;
    vertices[2] = coordsC;
}

vec3 triangulo::getNormal() {
    vec3 edge1 = vertices[1] - vertices[0];
    vec3 edge2 = vertices[2] - vertices[0];
    return -unit_vector(cross(edge1, edge2));
}