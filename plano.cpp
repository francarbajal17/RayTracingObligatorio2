#include "plano.h"

plano::plano(vec3 normal, point3 center, vec3 up, double width, double height, color ambient, color especular, color difuso, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia) {
	this->normal = unit_vector(normal);
	this->center = center;
	this->up = up;
	this->side = unit_vector(vec3(cross(up, normal)));
	this->width = width;
	this->height = height;

	this->ambiente = ambient;
	this->especular = especular;
	this->difuso = difuso;

	this->indice_reflexion = indice_reflexion;
	this->indice_refraccion = indice_refraccion;
	this->indice_especular = indice_especular;
	this->indice_transparencia = indice_transparencia;
}

hitRecord* plano::intersects(const rayo& ray) {
    // Calculate the denominator of the intersection formula
    float denom = dot(normal, ray.getDirection());

    // Check if the ray is parallel to the plane
    if (fabs(denom) < 1e-8)
        return nullptr;

    // Calculate the intersection distance t
    vec3 p = center - ray.getOrigin();
    float t = dot(p, normal) / denom;

    // Ensure the intersection distance is positive
    if (t < 0)
        return nullptr;

    // Calculate the intersection point
    vec3 candidate = ray.getOrigin() + t * ray.getDirection();

    // Check if the intersection point is within the plane bounds (height)
    float height_projection = dot(candidate - center, up);
    if (fabs(height_projection) > height / 2)
        return nullptr;

    // Check if the intersection point is within the plane bounds (length)
    float length_projection = dot(candidate - center, side);
    if (fabs(length_projection) > width / 2)
        return nullptr;

    hitRecord* closestHit = new hitRecord();

    closestHit->position = ray.getOrigin() + ray.getDirection() * t;
    closestHit->normal = normal;
    closestHit->hitDir = ray.getDirection();
    closestHit->angle = std::asin(dot(ray.getDirection(), closestHit->normal)); 
    closestHit->object = this;
    closestHit->distanceFromOrigin = t;

    //Arreglo del punto de interseccion
    closestHit->position = closestHit->position + closestHit->normal * 1e-4;

    return closestHit;
}