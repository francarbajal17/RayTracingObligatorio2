#include "plano.h"

plano::plano(vec3 normal, point3 center, vec3 up, double width, double height, color colorNormalizado, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia)
{
  this->normal = unit_vector(normal);
  this->center = center;
  this->up = up;
  this->side = unit_vector(vec3(cross(up, normal)));
  this->width = width;
  this->height = height;

  this->colorNormalizado = colorNormalizado;

  this->indice_reflexion = indice_reflexion;
  this->indice_refraccion = indice_refraccion;
  this->indice_especular = indice_especular;
  this->indice_transparencia = indice_transparencia;
}

hitRecord *plano::intersects(const rayo &ray)
{
  // Calculate the denominator of the intersection formula
  float denom = dot(normal, ray.obtenerDireccion());

  // Check if the ray is parallel to the plane
  if (fabs(denom) < 1e-8)
    return nullptr;

  // Calculate the intersection distance t
  vec3 p = center - ray.obtenerOrigen();
  float t = dot(p, normal) / denom;

  // Ensure the intersection distance is positive
  if (t < 0)
    return nullptr;

  // Calculate the intersection point
  vec3 candidate = ray.obtenerOrigen() + t * ray.obtenerDireccion();

  // Check if the intersection point is within the plane bounds (height)
  float height_projection = dot(candidate - center, up);
  if (fabs(height_projection) > height / 2)
    return nullptr;

  // Check if the intersection point is within the plane bounds (length)
  float length_projection = dot(candidate - center, side);
  if (fabs(length_projection) > width / 2)
    return nullptr;

  hitRecord *closestHit = new hitRecord();

  closestHit->position = ray.obtenerOrigen() + ray.obtenerDireccion() * t;
  closestHit->normal = normal;
  closestHit->hitDir = ray.obtenerDireccion();
  closestHit->angle = std::asin(dot(ray.obtenerDireccion(), closestHit->normal));
  closestHit->object = this;
  closestHit->distanceFromOrigin = t;

  // Arreglo del punto de interseccion
  closestHit->position = closestHit->position + closestHit->normal * 1e-4;

  return closestHit;
}