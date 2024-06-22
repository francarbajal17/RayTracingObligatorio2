#include "esfera.h"

using color = vec3;

esfera::esfera(vec3 center, double radius, color colorNormalizado, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia)
{
  this->center = center;
  this->radius = radius;

  this->colorNormalizado = colorNormalizado;

  this->indice_reflexion = indice_reflexion;
  this->indice_refraccion = indice_refraccion;
  this->indice_especular = indice_especular;
  this->indice_transparencia = indice_transparencia;
}

bool solveQuadratic(double a, double b, double c, double &root1, double &root2)
{
  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0)
  {
    return false;
  }

  root1 = (-b + std::sqrt(discriminant)) / (2 * a);
  root2 = (-b - std::sqrt(discriminant)) / (2 * a);

  return true;
}

hitRecord *esfera::intersects(const rayo &ray)
{
  double t0, t1;
  vec3 oc = ray.obtenerOrigen() - center;
  double a = dot(ray.obtenerDireccion(), ray.obtenerDireccion());
  double b = 2 * dot(oc, ray.obtenerDireccion());
  double c = dot(oc, oc) - radius * radius;
  double discriminant = b * b - 4 * a * c;
  if (discriminant > 0)
  {
    if (!solveQuadratic(a, b, c, t0, t1))
      return nullptr;
    if (t0 > t1)
    {
      double aux = t0;
      t0 = t1;
      t1 = aux;
    }
    if (t0 < 0)
    {
      t0 = t1;
      if (t0 < 0)
        return nullptr;
    }

    hitRecord *closestHit = new hitRecord();

    closestHit->position = ray.obtenerOrigen() + ray.obtenerDireccion() * t0;
    closestHit->normal = unit_vector(closestHit->position - center);
    closestHit->hitDir = ray.obtenerDireccion();
    closestHit->angle = std::asin(dot(ray.obtenerDireccion(), closestHit->normal));
    closestHit->object = this;
    closestHit->distanceFromOrigin = t0;
    // Arreglo del punto de interseccion
    closestHit->position = closestHit->position + closestHit->normal * 1e-4;

    return closestHit;
  }
  else
  {
    return nullptr;
  }
}
