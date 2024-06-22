#include "cilindro.h"

cilindro::cilindro(vec3 centro, double radio, double altura, vec3 direccion,
                   color colorNormalizado, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia)
{
  this->centro = centro;
  this->radio = radio;
  this->altura = altura;
  this->direccion = direccion;

  this->colorNormalizado = colorNormalizado;

  this->indice_reflexion = indice_reflexion;
  this->indice_refraccion = indice_refraccion;
  this->indice_especular = indice_especular;
  this->indice_transparencia = indice_transparencia;
}

hitRecord *cilindro::intersects(const rayo &ray)
{
  float tCaps;
  vec3 normalCaps;
  bool hitCaps = intersectsCaps(ray.obtenerOrigen(), ray.obtenerDireccion(), tCaps, normalCaps);

  float tSide;
  bool hitSide = intersectsSide(ray.obtenerOrigen(), ray.obtenerDireccion(), tSide);

  hitRecord *closestHit = new hitRecord();

  if (hitCaps && hitSide)
  {
    float t0 = (tCaps < tSide) ? tCaps : tSide;

    closestHit->position = ray.obtenerOrigen() + ray.obtenerDireccion() * t0;
    closestHit->normal = getNormalHitPoint(closestHit->position);
    closestHit->hitDir = ray.obtenerDireccion();
    closestHit->angle = std::asin(dot(ray.obtenerDireccion(), closestHit->normal));
    closestHit->object = this;
    closestHit->distanceFromOrigin = t0;
    // Arreglo del punto de interseccion
    closestHit->position = closestHit->position + closestHit->normal * 1e-4;

    return closestHit;
  }
  else if (hitCaps)
  {
    float t0 = tCaps;

    closestHit->position = ray.obtenerOrigen() + ray.obtenerDireccion() * t0;
    closestHit->normal = getNormalHitPoint(closestHit->position);
    closestHit->hitDir = ray.obtenerDireccion();
    closestHit->angle = std::asin(dot(ray.obtenerDireccion(), closestHit->normal));
    closestHit->object = this;
    closestHit->distanceFromOrigin = t0;
    // Arreglo del punto de interseccion
    closestHit->position = closestHit->position + closestHit->normal * 1e-4;

    return closestHit;
  }
  else if (hitSide)
  {
    float t0 = tSide;

    closestHit->position = ray.obtenerOrigen() + ray.obtenerDireccion() * t0;
    closestHit->normal = getNormalHitPoint(closestHit->position);
    closestHit->hitDir = ray.obtenerDireccion();
    closestHit->angle = std::asin(dot(ray.obtenerDireccion(), closestHit->normal));
    closestHit->object = this;
    closestHit->distanceFromOrigin = t0;
    // Arreglo del punto de interseccion
    closestHit->position = closestHit->position + closestHit->normal * 1e-4;

    return closestHit;
  }

  return nullptr;
}

bool cilindro::intersectsSide(const vec3 &rayOrigin, const vec3 &rayDirection, float &t) const
{
  vec3 originToCenterRay = rayOrigin - centro;
  vec3 centerCrossDireccion = cross(originToCenterRay, direccion);
  vec3 rayDirectionCrossDireccion = cross(rayDirection, direccion);
  double A = dot(rayDirectionCrossDireccion, rayDirectionCrossDireccion);
  double B = 2 * dot(rayDirectionCrossDireccion, centerCrossDireccion);
  double C = dot(centerCrossDireccion, centerCrossDireccion) - (radio * radio * dot(direccion, direccion));

  // Solve the quadratic equation A*t^2 + B*t + C = 0
  double discriminant = B * B - 4 * A * C;
  if (discriminant < 0)
    return false; // No intersection

  double sqrtDiscriminant = std::sqrt(discriminant);
  double t1 = (-B - sqrtDiscriminant) / (2 * A);
  double t2 = (-B + sqrtDiscriminant) / (2 * A);

  // Check the intersection points
  for (double t_val : {t1, t2})
  {
    if (t_val < 0)
      continue; // Intersection is behind the ray's origin

    vec3 intersectionPoint = rayOrigin + rayDirection * t_val;
    vec3 projectionOnAxis = centro + direccion * dot(direccion, intersectionPoint - centro);

    // Check if the intersection point is within the cylinder's height
    double distanceAlongAxis = dot(direccion, intersectionPoint - centro);
    if (distanceAlongAxis >= 0 && distanceAlongAxis <= altura)
    {
      t = t_val;
      return true;
    }
  }

  return false;
}

bool cilindro::intersectsCaps(const vec3 &rayOrigin, const vec3 &rayDirection, float &t, vec3 &normal) const
{
  float t1 = dot((centro - rayOrigin), direccion) / dot(rayDirection, direccion);
  float t2 = dot((centro + direccion * altura - rayOrigin), direccion) / dot(rayDirection, direccion);

  vec3 intersection1 = rayOrigin + rayDirection * t1;
  vec3 intersection2 = rayOrigin + rayDirection * t2;

  if ((intersection1 - centro).length() <= radio)
  {
    t = t1;
    normal = direccion;
    return true;
  }

  if ((intersection2 - (centro + direccion * altura)).length() <= radio)
  {
    t = t2;
    normal = -direccion;
    return true;
  }

  return false;
}

vec3 cilindro::getNormalHitPoint(vec3 hitPoint)
{
  vec3 toHitPoint = hitPoint - centro;
  vec3 projectionOnAxis = direccion * dot(toHitPoint, direccion);

  // Tolerance for determining if the hit point is on the caps
  float epsilon = 1e-3;

  // Check if the hit point is on the bottom cap
  if (fabs(dot(toHitPoint, direccion) + altura / 2.0) < epsilon)
  {
    return -direccion; // Normal points downwards for the bottom cap
  }

  // Check if the hit point is on the top cap
  if (fabs(dot(toHitPoint, direccion) - altura / 2.0) < epsilon)
  {
    return direccion; // Normal points upwards for the top cap
  }

  // Compute the normal vector for the side
  vec3 normal = unit_vector(toHitPoint - projectionOnAxis);
  return normal;
}