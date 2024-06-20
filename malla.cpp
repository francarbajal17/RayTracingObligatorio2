#include "malla.h"

malla::malla(std::vector<triangulo*> triangulos, color ambient, color especular, color difuso, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia) {
	this->triangulos = triangulos;
	
	this->ambiente = ambient;
	this->especular = especular;
	this->difuso = difuso;

	this->indice_reflexion = indice_reflexion;
	this->indice_refraccion = indice_refraccion;
	this->indice_especular = indice_especular;
	this->indice_transparencia = indice_transparencia;
}

hitRecord* malla::intersects(const rayo& ray) {
    constexpr float epsilon = std::numeric_limits<float>::epsilon();
    hitRecord* result = nullptr;
    float triangleDist = std::numeric_limits<float>::max();
    //int count = 0;

    for (auto triangulo : triangulos) {
        vec3 edge1 = triangulo->getCoordsB() - triangulo->getCoordsA();
        vec3 edge2 = triangulo->getCoordsC() - triangulo->getCoordsA();
        vec3 ray_cross_e2 = cross(ray.getDirection(), edge2);
        float det = dot(edge1, ray_cross_e2);

        if (det > -epsilon && det < epsilon)
            continue;    // This ray is parallel to this triangle.

        float inv_det = 1.0 / det;
        vec3 s = ray.getOrigin() - triangulo->getCoordsA();
        float u = inv_det * dot(s, ray_cross_e2);

        if (u < 0 || u > 1)
            continue;

        vec3 s_cross_e1 = cross(s, edge1);
        float v = inv_det * dot(ray.getDirection(), s_cross_e1);

        if (v < 0 || u + v > 1)
            continue;

        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = inv_det * dot(edge2, s_cross_e1);

        // Ray intersects
        if (t > epsilon) { 
            if (t < triangleDist) {
                result = new hitRecord();

                result->position = ray.getOrigin() + ray.getDirection() * t;
                result->normal = triangulo->getNormal();
                result->hitDir = ray.getDirection();
                result->angle = std::asin(dot(ray.getDirection(), result->normal));
                result->object = this;
                result->distanceFromOrigin = t;

                //Arreglo del punto de interseccion
                result->position = result->position + result->normal * 1e-4;

                triangleDist = t;
                //count++;
            }
            //if (count == 2) break;
            continue;
        }
        else               
            continue;
    }

    return result;
    
}