#include "vec3.h"
#include "objeto.h"
#include "esfera.h"
#include "luz.h"
#include "plano.h"
#include "triangulo.h"
#include "malla.h"

#include <vector>
#include <iostream>

#define M_PI 3.14159265358979323846

using namespace std;
using color = vec3;

std::vector<objeto*> objetos;
std::vector<luz*> luces;
float intensityAmbient;

hitRecord* closestIntersection(const rayo& ray) {  
    constexpr float object_dist = std::numeric_limits<float>::max();
    
    hitRecord* actual = new hitRecord();
    actual->distanceFromOrigin = object_dist;

    for (int i = 0; i < objetos.size(); i++) {
        hitRecord* hit = objetos[i]->intersects(ray);
        if (hit != nullptr && hit->distanceFromOrigin < actual->distanceFromOrigin) {
            actual = hit;
        }
    }
    if (actual != nullptr && actual->distanceFromOrigin < 1000)
        return actual;
    else
        return nullptr;
}

double fresnel(const vec3& direction, const vec3& normal, const double& inTransmissionCoefficient, const double& outTransmissionCoefficient) {
    double cosi = dot(direction, normal);

    if (cosi < -1.0) {
        cosi = -1.0;
    }
    if (cosi > 1.0) {
        cosi = 1.0;
    }

    double etai = inTransmissionCoefficient;
    double etat = outTransmissionCoefficient;
    if (cosi > 0) {
        std::swap(etai, etat);
    }
    // Compute sini using Snell's law
    double sint = etai / etat * sqrt(std::max(0.0, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) {
        return 1.0;
    }
    else {
        double cost = sqrt(std::max(0.0, 1 - sint * sint));
        cosi = abs(cosi);
        double Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        double Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
}

rayo getReflectiveRay(const rayo& ray, const hitRecord& hitRec) {
    return rayo(hitRec.position, hitRec.hitDir - 2 * dot(hitRec.hitDir, hitRec.normal) * hitRec.normal, ray.getMedium());
}

rayo getTransmissionRay(const rayo& ray, const hitRecord& hitRec) {
    bool outside = dot(hitRec.hitDir, hitRec.normal) < 0;
    vec3 bias = 0.01 * hitRec.normal;

    vec3 refractionRayOrig = outside ? hitRec.position - bias : hitRec.position + bias;

    double cosIncidenceNormal = dot(-hitRec.normal, unit_vector(hitRec.hitDir));

    if (cosIncidenceNormal < -1.0) {
        cosIncidenceNormal = -1.0;
    }
    if (cosIncidenceNormal > 1.0) {
        cosIncidenceNormal = 1.0;
    }

    double incidenceRefraction = ray.getMedium();
    double outRefraction = hitRec.object->indice_refraccion;

    vec3 normal = -hitRec.normal;
    if (cosIncidenceNormal < 0) {
        cosIncidenceNormal = -cosIncidenceNormal;
    }
    else { 
        normal = -hitRec.normal;
    }
    double eta = incidenceRefraction / outRefraction;
    double k = 1 - eta * eta * (1 - cosIncidenceNormal * cosIncidenceNormal);
    vec3 refractionDirection = k < 0 ? vec3(0, 0, 0) : eta * unit_vector(hitRec.hitDir) + (eta * cosIncidenceNormal - sqrt(k)) * normal;

    return rayo(refractionRayOrig, refractionDirection, hitRec.object->indice_refraccion);
}

color traza_RR(const rayo& ray, int depth);

color sombra_RR(const hitRecord* hitRec, const rayo& ray, int depth) {
    color diffuse = vec3(0, 0, 0);
    color specular = vec3(0, 0, 0);
    for (luz* luz : luces) {
        rayo rayo_s(hitRec->position, luz->getPosition() - hitRec->position, ray.getMedium());
        double newIntensity = luz->getIntensity();

        if (dot(hitRec->normal, rayo_s.getDirection()) > 0) {
            bool inShade = false;
            for (objeto* obj : objetos) {
                hitRecord* nextHit = obj->intersects(rayo_s);

                if (nextHit != nullptr && nextHit->distanceFromOrigin < (luz->getPosition() - nextHit->position).length()) {
                    if (nextHit->object != hitRec->object && nextHit->object->indice_transparencia == 1.0) {
                        inShade = true;
                        break;
                    }
                    else {
                        newIntensity *= 1 - nextHit->object->indice_transparencia;
                    }
                }
            }

            if (!inShade) {
                double angle = std::asin(dot(hitRec->normal, rayo_s.getDirection()));
                if (angle > 0) {
                    diffuse += angle * newIntensity * hitRec->object->difuso;

                    vec3 reflected_light = unit_vector(reflect(rayo_s.getDirection(), hitRec->normal));
                    specular += hitRec->object->indice_reflexion * std::pow(std::max(0.0, dot(reflected_light, (-hitRec->hitDir))), hitRec->object->indice_especular) * newIntensity * 0.3 * vec3(1, 1, 1);
                }
            }
        }
    }

    color final = color(0, 0, 0);

    if (depth < 5 && hitRec->object->indice_transparencia < 1 && (hitRec->object->indice_reflexion > 0 || hitRec->object->indice_refraccion > 0)) {
        double kr = fresnel(ray.getDirection(), hitRec->normal, ray.getMedium(), hitRec->object->indice_refraccion);

        color colorReflexion = vec3(0, 0, 0);
        if (hitRec->object->indice_reflexion > 0) {
            rayo reflectiveRay = getReflectiveRay(ray, *hitRec);
            colorReflexion = traza_RR(reflectiveRay, depth + 1);
        }

        color colorRefraccion = vec3(0, 0, 0);
        if (hitRec->object->indice_refraccion > 0) {
            if (kr < 1) {
                rayo refractionRay = getTransmissionRay(ray, *hitRec);
                colorRefraccion = traza_RR(refractionRay, depth + 1);
            }
        }

        final = (colorReflexion * kr + colorRefraccion * (1 - kr)) * (1 - hitRec->object->indice_transparencia);
    }

    final += (hitRec->object->ambiente * intensityAmbient + diffuse + specular) * hitRec->object->indice_transparencia;

    final[0] = final[0] * (final[0] < 1.f) + (final[0] >= 1);
    final[1] = final[1] * (final[1] < 1.f) + (final[1] >= 1);
    final[2] = final[2] * (final[2] < 1.f) + (final[2] >= 1);

    return final;
}

color traza_RR(const rayo& ray, int depth) {
    hitRecord* hitRec = closestIntersection(ray);
    if (hitRec != nullptr) {
        return sombra_RR(hitRec, ray, depth);
    }
    else {
        return color(0, 0, 0);
    }
}

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

void render() {
    float aspect = 5.0 / 5.0;

    const int width = 1600;
    const int height = int(width / aspect);
    double fov = 60;

    point3 cameraPosition = point3(0, 4.5, 6);
    point3 cameraLookAt = point3(0, 4.5, -4.5);
    vec3 cameraUp = vec3(0, 1, 0);

    vec3 w = unit_vector(cameraPosition - cameraLookAt);
    vec3 u = unit_vector(cross(cameraUp, w));
    vec3 v = cross(w, u);

    double windowDistance = (cameraPosition - cameraLookAt).length();
    double theta = degreesToRadians(fov);
    double h = tan(theta / 2);
    double windowHeight = 2 * h * windowDistance;
    double windowWidth = windowHeight * (double(width) / height);


    // Calculate the vectors across the horizontal and down the vertical window edges.
    vec3 window_u = windowWidth * u;
    vec3 window_v = windowHeight * -v;

    // Calculate the horizontal and vertical delta vectors from pixel to pixel.
    vec3 pixelDelta_u = window_u / width;
    vec3 pixelDelta_v = window_v / height;

    // Calculate the location of the upper left pixel.
    vec3 windowUpperLeft = cameraPosition - (windowDistance * w) - window_u / 2 - window_v / 2;
    vec3 upperLeftPixel = windowUpperLeft + 0.5 * (pixelDelta_u + pixelDelta_v);

    std::cout << "P3\n" << width << ' ' << height << "\n255\n";

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            vec3 pixelCenter = upperLeftPixel + (j * pixelDelta_u) + (i * pixelDelta_v);
            vec3 rayDirection = pixelCenter - cameraPosition;
            rayo ray(cameraPosition, rayDirection, 1.0);

            color pixel = traza_RR(ray, 1);

            auto r = pixel.x();
            auto g = pixel.y();
            auto b = pixel.z();

            int rbyte = int(255.999 * r);
            int gbyte = int(255.999 * g);
            int bbyte = int(255.999 * b);

            cout << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
        }
    }
}

malla* getMalla(vec3 position, double length, double height, double prof) {

    std::vector<triangulo*> triangulos;

    int triangles_per_unit = 1;

    //Caras delantera y trasera
    for (double i = 0; i < length * triangles_per_unit; i++) {
        for (double j = 0; j < height * triangles_per_unit; j++) {
            double xMin = position.x() + i / triangles_per_unit;
            double xMax = position.x() + (i + 1) / triangles_per_unit;
            double yMin = position.y() + j / triangles_per_unit;
            double yMax = position.y() + (j + 1) / triangles_per_unit;

            //Cara delantera
            triangulos.push_back(new triangulo(vec3(xMin, yMin, position.z()), vec3(xMax, yMin, position.z()), vec3(xMax, yMax, position.z())));
            triangulos.push_back(new triangulo(vec3(xMin, yMin, position.z()), vec3(xMax, yMax, position.z()), vec3(xMin, yMax, position.z())));

            //Cara trasera
            triangulos.push_back(new triangulo(vec3(xMin, yMin, position.z() - prof), vec3(xMax, yMin, position.z() - prof), vec3(xMin, yMax, position.z() - prof)));
            triangulos.push_back(new triangulo(vec3(xMax, yMin, position.z() - prof), vec3(xMax, yMax, position.z() - prof), vec3(xMin, yMax, position.z() - prof)));
        }
    }

    for (double i = 0; i < length * triangles_per_unit; i++) {
        for (double k = 0; k < prof * triangles_per_unit; k++) {
            double xMin = position.x() + i / triangles_per_unit;
            double xMax = position.x() + (i + 1) / triangles_per_unit;
            double zMin = position.z() - k / triangles_per_unit;
            double zMax = position.z() - (k + 1) / triangles_per_unit;

            // Cara superior
            triangulos.push_back(new triangulo(vec3(xMin, position.y() + height, zMin), vec3(xMin, position.y() + height, zMax), vec3(xMax, position.y() + height, zMax)));
            triangulos.push_back(new triangulo(vec3(xMin, position.y() + height, zMin), vec3(xMax, position.y() + height, zMax), vec3(xMax, position.y() + height, zMin)));

            // Cara inferior
            triangulos.push_back(new triangulo(vec3(xMin, position.y(), zMin), vec3(xMax, position.y(), zMin), vec3(xMax, position.y(), zMax)));
            triangulos.push_back(new triangulo(vec3(xMin, position.y(), zMin), vec3(xMax, position.y(), zMax), vec3(xMin, position.y(), zMax)));
        }
    }

    for (double j = 0; j < height * triangles_per_unit; j++) {
        for (double k = 0; k < prof * triangles_per_unit; k++) {
            double yMin = position.y() + j / triangles_per_unit;
            double yMax = position.y() + (j + 1) / triangles_per_unit;
            double zMin = position.z() - k / triangles_per_unit;
            double zMax = position.z() - (k + 1) / triangles_per_unit;

            // Cara Izquierda
            triangulos.push_back(new triangulo(vec3(position.x(), yMin, zMin), vec3(position.x(), yMax, zMax), vec3(position.x(), yMax, zMin)));
            triangulos.push_back(new triangulo(vec3(position.x(), yMin, zMin), vec3(position.x(), yMin, zMax), vec3(position.x(), yMax, zMax)));

            // Cara derecha
            triangulos.push_back(new triangulo(vec3(position.x() + length, yMin, zMin), vec3(position.x() + length, yMax, zMin), vec3(position.x() + length, yMax, zMax)));
            triangulos.push_back(new triangulo(vec3(position.x() + length, yMin, zMin), vec3(position.x() + length, yMax, zMax), vec3(position.x() + length, yMin, zMax)));
        }
    }
    return new malla(triangulos, color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), 0.0, 0.0, 0.0, 1.0);
}

int main() {   
    objetos.push_back(new esfera(vec3(-1.5, 3.5, -4.5), 1.5, color(1.0, 0.0, 0.0), color(1.0, 0.0, 0.0), color(1.0, 0.0, 0.0), 0.0, 1.5, 0.0, 0.2)); //Esfera de vidrio rosada
    objetos.push_back(new esfera(vec3(2, 3, -4.5), 1, color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), 1.0, 0.0, 0.5, 0.0));  //Esfera espejo
    objetos.push_back(getMalla(vec3(-3, 0, -3), 6, 2, 4)); //Mesa
    /*
    //objetos.push_back(new esfera(vec3(0, 4.5, -4.5), 1.5, color(0.0, 0.0, 1.0), color(0.0, 0.0, 1.0), color(0.0, 0.0, 1.0), 0.8, 0.0, 0.3, 1.0));  //Esfera solida azul
    
    //normal, center, up, width, height, color ambiente, color especular, color difuso, indice_reflexion, indice_refraccion, indice_especular, indice_transparencia
    
    //Rectangle scene
    
    //Bottom face
    objetos.push_back(new plano(vec3(0,1,0), point3(0,0,-4.5), vec3(0, 0, -1), 16, 9, color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), 1.0, 0.0, 0.0, 1.0));   
    //Up face
    objetos.push_back(new plano(vec3(0, -1, 0), point3(0, 9, -4.5), vec3(0, 0, -1), 16, 9, color(0.2, 0.2, 0.2), color(0.2, 0.2, 0.2), color(0.2, 0.2, 0.2), 1.0, 0.0, 0.0, 1.0));  
    //Back face
    objetos.push_back(new plano(vec3(0, 0, 1), point3(0, 4.5, -9), vec3(0, 0, -1), 16, 9, color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), 1.0, 0.0, 0.0, 1.0));  
    //Right face
    objetos.push_back(new plano(vec3(-1, 0, 0), point3(8, 4.5, -4.5), vec3(0, 0, 1), 9, 9, color(0.0, 0.4, 0.0), color(0.0, 0.4, 0.0), color(0.0, 0.4, 0.0), 1.0, 0.0, 0.0, 1.0));
    //Left face
    objetos.push_back(new plano(vec3(1, 0, 0), point3(-8, 4.5, -4.5), vec3(0, 0, 1), 9, 9, color(0.7, 0.0, 0.0), color(0.7, 0.0, 0.0), color(0.7, 0.0, 0.0), 1.0, 0.0, 0.0, 1.0));
    */

    //Square scene

    //Bottom face
    objetos.push_back(new plano(vec3(0, 1, 0), point3(0, 0, -4.5), vec3(0, 0, -1), 9, 9, color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), 1.0, 0.0, 0.0, 1.0));
    //Up face
    objetos.push_back(new plano(vec3(0, -1, 0), point3(0, 9, -4.5), vec3(0, 0, -1), 9, 9, color(0.2, 0.2, 0.2), color(0.2, 0.2, 0.2), color(0.2, 0.2, 0.2), 1.0, 0.0, 0.0, 1.0));
    //Back face
    objetos.push_back(new plano(vec3(0, 0, 1), point3(0, 4.5, -9), vec3(0, 0, -1), 9, 9, color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), color(1.0, 1.0, 1.0), 1.0, 0.0, 0.0, 1.0));
    //Right face
    objetos.push_back(new plano(vec3(-1, 0, 0), point3(4.5, 4.5, -4.5), vec3(0, 0, 1), 9, 9, color(0.0, 0.4, 0.0), color(0.0, 0.4, 0.0), color(0.0, 0.4, 0.0), 1.0, 0.0, 0.0, 1.0));
    //Left face
    objetos.push_back(new plano(vec3(1, 0, 0), point3(-4.5, 4.5, -4.5), vec3(0, 0, 1), 9, 9, color(0.7, 0.0, 0.0), color(0.7, 0.0, 0.0), color(0.7, 0.0, 0.0), 1.0, 0.0, 0.0, 1.0));


    luces.push_back(new luz(vec3(0, 8.9, -2.5), 0.3));
    luces.push_back(new luz(vec3(0, 8.9, -6.5), 0.3));

    intensityAmbient = 0.3;

    render();
}