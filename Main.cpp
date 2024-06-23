#include "vec3.h"
#include "objeto.h"
#include "esfera.h"
#include "luz.h"
#include "plano.h"
#include "triangulo.h"
#include "malla.h"
#include "cilindro.h"
#include "pugixml.hpp"
#include "FreeImage.h"

#include <vector>
#include <iostream>

#define M_PI 3.14159265358979323846

using namespace std;
using color = vec3;

// Variables globales
std::vector<objeto *> objetos;
std::vector<luz *> luces;
double intensityAmbient;
bool mostrarRefraccionBYN = false;
bool mostrarRefleccionBYN = false;

//Esfera grande
double esferaGrandeX;
double esferaGrandeY;
double esferaGrandeZ;
double indiceTransparenciaEG;
double indiceReflexionEG;
double indiceEspecularEG;
double indiceRefraccionEG;
double radioEsferaGrande;
double rColorEG;
double gColorEG;
double bColorEG;

double esferaChicaX;
double esferaChicaY;
double esferaChicaZ;
double rColorEC;
double gColorEC;
double bColorEC;
double radioEsferaChica;
double indiceTransparenciaEC;
double indiceReflexionEC;
double indiceEspecularEC;
double indiceRefraccionEC;

double cilindroX;
double cilindroY;
double cilindroZ;
double altoCilindro;
double radioCilindro;
double rColorCilindro;
double gColorCilindro;
double bColorCilindro;
double indiceTransparenciaCilindro;
double indiceReflexionCilindro;
double indiceEspecularCilindro;
double indiceRefraccionCilindro;

double luz1X;
double luz1Y;
double luz1Z;
double luz1Intensidad;

double mesaX;
double mesaY;
double mesaZ;
double mesaAncho;
double mesaAlto;
double mesaProfundidad;
double rColorMesa;
double gColorMesa;
double bColorMesa;
double indiceTransparenciaMesa;
double indiceReflexionMesa;
double indiceEspecularMesa;
double indiceRefraccionMesa;

double luz2X;
double luz2Y;
double luz2Z;
double luz2Intensidad;

int width;
int height;

point3 cameraPosition;
vec3 cameraLookAt;
vec3 cameraUp;

vec3 pixelDelta_u;
vec3 pixelDelta_v;

vec3 upperLeftPixel;

hitRecord *closestIntersection(const rayo &ray)
{
  constexpr float object_dist = std::numeric_limits<float>::max();

  hitRecord *actual = new hitRecord();
  actual->distanceFromOrigin = object_dist;

  for (int i = 0; i < objetos.size(); i++)
  {
    hitRecord *hit = objetos[i]->intersects(ray);
    if (hit != nullptr && hit->distanceFromOrigin < actual->distanceFromOrigin)
    {
      actual = hit;
    }
  }
  if (actual != nullptr && actual->distanceFromOrigin < 1000)
    return actual;
  else
    return nullptr;
}

double fresnel(const vec3 &direction, const vec3 &normal, const double &inTransmissionCoefficient, const double &outTransmissionCoefficient)
{
  double cosi = dot(direction, normal);

  if (cosi < -1.0)
  {
    cosi = -1.0;
  }
  if (cosi > 1.0)
  {
    cosi = 1.0;
  }

  double etai = inTransmissionCoefficient;
  double etat = outTransmissionCoefficient;
  if (cosi > 0)
  {
    std::swap(etai, etat);
  }
  // Compute sini using Snell's law
  double sint = etai / etat * sqrt(std::max(0.0, 1 - cosi * cosi));
  // Total internal reflection
  if (sint >= 1)
  {
    return 1.0;
  }
  else
  {
    double cost = sqrt(std::max(0.0, 1 - sint * sint));
    cosi = abs(cosi);
    double Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    double Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    return (Rs * Rs + Rp * Rp) / 2;
  }
}

rayo obtenerRayoReflejado(const rayo &ray, const hitRecord &hitRec)
{
  return rayo(hitRec.position, hitRec.hitDir - 2 * dot(hitRec.hitDir, hitRec.normal) * hitRec.normal, 1);
}

rayo getTransmissionRay(const rayo &ray, const hitRecord &hitRec)
{
  bool outside = dot(hitRec.hitDir, hitRec.normal) < 0;
  vec3 bias = 0.01 * hitRec.normal;

  vec3 refractionRayOrig = outside ? hitRec.position - bias : hitRec.position + bias;

  double cosIncidenceNormal = dot(-hitRec.normal, unit_vector(hitRec.hitDir));

  if (cosIncidenceNormal < -1.0)
  {
    cosIncidenceNormal = -1.0;
  }
  if (cosIncidenceNormal > 1.0)
  {
    cosIncidenceNormal = 1.0;
  }

  double incidenceRefraction = ray.obtenerMedio();
  double outRefraction = hitRec.object->indice_refraccion;

  vec3 normal = -hitRec.normal;
  if (cosIncidenceNormal < 0)
  {
    cosIncidenceNormal = -cosIncidenceNormal;
  }
  else
  {
    normal = -hitRec.normal;
  }
  double eta = incidenceRefraction / outRefraction;
  double k = 1 - eta * eta * (1 - cosIncidenceNormal * cosIncidenceNormal);
  vec3 refractionDirection = k < 0 ? vec3(0, 0, 0) : eta * unit_vector(hitRec.hitDir) + (eta * cosIncidenceNormal - sqrt(k)) * normal;

  return rayo(refractionRayOrig, refractionDirection, hitRec.object->indice_refraccion);
}

color traza_RR(const rayo &ray, int depth);

color sombra_RR(const hitRecord *hitRec, const rayo &ray, int depth)
{
  if (mostrarRefleccionBYN)
  {
    return color(hitRec->object->indice_reflexion, hitRec->object->indice_reflexion, hitRec->object->indice_reflexion);
  }
  else if (mostrarRefraccionBYN)
  {
    return color(hitRec->object->indice_refraccion, hitRec->object->indice_refraccion, hitRec->object->indice_refraccion);
  }
  color diffuse = vec3(0, 0, 0);
  color specular = vec3(0, 0, 0);
  for (luz *luz : luces)
  {
    rayo rayo_s(hitRec->position, luz->getPosition() - hitRec->position, ray.obtenerMedio());
    double newIntensity = luz->getIntensity();
    color colorNextHit = vec3(0, 0, 0);

    if (dot(hitRec->normal, rayo_s.obtenerDireccion()) > 0)
    {
      bool completeShade = false;
      bool inShade = false;
      for (objeto *obj : objetos)
      {
        hitRecord *nextHit = obj->intersects(rayo_s);

        if (nextHit != nullptr && nextHit->distanceFromOrigin < (luz->getPosition() - nextHit->position).length())
        {

          if (nextHit->object != hitRec->object && nextHit->object->indice_transparencia == 1.0)
          {
            completeShade = true;
            break;
          }
          else if (nextHit->object != hitRec->object && nextHit->object->indice_transparencia > 0.0)
          {
            inShade = true;
            colorNextHit = nextHit->object->colorNormalizado;
            newIntensity *= 1 - nextHit->object->indice_transparencia;
          }
          else
          {
            newIntensity *= 1 - nextHit->object->indice_transparencia;
          }
        }
      }
      if (!inShade && !completeShade)
      {
        double angle = std::asin(dot(hitRec->normal, rayo_s.obtenerDireccion()));
        if (angle > 0)
        {
          diffuse += angle * newIntensity * hitRec->object->colorNormalizado;

          vec3 reflected_light = unit_vector(reflect(rayo_s.obtenerDireccion(), hitRec->normal));
          specular += hitRec->object->indice_reflexion * std::pow(std::max(0.0, dot(reflected_light, (-hitRec->hitDir))), hitRec->object->indice_especular) * newIntensity * 0.3 * vec3(1, 1, 1);
        }
      }
      if (inShade && !completeShade)
      {
        double angle = std::asin(dot(hitRec->normal, rayo_s.obtenerDireccion()));
        if (angle > 0)
        {
          diffuse += angle * newIntensity * hitRec->object->colorNormalizado * colorNextHit;

          vec3 reflected_light = unit_vector(reflect(rayo_s.obtenerDireccion(), hitRec->normal));
          specular += hitRec->object->indice_reflexion * std::pow(std::max(0.0, dot(reflected_light, (-hitRec->hitDir))), hitRec->object->indice_especular) * newIntensity * 0.3 * vec3(1, 1, 1);
        }
      }
    }
  }

  color final = vec3(0, 0, 0);

  if (depth < 5 && (hitRec->object->indice_reflexion > 0 || hitRec->object->indice_refraccion > 0))
  {
    double kr = fresnel(ray.obtenerDireccion(), hitRec->normal, ray.obtenerMedio(), hitRec->object->indice_refraccion);

    color colorReflexion = vec3(0, 0, 0);
    if (hitRec->object->indice_reflexion > 0)
    {
      rayo reflectiveRay = obtenerRayoReflejado(ray, *hitRec);
      colorReflexion = traza_RR(reflectiveRay, depth + 1);
    }

    color colorRefraccion = vec3(0, 0, 0);
    if (hitRec->object->indice_refraccion > 0)
    {
      if (kr < 1)
      {
        rayo refractionRay = getTransmissionRay(ray, *hitRec);
        colorRefraccion = traza_RR(refractionRay, depth + 1);
      }
    }

    final = (colorReflexion * kr + colorRefraccion * (1 - kr));
  }

  final += (hitRec->object->colorNormalizado * intensityAmbient + diffuse + specular) * hitRec->object->indice_transparencia;

  final[0] = final[0] * (final[0] < 1.f) + (final[0] >= 1);
  final[1] = final[1] * (final[1] < 1.f) + (final[1] >= 1);
  final[2] = final[2] * (final[2] < 1.f) + (final[2] >= 1);

  return final;
}

color traza_RR(const rayo &ray, int depth)
{
  hitRecord *hitRec = closestIntersection(ray);
  if (hitRec != nullptr)
  {
    return sombra_RR(hitRec, ray, depth);
  }
  else
  {
    return color(0, 0, 0);
  }
}

double degreesToRadians(double degrees)
{
  return degrees * (M_PI / 180.0);
}

double clamp(double x, double min, double max)
{
  if (x < min)
    return min;
  if (x > max)
    return max;
  return x;
}


void saveAndShowGeneratedImage(std::vector<color> imagenFinal) {
    FIBITMAP* image = FreeImage_Allocate(width, height, 24);

    if (!image) {
        std::cerr << "Failed to allocate image" << std::endl;
        FreeImage_DeInitialise();
        throw new exception();
    }

    // Set the pixels
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            color& result = imagenFinal[i * width + j];

            int rbyte = int(256 * clamp(result.x(), 0.0, 0.999));
            int gbyte = int(256 * clamp(result.y(), 0.0, 0.999));
            int bbyte = int(256 * clamp(result.z(), 0.0, 0.999));

            RGBQUAD color;
            color.rgbRed = rbyte;
            color.rgbGreen = gbyte;
            color.rgbBlue = bbyte;
            FreeImage_SetPixelColor(image, j, height - i, &color);
        }
    }

    std::time_t now = std::time(nullptr);
    string timeString = std::to_string(now);
    string fileName = "./results/image_" + timeString + ".png";
    const char* imagePath = fileName.data();  // Replace with your image path

    // Save the image
    if (FreeImage_Save(FIF_PNG, image, imagePath, 0)) {
        std::cout << "Image saved successfully!" << std::endl;
    }
    else {
        std::cerr << "Failed to save image" << std::endl;
    }

    // Free resources
    FreeImage_Unload(image);
    FreeImage_DeInitialise();



    // Command to open image file based on OS
    #ifdef _WIN32
        std::string command = "start " + std::string(imagePath);
    #elif __APPLE__
        std::string command = "open " + std::string(imagePath);
    #else
        std::string command = "xdg-open " + std::string(imagePath);
    #endif

    // Execute the command
    int result = std::system(command.c_str());
    if (result != 0) {
        std::cerr << "Failed to open image file." << std::endl;
        throw new exception();
    }
}


void dibujarEscena()
{
  float aspect = 5.0 / 5.0;
  height = int(width / aspect);
  double fov = 60;

  std::vector<color> imagenFinal;
  cameraLookAt = point3(0, 4.5, -4.5);
  cameraUp = vec3(0, 1, 0);

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
  pixelDelta_u = window_u / width;
  pixelDelta_v = window_v / height;

  // Calculate the location of the upper left pixel.
  vec3 windowUpperLeft = cameraPosition - (windowDistance * w) - window_u / 2 - window_v / 2;
  upperLeftPixel = windowUpperLeft + 0.25 * (pixelDelta_u + pixelDelta_v);


  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {

      color pixel(0.0, 0.0, 0.0);

      vec3 pixelCenter = upperLeftPixel + (j * pixelDelta_u) + (i * pixelDelta_v);
      vec3 rayDirection = pixelCenter - cameraPosition;
      rayo ray(cameraPosition, rayDirection, 1.0);

      pixel += traza_RR(ray, 1);

      pixelCenter = upperLeftPixel + (j * pixelDelta_u) + ((i + 0.5) * pixelDelta_v);
      rayDirection = pixelCenter - cameraPosition;
      rayo ray2(cameraPosition, rayDirection, 1.0);

      pixel += traza_RR(ray2, 1);

      pixelCenter = upperLeftPixel + ((j + 0.5) * pixelDelta_u) + (i * pixelDelta_v);
      rayDirection = pixelCenter - cameraPosition;
      rayo ray3(cameraPosition, rayDirection, 1.0);

      pixel += traza_RR(ray3, 1);

      pixelCenter = upperLeftPixel + ((j + 0.5) * pixelDelta_u) + ((i + 0.5) * pixelDelta_v);
      rayDirection = pixelCenter - cameraPosition;
      rayo ray4(cameraPosition, rayDirection, 1.0);

      pixel += traza_RR(ray4, 1);

      color result = pixel / 4;

      imagenFinal.push_back(result);
    }
  }
  saveAndShowGeneratedImage(imagenFinal);
}

malla *getMalla(vec3 position, double length, double height, double prof, color color, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia)
{

  std::vector<triangulo *> triangulos;

  int triangles_per_unit = 1;

  // Caras delantera y trasera
  for (double i = 0; i < length * triangles_per_unit; i++)
  {
    for (double j = 0; j < height * triangles_per_unit; j++)
    {
      double xMin = position.x() + i / triangles_per_unit;
      double xMax = position.x() + (i + 1) / triangles_per_unit;
      double yMin = position.y() + j / triangles_per_unit;
      double yMax = position.y() + (j + 1) / triangles_per_unit;

      // Cara delantera
      triangulos.push_back(new triangulo(vec3(xMin, yMin, position.z()), vec3(xMax, yMin, position.z()), vec3(xMax, yMax, position.z())));
      triangulos.push_back(new triangulo(vec3(xMin, yMin, position.z()), vec3(xMax, yMax, position.z()), vec3(xMin, yMax, position.z())));

      // Cara trasera
      triangulos.push_back(new triangulo(vec3(xMin, yMin, position.z() - prof), vec3(xMax, yMin, position.z() - prof), vec3(xMin, yMax, position.z() - prof)));
      triangulos.push_back(new triangulo(vec3(xMax, yMin, position.z() - prof), vec3(xMax, yMax, position.z() - prof), vec3(xMin, yMax, position.z() - prof)));
    }
  }

  for (double i = 0; i < length * triangles_per_unit; i++)
  {
    for (double k = 0; k < prof * triangles_per_unit; k++)
    {
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

  for (double j = 0; j < height * triangles_per_unit; j++)
  {
    for (double k = 0; k < prof * triangles_per_unit; k++)
    {
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
  return new malla(triangulos, color, indice_reflexion, indice_refraccion, indice_especular, indice_transparencia);
}

void cargarObjetosYLuces()
{

  // PLANO
  // Cara inferior
  objetos.push_back(new plano(vec3(0, 1, 0), point3(0, 0, -4.5), vec3(0, 0, -1), 9, 9, color(.7, .7, .7), 0.0, 0.0, .7, 1.0));
  // Cara superior
  objetos.push_back(new plano(vec3(0, -1, 0), point3(0, 9, -4.5), vec3(0, 0, -1), 9, 9, color(0.2, 0.2, 0.2), 0.0, 0.0, 0.9, 1.0));
  // Cara trasera
  objetos.push_back(new plano(vec3(0, 0, 1), point3(0, 4.5, -9), vec3(0, 0, 1), 9, 9, color(.7, .7, .7), 0.0, 0.0, 0.7, 1.0));
  // Cara derecha
  objetos.push_back(new plano(vec3(-1, 0, 0), point3(4.5, 4.5, -4.5), vec3(0, 0, 1), 9, 9, color(0.0, 0.4, 0.0), 0.0, 0.0, 0.7, 1.0));
  // Cara izquierda
  objetos.push_back(new plano(vec3(1, 0, 0), point3(-4.5, 4.5, -4.5), vec3(0, 0, .7), 9, 9, color(0.7, 0.0, 0.0), 0.0, 0.0, 0.7, 1.0));

  objetos.push_back(new esfera(vec3(esferaGrandeX, esferaGrandeY, esferaGrandeZ), radioEsferaGrande, color(rColorEG, gColorEG, bColorEG),                                    // Esfera de vidrio rosada
                                    indiceReflexionEG, indiceRefraccionEG, indiceTransparenciaEG, indiceEspecularEG));         

  objetos.push_back(new esfera(vec3(esferaChicaX, esferaChicaY, esferaChicaZ), radioEsferaChica, color(rColorEC, gColorEC, bColorEC),                                       // Esfera espejo
      indiceReflexionEC, indiceRefraccionEC, indiceTransparenciaEC, indiceEspecularEC));
  
  objetos.push_back(getMalla(vec3(mesaX, mesaY, mesaZ), mesaAncho, mesaAlto, mesaProfundidad, color(rColorMesa, gColorMesa, bColorMesa),                                    // Mesa
                                indiceReflexionMesa, indiceRefraccionMesa, indiceEspecularMesa, indiceTransparenciaMesa));

  objetos.push_back(new cilindro(vec3(cilindroX, cilindroY, cilindroZ), radioCilindro, altoCilindro, vec3(0, 1, 0), color(rColorCilindro, gColorCilindro, bColorCilindro),  // Cilindro
                    indiceReflexionCilindro, indiceRefraccionCilindro, indiceEspecularCilindro, indiceTransparenciaCilindro));  

  luces.push_back(new luz(vec3(luz1X, luz1Y, luz1Z), luz1Intensidad));
  luces.push_back(new luz(vec3(luz2X, luz2Y, luz2Z), luz2Intensidad));
}

void cargarParametrosDesdeXML() {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file("../parametros.xml");

    width = doc.child("scene").child("window").attribute("width").as_int();

    esferaGrandeX = doc.child("scene").child("objects").child("esferaGrande").child("posicion").attribute("X").as_double();
    esferaGrandeY = doc.child("scene").child("objects").child("esferaGrande").child("posicion").attribute("Y").as_double();
    esferaGrandeZ = doc.child("scene").child("objects").child("esferaGrande").child("posicion").attribute("Z").as_double();
    radioEsferaGrande = doc.child("scene").child("objects").child("esferaGrande").child("dimensiones").attribute("radio").as_double();
    rColorEG = doc.child("scene").child("objects").child("esferaGrande").child("color").attribute("r").as_double();
    gColorEG = doc.child("scene").child("objects").child("esferaGrande").child("color").attribute("g").as_double();
    bColorEG = doc.child("scene").child("objects").child("esferaGrande").child("color").attribute("b").as_double();
    indiceTransparenciaEG = doc.child("scene").child("objects").child("esferaGrande").child("parametros").attribute("itransparencia").as_double();
    indiceReflexionEG = doc.child("scene").child("objects").child("esferaGrande").child("parametros").attribute("ireflexion").as_double();
    indiceEspecularEG = doc.child("scene").child("objects").child("esferaGrande").child("parametros").attribute("iespecular").as_double();
    indiceRefraccionEG = doc.child("scene").child("objects").child("esferaGrande").child("parametros").attribute("irefraccion").as_double();


    esferaChicaX = doc.child("scene").child("objects").child("esferaChica").child("posicion").attribute("X").as_double();
    esferaChicaY = doc.child("scene").child("objects").child("esferaChica").child("posicion").attribute("Y").as_double();
    esferaChicaZ = doc.child("scene").child("objects").child("esferaChica").child("posicion").attribute("Z").as_double();
    radioEsferaChica = doc.child("scene").child("objects").child("esferaChica").child("dimensiones").attribute("radio").as_double();
    rColorEC = doc.child("scene").child("objects").child("esferaChica").child("color").attribute("r").as_double();
    gColorEC = doc.child("scene").child("objects").child("esferaChica").child("color").attribute("g").as_double();
    bColorEC = doc.child("scene").child("objects").child("esferaChica").child("color").attribute("b").as_double();
    indiceTransparenciaEC = doc.child("scene").child("objects").child("esferaChica").child("parametros").attribute("itransparencia").as_double();
    indiceReflexionEC = doc.child("scene").child("objects").child("esferaChica").child("parametros").attribute("ireflexion").as_double();
    indiceEspecularEC = doc.child("scene").child("objects").child("esferaChica").child("parametros").attribute("iespecular").as_double();
    indiceRefraccionEC = doc.child("scene").child("objects").child("esferaChica").child("parametros").attribute("irefraccion").as_double();

    cilindroX = doc.child("scene").child("objects").child("cilindro").child("posicion").attribute("X").as_double();
    cilindroY = doc.child("scene").child("objects").child("cilindro").child("posicion").attribute("Y").as_double();
    cilindroZ = doc.child("scene").child("objects").child("cilindro").child("posicion").attribute("Z").as_double();
    altoCilindro = doc.child("scene").child("objects").child("cilindro").child("dimensiones").attribute("alto").as_double();
    radioCilindro = doc.child("scene").child("objects").child("cilindro").child("dimensiones").attribute("radio").as_double();
    rColorCilindro = doc.child("scene").child("objects").child("cilindro").child("color").attribute("r").as_double();
    gColorCilindro = doc.child("scene").child("objects").child("cilindro").child("color").attribute("g").as_double();
    bColorCilindro = doc.child("scene").child("objects").child("cilindro").child("color").attribute("b").as_double();
    indiceTransparenciaCilindro = doc.child("scene").child("objects").child("cilindro").child("parametros").attribute("itransparencia").as_double();
    indiceReflexionCilindro = doc.child("scene").child("objects").child("cilindro").child("parametros").attribute("ireflexion").as_double();
    indiceEspecularCilindro = doc.child("scene").child("objects").child("cilindro").child("parametros").attribute("iespecular").as_double();
    indiceRefraccionCilindro = doc.child("scene").child("objects").child("cilindro").child("parametros").attribute("irefraccion").as_double();

    mesaX = doc.child("scene").child("objects").child("mesa").child("posicion").attribute("X").as_double();
    mesaY = doc.child("scene").child("objects").child("mesa").child("posicion").attribute("Y").as_double();
    mesaZ = doc.child("scene").child("objects").child("mesa").child("posicion").attribute("Z").as_double();
    mesaAncho = doc.child("scene").child("objects").child("mesa").child("dimensiones").attribute("ancho").as_double();
    mesaAlto = doc.child("scene").child("objects").child("mesa").child("dimensiones").attribute("alto").as_double();
    mesaProfundidad = doc.child("scene").child("objects").child("mesa").child("dimensiones").attribute("profundidad").as_double();
    rColorMesa = doc.child("scene").child("objects").child("mesa").child("color").attribute("r").as_double();
    gColorMesa = doc.child("scene").child("objects").child("mesa").child("color").attribute("g").as_double();
    bColorMesa = doc.child("scene").child("objects").child("mesa").child("color").attribute("b").as_double();
    indiceTransparenciaMesa = doc.child("scene").child("objects").child("mesa").child("parametros").attribute("itransparencia").as_double();
    indiceReflexionMesa = doc.child("scene").child("objects").child("mesa").child("parametros").attribute("ireflexion").as_double();
    indiceEspecularMesa = doc.child("scene").child("objects").child("mesa").child("parametros").attribute("iespecular").as_double();
    indiceRefraccionMesa = doc.child("scene").child("objects").child("mesa").child("parametros").attribute("irefraccion").as_double();

    intensityAmbient = doc.child("scene").child("lights").child("luzAmbiente").attribute("intensity").as_double();
    luz1X = doc.child("scene").child("lights").child("light1").attribute("X").as_double();
    luz1Y = doc.child("scene").child("lights").child("light1").attribute("Y").as_double();
    luz1Z = doc.child("scene").child("lights").child("light1").attribute("Z").as_double();
    luz1Intensidad = doc.child("scene").child("lights").child("light1").attribute("intensity").as_double();

    luz2X = doc.child("scene").child("lights").child("light2").attribute("X").as_double();
    luz2Y = doc.child("scene").child("lights").child("light2").attribute("Y").as_double();
    luz2Z = doc.child("scene").child("lights").child("light2").attribute("Z").as_double();
    luz2Intensidad = doc.child("scene").child("lights").child("light2").attribute("intensity").as_double();

    cameraPosition = vec3(doc.child("scene").child("camera").attribute("X").as_double(),
        doc.child("scene").child("camera").attribute("Y").as_double(),
        doc.child("scene").child("camera").attribute("Z").as_double());
}

int main()
{

  cargarParametrosDesdeXML();
  
  cargarObjetosYLuces();

  dibujarEscena();
}