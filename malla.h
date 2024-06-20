#pragma once

#include "triangulo.h"
#include "objeto.h"

#include <vector>

class malla : public objeto
{
private:
	std::vector<triangulo*> triangulos;
public:
	malla(std::vector<triangulo*> triangulos, color ambient, color especular, color difuso, double indice_reflexion, double indice_refraccion, double indice_especular, double indice_transparencia);
	hitRecord* intersects(const rayo& ray);
};

