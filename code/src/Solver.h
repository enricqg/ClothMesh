#pragma once
#include "ParticleSystem.h"
#include <glm/vec3.hpp>
#include <glm/gtx/string_cast.hpp>


class Solver
{
public:
	virtual void updateParticles(ParticleSystem ps, glm::vec3* forces, glm::vec3 gravity, float dt, int idx, float elasticity, float friction, float sphereRadius, glm::vec3 spherePos, bool sphere, bool fixPosition) {};

protected:
	float findD(float A, float B, float C, glm::vec3 point);

	float quadraticEquation(float a, float b, float c);
};

