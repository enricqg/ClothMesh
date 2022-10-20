#pragma once
#include "Solver.h"
class Verlet : public Solver 
{
public:
	void updateParticles(ParticleSystem ps, glm::vec3* forces, glm::vec3 gravity, float dt, int idx, float elasticity, float friction, float sphereRadius, glm::vec3 spherePos, bool sphere, bool fixPosition); //pos + prev_pos
};
