#pragma once
#include <glm/vec3.hpp>
#include <glm/glm.hpp>

class ParticleSystem {
public:
	ParticleSystem(int _numParticles);
	int numParticles;
	glm::vec3* positions;
	glm::vec3* extra;
private:
};