#include "Euler.h"

namespace ClothMesh {
	extern void updateClothMesh(float* array_data);
	extern const int numCols;
	extern const int numRows;
}

void Euler::updateParticles(ParticleSystem ps, glm::vec3* forces, glm::vec3 gravity, float dt, int idx, float elasticity, float friction, float sphereRadius, glm::vec3 spherePos, bool sphere, bool fixPosition)
{

	glm::vec3 newPosition;
	glm::vec3 newVelocity;
	//glm::vec3 acc = forces[idx] + gravity; //com que la massa es 1, per la 2a llei de newton f=m*a, força = acceleracio
	glm::vec3 acc = gravity + forces[idx]; //com que la massa es 1, per la 2a llei de newton f=m*a, força = acceleracio

	newPosition = ps.positions[idx] + dt * ps.extra[idx];
	newVelocity = ps.extra[idx] + dt * acc;

	//colisions
	//plane collision
	// X
	if (newPosition.x <= -5 || newPosition.x >= 5) {
		float aux = newPosition.x < 0 ? aux = -1 : aux = 1;
		float D = findD(aux, 0.f, 0.f, glm::vec3(5 * aux, 0.f, 0.f));

		newPosition = newPosition - (1 + elasticity) * (glm::dot(glm::vec3(aux, 0.f, 0.f), newPosition) + D) * glm::vec3(aux, 0.f, 0.f);
		newVelocity = newVelocity - (1 + elasticity) * glm::dot(glm::vec3(aux, 0.f, 0.f), newVelocity) * glm::vec3(aux, 0.f, 0.f);

		glm::vec3 tangentialVelocity = newVelocity - ((glm::vec3(aux, 0.f, 0.f) * newVelocity) * glm::vec3(aux, 0.f, 0.f));
		newVelocity = newVelocity - friction * tangentialVelocity;
	}

	//Y
	if (newPosition.y <= 0 || newPosition.y >= 10) {
		float aux = newPosition.y < 0 ? aux = -1 : aux = 1;
		float D = findD(0.f, aux, 0.f, glm::vec3(0.f, 5 * aux + 5, 0.f));

		newPosition = newPosition - (1 + elasticity) * (glm::dot(glm::vec3(0.f, aux, 0.f), newPosition) + D) * glm::vec3(0.f, aux, 0.f);
		newVelocity = newVelocity - (1 + elasticity) * glm::dot(glm::vec3(0.f, aux, 0.f), newVelocity) * glm::vec3(0.f, aux, 0.f);

		glm::vec3 tangentialVelocity = newVelocity - ((glm::vec3(0.f, aux, 0.f) * newVelocity) * glm::vec3(0.f, aux, 0.f));

	}
	//Z
	if (newPosition.z <= -5 || newPosition.z >= 5) {
		float aux = newPosition.z < 0 ? aux = -1 : aux = 1;
		float D = findD(0.f, 0.f, aux, glm::vec3(0.f, 0.f, 5 * aux));

		newPosition = newPosition - (1 + elasticity) * (glm::dot(glm::vec3(0.f, 0.f, aux), newPosition) + D) * glm::vec3(0.f, 0.f, aux);
		newVelocity = newVelocity - (1 + elasticity) * glm::dot(glm::vec3(0.f, 0.f, aux), newVelocity) * glm::vec3(0.f, 0.f, aux);

		glm::vec3 tangentialVelocity = newVelocity - ((glm::vec3(0.f, 0.f, aux) * newVelocity) * glm::vec3(0.f, 0.f, aux));
		newVelocity = newVelocity - friction * tangentialVelocity;
	}

	//sphere collision
	if (sphere && glm::distance(newPosition, spherePos) <= sphereRadius)
	{
		float a = pow(ps.extra[idx].x, 2) + pow(ps.extra[idx].y, 2) + pow(ps.extra[idx].z, 2);

		float b = 2.f * ps.positions[idx].x * ps.extra[idx].x - 2.f * spherePos.x * ps.extra[idx].x +
			2.f * ps.positions[idx].y * ps.extra[idx].y - 2.f * spherePos.y * ps.extra[idx].y +
			2.f * ps.positions[idx].z * ps.extra[idx].z - 2.f * spherePos.z * ps.extra[idx].z;

		float c = pow(ps.positions[idx].x, 2) + pow(spherePos.x, 2) - 2. * ps.positions[idx].x * spherePos.x +
			pow(ps.positions[idx].y, 2) + pow(spherePos.y, 2) - 2. * ps.positions[idx].y * spherePos.y +
			pow(ps.positions[idx].z, 2) + pow(spherePos.z, 2) - 2. * ps.positions[idx].z * spherePos.z
			- pow(sphereRadius, 2);

		float lambda = quadraticEquation(a, b, c);

		glm::vec3 collisionPoint = ps.positions[idx] + lambda * ps.extra[idx];
		glm::vec3 collisionNormal = normalize(collisionPoint - spherePos);

		float D = findD(collisionNormal.x, collisionNormal.y, collisionNormal.z, collisionPoint);

		newPosition = newPosition - (1 + elasticity) * (glm::dot(collisionNormal, newPosition) + D) * collisionNormal;
		newVelocity = newVelocity - (1 + elasticity) * glm::dot(collisionNormal, newVelocity) * collisionNormal;

		glm::vec3 tangentialVelocity = newVelocity - ((collisionNormal * newVelocity) * collisionNormal);
		newVelocity = newVelocity - friction * tangentialVelocity;
	}

	if (idx != 0 && idx != ClothMesh::numCols - 1) {
		ps.positions[idx] = newPosition;
		ps.extra[idx] = newVelocity;
	}
	else if (!fixPosition) {
		ps.positions[idx] = newPosition;
		ps.extra[idx] = newVelocity;
	}
}

