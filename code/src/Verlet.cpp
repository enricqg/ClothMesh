#include "Verlet.h"

namespace ClothMesh {
	extern void updateClothMesh(float* array_data);
	extern const int numCols;
	extern const int numRows;
}

void Verlet::updateParticles(ParticleSystem ps, glm::vec3* forces, glm::vec3 gravity, float dt, int idx, float elasticity, float friction, float sphereRadius, glm::vec3 spherePos, bool sphere, bool fixPosition)
{

	glm::vec3 newPosition;
	glm::vec3 newVelocity;
	glm::vec3 previousPosition;
	//glm::vec3 acc = forces[idx] + gravity; //com que la massa es 1, per la 2a llei de newton f=m*a, força = acceleracio
	glm::vec3 acc = gravity + forces[idx]; //com que la massa es 1, per la 2a llei de newton f=m*a, força = acceleracio

	//newPosition = ps.positions[idx] + dt * ps.extra[idx];
	//newVelocity = ps.extra[idx] + dt * acc;

	newPosition = ps.positions[idx] + (ps.positions[idx] - ps.extra[idx]) + (acc * dt * dt);
	newVelocity = (newPosition - ps.positions[idx]) / dt;

	previousPosition = ps.positions[idx];

	//colisions
	//plane collision
	// X
	if (newPosition.x <= -5 || newPosition.x >= 5) {
		float aux = newPosition.x < 0 ? aux = -1 : aux = 1;
		float D = findD(aux, 0.f, 0.f, glm::vec3(5 * aux, 0.f, 0.f));

		newPosition = newPosition - (1 + elasticity) * (glm::dot(glm::vec3(aux, 0.f, 0.f), newPosition) + D) * glm::vec3(aux, 0.f, 0.f);

		previousPosition = previousPosition - (1 + elasticity) * (glm::dot(glm::vec3(aux, 0.f, 0.f), previousPosition) + D) * glm::vec3(aux, 0.f, 0.f);


		newVelocity = (newPosition - previousPosition) / dt;

		glm::vec3 newVelocity_n = (newVelocity * glm::vec3(aux, 0.f, 0.f)) * glm::vec3(aux, 0.f, 0.f);
		glm::vec3 newVelocity_tg = newVelocity - newVelocity_n;

		newVelocity -= friction * newVelocity_tg;

		previousPosition = newPosition - newVelocity * dt;
	}

	//Y
	if (newPosition.y <= 0 || newPosition.y >= 10) {
		float aux = newPosition.y < 0 ? aux = -1 : aux = 1;
		float D = findD(0.f, aux, 0.f, glm::vec3(0.f, 5 * aux + 5, 0.f));

		newPosition = newPosition - (1 + elasticity) * (glm::dot(glm::vec3(0.f, aux, 0.f), newPosition) + D) * glm::vec3(0.f, aux, 0.f);
		previousPosition = previousPosition - (1 + elasticity) * (glm::dot(glm::vec3(0.f, aux, 0.f), previousPosition) + D) * glm::vec3(0.f, aux, 0.f);

		newVelocity = (newPosition - previousPosition) / dt;

		glm::vec3 newVelocity_n = (newVelocity * glm::vec3(0.f, aux, 0.f)) * glm::vec3(0.f, aux, 0.f);
		glm::vec3 newVelocity_tg = newVelocity - newVelocity_n;

		newVelocity -= friction * newVelocity_tg;

		previousPosition = newPosition - newVelocity * dt;
	}
	//Z
	if (newPosition.z <= -5 || newPosition.z >= 5) {
		float aux = newPosition.z < 0 ? aux = -1 : aux = 1;
		float D = findD(0.f, 0.f, aux, glm::vec3(0.f, 0.f, 5 * aux));

		newPosition = newPosition - (1 + elasticity) * (glm::dot(glm::vec3(0.f, 0.f, aux), newPosition) + D) * glm::vec3(0.f, 0.f, aux);
		previousPosition = previousPosition - (1 + elasticity) * (glm::dot(glm::vec3(0.f, 0.f, aux), previousPosition) + D) * glm::vec3(0.f, 0.f, aux);

		newVelocity = (newPosition - previousPosition) / dt;

		glm::vec3 newVelocity_n = (newVelocity * glm::vec3(0.f, 0.f, aux)) * glm::vec3(0.f, 0.f, aux);
		glm::vec3 newVelocity_tg = newVelocity - newVelocity_n;

		newVelocity -= friction * newVelocity_tg;

		previousPosition = newPosition - newVelocity * dt;
	}

	//sphere collision
	if (sphere && glm::distance(newPosition, spherePos) <= sphereRadius)
	{
		float a = pow(newVelocity.x, 2) + pow(newVelocity.y, 2) + pow(newVelocity.z, 2);

		float b = 2.f * newPosition.x * newVelocity.x - 2.f * spherePos.x * newVelocity.x +
			2.f * newPosition.y * newVelocity.y - 2.f * spherePos.y * newVelocity.y +
			2.f * newPosition.z * newVelocity.z - 2.f * spherePos.z * newVelocity.z;

		float c = pow(newPosition.x, 2) + pow(spherePos.x, 2) - 2. * newPosition.x * spherePos.x +
			pow(newPosition.y, 2) + pow(spherePos.y, 2) - 2. * newPosition.y * spherePos.y +
			pow(newPosition.z, 2) + pow(spherePos.z, 2) - 2. * newPosition.z * spherePos.z
			- pow(sphereRadius, 2);

		float lambda = quadraticEquation(a, b, c);

		glm::vec3 collisionPoint = ps.positions[idx] + lambda * newVelocity;
		glm::vec3 collisionNormal = normalize(collisionPoint - spherePos);

		float D = findD(collisionNormal.x, collisionNormal.y, collisionNormal.z, collisionPoint);

		newPosition = newPosition - (1 + elasticity) * (glm::dot(collisionNormal, newPosition) + D) * collisionNormal;
		previousPosition = previousPosition - (1 + elasticity) * (glm::dot(collisionNormal, previousPosition) + D) * collisionNormal;

		newVelocity = (newPosition - previousPosition) / dt;

		glm::vec3 newVelocity_n = (newVelocity * collisionNormal) * collisionNormal;
		glm::vec3 newVelocity_tg = newVelocity - newVelocity_n;

		newVelocity -= friction * newVelocity_tg;

		previousPosition = newPosition - newVelocity * dt;
	}

	if (idx != 0 && idx != ClothMesh::numCols - 1) {
		ps.extra[idx] = previousPosition;
		ps.positions[idx] = newPosition;
	}
	else if (!fixPosition) {
		ps.extra[idx] = previousPosition;
		ps.positions[idx] = newPosition;
	}

}

