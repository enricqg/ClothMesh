#include "Mesh.h"

Mesh::Mesh() : Mesh(ClothMesh::numCols, ClothMesh::numRows, 1, 1, 1, 1, 1 ,1 ,1, false) {}; 

Mesh::Mesh(int _width, int _height, float k_e_struct, float k_d_struct, float k_e_shear, float k_d_shear, float k_e_bend, float k_d_bend, float rest_distance, bool isEuler) 
	: width(_width), height(_height), ParticleSystem(_width* _height)
	, k_e_struct(k_e_struct),k_d_struct(k_d_struct), k_e_shear(k_e_shear), k_d_shear(k_d_shear),k_e_bend(k_e_bend), k_d_bend(k_d_bend), rest_distance(rest_distance) {

	float y = 8.0f;
	//initialize mesh positions
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			positions[get_index(row, col)] = glm::vec3(row * rest_distance, y, col * rest_distance);
			if(isEuler) extra[get_index(row, col)] = glm::vec3(0.0f, 0.0f, 0.0f);
			else extra[get_index(row, col)] = glm::vec3(row * rest_distance, y, col * rest_distance);
		}
	}
}

glm::vec3* Mesh::Get_spring_forces(bool fixPosition, bool isEuler, float dt, float _k_e_struct, float _k_d_struct, float _k_e_shear, float _k_d_shear, float _k_e_bend, float _k_d_bend, float _rest_distance) {
	glm::vec3* forces = new glm::vec3[width * height];

	for (int i = 0; i < width * height; i++)
	{
		forces[i] = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	glm::vec3 tempForce(0.0f, 0.0f, 0.0f);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {

			//structural
			if (j < width - 1) {

				ApplyConstraints(get_index(i, j), get_index(i , j + 1), _rest_distance, fixPosition);
				tempForce = get_spring_force(isEuler, dt, positions[get_index(i, j)], positions[get_index(i, j + 1)], extra[get_index(i, j)], extra[get_index(i, j + 1)], _k_e_struct, _k_d_struct, _rest_distance);
				forces[get_index(i, j)] += tempForce;
				forces[get_index(i, j + 1)] -= tempForce;
			}
			if (i < height - 1) {
				ApplyConstraints(get_index(i, j), get_index(i + 1, j), _rest_distance, fixPosition);
				tempForce = get_spring_force(isEuler, dt, positions[get_index(i, j)], positions[get_index(i + 1, j)], extra[get_index(i, j)], extra[get_index(i + 1, j)], _k_e_struct, _k_d_struct, _rest_distance);
				forces[get_index(i, j)] += tempForce;
				forces[get_index(i + 1, j)] -= tempForce;
			}

			//shear
			if (j < width - 1 && i < height - 1) {
				ApplyConstraints(get_index(i, j), get_index(i + 1, j + 1), sqrt(_rest_distance * _rest_distance + _rest_distance * _rest_distance), fixPosition);
				tempForce = get_spring_force(isEuler, dt, positions[get_index(i, j)], positions[get_index(i + 1, j + 1)], extra[get_index(i, j)], extra[get_index(i + 1, j + 1)], _k_e_shear, _k_d_shear, sqrt(_rest_distance * _rest_distance + _rest_distance * _rest_distance));
				forces[get_index(i, j)] += tempForce;
				forces[get_index(i + 1, j + 1)] -= tempForce;

				ApplyConstraints(get_index(i + 1, j), get_index(i, j + 1), sqrt(_rest_distance * _rest_distance + _rest_distance * _rest_distance), fixPosition);
				tempForce = get_spring_force(isEuler, dt, positions[get_index(i + 1, j)], positions[get_index(i, j + 1)], extra[get_index(i + 1, j)], extra[get_index(i, j + 1)], _k_e_shear, _k_d_shear, sqrt(_rest_distance * _rest_distance + _rest_distance * _rest_distance));
				forces[get_index(i + 1, j)] += tempForce;
				forces[get_index(i, j + 1)] -= tempForce;

			}

			//bend
			if (j < width - 2) {

				tempForce = get_spring_force(isEuler, dt, positions[get_index(i, j)], positions[get_index(i, j + 2)], extra[get_index(i, j)], extra[get_index(i, j + 2)], _k_e_bend, _k_d_bend, _rest_distance * 2);
				forces[get_index(i, j)] += tempForce;
				forces[get_index(i, j + 2)] -= tempForce;
			}
			if (i < height - 2) {
				tempForce = get_spring_force(isEuler, dt, positions[get_index(i, j)], positions[get_index(i + 2, j)], extra[get_index(i, j)], extra[get_index(i + 2, j)], _k_e_bend, _k_d_bend, _rest_distance * 2);
				forces[get_index(i, j)] += tempForce;
				forces[get_index(i + 2, j)] -= tempForce;
			}
		}
	}
	return forces;
}

void Mesh::ResetMesh(int _width, int _height, float rest_distance, bool isEuler)
{
	float y = 8.0f;
	//initialize mesh positions
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			positions[get_index(row, col)] = glm::vec3(row * rest_distance, y, col * rest_distance);
			if (isEuler) extra[get_index(row, col)] = glm::vec3(0.0f, 0.0f, 0.0f);
			else extra[get_index(row, col)] = glm::vec3(row * rest_distance, y, col * rest_distance);
		}
	}
}

void Mesh::ApplyConstraints(int i, int j, float rest_distance, bool fixPosition)
{
	//mirar 
    //https://gamedevelopment.tutsplus.com/tutorials/simulate-tearable-cloth-and-ragdolls-with-simple-verlet-integration--gamedev-519


	glm::vec3 diff = positions[i] - positions[j];
	float d = glm::length(diff);

	float difference = (rest_distance - d) / d;

	glm::vec3 translate = diff * 0.1f * difference;

	if (i != 0 && i != ClothMesh::numCols - 1) positions[i] += translate;
	else if (!fixPosition) positions[i] += translate;
	if (j != 0 && j != ClothMesh::numCols - 1 ) positions[j] -= translate;
	else if (!fixPosition) positions[j] -= translate;
}

int Mesh::get_index(int row, int col) {
	return row * width + col;
}

glm::vec3 Mesh::get_spring_force(bool isEuler, float dt, glm::vec3 p1, glm::vec3 p2, glm::vec3 extra1, glm::vec3 extra2, float k_e, float k_d, float rest_distance)
{
	//verlet (cal calcular velocitat)
	if (!isEuler) {
		glm::vec3 v1, v2;
		v1 = (p1 - extra1) / dt;
		v2 = (p2 - extra2) / dt;
		return -(k_e * (glm::distance(p1, p2) - rest_distance) + k_d * glm::dot((v1 - v2), glm::normalize(p1 - p2))) * glm::normalize(p1 - p2);
	}
	//euler
	else return -(k_e * (glm::distance(p1, p2) - rest_distance) + k_d * glm::dot((extra1 - extra2), glm::normalize(p1 - p2))) * glm::normalize(p1 - p2);
}