#pragma once
#include "ParticleSystem.h"
#include <map>

namespace ClothMesh {
	extern void updateClothMesh(float* array_data);
	extern const int numCols;
	extern const int numRows;
}
extern bool renderCloth;
/////////////

class Mesh: public ParticleSystem
{
public:
	int width, height;
	float k_e_struct, k_d_struct;
	float k_e_shear, k_d_shear;
	float k_e_bend, k_d_bend;
	float rest_distance;

	Mesh();
	Mesh(int _width, int _height, float k_e_struct, float k_d_struct, float k_e_shear, float k_d_shear,float k_e_bend, float k_d_bend, float rest_distance, bool isEuler);
	//calcula les spring forces de tota la mesh
	glm::vec3* Get_spring_forces(bool fixPosition, bool isEuler, float dt, float _k_e_struct, float _k_d_struct, float _k_e_shear, float _k_d_shear, float _k_e_bend, float _k_d_bend, float _rest_distance);
	void ResetMesh(int _width, int _height, float rest_distance, bool isEuler);

private:
	void ApplyConstraints(int i, int j, float rest_distance, bool fixPosition);
	int get_index(int row, int col);
	//calcula la força d'una spring
	glm::vec3 get_spring_force(bool isEuler, float dt, glm::vec3 p1, glm::vec3 p2, glm::vec3 extra1, glm::vec3 extra2, float k_e, float k_d, float rest_distance);
};

