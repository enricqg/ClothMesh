#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include "ParticleSystem.h"
#include "Mesh.h"
#include "Solver.h"
#include "Verlet.h"
#include "Euler.h"


float randomFloat(float a, float b)
{
	return a + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (b - a)));
}

/////////Forward declarations
namespace LilSpheres {
	extern const int maxParticles;
	extern int firstParticleIdx;
	extern int particleCount;
	extern void updateParticles(int startIdx, int count, float* array_data);
}
extern bool renderParticles;

namespace Sphere {

	glm::vec3 spherePos = glm::vec3(randomFloat(-5.f, 5.f), randomFloat(0.f, 10.f), randomFloat(-5.f, 5.f));
	float sphereRadius = randomFloat(1.f, 2.f);

	extern void setupSphere(glm::vec3 pos = glm::vec3(spherePos), float radius = sphereRadius);
	extern void cleanupSphere();
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
}
extern bool renderSphere;

namespace ClothMesh {
	extern void updateClothMesh(float* array_data);
	extern const int numCols;
	extern const int numRows;
}
extern bool renderCloth;
/////////////

ParticleSystem ps(ClothMesh::numCols* ClothMesh::numRows);
Mesh mesh;
Solver* solver;

float timer = 0.f;

glm::vec3 gravity = glm::vec3(.0f, -9.81f, 0.f);
float rest_distance = 0.2f;

float elasticity = 0.5;
float friction = 0.5;

float k_e_struct = 1000;
float k_d_struct = 30;
float k_e_shear = 1000;
float k_d_shear = 30;
float k_e_bend = 1000;
float k_d_bend = 30;

bool isEuler = false;
bool fixPosition = true;
bool resetSim = false;

/////////////

//deinicio interficie grafica
bool show_test_window = false;
void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
	}
	//lo nuestro
	ImGui::Text("", "");
	if (ImGui::Button("Reset simulation") && !resetSim) {
		resetSim = true;
	};
	ImGui::SameLine();
	ImGui::Text("%i", (int)(20.f - timer));

	ImGui::Text("", "");
	if (ImGui::Button("Change solver")) {
		isEuler = !isEuler;
		resetSim = true;
	}
	ImGui::SameLine();
	if (isEuler) ImGui::Text("Using Euler", "");
	else ImGui::Text("Using Verlet", "");

	ImGui::Text("", "");
	if (ImGui::Button("Fix mesh")) {
		fixPosition = !fixPosition;
	}
	ImGui::SameLine();
	ImGui::Text("%s", (fixPosition ? "true" : "false"));

	ImGui::Text("", "");
	ImGui::SliderFloat3("gravity", &gravity.x, -10.f, 10.f, "%.1f");
	ImGui::SliderFloat("elasticity coeff", &elasticity, 0.f, 1.f);
	ImGui::SliderFloat("friction coeff", &friction, 0.f, 1.f);

	ImGui::Text("", "");
	ImGui::SliderFloat("rest distance", &rest_distance, 0.1f, 0.6f);
	ImGui::Spacing();
	ImGui::SliderFloat("structural elasticity", &k_e_struct, 200.f, 2000.f);
	ImGui::SliderFloat("structural damping", &k_d_struct, 20.f, 70.f);
	ImGui::Spacing();
	ImGui::SliderFloat("shear elasticity", &k_e_shear, 200.f, 2000.f);
	ImGui::SliderFloat("shear damping", &k_d_shear, 20.f, 70.f);
	ImGui::Spacing();
	ImGui::SliderFloat("bending elasticity", &k_e_bend, 200.f, 2000.f);
	ImGui::SliderFloat("bending damping", &k_d_bend, 20.f, 70.f);

	ImGui::Text("", "");

	if (ImGui::Button("Sphere"))renderSphere = !renderSphere;
	if (renderSphere) {
		ImGui::SliderFloat3("sphere pos", &Sphere::spherePos.x, -5.f, 10.f, "%.1f");
		ImGui::SliderFloat("sphere radius", &Sphere::sphereRadius, 0.1f, 5.f, "%.1f");
	}

	if (ImGui::Button("Show Particles"))renderParticles = !renderParticles;

	ImGui::End();
}

//prepara setup de la simulacio
void PhysicsInit() {

	if(isEuler) solver = new Euler();
	else solver = new Verlet();

	renderParticles = false;
	renderSphere = true;

	//malla
	mesh = Mesh(ClothMesh::numCols, ClothMesh::numRows,  k_e_struct,  k_d_struct,  k_e_shear,  k_d_shear,  k_e_bend,  k_d_bend, rest_distance, isEuler);
	renderCloth = true;

	LilSpheres::particleCount = mesh.width * mesh.height;
}

//per cada loop es crida el update
void PhysicsUpdate(float dt) {

	// timer (20 segons)
	timer += dt;

	// controla el reset de la simulacio
	if (resetSim || (timer >= 20.f)) {
		if (isEuler) {
			solver = new Euler();
			isEuler = true;
		}
		else {
			solver = new Verlet();
			isEuler = false;
		}
		mesh.ResetMesh(ClothMesh::numCols, ClothMesh::numRows, rest_distance, isEuler);

		Sphere::spherePos = glm::vec3(randomFloat(-5.f, 5.f), randomFloat(0.f, 10.f), randomFloat(-5.f, 5.f));
		Sphere::sphereRadius = randomFloat(1.f, 2.f);

		gravity = glm::vec3(0.0f, -9.81f, 0.0f);
		resetSim = false;
		timer = 0.f;
	}

	// 15 iteracions per frame
	for (int i = 0; i < 15; i++)
	{
		glm::vec3* forces = mesh.Get_spring_forces(fixPosition, isEuler, dt/15, k_e_struct,  k_d_struct,  k_e_shear,  k_d_shear,  k_e_bend,  k_d_bend,  rest_distance);
		//sumar gravetat

		for (int i = 0; i < ps.numParticles; i++)
		{
			solver->updateParticles(mesh, forces, gravity, dt / 15, i, elasticity, friction, Sphere::sphereRadius, Sphere::spherePos, renderSphere, fixPosition);
		}

		delete forces;
		forces = nullptr;
	}

	//malla
	ClothMesh::updateClothMesh(&mesh.positions[0].x);
	if (renderParticles) LilSpheres::updateParticles(0, mesh.width * mesh.height, &mesh.positions[0].x);
	//spehre
	Sphere::updateSphere(Sphere::spherePos, Sphere::sphereRadius);
}

//al tancar el programa per alliberar memoria
void PhysicsCleanup() {
	delete solver;
	solver = nullptr;
}