#include "ParticleSystem.h"
//#include "CircularBuffer.h"
#include <stdio.h>
#include <glm/gtx/string_cast.hpp>
#include <iostream>


/////////Forward declarations
namespace LilSpheres {
	//extern const int maxParticles;
	extern int firstParticleIdx;
	extern int particleCount;
	extern void updateParticles(int startIdx, int count, float* array_data);
}
extern bool renderParticles;
/////////////

ParticleSystem::ParticleSystem(int _numParticles){
	positions = new glm::vec3[_numParticles];
	extra = new glm::vec3[_numParticles];
	numParticles = _numParticles;
}

///////////Methods
float quadraticEquation(float a, float b, float c) //bool addition->true, es fa la suma / addition->false, es fa la resta
{

	//float add = (-b + sqrt((b * b) - 4.f * a * c)) / 2.f * a;
	//float add2 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);

	//float subs = (-b - sqrt((b * b) - 4.f * a * c)) / 2.f * a;

	//return glm::min(add, subs);

	return glm::min((-b + sqrt(pow(b, 2) - 4. * a * c)) / (2.f * a), (-b - sqrt(pow(b, 2) - 4. * a * c)) / (2.f * a));
}
float findD(float A, float B, float C, glm::vec3 point)
{
	return -(A * point.x) - (B * point.y) - (C * point.z);
}
////////////





