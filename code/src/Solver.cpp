#include "Solver.h"

float Solver::findD(float A, float B, float C, glm::vec3 point)
{
	return -(A * point.x) - (B * point.y) - (C * point.z);
}

float Solver::quadraticEquation(float a, float b, float c)
{
	return glm::min((-b + sqrt(pow(b, 2) - 4. * a * c)) / (2.f * a), (-b - sqrt(pow(b, 2) - 4. * a * c)) / (2.f * a));
}
