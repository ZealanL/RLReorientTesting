#include "RLUHelp.h"

mat3 RLUHelp::Convert(const RotMat& rot) {
	mat3 result;
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			result(j, i) = rot[i][j];
	
	return result;
}

RotMat RLUHelp::Convert(const mat3& mat) {
	RotMat result;
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			result[i][j] = mat(j, i);

	return result;
}

vec3 RLUHelp::Convert(const Vec& vec) {
	return vec3{ vec.x, vec.y, vec.z };
}

Vec RLUHelp::Convert(const vec3& vec) {
	return Vec(vec[0], vec[1], vec[2]);
}