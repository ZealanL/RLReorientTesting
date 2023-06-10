#pragma once
#include "../Framework.h"
#include "../../RLUtilities/inc/simulation/car.h"

namespace RLUHelp {
	mat3 Convert(const RotMat& rot);
	RotMat Convert(const mat3& mat);

	vec3 Convert(const Vec& vec);
	Vec Convert(const vec3& vec);
}