#pragma once
#include "../../RocketSim/src/Math/Math.h"

namespace Math {
	float RotMatDist(const RotMat& a, const RotMat& b);

	float ErrorToScorePercent(float error, float scale);
}