#include "Math.h"

float Math::RotMatDist(const RotMat& a, const RotMat& b) {
	btQuaternion qA, qB;
	((btMatrix3x3)a).getRotation(qA);
	((btMatrix3x3)b).getRotation(qB);

	return qA.angleShortestPath(qB);
}

float Math::ErrorToScorePercent(float error) {
	return (1 / (1 + error / 16)) * 100;
}