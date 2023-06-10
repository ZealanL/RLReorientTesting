#pragma once
#include "../Framework.h"

struct RControls {
	float pitch, yaw, roll;

	constexpr RControls() {
		pitch = yaw = roll = 0;
	}

	RControls(float pitch, float yaw, float roll)
		: pitch(pitch), yaw(yaw), roll(roll) {
	}
};

#define RM_RUN_ARGS const RotMat& rot, const Vec& angVel, const RotMat& targetRot

// For once, I'm going to use virtual stuff
class ReorientMethod {
public:
	virtual string GetName() const = 0;
	virtual RControls Run(RM_RUN_ARGS) = 0;
};