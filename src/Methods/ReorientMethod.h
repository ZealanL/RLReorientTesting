#pragma once
#include "../Framework.h"

struct RControls {
	float pitch, yaw, roll;

	constexpr RControls() {
		pitch = yaw = roll = 0;
	}

	constexpr RControls(float pitch, float yaw, float roll)
		: pitch(RS_CLAMP(pitch, -1, 1)), yaw(RS_CLAMP(yaw, -1, 1)), roll(RS_CLAMP(roll, -1, 1)) {}
};

#define RM_RUN_ARGS const RotMat& rot, const Vec& angVel, const RotMat& targetRot

// For once, I'm going to use virtual stuff
class ReorientMethod {
public:
	static vector<ReorientMethod*>& GetAllMethods();

	virtual string GetName() const = 0;
	virtual RControls Run(RM_RUN_ARGS) = 0;
};