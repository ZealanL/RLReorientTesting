#include "RM_RedUtils.h"

float SteerPD(float angle, float rate) {
	// Code copied from SteerPD() in https://github.com/ItsCodeRed/RedUtils/blob/master/RedUtils/Tools.cs
	return RS_CLAMP(powf(35 * (angle + rate), 3) / 10, -1, 1);
}

RControls RM_RedUtils::Run(RM_RUN_ARGS) {
	
	// Code copied from AimAt() in https://github.com/ItsCodeRed/RedUtils/blob/master/RedUtils/Tools.cs
	Vec localForward = rot.Dot(targetRot.forward);
	Vec localUp = rot.Dot(targetRot.up);
	Vec localAngVel = rot.Dot(angVel);
	Angle targetAngles = {
		atan2f(localForward.y,	localForward.x), // Yaw
		atan2f(localForward.z,	localForward.x), // Pitch
		atan2f(localUp.y,		localUp.z)		 // Roll
	};

	return {
		SteerPD(targetAngles.pitch,	localAngVel.y * 0.20f), // Pitch
		SteerPD(targetAngles.yaw,  -localAngVel.z * 0.15f), // Yaw
		SteerPD(targetAngles.roll,	localAngVel.x * 0.25f)  // Roll
	};
}