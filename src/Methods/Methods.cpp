#include "Methods.h"

#include "../../RLUtilities/inc/mechanics/reorient.h"
#include "../../RLUtilities/inc/mechanics/reorient_ML.h"
#include "../RLUHelp/RLUHelp.h"
#include "../RLUTModelWeights.h"

std::vector<ReorientMethod*> g_Methods = {};

///////////////////

RControls RM_RLU_ML::Run(RM_RUN_ARGS) {
	static bool first = true;
	if (first) {
		first = false;
		ReorientML::set_model(Model(
			std::vector<float>(
				(float*)RLUT_MODEL_WEIGHTS,
				(float*)RLUT_MODEL_WEIGHTS + (sizeof(RLUT_MODEL_WEIGHTS) / sizeof(float))
				)
		));
	}

	Car rluCar = Car();
	rluCar.angular_velocity = RLUHelp::Convert(angVel);
	rluCar.orientation = RLUHelp::Convert(rot);

	ReorientML rluReorient = ReorientML(rluCar);
	rluReorient.target_orientation = RLUHelp::Convert(targetRot);
	rluReorient.step(TICKTIME);

	return {
		rluReorient.controls.pitch,
		rluReorient.controls.yaw,
		rluReorient.controls.roll
	};
}

RControls RM_RLU::Run(RM_RUN_ARGS) {

	Car rluCar = Car();
	rluCar.angular_velocity = RLUHelp::Convert(angVel);
	rluCar.orientation = RLUHelp::Convert(rot);

	Reorient rluReorient = Reorient(rluCar);
	rluReorient.eps_phi = 0;
	rluReorient.eps_omega = 0;
	rluReorient.target_orientation = RLUHelp::Convert(targetRot);
	rluReorient.step(TICKTIME);

	return {
		rluReorient.controls.pitch,
		rluReorient.controls.yaw,
		rluReorient.controls.roll
	};
}

float SteerPD(float angle, float rate, float scale, float power, float div) {
	// Code copied from SteerPD() in https://github.com/ItsCodeRed/RedUtils/blob/master/RedUtils/Tools.cs
	return RS_CLAMP(powf(scale * (angle + rate), power) / div, -1, 1);
}

RControls RunSteerPDs(Angle targetAngles, Vec localAngVel, Vec localAngVelScale, float spdScale, float spdPower, float spdDiv) {
	Vec scaledLocalAngVel = localAngVel * localAngVelScale;
	return {
		SteerPD(targetAngles.pitch,	scaledLocalAngVel.y, spdScale, spdPower, spdDiv),
		SteerPD(targetAngles.yaw,  -scaledLocalAngVel.z, spdScale, spdPower, spdDiv),
		SteerPD(targetAngles.roll,	scaledLocalAngVel.x, spdScale, spdPower, spdDiv)
	};
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

	constexpr float
		SPD_SCALE = 35,
		SPD_POWER = 3,
		SPD_DIV = 10;

	return RunSteerPDs(
		targetAngles, localAngVel, Vec(0.25f, 0.20f, 0.15f), 35, 3, 10);

	return {
		SteerPD(targetAngles.pitch,	localAngVel.y * 0.20f, SPD_SCALE, SPD_POWER, SPD_DIV), // Pitch
		SteerPD(targetAngles.yaw,  -localAngVel.z * 0.15f, SPD_SCALE, SPD_POWER, SPD_DIV), // Yaw
		SteerPD(targetAngles.roll,	localAngVel.x * 0.25f, SPD_SCALE, SPD_POWER, SPD_DIV)  // Roll
	};
}