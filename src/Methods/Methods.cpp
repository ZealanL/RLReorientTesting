#include "Methods.h"

#include "../../RLUtilities/inc/mechanics/reorient.h"
#include "../../RLUtilities/inc/mechanics/reorient_ML.h"
#include "../../RocketSim/libsrc/bullet3-3.24/LinearMath/btTransformUtil.h"
#include "../../RocketSim/src/RLConst.h"
#include "../RLUHelp/RLUHelp.h"
#include "../RLUTModelWeights.h"
#include "../Math/Math.h"

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

RControls RunSteerPDs(RM_RUN_ARGS, Vec localAngVelScale, float spdScale, float spdPower, float spdDiv) {

	// Replicated from AimAt()/SteerPD() in https://github.com/ItsCodeRed/RedUtils/blob/master/RedUtils/Tools.cs

	Vec localForward = rot.Dot(targetRot.forward);
	Vec localUp = rot.Dot(targetRot.up);
	Vec localAngVel = rot.Dot(angVel);

	Angle targetAngles = {
		atan2f(localForward.y,	localForward.x), // Yaw
		atan2f(localForward.z,	localForward.x), // Pitch
		atan2f(localUp.y,		localUp.z)		 // Roll
	};

	Vec scaledLocalAngVel = localAngVel * localAngVelScale;
	return {
		SteerPD(targetAngles.pitch,	scaledLocalAngVel.y, spdScale, spdPower, spdDiv),
		SteerPD(targetAngles.yaw,  -scaledLocalAngVel.z, spdScale, spdPower, spdDiv),
		SteerPD(targetAngles.roll,	scaledLocalAngVel.x, spdScale, spdPower, spdDiv)
	};
}

RControls RM_RedUtils::Run(RM_RUN_ARGS) {
	return RunSteerPDs(rot, angVel, targetRot, Vec(0.25f, 0.20f, 0.15f), 35, 3, 10);
}