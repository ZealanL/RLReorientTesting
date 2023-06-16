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

RControls RM_BruteForcer::Run(RM_RUN_ARGS) {
	using namespace RLConst;

	btTransform transform = btTransform(rot);

	btVector3
		dirPitch_right = -rot.right,
		dirYaw_up = rot.up,
		dirRoll_forward = -rot.forward;

	RControls bestControls = RControls(0, 0, 0);
	float lowestDist = FLT_MAX;
	for (float yaw = -1; yaw <= 1; yaw++) {
		for (float pitch = -1; pitch <= 1; pitch++) {
			for (float roll = -1; roll <= 1; roll += 0.5f) {
				Vec newAngVel = angVel;

				Vec torque;
				if (yaw || pitch || roll) {
					torque = (pitch * dirPitch_right * CAR_AIR_CONTROL_TORQUE.x) +
						(yaw * dirYaw_up * CAR_AIR_CONTROL_TORQUE.y) +
						(roll * dirRoll_forward * CAR_AIR_CONTROL_TORQUE.z);
				} else {
					torque = { 0, 0, 0 };
				}

				float
					dampPitch = dirPitch_right.dot(angVel) * CAR_AIR_CONTROL_DAMPING.x * (1 - abs(pitch)),
					dampYaw = dirYaw_up.dot(angVel) * CAR_AIR_CONTROL_DAMPING.y * (1 - abs(yaw)),
					dampRoll = dirRoll_forward.dot(angVel) * CAR_AIR_CONTROL_DAMPING.z;

				Vec damping =
					(dirYaw_up * dampYaw) +
					(dirPitch_right * dampPitch) +
					(dirRoll_forward * dampRoll);

				newAngVel = angVel + (torque - damping) * CAR_TORQUE_SCALE * TICKTIME;

				btTransform newTransform;
				btTransformUtil::integrateTransform(transform, btVector3(), newAngVel, TICKTIME, newTransform);

				RotMat newRot = (RotMat)newTransform.getBasis();

				float dist = newRot.forward.Dist(targetRot.forward);
				if (dist < lowestDist) {
					lowestDist = dist;
					bestControls = { pitch, yaw, roll };
				}
			}
		}
	}

	return bestControls;
}