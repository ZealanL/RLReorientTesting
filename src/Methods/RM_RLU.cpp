#include "RM_RLU.h"

#include "../RLUHelp/RLUHelp.h"
#include "../../RLUtilities/inc/mechanics/reorient.h"

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