#include "RM_RLU.h"

#include "../../RLUtilities/inc/simulation/car.h"
#include "../../RLUtilities/inc/mechanics/reorient.h"

RControls RM_RLU::Run(RM_RUN_ARGS) {
	
	Car rluCar = Car();
	rluCar.angular_velocity = vec3{ angVel.x, angVel.y, angVel.z };
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			rluCar.orientation(i, j) = rot[i][j];

	Reorient rluReorient = Reorient(rluCar);
	rluReorient.step(TICKTIME);

	return {
		rluReorient.controls.pitch,
		rluReorient.controls.yaw,
		rluReorient.controls.roll
	};
}