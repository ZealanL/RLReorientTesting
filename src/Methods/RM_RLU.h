#pragma once
#include "../ReorientMethod.h"

class RM_RLU : public ReorientMethod {
public:
	string GetName() const {
		return "RLUtilities";
	}

	RControls Run(RM_RUN_ARGS);
};