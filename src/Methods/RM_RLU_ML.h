#pragma once
#include "../ReorientMethod.h"

class RM_RLU_ML : public ReorientMethod {
public:

	string GetName() const {
		return "RLUtilities ML";
	}

	RControls Run(RM_RUN_ARGS);
};