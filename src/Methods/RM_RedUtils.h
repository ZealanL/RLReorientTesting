#pragma once
#include "../ReorientMethod.h"

class RM_RedUtils : public ReorientMethod {
public:
	string GetName() const {
		return "RedUtils";
	}

	RControls Run(RM_RUN_ARGS);
};