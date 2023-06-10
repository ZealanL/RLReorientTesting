#pragma once
#include "ReorientMethod.h"

#define MAKE_RM(className, strName) \
class className : public ReorientMethod { \
public: \
	className() { \
		ReorientMethod::GetAllMethods().push_back(this); \
	} \
	string GetName() const { \
		return strName; \
	} \
	RControls Run(RM_RUN_ARGS); \
}; \
inline className* g_##className = new className();

MAKE_RM(RM_RLU, "RLUtilities");
MAKE_RM(RM_RLU_ML, "RLUtilities_ML");
MAKE_RM(RM_RedUtils, "RedUtils");
MAKE_RM(RM_BruteForcer, "BruteForcer");