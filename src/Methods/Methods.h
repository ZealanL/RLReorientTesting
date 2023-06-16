#pragma once
#include "ReorientMethod.h"

#define MAKE_RM(className, strName, shouldAddToMethods) \
class className : public ReorientMethod { \
public: \
	className(bool addToMethods) : ReorientMethod(addToMethods) {} \
	string GetName() const { \
		return strName; \
	} \
	RControls Run(RM_RUN_ARGS); \
}; \
inline className* g_##className = new className(shouldAddToMethods);

MAKE_RM(RM_RLU, "RLUtilities", true);
MAKE_RM(RM_RLU_ML, "RLUtilities_ML", true);
MAKE_RM(RM_RedUtils, "RedUtils", true);
MAKE_RM(RM_BruteForcer, "BruteForcer", true);