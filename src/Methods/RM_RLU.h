#include "../ReorientMethod.h"

class RM_RLU : public ReorientMethod {
public:

	const char* GetName() const {
		return "RLUtilities";
	}

	RControls Run(RM_RUN_ARGS);
};