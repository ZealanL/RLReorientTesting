#include "ReorientMethod.h"

vector<ReorientMethod*>& ReorientMethod::GetAllMethods() {
	static vector<ReorientMethod*> methods;
	return methods;
}