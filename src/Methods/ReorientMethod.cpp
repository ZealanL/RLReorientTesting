#include "ReorientMethod.h"

vector<ReorientMethod*>& ReorientMethod::GetAllMethods() {
	static vector<ReorientMethod*> methods;
	return methods;
}

ReorientMethod::ReorientMethod(bool addToMethods) {
	if (addToMethods)
		GetAllMethods().push_back(this);
}