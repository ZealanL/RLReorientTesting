#pragma once
#include "Framework.h"

struct TestResult {
	float error = 0;

	// Timed out before reaching the target rotation
	bool dnf = false;
};