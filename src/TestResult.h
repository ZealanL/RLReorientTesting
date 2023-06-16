#pragma once
#include "Framework.h"

struct TestResult {
	float
		forwardError = 0,
		overshootError = 0,
		initialReachTimeError = 0;

	// Timed out before reaching the target rotation
	bool dnf = false;
};