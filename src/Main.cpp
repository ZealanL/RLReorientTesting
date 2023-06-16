#include "Framework.h"

#include "Methods/Methods.h"
#include "TestCase.h"
#include "TestResult.h"

#include "../RocketSim/src/RocketSim.h"
#include "Math/Math.h"

Arena* g_Arena = NULL;
Car* g_Car = NULL;

Vec MakeRandomAngVel() {
	return Vec(
		Math::RandFloat(-1, 1),
		Math::RandFloat(-1, 1),
		Math::RandFloat(-1, 1)
	).Normalized() * Math::RandFloat(0, RLConst::CAR_MAX_ANG_SPEED);
}

RotMat MakeRandomRot() {
	// I'm lazy so lets just use euler angles
	Angle ang = Angle(
		Math::RandFloat(-M_PI, M_PI),
		Math::RandFloat(-M_PI / 2, M_PI / 2),
		Math::RandFloat(-M_PI, M_PI)
	);
	return ang.ToRotMat();
}

// Rough estimate of lower bound of error for a rotation of dist, multiplied by scale
float ComputeDistErrorLowerBound(float dist) {
	float timeLowerBound = RS_MAX(dist * 0.3f - 0.08f, 0);
	return powf(timeLowerBound, 6.4f) * 13.24f;
}

TestResult RunTest(ReorientMethod* method, const TestCase& testCase) {
	constexpr float
		STOP_ANGLE_THRESHOLD = 0.1f, // Angle distance under this is considered to have reached the target position
		STOP_ANGVEL_THRESHOLD = 0.1f, // Anglular velocity threshold under this is considered to have stopped
		UP_ERROR_SCALE = 0.4f, // Upwards error matters less than forwards error, so its scaled down
		TIMEOUT_SECONDS = 10;

	CarState startState = {};
	startState.rotMat = testCase.rot;
	startState.angVel = testCase.angVel;
	g_Car->SetState(startState);

	TestResult result;
	result.dnf = true;

	// Compute minimum possible error (very naive)
	// Does not include overshoot
	float 
		startForwardDist = testCase.rot.forward.Dist(testCase.targetRot.forward),
		startAngleDist = Math::RotMatDist(testCase.rot, testCase.targetRot);
	float 
		forwardErrorLowerBound = ComputeDistErrorLowerBound(startForwardDist) * 1.0f,
		timeLowerBound = RS_MAX(((startAngleDist - STOP_ANGLE_THRESHOLD) / RLConst::CAR_MAX_ANG_SPEED) - TICKTIME, 0);

	result.initialReachTimeError = TIMEOUT_SECONDS;

	bool reached = false;
	float lastAngleDist = 0;
	for (float t = 0; t < TIMEOUT_SECONDS; t += TICKTIME) {
		{ // Move ball out of the way
			BallState ballState = {};
			ballState.pos = Vec(-1000, 0, -1000);
			ballState.vel = Vec();
			g_Arena->ball->SetState(ballState);
		}

		// Reset car pos and vel
		CarState carState = g_Car->GetState();
		carState.pos = carState.vel = Vec(0, 0, 0);
		g_Car->SetState(carState);

		// Update controls from reorient method
		RControls reorientControls = method->Run(carState.rotMat, carState.angVel, testCase.targetRot);
		g_Car->controls.pitch = reorientControls.pitch;
		g_Car->controls.yaw = reorientControls.yaw;
		g_Car->controls.roll = reorientControls.roll;

		// Update simulation and carState
		g_Arena->Step(1);
		carState = g_Car->GetState();

		// Calculate forward error
		float 
			forwardDist = carState.rotMat.forward.Dist(testCase.targetRot.forward),
			forwardError = (forwardDist / 2) * t;

		// Calculate up error
		float
			upDist = carState.rotMat.up.Dist(testCase.targetRot.up),
			upError = (upDist / 2) * t;

		float angleDist = Math::RotMatDist(carState.rotMat, testCase.targetRot);

		if (angleDist < STOP_ANGLE_THRESHOLD) {
			result.initialReachTimeError = RS_MIN(result.initialReachTimeError, t);
			if (carState.angVel.LengthSq() < (STOP_ANGVEL_THRESHOLD * STOP_ANGVEL_THRESHOLD)) {
				// We're done
				result.dnf = false;
				break;
			} else {
				// We reached the target angle
				// Keep track of this so that we can mark further error as overshooting
				reached = true;
			}
		}

		if (reached && angleDist > lastAngleDist) {
			result.overshootError += forwardError + upError;
		} else {
			result.forwardError += forwardError;
		}

		lastAngleDist = angleDist;
	}

	assert(result.forwardError >= forwardErrorLowerBound);
	assert(result.initialReachTimeError >= timeLowerBound);

	result.forwardError	-= forwardErrorLowerBound;
	result.initialReachTimeError -= timeLowerBound;
	return result;
}

int main() {
	RocketSim::Init(RS_COLLISION_MESHES_PATH);
	g_Arena = Arena::Create(GameMode::THE_VOID);
	g_Car = g_Arena->AddCar(Team::BLUE);
	
	vector<ReorientMethod*>& methods = ReorientMethod::GetAllMethods();

	if (!methods.empty()) {
		RS_LOG("Running reorient tests on " << methods.size() << " methods.");
	} else {
		RS_LOG("No reorient methods to run!");
		return EXIT_FAILURE;
	}

	// Create test cases
	constexpr size_t TEST_AMOUNT =
#ifdef _DEBUG
		100;
#else
		1000;
#endif

	TestCase* tests = new TestCase[TEST_AMOUNT];
	for (size_t i = 0; i < TEST_AMOUNT; i++) {
		TestCase& test = tests[i];
		test.angVel = MakeRandomAngVel();
		test.rot = MakeRandomRot();
		test.targetRot = MakeRandomRot();
	}

	// Run tests for all methods
	for (ReorientMethod* method : methods) {
		RS_LOG("======================================");
		RS_LOG("Method: " << method->GetName());

		float
			totalForwardError = 0,
			totalTimeError = 0,
			totalOvershootError = 0;

		size_t numFailed = 0;
		for (size_t i = 0; i < TEST_AMOUNT; i++) {
			TestResult testResult = RunTest(method, tests[i]);

			if (!testResult.dnf) {
				totalForwardError = testResult.forwardError;
				totalTimeError += testResult.initialReachTimeError;
				totalOvershootError += testResult.overshootError;
			} else {
				numFailed++;
			}
		}

		float 
			avgForwardError = totalForwardError / TEST_AMOUNT,
			avgTimeError = totalTimeError / TEST_AMOUNT,
			avgOvershootError = totalOvershootError / TEST_AMOUNT;

		if (numFailed != 0) {
			RS_LOG(" > FAILED to finish " << numFailed << " / " << TEST_AMOUNT << " tests.");
		}

		float
			forwardSpeed = Math::ErrorToScorePercent(avgForwardError, 0.5f),
			overallSpeed = Math::ErrorToScorePercent(avgTimeError, 4),
			
			accuracy = Math::ErrorToScorePercent(avgOvershootError, 0.5f);

		const auto fnLogResult = [](const char* name, float percentScore, float error) {
			RS_LOG(
				" > " << name << ":" << string(RS_MAX(20 - strlen(name), 1), ' ') <<
				std::fixed << std::setprecision(2) << percentScore << "%" <<
				" (" << std::setprecision(3) << error << " error)"
			);
		};

		fnLogResult("Forward Speed", forwardSpeed, avgForwardError);
		fnLogResult("Overall Speed", overallSpeed, avgTimeError);
		
		fnLogResult("Stopping Accuracy", accuracy, avgOvershootError);
	}

	delete[] tests;
	delete g_Arena;
	return EXIT_SUCCESS;
}