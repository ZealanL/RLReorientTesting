#include "Framework.h"

#include "Methods/RM_RLU.h"
#include "Methods/RM_RLU_ML.h"
#include "Methods/RM_RedUtils.h"
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

TestResult RunTest(ReorientMethod* method, const TestCase& testCase) {
	constexpr float
		STOP_ANGLE_THRESHOLD = 0.05f, // Angle distance under this is considered to have reached the target position
		STOP_ANGVEL_THRESHOLD = 0.05f, // Anglular velocity threshold underthis is considered to have stopped
		UP_ERROR_SCALE = 0.4f, // Upwards error matters less than forwards error, so its scaled down
		TIMEOUT_SECONDS = 10;

	CarState startState = {};
	startState.rotMat = testCase.rot;
	startState.angVel = testCase.angVel;
	g_Car->SetState(startState);

	TestResult result;
	result.dnf = true;
	
	bool reached = false;
	float lastError = 0;

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

		// Calculate error
		float error = (Math::RotMatDist(carState.rotMat, testCase.targetRot) / M_PI) * t;
		if (error < STOP_ANGLE_THRESHOLD) {
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

		if (reached && error > lastError) {
			result.overshootError += error;
		} else {
			result.error += error;
		}

		lastError = error;
	}

	return result;
}

int main() {
	RocketSim::Init(RS_COLLISION_MESHES_PATH);
	g_Arena = Arena::Create(GameMode::THE_VOID);
	g_Car = g_Arena->AddCar(Team::BLUE);

	vector<ReorientMethod*> methods = {
		new RM_RLU(),
		new RM_RLU_ML(),
		new RM_RedUtils()
	};
	
	if (!methods.empty()) {
		RS_LOG("Running reorient tests on " << methods.size() << " methods.");
	} else {
		RS_LOG("No reorient methods to run!");
		return EXIT_FAILURE;
	}

	// Create test cases
	constexpr size_t TEST_AMOUNT = 100;
	TestCase tests[TEST_AMOUNT];
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
			totalError = 0,
			totalOvershootError = 0;

		size_t numFailed = 0;
		for (size_t i = 0; i < TEST_AMOUNT; i++) {
			TestResult testResult = RunTest(method, tests[i]);

			if (!testResult.dnf) {
				totalError += testResult.error;
				totalOvershootError += testResult.overshootError;
			} else {
				numFailed++;
			}
		}

		float avgError = totalError / TEST_AMOUNT;
		float avgOvershootError = totalOvershootError / TEST_AMOUNT;

		if (numFailed == 0) {
			RS_LOG(" > Finished all tests.");
		} else {
			RS_LOG(" > FAILED to finish " << numFailed << " / " << TEST_AMOUNT << " tests.");
		}

		float speed = Math::ErrorToScorePercent(avgError);
		float accuracy = Math::ErrorToScorePercent(avgOvershootError);
		RS_LOG(" > Speed: " << std::setprecision(4) << speed << "%");
		RS_LOG(" > Accuracy: " << std::setprecision(4) << accuracy << "%");
	}

	delete g_Arena;
	return EXIT_SUCCESS;
}