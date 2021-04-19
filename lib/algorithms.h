#include "common.h"

extern "C" void collisionAvoidance(int totalObsNum, int* appeared, int staticObsNum, Point *staticObsVerdices, int* probe, Point * obsPosition, RotationalState * rotationalObsStates,
	LinearState * linearObsStates, Point * avPosition, RotationalState * rotationalAVStates, LinearState * linearAVStates,
	Point * goalPosition, Point * path, int pathLen, RotationalState * rotationalGoalStates, LinearState * linearGoalStates);
