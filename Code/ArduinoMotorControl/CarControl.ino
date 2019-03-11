//libraries
#include "motordrivesysupdated.h"

//globals
controlsys motor_left, motor_right;

void setup()
{
	const int directionpin_left = 5;
	const int directionpin_right = 6;
	const int tempbuttonpins_left[3] = { 14,15,16 };
	const int tempbuttonpins_right[3] = { 14,15,16 };
	motor_left.setup(openloop, 5, directionpin_left, tempbuttonpins_left, A5, int_0, 400, 100, 49, 150, 1000);
	motor_right.setup(openloop, 5, directionpin_right, tempbuttonpins_right, A5, int_1, 400, 100, 49, 150, 1000);
}

void loop()
{
	motor_left.routine();
	motor_right.routine();
}