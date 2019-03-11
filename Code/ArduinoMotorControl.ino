/*
	Name:       ArduinoMotorControl.ino
	Created:	02/03/2019 14:49:29
	Author:     DESKTOP-BFVU60N\User
*/

// libraries
#include "motordrivesysupdated.h"

// globals
controlsys motorsys;

void setup()
{
	const int tempdirpins[2] = { 7,8 };
	const int tempbuttonpins[3] = { 14,15,16 };
	const double gain = 0.000005;
	motorsys.setup(closedloop, 5, tempdirpins, tempbuttonpins, A5, int_0, 400, 100, 6, 150, 1000);
	motorsys.setup_pid(20*gain, 150*gain, 0*gain);
}

void loop()
{
	motorsys.routine();
}
