#ifndef CARCONTROL_H
#define CARCONTROL_H
#include "motordrivesysupdated.h"

class carcontroller
{
private:
	//objects
	controlsys motor_left, motor_right;
	//flags
	controllertype cntrltype = indef;
	bool setup_flag, init_flag, pidflag = false;
	//variables

	//private functions

};







#endif