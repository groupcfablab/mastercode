#ifndef CARCONTROL_H
#define CARCONTROL_H
#include "motordrivesysupdated.h"



class motorcontrol
{
private:
	//objects
	InterruptSpeedMeasure rotation_counter;
	basic_speed_PID pidcontrol;
	HBridgeDCmotor motor;
	IntervalCheckTimer refreshrate;
	const unsigned long default_refresh_time = 250;

	//flags
	bool setup_flag, init_flag, dir_flag = false;

	//variables
	int outputpwm, minpwm;
	double current_speed, ratedrpm;

	//private functions

public:
	//constructor
	motorcontrol()
	{
		init_flag = true;
	}
	motorcontrol(const double kp, const double ki, const double kd, const int motorpin, const int dirpin, const ArduinoInterruptNames rpmmeasurepin, const int interruptsoncircle = 49, const uint8_t in_minpwm = 55, const double in_ratedrpm = 6000)
	{
		init_flag = true;
		setup(kp, ki, kd, motorpin, dirpin, rpmmeasurepin, interruptsoncircle, in_minpwm, in_ratedrpm);
	}

	void setup(const double kp, const double ki, const double kd, const int motorpin, const int dirpin, const ArduinoInterruptNames rpmmeasurepin, const int interruptsoncircle = 49, const uint8_t in_minpwm = 55, const double in_ratedrpm = 6000)
	{
		if (init_flag && !setup_flag)
		{
			rotation_counter.setupSpeedMeasure(rpmmeasurepin, interruptsoncircle);
			pidcontrol.set_gainvals(kp, ki, kd);
			motor.setup_HBridgeDCmotor(motorpin, dirpin);
			refreshrate.setInterCheck(default_refresh_time);
			minpwm = in_minpwm;
			ratedrpm = in_ratedrpm;
			setup_flag = true;
		}
	}

	void drive(double targetrpm, bool echo = false)
	{
		if (setup_flag&&init_flag)
		{
			if (refreshrate.isMinChekTimeElapsedAndUpdate())
			{
				current_speed = rotation_counter.getRPMandUpdate();
				if ((targetrpm < 0)&&(!dir_flag))
				{
					motor.changedir();
					dir_flag = true;
				}
				double outputrpm = pidcontrol.ComputePID_output(abs(targetrpm), current_speed);
				outputpwm = map(outputrpm, 0, ratedrpm, minpwm, 255);
				if (!motor.isStarted()) { motor.start(); }
				motor.setSpeedPWM(outputpwm);
				if (echo)
				{
					Serial.print(abs(targetrpm));
					Serial.print(',');
					Serial.print(current_speed);
					Serial.print(',');
					Serial.println(outputrpm);
				}
			}
		}
	}
};

class carcontroller
{
private:
	//objects
	motorcontrol motor_left, motor_right; 

	//flags
	bool setup_flag, init_flag = false;
	
	//variables
	double driving_angle;
	//private functions
public:
	carcontroller()
	{
		init_flag = true;
	}
	carcontroller(const double kp, const double ki, const double kd, const int motorpin_left, const int motorpin_right, const int dirpin_left, const int dirpin_right, const ArduinoInterruptNames rpmmeasurepin_left, const ArduinoInterruptNames rpmmeasurepin_right, const int interruptsoncircle = 49, const uint8_t in_minpwm = 55, const double in_ratedrpm = 6000)
	{
		init_flag = true;
		setup(kp, ki, kd, motorpin_left, motorpin_right, dirpin_left, dirpin_right, rpmmeasurepin_left, rpmmeasurepin_right, interruptsoncircle, in_minpwm, in_ratedrpm);
	}
	void setup(const double kp, const double ki, const double kd, const int motorpin_left, const int motorpin_right, const int dirpin_left, const int dirpin_right, const ArduinoInterruptNames rpmmeasurepin_left, const ArduinoInterruptNames rpmmeasurepin_right, const int interruptsoncircle = 49, const uint8_t in_minpwm = 55, const double in_ratedrpm = 6000)
	{
		if (init_flag && !setup_flag)
		{
			motor_left.setup(kp, ki, kd, motorpin_left, dirpin_left, rpmmeasurepin_left, interruptsoncircle, in_minpwm, in_ratedrpm);
			motor_right.setup(kp, ki, kd, motorpin_right, dirpin_right, rpmmeasurepin_right, interruptsoncircle, in_minpwm, in_ratedrpm);
			setup_flag = true;
		}
	}

	void drive(double in_angle, double in_target_speed)
	{
		driving_angle = in_angle;
		double left_speed;//FINISH THIS
		//motor_left.drive();

	}

};







#endif