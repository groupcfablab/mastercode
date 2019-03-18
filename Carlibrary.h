#ifndef CARCONTROL_H
#define CARCONTROL_H
#include "motordrivesysupdated.h"
#include <math.h>


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
				else if((targetrpm >= 0)&&(dir_flag))
				{
					motor.changedir();
					dir_flag = false;
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
	
	double distance_travelled(const int wheel_diameter,const int interruptsoncircle = 49)
	{
		int rotations = rotation_counter.GetkDistanceCount();
		return ((double)(rotations)/interruptsoncircle)*(wheel_diameter*PI);
	}
	void reset
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

	bool drive_turn(double in_angle, double in_target_speed, double turn_radius)
	{
		double driving_angle = abs(in_angle)/180;//percentage
		double left_speed=((turn_radius+5)/turn_radius)*in_target_speed;
		double right_speed=((turn_radius-5)/turn_radius)*in_target_speed;
		double avg_distance=(motor_left.distance_travelled(57)+motor_right.distance_travelled(57))/2;
		double target_distance=(turn_radius*PI)*driving_angle;//ALL IN MILIMETRES
		if(avg_distance<target_distance)
		{
			if(in_angle>0)
			{
				motor_left.drive(left_speed);
				motor_right.drive(right_speed);
			}
			else if (in_angle<0)
			{
				motor_left.drive(right_speed);
				motor_right.drive(left_speed);
			}
			else
			{
				motor_left.drive(in_target_speed);
				motor_right.drive(in_target_speed);
			}
			Serial.println(avg_distance);
			return false;
			
		}
		else
		{
			return true;
		}

	}

};







#endif
