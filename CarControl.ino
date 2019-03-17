//libraries
#include "Carlibrary.h"

//globals
//controlsys motor_left, motor_right;
motorcontrol motor_left, motor_right;

void setup()
{
	Serial.begin(9600);
	const controllertype system_type = closedloop;
	const int pwmpin__left = 5;
	const int pwmpin__right = 6;

	const int directionpin_left = 52;
	const int directionpin_right = 22;

	const int buttonpins[3] = { 38,40,42 };
	const int potentiometerpin = A0;

	const ArduinoInterruptNames interruptpin_left = int_0;//int_0=D2
	const ArduinoInterruptNames interruptpin_right = int_1;//int_1=D3

	const int reading_rate_ms = 400;
	const int pwm_update_rate_ms = 100;

	const int magnets_on_circle = 49;
	const int min_pwm_for_motor = 20;
	const int max_rated_rpm = 230;
	double k = 0.0002;
	motor_left.setup(20 * k, 150 * k, 0.3*k, pwmpin__left, directionpin_left, interruptpin_left, magnets_on_circle, min_pwm_for_motor, max_rated_rpm);
	motor_right.setup(20 * k, 150 * k, 0.3*k, pwmpin__right, directionpin_right, interruptpin_right, magnets_on_circle, min_pwm_for_motor, max_rated_rpm);

}

void loop()
{
	motor_left.drive(80, false);
	motor_right.drive(140, true);
}
