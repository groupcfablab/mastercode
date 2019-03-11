//libraries
#include "motordrivesysupdated.h"

//globals
controlsys motor_left, motor_right;

void setup()
{
	const controllertype system_type = openloop;
	const int pwmpin__left = 5;
	const int pwmpin__right = 6;

	const int directionpin_left = 52;
	const int directionpin_right = 22;

	const int buttonpins[3] = { 38,40,42 };
	const int potentiometerpin = A0;

	const ArduinoInterruptNames interruptpin_left = int_4;//int_4=D2
	const ArduinoInterruptNames interruptpin_right = int_5;//int_5=D3

	const int reading_rate_ms = 400;
	const int pwm_update_rate_ms = 100;

	const int magnets_on_circle = 49;
	const int min_pwm_for_motor = 60;

	const int max_rated_rpm = 2000;

	motor_left.setup(system_type, pwmpin__left, directionpin_left, buttonpins, potentiometerpin, interruptpin_left, reading_rate_ms, pwm_update_rate_ms, magnets_on_circle, min_pwm_for_motor, max_rated_rpm);
	motor_right.setup(system_type, pwmpin__right, directionpin_right, buttonpins, potentiometerpin, interruptpin_right, reading_rate_ms, pwm_update_rate_ms, magnets_on_circle, min_pwm_for_motor, max_rated_rpm);
}

void loop()
{
	motor_left.routine();
	motor_right.routine();
}