//libraries
#include "motordrivesysupdated.h"

//globals
controlsys motor_left, motor_right;

void setup()
{
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

	const int max_rated_rpm = 300;
      
	motor_left.setup(system_type, pwmpin__left, directionpin_left, buttonpins, potentiometerpin, interruptpin_left, reading_rate_ms, pwm_update_rate_ms, magnets_on_circle, min_pwm_for_motor, max_rated_rpm);
	motor_right.setup(system_type, pwmpin__right, directionpin_right, buttonpins, potentiometerpin, interruptpin_right, reading_rate_ms, pwm_update_rate_ms, magnets_on_circle, min_pwm_for_motor, max_rated_rpm);
        double k=0.0001;
        motor_left.setup_pid(20*k,150*k,0.3*k);
        //motor_right.setup_pid(20*k,150*k,0.3*k);
}

void loop()
{
	motor_left.routine();
	//motor_right.routine();
}
