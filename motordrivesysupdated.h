#ifndef MOTORDRIVESYSUPDATED_H
#define MOTORDRIVESYSUPDATED_H
/*there are 5 inputs to the system, 3pusbuttons, potentiometer and hallFX sensor
we need to attach the hallfx sensor to an interrupt, as its readings are critical
potentiometer is to just adjust the target value for speed and the buttons are just 
user interface for the system (on, motor stop, reverse direction)
*/

// libraries used
#include "InterruptBasedSpeedMeasure.h"
#include "IntervalCheckTimer.h"
#include "DCmotor.h"
#include "basic_speed_PID.h"
#include "inputlib.h"
#include <math.h>
//----


// globals
bool serial_flag = false;
enum ctrl_action { idle, do_reading, do_action } ;
enum { do_nothing, changedir, changedircomplete } dir_flag;
enum controllertype { indef, openloop, closedloop };
enum buttonflags { off, pressed, released };
command_list_enum commands[3] = { switch_on,switch_off,change_spin_dir };
struct coords
{
	int time;
	int value;
};
//for the moving average
const int numReadings = 2;
int readings[numReadings];  
//----

class inputsystem {
protected:
	//objects
	InterruptSpeedMeasure rotation_counter;
	inputlib potentiometer;
	inputs pushbuttons;

	//flags
	bool init_flag, setup_flag = false;

	//protected functions
	bool pushbuttonstoinputs(const int* buttonpins)
	{
		for (int i = 0; i < 3; i++)
		{
			in_push_button tempbutton(buttonpins[i], commands[i]);
			pushbuttons.add_in_push_button(tempbutton);
		}
		return true;
	}

public:
	//constructor without setup
	inputsystem()
	{
		init_flag = true;
	}
	//constructor with setup
	inputsystem(const int* buttonpins, const int potentiometerpin, const ArduinoInterruptNames interruptpin, const int interruptsoncircle = 6)
	{
		init_flag = true;
		if (setup(buttonpins, potentiometerpin, interruptpin, interruptsoncircle))
		{
			setup_flag = true;
		}
		else 
		{ 
			if (serial_flag) { Serial.println("Error in system setup"); }
		}
	}

	bool setup(const int* buttonpins, const int potentiometerpin, const  ArduinoInterruptNames interruptpin, const int interruptsoncircle = 6)
	{
		Serial.begin(9600);
		serial_flag = true;
		bool success = false;
		if (init_flag && !setup_flag)
		{
			success = pushbuttonstoinputs(buttonpins);
			if (!success&&serial_flag) { Serial.println("Pushbuttons FAIL"); }
			success &= potentiometer.setup_potentiometer(potentiometerpin);
			if (!success&&serial_flag) { Serial.println("Pot FAIL"); }
			rotation_counter.setupSpeedMeasure(interruptpin, interruptsoncircle);
			success &= rotation_counter.isEnabled();
			if (!success&&serial_flag) { Serial.println("Interrupt FAIL"); }
			setup_flag = true;
			return success;
		}
		else { return success; }
	}

	bool issetupandworking() { return setup_flag; }

	bool takereadings(double& rpm, command_list_enum& command, int& potvoltage)
	{
		if (setup_flag)
		{
			rpm = rotation_counter.getRPMandUpdate();//not my libary so cant change it to return success
			bool buttonfound = pushbuttons.check_n_get_command(command);
			bool success = potentiometer.read_input(potvoltage);
			if (!success&&serial_flag) { Serial.println("potread fail"); }
			return buttonfound;
		}
	}


};

class outputsystem {
protected:
	//objects
	HBridgeDCmotor motor;
	di_LED dirpin2;
	//flags
	bool inv_flag, init_flag, setup_flag = false;
	//protected functions
	void dirchange()
	{
		if (motor.isStarted())
		{
			motor.stop();
			delay(500);
			motor.changedir();
			if (inv_flag)
			{
				if (dirpin2.isOn())
				{
					dirpin2.switch_off();
				}
				else
				{
					dirpin2.switch_on();
				}
			}
			motor.start();
			
		}
		else
		{
			motor.changedir();
		}
		dir_flag = changedircomplete;
	}
	//setspeed with 8bit int
	bool setspeedndir(uint8_t speedpwm)
	{
		if (motor.isStarted())
		{

			if (dir_flag == changedir)
			{
				dirchange();
				if (dir_flag == changedircomplete)
				{
					dir_flag = do_nothing;
					return true;
				}
				else { return false; }
			}
			else
			{
				if (dir_flag == changedircomplete)
				{
					dir_flag = do_nothing;
				}
				motor.setSpeedPWM(speedpwm);
				return true;
			}

		}

	}

	bool checkifsetup()
	{
		if (setup_flag)
		{
			return true;
		}
		else
		{
			if (serial_flag) { Serial.println("Error - system not setup"); }
			return false;
		}
	}


public:
	//blank constructor
	outputsystem()
	{
		init_flag = true;
	}

	//constructor with inverter&setup
	outputsystem(int motorpin,int dirpin)
	{
		init_flag = true;
		{
			if (setupinv(motorpin, dirpin))
			{
				setup_flag = true;
			}
			else
			{
				if (serial_flag) { Serial.println("Error - failed to set up outputsys"); }
			}
		}
	}

	//constructor without inverter&setup
	outputsystem(int motorpin, int* dirpins)
	{
		init_flag = true;
		{
			if (setupnoinv(motorpin, dirpins))
			{
				setup_flag = true;
			}
			else
			{
				if (serial_flag) { Serial.println("Error - failed to set up outputsys"); }
			}
		}
	}

	// setup with inverter
	bool setupinv(int motorpin,int dirpin)
	{
		if (init_flag && !setup_flag) 
		{
			motor.setup_HBridgeDCmotor(motorpin, dirpin);
			motor.start();
			return true;
		}
		else
		{
			if (serial_flag) { Serial.println("Error in setting up ouptupsys- already setup"); }
			return false;
		}
	}

	// setup without inverter
	bool setupnoinv(const int motorpin, const int* dirpins)
	{
		if (init_flag && !setup_flag) 
		{
			if (true)//(sizeof(dirpins) / sizeof(dirpins[0])) this didnt work during debug
			{
				motor.setup_HBridgeDCmotor(motorpin, dirpins[0]);
				dirpin2.setup_LED(dirpins[1]);
				dirpin2.switch_off();//starts normally off
				motor.start();
				inv_flag = true;
				return true;
			}
			else
			{
				if (serial_flag) { Serial.println("Error in setting outputsys, dir pins not 2"); }
				return false;
			}
		}
		else
		{
			if (serial_flag) { Serial.println("Error in setting up ouptupsys- already setup"); }
			return false;
		}
	}

	bool setspeedadjusted(uint8_t pwm)
	{
		return setspeedndir(pwm);
	}

	//minpwm is the smallest 8bit value with which the motor is still able to rotate
	bool setspeedunadjusted(double rpm, uint8_t minpwm, double ratedrpm)
	{
		uint8_t pwm = (minpwm + (int)((rpm / ratedrpm)*(255 - minpwm)));
		return setspeedndir(pwm);
	}

	bool kickstart(int duration = 1000)
	{
		int starttime = millis();
		while (duration > (millis() - starttime))
		{
			setspeedadjusted(255);
		}
		return true;
	}

};

class controlsys {
protected:
	//objects
	inputsystem inputsysobj;
	outputsystem outputsysobj;
	basic_speed_PID pidcontrol;
	IntervalCheckTimer input_check, output_refreshrate, buttonstate_rate;

	//flags
	bool init_flag, setup_flag, pid_flag, kickstart_flag = false;
	controllertype systype = indef;
	ctrl_action state = idle;
	buttonflags onbuttonflag, stopbuttonflag, dirchangebuttonflag = off;
	bool syson, motoron, buttonfound, steprespcomplete = false;
	coords lastrise, lastpeak, lastmin, lastsettle;
	
	bool risecomplete, peakcomplete, mincomplete, settlecomplete, envelopeflag = false;
	//variables
	double pidoutputspeed, targetrpm, measuredrpm, max_rpm;
	command_list_enum buttoncommand = simple_none;
	uint8_t min_pwm;
	int potvoltage;
	uint8_t timeschecked = 0;
	const uint8_t envelope = 2;// percentage value
	
	//private functions
	double adjustrpmfromNbit(int input, int maxval, int Nbits)
	{
		return (((double)input) / (pow(2, Nbits)))*maxval;
	}
	bool checkforstate()
	{
		//for performance benefits, only check every interval, and dont do anything else
		// also wait for state to be changed to idle before changing it
		if (input_check.isMinChekTimeElapsedAndUpdate())
		{
			state = do_reading;
			
		}
		else if (output_refreshrate.isMinChekTimeElapsedAndUpdate())
		{
			state = do_action;
		}
		return true;
	}
	void buttoncheckNsetFlag(bool foundflag)
	{
		if (foundflag)
		{
			if (buttoncommand == commands[1])
			{
				if (onbuttonflag != released)
				{
					onbuttonflag = pressed;
				}

			}
			else if (buttoncommand == commands[2])
			{
				if (stopbuttonflag != released)
				{
					stopbuttonflag = pressed;
				}
			}
			else
			{
				if (dirchangebuttonflag != released)
				{
					dirchangebuttonflag = pressed;
				}

			}
		}
		else
		{	//first time after release, flag set to released state
			if (onbuttonflag == pressed) { onbuttonflag = released; }
			else if (stopbuttonflag == pressed) { stopbuttonflag = released; }
			else if (dirchangebuttonflag == pressed) { dirchangebuttonflag = released; }
		}
	}
	void xorcommand(bool& input)
	{
		if (input)
		{
			input = false;
		}
		else { input = true; }
	}

	void steprespupdate()
	{
		const unsigned long period = input_check.getInterCheck()/1000;
		if (!steprespcomplete)
		{
			//rise time
			if (!risecomplete)
			{
				if (measuredrpm <= targetrpm * 0.9)
				{
					lastrise.value = measuredrpm;
					lastrise.time = timeschecked * period;
					lastpeak.value = lastrise.value;
					
				}
				else { risecomplete = true; }
			}
			else
			{
				//settletime
				if (measuredrpm > ((1 - (double)envelope / 100))*targetrpm)
				{
					lastsettle.value = measuredrpm;
					lastsettle.time = timeschecked * period;
					envelopeflag = true;
				}
				else if (measuredrpm > ((1 + (double)envelope / 100))*targetrpm)
				{
					lastsettle.value = 0;//reset if leaves envelope
					lastsettle.time = 0;
					envelopeflag = false;
				}
				//peak time
				if (!peakcomplete)
				{
					if (measuredrpm > lastpeak.value)
					{
						lastpeak.value = measuredrpm;
						lastmin.value = lastpeak.value;
						lastpeak.time = timeschecked * period;
					}
				}
				if (!mincomplete)
				{
					if (measuredrpm < lastmin.value)
					{
						lastmin.value = measuredrpm;
						lastmin.time = timeschecked * period;
					}
				}
			}
			if (timeschecked * period > 20)//if sampling time exceeds max seconds, just takes the existing settle values
			{
				steprespcomplete = true;
				peakcomplete = true;
				mincomplete = true;
			}
		}
	}

	bool routineopenloop()
	{
		if (checkforstate())
		{
			if (state == do_reading)
			{
				buttonfound = inputsysobj.takereadings(measuredrpm, buttoncommand, potvoltage);
				//check if any fo the buttons is pressed down to flag
				buttoncheckNsetFlag(buttonfound);
				if (buttonfound)
				{
					if (serial_flag)
					{
						Serial.println("Button pressed");
					}
				}
				if (measuredrpm < (0.02*max_rpm))//if measured rpm is less than 2 percent rated rpm do kickstart
				{
					kickstart_flag = true;
				}
				if (onbuttonflag == released)
				{
					if (serial_flag) { Serial.println("On button released"); }
					onbuttonflag = off;
					xorcommand(syson);
				}
				if (!syson) {
					if (buttonstate_rate.isMinChekTimeElapsed()) { Serial.println("System off"); }
					state = idle;
					goto CHECKMOTOR;
				}

				if (serial_flag)
				{
					Serial.print("Current RPM: ");
					Serial.print(measuredrpm);
					Serial.print("Target %: ");
					Serial.println(adjustrpmfromNbit(potvoltage, 100, 10));
				}
				state = idle;
			}
			else if ((state == do_action) && syson)
			{
				if (stopbuttonflag == released)
				{
					if (serial_flag) { Serial.println("Stop button released"); }
					stopbuttonflag = off;
					xorcommand(motoron);
				}
				if (!motoron)
				{
				CHECKMOTOR:	outputsysobj.setspeedadjusted(0);
					if (buttonstate_rate.isMinChekTimeElapsedAndUpdate()) { Serial.println("motor off"); }
					state = idle;
					goto ENDOPENLOOP;
				}
				targetrpm = adjustrpmfromNbit(potvoltage, max_rpm, 10);
				if (false)//kickstart_flag// my motor is very unstable so kickstarting is not feasible
				{
					if (outputsysobj.kickstart())
					{
						kickstart_flag = false;
					}
				}
				else
				{
					if (dirchangebuttonflag == released)
					{

						if (buttonstate_rate.isMinChekTimeElapsedAndUpdate()) { Serial.println("Direction button released"); }
						dirchangebuttonflag = off;
						dir_flag = changedir;
					}
					if (outputsysobj.setspeedunadjusted(targetrpm, min_pwm, max_rpm))
					{
						state = idle;
					}
					else
					{
						if (serial_flag) { Serial.println("Error in adjusting speed"); }
						return false;
					}
				}

			ENDOPENLOOP:return true;
			}
			else
			{
				//						if (serial_flag) { Serial.println("System off"); }
				return false;
			}

		}
		else
		{
			if (serial_flag) { Serial.println("Error in routine, could not check if time for update"); }
			return false;
		}
	}

	bool routineclosedloop()
	{
		if (pid_flag)
		{
			if (checkforstate())
			{
				if (state == do_reading)
				{
					//check if any fo the buttons is pressed down to flag
					buttonfound = inputsysobj.takereadings(measuredrpm, buttoncommand, potvoltage);
					/*if (!steprespcomplete) 
					{ 
						timeschecked++;
					}*/

					buttoncheckNsetFlag(buttonfound);
					/*if (buttonfound)
					{
						if (serial_flag)
						{
							Serial.println("Button pressed");
						}
					}*/
					if (onbuttonflag == released)
					{
//						if (serial_flag) { Serial.println("On button released"); }
						onbuttonflag = off;
						xorcommand(syson);
					}
					if (!syson) {
						if (buttonstate_rate.isMinChekTimeElapsed()) { Serial.println("System off"); }
						goto CHECKMOTORCL;
					}
					state = idle;
				}
				else if (state == do_action && syson)
				{
					if (stopbuttonflag == released)
					{
						stopbuttonflag = off;
						xorcommand(motoron);
					}
					if (!motoron)
					{
					CHECKMOTORCL:	outputsysobj.setspeedadjusted(0);
						if (buttonstate_rate.isMinChekTimeElapsedAndUpdate()) { Serial.println("motor off"); }
						goto ENDCLOSEDLOOP;
					}
					targetrpm = adjustrpmfromNbit(potvoltage, max_rpm, 10);
					measuredrpm = movavg(measuredrpm);
					pidoutputspeed = pidcontrol.ComputePID_output(potvoltage, measuredrpm);
					pidoutputspeed = adjustrpmfromNbit(pidoutputspeed, max_rpm, 8);
					steprespupdate();
					if (serial_flag)
					{
//						Serial.print(timeschecked*input_check.getInterCheck() / 1000);
//						Serial.print(",");
						Serial.print(measuredrpm);
						Serial.print(",");
						Serial.print(targetrpm);
						Serial.print(",");
						Serial.print(pidoutputspeed);
/*
						Serial.print(",");
						//				Serial.print("Rise time and value");
						Serial.print(lastrise.value);
						Serial.print(",");
						//				Serial.print(lastrise.time);
						//				Serial.print("Peak time and value");
						Serial.print(lastpeak.value);
						Serial.print(",");
						//				Serial.print(lastpeak.time);
						//				Serial.print("Min time and value");
						Serial.print(lastmin.value);
						Serial.print(",");
						//				Serial.print(lastmin.time);
						//				Serial.print("Settle time and value");
						Serial.print(lastsettle.value);
						//				Serial.println(lastsettle.time);*/
						Serial.println();


					}
					if (dirchangebuttonflag == released)
					{
						//if (buttonstate_rate.isMinChekTimeElapsedAndUpdate()) { Serial.println("Direction button released"); }
						dirchangebuttonflag = off;
						dir_flag = changedir;
					}
					if (outputsysobj.setspeedunadjusted(pidoutputspeed, min_pwm, max_rpm))
					{
						state = idle;
					}
					else
					{
						//if (serial_flag) { Serial.println("Error in adjusting speed"); }
						return false;
					}
				ENDCLOSEDLOOP:return true;
				}
				else
				{
					//						if (serial_flag) { Serial.println("System off"); }
					return false;
				}
			}
			else
			{
				//if (serial_flag) { Serial.println("Error in routine, could not check if time for update"); }
				return false;
			}
		}
		// i only allow custom pid values
		else
		{
			//if (serial_flag) { Serial.println("Error in routine, pid values not setup"); }
			return false;
		}

	}

	//for the moving average function
    // the readings from the analog input
    int readIndex = 0;              // the index of the current reading
    int total = 0;                  // the running total
    int average = 0;                // the average
    double movavg(int reading)
    {
        // subtract the last reading:
        total = total - readings[readIndex];
        // read from the sensor:
        readings[readIndex] = reading;
        // add the reading to the total:
        total = total + readings[readIndex];
        // advance to the next position in the array:
        readIndex = readIndex + 1;
        
        // if we're at the end of the array...
        if (readIndex >= numReadings) {
            // ...wrap around to the beginning:
            readIndex = 0;
        }
        
        // calculate the average:
        average = total / numReadings;
        // send it to the computer as ASCII digits
        return average;
    }

	
    
public:
	//blank constructor
	controlsys()
	{
		init_flag = true;
	}
	// with hex inverter
	controlsys(const controllertype system_type, const int motorpin, const int dirpin, const int* buttonpins, const int potpin, const ArduinoInterruptNames interruptpin, const int checkinterval = 250, const int refreshrate = 100, const int interruptsoncircle = 6, const uint8_t minpwm = 55, const double ratedrpm = 6000)
	{
		init_flag = true;
		bool success = setup(system_type, motorpin, dirpin, buttonpins, potpin, interruptpin, checkinterval, refreshrate, interruptsoncircle, minpwm, ratedrpm);
		state = idle;
		
	}
	
	//without hex inverter
	controlsys(const controllertype system_type, const int motorpin, const int* dirpins, const int* buttonpins, const int potpin, const ArduinoInterruptNames interruptpin, const int checkinterval = 250, const int refreshrate = 100, const int interruptsoncircle = 6, const uint8_t minpwm = 55, const double ratedrpm = 6000)
	{
		init_flag = true;
		bool success = setup(system_type, motorpin, dirpins, buttonpins, potpin, interruptpin, checkinterval, refreshrate, interruptsoncircle, minpwm, ratedrpm);

		state = idle;
		setup_flag = success;

	}
	
	bool setup(const controllertype system_type, const int motorpin, const int dirpin, const int* buttonpins, const int potpin, const ArduinoInterruptNames interruptpin, const int checkinterval = 250, const int refreshrate = 100, const int interruptsoncircle = 6, const uint8_t minpwm = 55, const double ratedrpm = 6000)
	{
		Serial.begin(9600);
		serial_flag = true;
		if (init_flag && !setup_flag)
		{
			bool success = inputsysobj.setup(buttonpins, potpin, interruptpin, interruptsoncircle);
			success &= outputsysobj.setupinv(motorpin, dirpin);
			input_check.setInterCheck(checkinterval);
			output_refreshrate.setInterCheck(refreshrate);
			buttonstate_rate.setInterCheck(2000);
			max_rpm = ratedrpm;
			min_pwm = minpwm;
			systype = system_type;
			setup_flag = success;
			return success;
		}
		else
		{
			if (serial_flag) { Serial.println("Error in setup, system already set up"); }
		}
	}
	
	bool setup(const controllertype system_type, const int motorpin, const int* dirpins, const int* buttonpins, const int potpin, const ArduinoInterruptNames interruptpin, const int checkinterval = 250, const int refreshrate = 100, const int interruptsoncircle = 6, const uint8_t minpwm = 55, const double ratedrpm = 6000)
	{
		Serial.begin(9600);
		serial_flag = true;
        for (int thisReading = 0; thisReading < numReadings; thisReading++)
        {
            readings[thisReading] = 0;
            
        }
		if (init_flag && !setup_flag)
		{
			bool success = inputsysobj.setup(buttonpins, potpin, interruptpin, interruptsoncircle);
			if (!success) { Serial.println("INPUTS FAIL"); }
			success &= outputsysobj.setupnoinv(motorpin, dirpins);
			if (!success) { Serial.println("OUTPUTS FAIL"); }
			input_check.setInterCheck(checkinterval);
			output_refreshrate.setInterCheck(refreshrate);
			buttonstate_rate.setInterCheck(2000);
			max_rpm = ratedrpm;
			min_pwm = minpwm;
			systype = system_type;
			setup_flag = success;
			return success;
		}
		else
		{
			if (serial_flag) { Serial.println("Error in setup, system already set up"); }
		}

	}
	
	bool setup_pid(double kp, double ki, double kd)
	{
		pidcontrol.set_gainvals(kp, ki, kd);
		pid_flag = true;
		return true;
	}

	bool routine()
	{
		if (init_flag&&setup_flag)
		{
			if (systype == openloop)
			{
				routineopenloop();
			}
			else if (systype == closedloop)
			{
				routineclosedloop();
			}
			else
			{
				if (serial_flag) { Serial.println("Error in routine, system type incorrect"); }
				return false;
			}
		}
		else
		{
			if (serial_flag) { Serial.println("Error in routine, not setup or initialised"); }
			return false;
		}
	}
};
#endif
