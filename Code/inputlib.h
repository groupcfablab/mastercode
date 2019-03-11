#ifndef sensing_unit_h
#define sensing_unit_h
#include "Basic_Input.h"
// this library implements the "sensing unit" of my system
class inputlib {
protected:
	in_analog inp_pin;
	boolean enabled;
public:
	// default constructor: does not initialize the analog inputs
	inputlib() { enabled = false; }
	// constructor with argiuments (input pins): initialize both analog inputs (and checks)
	inputlib(int pin) : inp_pin(pin)
	{
		if (inp_pin.isInitialized())
			enabled = true;
		else
			enabled = false;
	}
	// checks if the unit is enabled
	boolean isEnabled() { return enabled; }
	// setup potentiometer
	bool setup_potentiometer(int pin)
	{
		if (!isEnabled())
		{
			if (!inp_pin.isInitialized()) inp_pin.setup_in_analog(pin);
			if (inp_pin.isInitialized()) enabled = true;
			return true;
		}
		else { return false; }
	}
	// reads both inputs
	boolean read_input(int& val)
	{
		boolean success = inp_pin.read_input(val);
		return (success);
	}
};
#endif
