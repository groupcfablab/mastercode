#ifndef ProperLED_h
#define ProperLED_h

#include <Arduino.h>


// abstract led
class abstr_LED{
protected:
	int out_pin;	
	boolean LED_enabled;
	boolean led_on;
	//
	inline void setEnabled(boolean enabled){LED_enabled=enabled;}	
	inline void setOn(boolean isOn){led_on=isOn;}	
	
	// set the pin to the ON vlue
	virtual void set_pin_on()=0;
	
	// set the pin to the OFF vlue
	virtual void set_pin_off()=0;
	//
public:
	abstr_LED(){out_pin=-1; setEnabled(false);}
	
	inline boolean isEnabled(){return LED_enabled;}
	inline boolean isOn(){return led_on;}
	
	void toggle() {isOn()?switch_off():switch_on();}
	
	// switch-on the LED (it's OFF by default)
	virtual void switch_on()=0;
	
	// switch-off the LED (it's OFF by default)
	virtual void switch_off()=0;
	
	// get current birgtness (when supported)
	virtual int get_brightness()=0;	
	
	virtual void setup_LED(int out_pin_numb)
	{
		if(out_pin_numb>=0)
		{
			out_pin=out_pin_numb;
			pinMode(out_pin,OUTPUT);
			setEnabled(true);
			//switch off by default
			setOn(true);
			switch_off();
			//
		}
	}
	//
	virtual void blink(int waiting_ms)
	{
		if (isEnabled() && (waiting_ms>0))
		{
			switch_on();
			delay(waiting_ms);
			//
			switch_off();
			delay(waiting_ms);
		}
	}
	virtual void brighter() {;}
	virtual void dimmer() {;}
	virtual void set_brightness(int in_pwm) {;}
	virtual void set_max_bright() {;}	
	//
};


// digital led (ON/OFF)
class di_LED: public abstr_LED{
protected:	
	virtual void set_pin_on() {digitalWrite(out_pin, HIGH);}
	virtual void set_pin_off()	{digitalWrite(out_pin, LOW);}
public:
	di_LED() : abstr_LED() {;}
	di_LED(int out_pin_numb) : abstr_LED(){setup_LED(out_pin_numb);}	
	//
	virtual void switch_on()
	{
		if( isEnabled() && !isOn() )
		{
			// switch LED ON
			set_pin_on();
			setOn(true);
		}

	}
	//
	virtual void switch_off()
	{
		if( isEnabled() && isOn() )
		{
			// switch LED OFF
			set_pin_off();
			setOn(false);
		}	
	}
	//
	virtual int get_brightness() {return (isOn()?255:0 );}
};



// digital led (ON/OFF) with inverter (set tension to low to shine led)
class di_LED_inv : public di_LED{
protected:	
	virtual void set_pin_on(){digitalWrite(out_pin, LOW);}
	virtual void set_pin_off(){digitalWrite(out_pin, HIGH);}
public:
	di_LED_inv() : di_LED() {;}
	di_LED_inv(int out_pin_numb) : di_LED(out_pin_numb) {set_pin_off();setOn(false);}
};



// analog led (PWM)
class an_LED: public abstr_LED{
protected:
	int current_pwm;
	int prev_pwm;
	int pwm_increment;
	int pwm_off_value;
	virtual void set_pin_on() {analogWrite(out_pin,current_pwm);}
	virtual void set_pin_off() {analogWrite(out_pin,pwm_off_value);}
	void setup_default_vals(){current_pwm=127;prev_pwm=127;pwm_increment=2;pwm_off_value=0;}
public:
	an_LED(): abstr_LED(){setup_default_vals();}
	an_LED(int out_pin_numb):abstr_LED(){setup_default_vals();setup_LED(out_pin_numb);}	
	//
	virtual void switch_on()
	{
		//// temp test [ in setup() add this line: Serial.begin(9600); ]
		//Serial.print("Switching on ");	
		//Serial.println();		
		////

		if( isEnabled() && !isOn() )
		{
			// if PWM is zero, resume previous value
			//if(current_pwm==pwm_off_value)
			current_pwm=prev_pwm;

			// switch LED ON
			set_pin_on();
			setOn(true);
		}

	}
	//
	virtual void switch_off()
	{
		//// temp test [ in setup() add this line: Serial.begin(9600); ]
		//Serial.print("Switching off ");	
		//Serial.println();		
		////

		if( isEnabled() && isOn() )
		{
			// store PWM previous value
			prev_pwm=current_pwm;

			// switch LED OFF
			set_pin_off();
			setOn(false);

			// update PWM value
			current_pwm=pwm_off_value;
		}

	}
	//
	virtual void brighter()
	{
		int incremented_pwm;

		//// temp test [ in setup() add this line: Serial.begin(9600); ]
		//Serial.print("brighter ; current pwm is ");
		//Serial.print(current_pwm);
		//Serial.println();		
		////
		if( isEnabled() )
		{	
			if( isOn() )
			{// led is on: make brighter
				if(current_pwm<255)
				{
					// store PWM previous value
					prev_pwm=current_pwm;		

					// set a higher (brighter) value
					incremented_pwm=current_pwm + pwm_increment;
					//incremented_pwm=current_pwm + 4;

					current_pwm=min( incremented_pwm , 255 );
					set_pin_on();
				}
			}
			else
			{	// led is off: ajust it so that it will be brighter next time it is on
				if(prev_pwm<255)
				{
					// set a higher (brighter) value
					incremented_pwm=prev_pwm + pwm_increment;
					prev_pwm=min( incremented_pwm , 255 );
				}
			}
		}
	}
	//
	virtual void dimmer()
	{
		int decremented_pwm;

		//// temp test [ in setup() add this line: Serial.begin(9600); ]
		//Serial.print("dimmer ; current pwm is ");
		//Serial.print(current_pwm);
		//Serial.println();		
		////
		if( isEnabled() )
		{	
			if( isOn() )
			{// led is on: make dimmer
				if(current_pwm>0)
				{
					// store PWM previous value
					prev_pwm=current_pwm;		

					// set a lower (dimmer) value
					decremented_pwm=current_pwm - pwm_increment; 
					current_pwm=max(decremented_pwm , 0 );
					set_pin_on();
				}
			}
			else
			{	// led is off: ajust it so that it will be dimmer next time it is on
				if(prev_pwm>0)
				{
					// set a lower (dimmer) value
					decremented_pwm=prev_pwm - pwm_increment;
					prev_pwm=min( decremented_pwm , 0 );
				}
			}
		}
	}
	//
	virtual void set_brightness(int in_pwm)
	{
		//// temp test [ in setup() add this line: Serial.begin(9600); ]
		//Serial.print("set_brightness ; in_pwm pwm is ");
		//Serial.print(in_pwm);
		//Serial.println();		
		////

		if( isEnabled() )
		{
			boolean switch_back_off=false;
			in_pwm=abs(in_pwm);

			// switch on if needed
			if(!isOn())
			{
				current_pwm=pwm_off_value;
				prev_pwm=pwm_off_value;
				switch_on();
				switch_back_off=true;
			}

			// store PWM previous value
			prev_pwm=current_pwm;		

			// set a  value
			current_pwm=min( in_pwm , 255 );
			set_pin_on();

			// switch off if needed
			if(switch_back_off)
				switch_off();
		}
	}
	//
	virtual void set_max_bright() {set_brightness(255);}
	virtual int get_brightness() {return current_pwm;}
};


// analog led (PWM) with inverter (reduce PWM to make brighter)
class an_LED_inv: public an_LED{
public:
	an_LED_inv(): an_LED(){pwm_off_value=255;}
	an_LED_inv(int out_pin_numb):an_LED(out_pin_numb){pwm_off_value=255;switch_off();}	
	//
	virtual void brighter() {an_LED::dimmer();}
	virtual void dimmer() {an_LED::brighter();}
	virtual void set_brightness(int in_pwm) {an_LED::set_brightness(255 - in_pwm);}
	virtual int get_brightness() {return (255 - current_pwm);}
};







// */

#endif