#ifndef SERIALPRINTERPLOTTER_H
#define SERIALPRINTERPLOTTER_H

#include <Arduino.h>



class SerialPrinterPlotter{
protected:
	static const int MAX_CHARACTERS=75;
	static const int MAX_VARS=5;

	char printstring[MAX_CHARACTERS+2];
	
	
	
	//int tot_characters;
	double min_val[MAX_VARS], max_val[MAX_VARS];
	
	//
	char var_symbol[MAX_VARS];
	char target_symbol[MAX_VARS];
	//int approximation_pct[MAX_VARS];

	void reset_printstring()
	{
		int index;
		for(index=0; index<=MAX_CHARACTERS; index++)
			printstring[index]=' ';
		printstring[index]='\0';
	}
public:
	SerialPrinterPlotter()
	{
		static const int MAX_SAMPLES=5;
		char sample_var_symbols[MAX_SAMPLES]={'*','o','.','x', '+'};
		char sample_target_symbols[MAX_SAMPLES]={'|','I','!','i', 'V'};

		for(int i=0; i<MAX_VARS; i++)
		{
			min_val[i]=0.0;
			max_val[i]=100.0;
			//approximation_pct[i]=0;
			var_symbol[i]=sample_var_symbols[i%MAX_SAMPLES];
			target_symbol[i]=sample_target_symbols[i%MAX_SAMPLES];
		}

		reset_printstring();
	}

	// this sets the bounds for the values that can be printed (clipped to stay inside these bounds):
	// a value close to min os near the left edge of the screen; close to max, MAX_CHARACTERS on the right;
	void set_bounds(int varindex, double in_min_val, double in_max_val)
	{
		if(varindex>=0 && varindex<MAX_VARS)
		{
			min_val[varindex]=in_min_val;
			max_val[varindex]=in_max_val;
			//
			Serial.begin(9600);
			//
		}
	}

	// this appends the current val to the print_string (prepare for printing)	
	void appendval(int varindex, double val, double target, int approx_pctg=0)
	{
		int plot_val_index, min_val_var, max_val_var;
		double plotval;
		char symbol, tar_symbol;

		if(varindex>=0 && varindex<MAX_VARS)
		{
			min_val_var=min_val[varindex];
			max_val_var=max_val[varindex];
			symbol=var_symbol[varindex];
			tar_symbol=target_symbol[varindex];
		}
		else
			return;
		
		// print a band around the target value that represtns the required % error
		if(approx_pctg>0)
		{
			// upper band: + approx_pctg/2;
			plotval = target*( 1 + ( (double)approx_pctg )/200 );  
			plot_val_index=(int)map(plotval , min_val_var, max_val_var, 0, MAX_CHARACTERS);
			plot_val_index=constrain(plot_val_index, 0, MAX_CHARACTERS); 
			if(printstring[plot_val_index]==' ')
				printstring[plot_val_index]=']';

			// lower band: - approx_pctg/2;
			plotval = target*( 1 - ( (double)approx_pctg )/200 );  
			plot_val_index=(int)map(plotval , min_val_var, max_val_var, 0, MAX_CHARACTERS);
			plot_val_index=constrain(plot_val_index, 0, MAX_CHARACTERS); 
			if(printstring[plot_val_index]==' ')
				printstring[plot_val_index]='[';
		}

		// target value
		plotval=target;  
		plot_val_index=(int)map(plotval , min_val_var, max_val_var, 0, MAX_CHARACTERS);
		plot_val_index=constrain(plot_val_index, 0, MAX_CHARACTERS); 
		if(printstring[plot_val_index]==' ')
			printstring[plot_val_index]=tar_symbol;

		// current value
		plotval=val;  
		plot_val_index=(int)map(plotval , min_val_var, max_val_var, 0, MAX_CHARACTERS);
		plot_val_index=constrain(plot_val_index, 0, MAX_CHARACTERS); 
		printstring[plot_val_index]=symbol;
		//
	}
	
	// this appends the current val to the print_string (prepare for printing)
	void print_the_string()
	{
		Serial.println(printstring);
		reset_printstring();
	}
};



#endif