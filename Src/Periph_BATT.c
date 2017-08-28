/*---------------			 		 PROGRAM FILE            ------------------
--------------- PERIPHERAL FUNCTIONS: BATTERY CONTROL ---------------

Company: ROBOTAERO
Project: UGV
Author: Gabriel Galvao (intern)/ Andrey Shirokov (tutor)
Description:	
This peripheral file contains programs that calculate battery State Of Charge (SOC) based on ADC measured output battery voltage.
The input electrical topology is a series of voltage dividers, for each cell in the battery pack, as follows:
GND
CH1: RB=51k / RP=15k
CH2: RB=15k / RP=25,5k (two 51k in parallel)
CH3: RB=100k / RP=300k
The RB are the resistors between GND and ADC input channels, and the RP resistors are the ones between the battery output channels
and the ADC input channels.
The SOC is calculated using a lookup table found in the internet, for LiPo batteries. We use a linear interpolation between neighbour values.
To reduce noise in the ADC, several samples are taken within a second (approximately 54) and afterwards their mean is the value used for calculations.
The values used for voltage dividers and offsets were calculated/measured with a multimeter, and may be susceptible to changes if the resistors 
or ADC channels used are changed.
-----------------------------------------------------------------
*/

/* ------- INCLUDES ------- */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "Periph_BATT.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

/* ------- PRIVATE DEFINITIONS ------- */

float ADC_voltages [BATT_N_CELLS];				//Sampled voltage read of ADC
float BATT_voltages [BATT_N_CELLS];				//Converted battery cell voltages
float Batt_SOC [BATT_N_CELLS];						//Batteries' State Of Charge

/* ------- FUNCTION IMPLEMENTATION ------- */

float Battery_SOC (float voltage){
	uint8_t i = 0;
	float soc;
	if(voltage >= voltage_levels[20])
		return 100.0;
	if(voltage <= voltage_levels[0])
		return 0.0;
	while(voltage > voltage_levels[i])
		i++;
	soc = voltage-voltage_levels[i-1];
	soc = 5.0*soc/(voltage_levels[i]-voltage_levels[i-1]);
	soc += SOC_levels[i-1];
	return soc;
}

float TrueCellVoltage (float read_v, float base_v, int n_cell){
	float out_v;
	out_v = Batt_Voltage_Dividers[n_cell]*(read_v+ADC_offsets[n_cell]) - base_v;
	out_v = max(out_v,voltage_levels[0]);																						//We suppose battery voltages can't be out of these boundaries
	out_v = min(out_v,voltage_levels[20]);																					//If they are, that is not a good sign (shown by SOC)
	return out_v;
}

float BATT_Conversions (void){
	float v_accum = 0;
	for(int i = 0; i<BATT_N_CELLS; i++){														
		sprintf(V_lvls[i], "%.3f M\n\r", ADC_voltages[i]);																												//Outputs directly ADC measured voltages
		BATT_voltages[i] = LP_alpha*TrueCellVoltage(ADC_voltages[i],v_accum,i) + (1-LP_alpha)*BATT_voltages[i];		//Reconstitutes battery cells' voltages and applies Low-pass filter
		sprintf(V_lvls[i+BATT_N_CELLS], "%.3f B\n\r", BATT_voltages[i]);																					//Outputs battery reconstituted voltages
		v_accum += BATT_voltages[i];																																							//Variable used for accumulating voltages, given the electrical topology used
		Batt_SOC[i] = Battery_SOC(BATT_voltages[i]);																															//Calculates battery State of Charge by interpolating a lookup table
		sprintf(V_lvls[i+2*BATT_N_CELLS], "%.3f S\n\r", Batt_SOC[i]);																							//Outputs the SOC result calculations
	}
	return Battery_SOC(v_accum/(float)BATT_N_CELLS);																														//Returns the mean SOC value
}

void BATT_DebugAppendToString (char * outstr){
	for(int i = 0; i<3*BATT_N_CELLS; i++){																				
		strcat(outstr,(const char *) V_lvls[i]);														//Concatenates string with ADC measures/treatment
	}
}
