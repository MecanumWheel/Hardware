/*--------------- 					 HEADER FILE						-----------------
--------------- PERIPHERAL FUNCTIONS: BATTERY CONTROL ---------------

Company: ROBOTAERO
Project: UGV
Author: Gabriel Galvao (intern)/ Andrey Shirokov (tutor)
Description:	
This peripheral file contains programs that calculate battery State Of Charge (SOC) based on ADC measured output battery voltage.
The input electrical topology is a series of voltage dividers, for each cell in the battery pack, as follows:
GND to the analog ground reference (usually connected to normal GND)
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

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

/* ------- DEFINES ------- */
# define max(a,b)   (a<b ? b : a)   			//Auxiliary macros of maximum and minimum     
# define min(a,b)   (a>b ? b : a) 
#define BATT_N_CELLS 		3									//Number of battery cells to be measured
#define v_step 					0.0008056640625		//minimum resolution of ADC for 12 bits
#define LP_alpha 				0.5								//Low-pass filter alpha value

/* ------- PRIVATE DEFINITIONS ------- */

static int SOC_levels [21] = {0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};
static float voltage_levels [21] = {3.27,3.61,3.69,3.71,3.73,3.75,3.77,3.79,3.80,3.82,3.84,3.85,3.87,3.91,3.95,3.98,4.02,4.08,4.11,4.15,4.20};
static float ADC_offsets [BATT_N_CELLS] = {-0.16,-0.03,-0.01};							//Input offset of ADC (value systematically added to the real measures to get ADC read value)
static float Batt_Voltage_Dividers [BATT_N_CELLS] = {1.291,2.727,4.009};		//Multipliers to raise up to scale the values measured

static char V_lvls [3*BATT_N_CELLS][11];	//VOLTAGE levels strings - for debug (UART SEND)

/* ------- FUNCTION PROTOTYPES ------- */
// - Function for determining Battery State of Charge based on a linear interpolation of a lookup table - //
float Battery_SOC (float voltage);
// - Function for rescaling voltage to real battery voltage, based of electrical input topology - //
float TrueCellVoltage (float read_v, float base_v, int n_cell);
// - Function for converting voltages to SOC -> Returns average SOC - //
float BATT_Conversions (void);
// - Function to output debugging values to string - //
void BATT_DebugAppendToString (char * outstr);
