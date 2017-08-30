	
/* - PROGRAM FILE FOR ULTRASONIC PERIPHERAL - MECANUM ROBOT - */
	#include "main.h"
	#include "stm32f4xx_hal.h"
	#include "cmsis_os.h"
	#include "float.h"
	
	extern uint16_t Ultrasonic_flags;
	uint16_t US_time_acq [4][2];							//For every sensor we get two values, rising edge and falling
	float US_distance [4],US_min_dist;				//Array to hold distances in mm and minimum distance
	extern struct Ultrasonic_Distances US_RxStruct;	//Struct to be sent to the PC, with distances
	extern uint32_t channel;												//For timer channel selection
	
void Ultrasonic_DistCalculations(void){
	US_min_dist = FLT_MAX;
	for(int i=0; i<4; i++){																				//Distances calculations
		if((Ultrasonic_flags & (ULTRASONIC_CAPTURE_1 << i)) == 0){	//Checks if sensor has been measured
			//US_distance[i] = 0;																				//If not, set distance to 0
			continue;
		}
		if(US_time_acq[i][0] < US_time_acq[i][1])
			US_time_acq[i][1] = US_time_acq[i][1] - US_time_acq[i][0];
		else
			US_time_acq[i][1] = 0xFFFF - US_time_acq[i][0] + US_time_acq[i][1];
		US_distance[i] = US_time_acq[i][1]*(float)340/(float)(2*1000);					//Each timer tick should be one microsecond -> distance in mm 
		if(US_min_dist > US_distance[i] && US_distance[i] != 0)
			US_min_dist = US_distance[i];													//Gets minimum distance
		if(US_distance[i] > 4000)																		//If distance is more than 4m, then we send signal
			US_distance[i] = -1;
	}
}

void Ultrasonic_CheckConditions(void){
	if(US_min_dist < ULTRASONIC_T_ZERO){
		Ultrasonic_flags |= ULTRASONIC_WARNING;		//Sets hit collision avoidance state
		//SEND WARNING (could be just the flag)
	}
	if(US_min_dist < ULTRASONIC_T_MINUS){
		Ultrasonic_flags |= ULTRASONIC_DEAD;				//Sets dead state
	}
	if(US_min_dist > ULTRASONIC_T_ZERO && US_min_dist < ULTRASONIC_T_PLUS){
		Ultrasonic_flags &= ~ULTRASONIC_DEAD;
		Ultrasonic_flags |= ULTRASONIC_WARNING;
	}
	else if(US_min_dist > ULTRASONIC_T_PLUS){
		Ultrasonic_flags &= ~ULTRASONIC_DEAD;
		Ultrasonic_flags &= ~ULTRASONIC_WARNING;
	}
}
