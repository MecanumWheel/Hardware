/* - HEADER FILE FOR ULTRASONIC PERIPHERAL - MECANUM ROBOT - */
	#include "main.h"
	#include "stm32f4xx_hal.h"
	#include "cmsis_os.h"
	
	// - FUNCTION PROTOTYPES - //
	void Ultrasonic_DistCalculations(void);
	void Ultrasonic_CheckConditions(void);
