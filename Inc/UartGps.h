#pragma once

#include "cmsis_os.h"

typedef struct
{
    float lan, lon;
    char ns, we;
} GpsMessage;

extern QueueHandle_t gpsTxQueue;

void GpsReceiverFunc(void const * argument)
{
    // Setup
    
    // Infinite loop
    while(1) 
    {
        
    }
}
