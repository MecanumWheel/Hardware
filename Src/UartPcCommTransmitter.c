#include "UartPcCommTransmitter.h"

QueueHandle_t motorTaskTxQueue, usTaskTxQueue, gpsTxQueue, eventTxQueue;

void UartPcCommTransmitterFunc(void const * argument)
{
    // Setup
    
    // Infinite loop
    while (1)
    {
        if (uxQueueMessagesWaiting(motorTaskTxQueue))
        {
            //...
        }
    }
}
