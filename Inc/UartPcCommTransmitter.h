#pragma once

#include "cmsis_os.h"

extern QueueHandle_t motorTaskTxQueue, usTaskTxQueue, gpsTxQueue, eventTxQueue;

void UartPcCommTransmitterFunc(void const * argument);
