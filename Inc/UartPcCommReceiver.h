#pragma once

#include "cmsis_os.h"

extern QueueHandle_t motorTaskRxQueue, usTaskRxQueue, gpsRxQueue, eventRxQueue;

void UartPcCommReceiverFunc(void const * argument);
