#pragma once

#include "cmsis_os.h"

typedef enum
{
    EVT_GETTING_CLOSE_TO_WALL = 1 << 1,
    EVT_CLOSE_TO_WALL         = 1 << 2
} Event;

extern QueueHandle_t eventQueue;
extern QueueHandle_t eventAckQueue;

void EventControllerTaskFunc(const void *params);

uint8_t eventCheckGettingCloseToWall(void);
uint8_t eventCheckCloseToWall(void);
