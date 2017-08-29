#include "EventController.h"

QueueHandle_t eventQueue;
QueueHandle_t eventAckQueue;

volatile uint32_t eventStatus = 0;

void EventControllerTaskFunc(const void *params)
{
    while (1)
    {
        uint32_t buf = 0;
        xQueueReceive(eventQueue, &buf, 0);
        eventStatus |= buf;
        buf = 0;
        xQueueReceive(eventAckQueue, &buf, 0);
        eventStatus &= ~buf;
    }
}
    
uint8_t eventCheckGettingCloseToWall(void)
{
    return eventStatus | EVT_GETTING_CLOSE_TO_WALL;
}

uint8_t eventCheckCloseToWall(void)
{
    return eventStatus | EVT_CLOSE_TO_WALL;
}
