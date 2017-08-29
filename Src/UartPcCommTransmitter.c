#include "UartPcCommTransmitter.h"

#include "cmsis_os.h"
#include <stm32f4xx_hal.h>

#include <JSON.h>

#include "main.h"

extern QueueHandle_t UltrasonicToPC;
extern UART_HandleTypeDef huart2;

void UartPcCommTransmitterFunc(void const * argument)
{
    // Setup
    HAL_UART_Transmit(&huart2, (uint8_t*)"TEST\r\n", 6, HAL_MAX_DELAY);

    // Infinite loop
    while (1)
    {
        if (huart2.gState != HAL_UART_STATE_READY)
            continue;
        if (uxQueueMessagesWaiting(UltrasonicToPC))
        {
            struct Ultrasonic_Distances qMsg;
            xQueueReceive(UltrasonicToPC, (void*)&qMsg, portMAX_DELAY);
            float values[] = {qMsg.Right, qMsg.Front, qMsg.Left, qMsg.Back};
            char *msg = 0;
            size_t size = 0;
            jsonStart(&msg, &size);
            jsonAddString(&msg, &size, "s", "us");
            jsonAddFloatArray(&msg, &size, "v", values, 4);
            jsonEnd(&msg, &size);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            free(msg);
        }
    }
}
