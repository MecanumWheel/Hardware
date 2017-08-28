//#include "UartPcCommTransmitter.h"

//#include "cmsis_os.h"
//#include <stm32f4xx_hal.h>

//#include <jansson.h>
//#include <string.h>

//extern QueueHandle_t usQ;
//extern UART_HandleTypeDef huart1;

//void UartPcCommTransmitterFunc(void const * argument)
//{
//    // Setup
//    
//    // Infinite loop
//    while (1)
//    {
//        if (uxQueueMessagesWaiting(usQ))
//        {
//            US_RxStruct qMsg;
//            xQueueReceive(usQ, (void*)&qMsg, portMAX_DELAY);
//            double values[] = {qMsg.Right, qMsg.Front, qMsg.Left, qMsg.Back};
//            json_t *jsonObj = json_pack("{sss[ffff]}", "s", "us", "v", values[0], values[1], values[2], values[3]);
//            char *msg = json_dumps(jsonObj, 0);
//            json_decref(jsonObj);
//            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg));
//            free(msg);
//        }
//    }
//}
