#define JS_OPT 2

#if (JS_OPT == 1)

#include "UartPcCommReceiver.h"

#include <stm32f4xx_hal.h>

#include <string.h>
#include <jansson.h>

#define CANCEL_IF(arg) if (arg){jsonRxBufferUsed = 0; jsonRxBufferState = JSON_BUF_PROCESSED; continue;}

extern UART_HandleTypeDef huart2;

typedef struct
{
    uint8_t fr, fl, bl, br;
} MotorCmd;

enum JsonBufferState
{
    JSON_BUF_RECEIVING,
    JSON_BUF_RECEIVED,
    JSON_BUF_PROCESSING,
    JSON_BUF_PROCESSED
};

const char *typeKeyStr = "c";
const char *valueKeyStr = "c";
const char *typeMotorsStr = "m";
const char *typeEventAckStr = "ea";

uint8_t dmaRxBuffer[2] = {0};
uint8_t jsonRxBuffer[512] = {0};
volatile size_t jsonRxBufferUsed = 0;
volatile uint8_t jsonRxBufferState = JSON_BUF_PROCESSED;

static void handleIncomingByte(uint8_t byte)
{
    if (jsonRxBufferState == JSON_BUF_PROCESSED && byte != '{' 
        || jsonRxBufferState == JSON_BUF_PROCESSING 
        || jsonRxBufferState == JSON_BUF_RECEIVED)
        return;
    if (byte == '}')
        jsonRxBufferState = JSON_BUF_RECEIVED;
    else
        jsonRxBufferState = JSON_BUF_RECEIVING;
    jsonRxBuffer[jsonRxBufferUsed] = byte;
    jsonRxBufferUsed++;
}

static void processMotorsRequest(json_t *root)
{
    static MotorCmd motorCmd;
    json_t *valuesArr = json_object_get(root, valueKeyStr);
    if (!json_is_array(valuesArr))
        return;
    for (int i = 0; i < json_array_size(valuesArr); i++)
    {
        json_t *el = json_array_get(valuesArr, i);
        ((uint8_t*)&motorCmd)[i] = json_integer_value(el);
    }
}

static void processEventAckRequest(json_t *root)
{
    HAL_UART_Transmit_DMA(&huart2, jsonRxBuffer, jsonRxBufferUsed);
}

void Uart2HalfCpltHandler(void)
{
    handleIncomingByte(dmaRxBuffer[0]);
}

void Uart2CpltHandler(void)
{
    handleIncomingByte(dmaRxBuffer[1]);
}

void UartPcCommReceiverFunc(void const * argument)
{
    // Setup
    HAL_UART_Receive_DMA(&huart2, dmaRxBuffer, sizeof(dmaRxBuffer));
    
    // Infinite loop
    while (1)
    {
        if (jsonRxBufferState != JSON_BUF_RECEIVED)
            continue;
        jsonRxBufferState = JSON_BUF_PROCESSING;
        json_t *obj = json_loadb((const char *)jsonRxBuffer, jsonRxBufferUsed, 0, 0);
        CANCEL_IF(!obj);
        json_t *cmd = json_object_get(obj, typeKeyStr);
        CANCEL_IF(!cmd || !json_is_string(cmd));
        const char *cmdStr = json_string_value(cmd);
        if (!strcmp(cmdStr, typeMotorsStr))
            processMotorsRequest(obj);
        else if (!strcmp(cmdStr, typeEventAckStr))
            processEventAckRequest(obj);
        json_decref(obj);
        jsonRxBufferUsed = 0;
        jsonRxBufferState = JSON_BUF_PROCESSED;
    }
}

#elif (JS_OPT == 2)

#include "UartPcCommReceiver.h"
#include "motor.h"

#include <stm32f4xx_hal.h>

#include <string.h>
#include <jansson.h>

/**
 * Resources and management
 * - Parser unlocks uart receiver
 */

#define CANCEL_IF(arg) if (arg){jsonRxBufferUsed[!jsonCurrentRxBuffer] = 0; jsonJsonBufferState = JSON_BUF_PROCESSED; continue;}

extern UART_HandleTypeDef huart2;
extern xQueueHandle motorQueue;

enum JsonBufferState
{
    JSON_BUF_WAITING_DATA = 1 << 1,
    JSON_BUF_RECEIVING = 1 << 2,
    JSON_BUF_RECEIVED = 1 << 3,
    JSON_BUF_PROCESSING = 1 << 4,
    JSON_BUF_PROCESSED = 1 << 5
};

static const char *typeKeyStr = "c";
static const char *valueKeyStr = "v";
static const char *typeMotorsStr = "m";
static const char *typeEventAckStr = "ea";

static uint8_t dmaRxBuffer[2] = {0};
static uint8_t jsonRxBuffer[2][512];
static volatile uint8_t jsonCurrentRxBuffer = 0;
static volatile size_t jsonRxBufferUsed[2] = {0};
static volatile uint8_t jsonRxBufferState = JSON_BUF_WAITING_DATA;
static volatile uint8_t jsonJsonBufferState = JSON_BUF_PROCESSED;

static void handleIncomingByte(uint8_t byte)
{
    if (jsonRxBufferState == JSON_BUF_WAITING_DATA && byte != '{'
        || jsonRxBufferState == JSON_BUF_RECEIVED)
        return;
    if (byte == '}')
        jsonRxBufferState = JSON_BUF_RECEIVED;
    else
        jsonRxBufferState = JSON_BUF_RECEIVING;
    jsonRxBuffer[jsonCurrentRxBuffer][jsonRxBufferUsed[jsonCurrentRxBuffer]] = byte;
    jsonRxBufferUsed[jsonCurrentRxBuffer]++;
}

static void processMotorsRequest(json_t *root)
{
    static FullMotorPackage motorCmd;
    json_t *valuesArr = json_object_get(root, valueKeyStr);
    if (!json_is_array(valuesArr))
        return;
    for (int i = 0; i < json_array_size(valuesArr); i++)
    {
        json_t *el = json_array_get(valuesArr, i);
        int8_t dir = json_integer_value(el);
        SingleMotor *ptrs[4] = {&(motorCmd.FrontRight), &motorCmd.FrontLeft, &motorCmd.BackLeft, &motorCmd.BackRight};
        ptrs[i]->direction = dir > 0 ? Forward : Backward;
        ptrs[i]->speed = dir > 0 ? dir : -dir;
    }
    xQueueSend(motorQueue, (void*)(&motorCmd), 0);
}

static void processEventAckRequest(json_t *root)
{
    HAL_UART_Transmit_DMA(&huart2, jsonRxBuffer[!jsonCurrentRxBuffer], jsonRxBufferUsed[!jsonCurrentRxBuffer]);
}

void Uart2HalfCpltHandler(void)
{
    handleIncomingByte(dmaRxBuffer[0]);
}

void Uart2CpltHandler(void)
{
    handleIncomingByte(dmaRxBuffer[1]);
}

void UartPcCommReceiverFunc(void const * argument)
{
    // Setup
    HAL_UART_Receive_DMA(&huart2, dmaRxBuffer, sizeof(dmaRxBuffer));
    
    // Infinite loop
    while (1)
    {
        if (jsonJsonBufferState == JSON_BUF_PROCESSED)
        {
            if (jsonRxBufferState != JSON_BUF_RECEIVED)
                continue;
            else
            {
                jsonCurrentRxBuffer = !jsonCurrentRxBuffer;
                jsonJsonBufferState = JSON_BUF_PROCESSING;
                jsonRxBufferState = JSON_BUF_WAITING_DATA;
            }
        }
        json_t *obj = json_loadb((const char *)(jsonRxBuffer[!jsonCurrentRxBuffer]),
                                 jsonRxBufferUsed[!jsonCurrentRxBuffer], 0, 0);
        CANCEL_IF(!obj);
        json_t *cmd = json_object_get(obj, typeKeyStr);
        CANCEL_IF(!cmd || !json_is_string(cmd));
        const char *cmdStr = json_string_value(cmd);
        if (!strcmp(cmdStr, typeMotorsStr))
            processMotorsRequest(obj);
        else if (!strcmp(cmdStr, typeEventAckStr))
            processEventAckRequest(obj);
        json_decref(obj);
        jsonRxBufferUsed[!jsonCurrentRxBuffer] = 0;
        jsonJsonBufferState = JSON_BUF_PROCESSED;
    }
}

#endif
