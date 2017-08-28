#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "motor.h"

extern UART_HandleTypeDef huart3;

const uint8_t START_SPEED = 0x00;
const uint8_t MOTOR_MAX_SPEED = 0x7F;
const uint8_t PACKAGE_SIZE = 5;
const uint8_t MOTOR_CMD_OFFSET = 4; // offset for right motors


uint16_t crc16(uint8_t *data_p, uint16_t length)
{
	uint16_t crc = 0;
	for (uint16_t i=0; i<length; i++)
	{
		crc = crc ^ (data_p[i]<<8);
		for (uint16_t j=0; j<8; j++)
		{
			if((crc & 0x8000) != 0)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}
	return crc;
}

void CalcCRC(uint8_t* message)
{
	uint16_t crc = crc16(message, 3);
	message[3] = 0xFF & (crc >> 8);
	message[4] = 0xFF & (crc);
}






void GeneratePackage(uint8_t* package, uint8_t address, uint8_t cmd, uint8_t value)
{
	package[0] = address;
	package[1] = cmd;
	package[2] = value;
	
	uint16_t crc = crc16(package, 3);
	package[3] = 0xFF & (crc >> 8);
	package[4] = 0xFF & (crc);
}

uint8_t MoveMotor(Position motorPosition, SingleMotor motor)
{
	if (motor.speed & ~MOTOR_MAX_SPEED)
		return 0x01; // speed is too big. Speed must bee from 0 to 127
	
	uint8_t package[PACKAGE_SIZE];
		
	switch(motorPosition)
	{
		case FrontLeft:
			GeneratePackage(package, 128, (uint8_t)motor.direction, motor.speed);
			break;
		case FrontRight:
			GeneratePackage(package, 128, (uint8_t)motor.direction+MOTOR_CMD_OFFSET, motor.speed);
			break;
		case BackLeft:
			GeneratePackage(package, 129, (uint8_t)motor.direction, motor.speed);
			break;
		case BackRight:
			GeneratePackage(package, 129, (uint8_t)motor.direction+MOTOR_CMD_OFFSET, motor.speed);
			break;
			
	}
	HAL_UART_Transmit(&huart3, package, PACKAGE_SIZE, 500);
	return 0x00;
}

