#ifndef MOTOR_H
#define MOTOR_H

typedef enum
{
  FrontLeft,
  FrontRight,
	BackLeft,
	BackRight
} Position;

typedef enum
{
  Forward = 0,
  Backward = 1
} Direction;


typedef struct
{
	Direction direction;
	uint8_t speed;
} SingleMotor;

typedef struct
{
	SingleMotor FrontLeft;
	SingleMotor FrontRight;
	SingleMotor BackLeft;
	SingleMotor BackRight;
} FullMotorPackage;

uint8_t MoveMotor(Position motorPosition, SingleMotor motor);

#endif
