/*
 * sPIder.c
 *
 *  Created on: Dec 9, 2020
 *      Author: acapone13
 */

/* Includes ------------------------------------------------------------------*/
#include "spider.h"
#include "pca9685.h"
#include "usart.h"


void homing(float refPos[4][3])
{
	pos2Angle(refPos, 1);
}

void inverseCinematic(float *alpha, float *beta, float *gamma, float x, float y, float z)
{
	//	Theta 1: First Joint variable -> alpha
	float theta1 = atan2(y, x);
	*alpha = theta1 / PI * 180;

	//	Internal variables for calculations
	float d = sqrt((x*x) + (y*y));
	float h = d - lengthA;
	float r = sqrt(h*h + z*z);

	//	Theta 2: Second Joint variable -> beta
	float theta2a = atan2(z, h);
	float theta2b = acos(((lengthB*lengthB) + (r*r) - (lengthC*lengthC)) / (2 * lengthB * r));
	float theta2 = theta2a + theta2b + (PI/2);
	*beta = theta2 / PI * 180;

	//	Theta 3: Second Joint variable -> gamma
	float temp = sin(theta2b) * (r / lengthC);
	float theta3 = asin(abs(temp) > 1 ? round(temp) : temp);
//	float epsilon = (PI / 2) - theta2;
	*gamma = theta3 / PI * 180;
}

void setPosition(int leg, float x, float y, float z)
{
	expectedPos[leg][0] = x;
	expectedPos[leg][1] = y;
	expectedPos[leg][2] = z;
}

void setStep(int leg)
{
	// Check X axis
	float xStep = abs(expectedPos[leg][0] - actualPos[leg][0]) > 20 ? absoluteStep : (float)(fabs(expectedPos[leg][0] - actualPos[leg][0]));
	if (expectedPos[leg][0] > actualPos[leg][0])
	{
		actualPos[leg][0] += xStep;
	}
	else if (expectedPos[leg][0] < actualPos[leg][0])
	{
		actualPos[leg][0] -= xStep;
	}

	//	Check Y axis
	float yStep = abs(expectedPos[leg][1] - actualPos[leg][1]) > 20 ? absoluteStep : (float)fabs(expectedPos[leg][1] - actualPos[leg][1]);
	if (expectedPos[leg][1] > actualPos[leg][1])
	{
		actualPos[leg][1] += yStep;
	}
	else if (expectedPos[leg][1] < actualPos[leg][1])
	{
		actualPos[leg][1] -= yStep;
	}

	//	Check Y axis
	float zStep = abs(expectedPos[leg][2] - actualPos[leg][2]) > 20 ? absoluteStep : (float)fabs(expectedPos[leg][2] - actualPos[leg][2]);
	if (expectedPos[leg][2] > actualPos[leg][2])
	{
		actualPos[leg][2] += zStep;
	}
	else if (expectedPos[leg][2] < actualPos[leg][2])
	{
		actualPos[leg][2] -= zStep;
	}
}

void pos2Angle(float position[4][3], int homing)
{
	uint16_t refAngles[SERVO_COUNT];
	static float alpha, beta, gamma;
	int leg;
	for (leg = 0; leg < LEG_COUNT; leg++)
	{
		uint8_t bottomServo = ((leg + 1) * 3) - 3;
		uint8_t middleServo = ((leg + 1) * 3) - 2;
		uint8_t topServo = ((leg + 1) * 3) - 1;

		// Calculate servo angles from cartesian variables with inverse cinematic
		inverseCinematic(&alpha, &beta, &gamma, position[leg][0], position[leg][1], position[leg][2]);
		refAngles[bottomServo] = (uint16_t)alpha;
		refAngles[middleServo] = (uint16_t)beta;
		refAngles[topServo] = (uint16_t)gamma;

		// Update actual Position
		if (homing)
		{
			actualPos[leg][0] = position[leg][0];
			actualPos[leg][1] = position[leg][1];
			actualPos[leg][2] = position[leg][2];
		}
	}
	checkLimits(refAngles);
	setServo(refAngles);
}

void servoMovement()
{
	float tempPos[4][3];
	int i, j;
	for (i = 0; i < LEG_COUNT; i++)
	{
		for (j = 0; j < 3; j++)
		{
			tempPos[i][j] = actualPos[i][j];
		}
	}
	pos2Angle(tempPos, 0);
}

void waitReach(int leg){
	while(expectedPos[leg][0] != actualPos[leg][0] || expectedPos[leg][1] != actualPos[leg][1] \
			|| expectedPos[leg][2] != actualPos[leg][2])
	{
		setStep(leg);
		servoMovement();
		if (speed == legSpeed)
		{
			HAL_Delay(40);
		}
	}
}

void waitAllReach()
{
	int i;
	for (i = 0; i < LEG_COUNT; i++)
	{
		waitReach(i);
	}
}

void setServo(uint16_t *angles)
{
	int leg, servo;
	for (leg = 0; leg < LEG_COUNT; leg++)
	{
		uint8_t bottomServo = ((leg + 1) * 3) - 3;
		uint8_t middleServo = ((leg + 1) * 3) - 2;
		uint8_t topServo = ((leg + 1) * 3) - 1;
		//	Set angles for each servo
		if (leg == 0)
		{
			angles[bottomServo] = 180 - angles[bottomServo];
			angles[topServo] = 180 - angles[topServo];
		}
		else if (leg == 1)
		{
			angles[middleServo] = 180 - angles[middleServo];
		}
		else if (leg == 2)
		{
			angles[bottomServo] = 180 - angles[bottomServo];
			angles[topServo] = 180 - angles[topServo];
		}
		else if (leg == 3)
		{
			angles[middleServo] = 180 - angles[middleServo];
		}
	}
	for (servo = 0; servo < SERVO_COUNT; servo++)
	{
		PCA9685_SetServoAngle(servo, angles[servo], 0);
	}
}

void checkLimits(uint16_t *angles)
{
	int i;
	for (i = 0; i < SERVO_COUNT; i++)
	{
		angles[i] = angles[i] > 180 ? 180 : angles[i];
		angles[i] = angles[i] < 0 ? 0 : angles[i];
	}
}

void forward_walk()
{
	speed = legSpeed;
	if (actualPos[2][1] == yMax)
	{
		//	Legs 1 & 3 move
		//	Leg 3 moves to the front
		setPosition(2, xRef, yMax, zRef);
		waitReach(2);
		setPosition(2, xmidMax, ymidMin, zRef);
		waitReach(2);
		setPosition(2, xmidMax, ymidMin, zMin);
		waitReach(2);

		//	Body movement
		setPosition(0, xmidMax, ymidMin, zMin);
		setPosition(1, xRef, yMax, zMin);
		setPosition(2, xMid, yMid, zMin);
		setPosition(3, xMid, yMid, zMin);
		waitAllReach();

//		//	Leg 1 moves to the front
		setPosition(0, xMid, yMid, 0);
		waitReach(0);
		setPosition(0, xRef, yMax, zRef);
		waitReach(0);
		setPosition(0, xRef, yMax, zMin);
		waitReach(0);
	}
	else
	{
		//	Legs 2 & 4 move
		//	Leg 2 moves to the front
		setPosition(1, xRef, yMax, zRef);
		waitReach(1);
		setPosition(1, xmidMax, ymidMin, zRef);
		waitReach(1);
		setPosition(1, xmidMax, ymidMin, zMin);
		waitReach(1);

		//	Body movement
		setPosition(0, xMid, yMid, zMin);
		setPosition(1, xMid, yMid, zMin);
		setPosition(2, xRef, yMax, zMin);
		setPosition(3, xmidMax, ymidMin, zMin);
		waitAllReach();

		//	Leg 4 moves to the front
		setPosition(3, xMid, yMid, zRef);
		waitReach(3);
		setPosition(3, xRef, yMax, zRef);
		waitReach(3);
		setPosition(3, xRef, yMax, zMin);
		waitReach(3);
	}
}

void backward_walk()
{
	speed = legSpeed;
	if (actualPos[2][1] == yMax)
	{
		//	Legs 2 & 4 move
		//	Leg 2 moves to the front
		setPosition(3, xRef, yMax, zRef);
		waitReach(3);
		setPosition(3, xmidMax, ymidMin, zRef);
		waitReach(3);
		setPosition(3, xmidMax, ymidMin, zMin);
		waitReach(3);

		//	Body movement
		setPosition(0, xRef, yMax, zMin);
		setPosition(1, xmidMax, ymidMin, zMin);
		setPosition(2,  xMid, yMid, zMin);
		setPosition(3, xMid, yMid, zMin);
		waitAllReach();

		//	Leg 4 moves to the front
		setPosition(1, xMid, yMid, zRef);
		waitReach(1);
		setPosition(1, xRef, yMax, xRef);
		waitReach(1);
		setPosition(1, xRef, yMax, zMin);
		waitReach(1);
	}
	else
	{
		//	Legs 3 & 1 move
		//	Leg 3 moves to the front
		setPosition(0, xRef, yMax, zRef);
		waitReach(0);
		setPosition(0, xmidMax, ymidMin, zRef);
		waitReach(0);
		setPosition(0, xmidMax, ymidMin, zMin);
		waitReach(0);

		//	Body movement
		setPosition(0, xMid, yMid, zMin);
		setPosition(1, xMid, yMid, zMin);
		setPosition(2, xmidMax, ymidMin, zMin);
		setPosition(3, xRef, yMax, zMin);
		waitAllReach();

		//	Leg 1 moves to the front
		setPosition(2, xMid, yMid, zRef);
		waitReach(2);
		setPosition(2, xRef, yMax, zRef);
		waitReach(2);
		setPosition(2, xRef, yMax, zMin);
		waitReach(2);
	}
}

void turn_left()
{
	speed = legSpeed;
	if (actualPos[2][1] == yMax)
	{
		//	Legs 2 & 1 move
		//	Leg 2 moves to the front
		setPosition(2, xRef, yMax, zRef);
		waitReach(2);
		setPosition(2, xmidMax, ymidMin, zRef);
		waitReach(2);

		// Body movement and final Leg 2 movement
		setPosition(0, xMid, yMid, zMin);
		setPosition(1, xMax, yRef, zMin);
		setPosition(2, xMax, yRef, zMin);
		setPosition(3, xmidMin, xmidMax, zMin);
		waitAllReach();

		//	Leg 1 moves to the reference
		setPosition(1, xMax, yRef, zRef);
		waitReach(1);
		setPosition(1, xMid, yMid, zRef);
		waitReach(1);

		// Body movement and final Leg 1 movement
		setPosition(0, xRef, yMax, zMin);
		setPosition(1, xRef, yMax, zMin);
		setPosition(2, xMid, yMid, zMin);
		setPosition(3, xMid, yMid, zMin);
		waitAllReach();
	}
	else
	{
		//	Legs 3 & 0 move
		//	Leg 0 moves to the front
		setPosition(0, xRef, yMax, zRef);
		waitReach(0);
		setPosition(0, xmidMax, ymidMin, zRef);
		waitReach(0);

		// Body movement and final Leg 0 movement
		setPosition(0, xMax, yRef, zMin);
		setPosition(1, xmidMin, ymidMax, zMin);
		setPosition(2, xMid , yMid, zMin);
		setPosition(3, xMax, yRef, zMin);
		waitAllReach();

		//	Leg 3 moves to reference
		setPosition(3, xMax, yRef, zRef);
		waitReach(3);
		setPosition(3, xMid, yMid, zRef);
		waitReach(3);

		// Body movement and final Leg 3 movement
		setPosition(0, xMid, yMid, zMin);
		setPosition(1, xMid, yMid, zMin);
		setPosition(2, xRef, yMax, zMin);
		setPosition(3, xRef, yMax, zMin);
		waitAllReach();
	}
}

void turn_right()
{
	speed = legSpeed;
	if (actualPos[2][1] == yMax)
	{
		//	Legs 3 & 0 move
		//	Leg 3 moves to the front
		setPosition(3, xRef, yMax, zRef);
		waitReach(3);
		setPosition(3, xmidMax, ymidMin, zRef);
		waitReach(3);

		// Body movement and final Leg 3 movement
		setPosition(0, xMax, yRef, zMin);
		setPosition(1, xMid , yMid,zMin);
		setPosition(2, xmidMin, ymidMax, zMin);
		setPosition(3, xMax, yRef, zMin);
		waitAllReach();

		//	Leg 0 moves to reference
		setPosition(0, xMax, yRef, zRef);
		waitReach(0);
		setPosition(0, xMid, yMid, zRef);
		waitReach(0);
		setPosition(0, xRef, yMax, zMin);
		waitReach(0);

		// Body movement and final Leg 0 movement
		setPosition(1, xRef, yMax, zMin);
		setPosition(2, xMid, yMid, zMin);
		setPosition(3, xMid, yMid, zMin);
		waitAllReach();
	}
	else
	{
		//	Legs 2 & 1 move
		//	Leg 1 moves to the front
		setPosition(1, xRef, yMax, zRef);
		waitReach(1);
		setPosition(1, xmidMax, ymidMin, zRef);
		waitReach(1);
		setPosition(1, xMax, yRef, zMin);
		waitReach(1);

		// Body movement and final Leg 1 movement
		setPosition(0, xmidMin, ymidMax, zMin);
//		setPosition(1, xMax, 0, zMin);
		setPosition(2, xMax, yRef, zMin);
		setPosition(3, xMid, yMid, zMin);
		waitAllReach();


		//	Leg 2 moves to the reference
		setPosition(2, xMax, yRef, zRef);
		waitReach(2);
		setPosition(2, xMid, yMid, zRef);
		waitReach(2);

		// Body movement and final Leg 0 movement
		setPosition(0, xMid, yMid, zMin);
		setPosition(1, xMid, yMid, zMin);
		setPosition(2, xRef, yMax, zMin);
		setPosition(3, xRef, yMax, zMin);
		waitAllReach();
	}
}

void sit()
{
	speed = bodySpeed;
	int i;
	for (i = 0; i < LEG_COUNT; i++)
	{
		setPosition(i, xMid, yMid, xMax);
	}
	waitAllReach();
}

void stand()
{
	speed = bodySpeed;
	int i;
	for (i = 0; i < LEG_COUNT; i++)
	{
		setPosition(i, xMid, yMid, zMin);
	}
	waitAllReach();
}

void right_walk(uint16_t *angles)
{
	speed = legSpeed;
//	angles[2] = angles[2] < 100 ? angles[2] + 30 : angles[2];
//	PCA9685_SetServoAngle(2, angles[2], 0);
//	HAL_Delay(200);
//
//	legControl(angles, 3, 'U', 30);
//	angles[8] = angles[8] > 0 ? angles[8] - 30 : angles[8];
//	PCA9685_SetServoAngle(8, angles[8], 0);
//	HAL_Delay(200);
//
//	legControl(angles, 2, 'U', 30);
//	angles[5] = angles[5] < 140 ? angles[5] + 30 : angles[5];
//	PCA9685_SetServoAngle(5, angles[5], 0);
//	HAL_Delay(200);
//
//	legControl(angles, 1, 'D', 30);
//	angles[] = angles
}

void left_walk(uint16_t *angles)
{
	speed = legSpeed;
}

void stop()
{
	int i, j;
	for (i = 0; i < LEG_COUNT; i++)
	{
		for (j = 0; j < 3;  j++)
		{
			expectedPos[i][j] = actualPos[i][j];
		}
	}
	servoMovement();
}

void servoControl(uint16_t *angles, uint8_t servo, uint16_t angle)
{
	PCA9685_SetServoAngle(servo, angle, 0);
	angles[servo] = angle;
}

void legControl(uint16_t *angles, uint8_t leg, char movement, uint16_t step)
{
	//	Select the corresponding servos for each leg
	uint8_t bottomServo = (leg * 3) - 3;
	uint8_t middleServo = (leg * 3) - 2;
	uint8_t topServo = (leg * 3) - 1;
	switch(leg)
	{
		case 1:
		case 3:
			switch(movement)
			{
				case 'U':
					if (angles[middleServo] < MAX_ANGLE && angles[topServo] < MAX_ANGLE)
					{
						angles[middleServo] += step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
						angles[topServo] -= 5;
						PCA9685_SetServoAngle(topServo, angles[topServo], 0);
					}
					else if (angles[middleServo] < MAX_ANGLE)
					{
						angles[middleServo] += step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
					}
					else
					{
						angles[topServo] -= 5;
						PCA9685_SetServoAngle(topServo, angles[topServo], 0);
					}

					break;
				case 'D':
					if (angles[middleServo] > MIN_ANGLE && angles[topServo] > MIN_ANGLE)
					{
						angles[middleServo] -= step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
						angles[topServo] -= 5;
						PCA9685_SetServoAngle(topServo, angles[topServo] - 5, 0);

					}
					else if (angles[middleServo] < MAX_ANGLE)
					{
						angles[middleServo] -= step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
					}
					else
					{
						angles[topServo] += 5;
						PCA9685_SetServoAngle(topServo, angles[topServo], 0);
					}

					break;
				case 'R':
					if (angles[bottomServo] > MIN_ANGLE)
					{
						angles[bottomServo] -= step;
						PCA9685_SetServoAngle(bottomServo, angles[bottomServo], 0);
					}

					break;
				case 'L':
					if (angles[bottomServo] < MAX_ANGLE)
					{
						angles[bottomServo] += step;
						PCA9685_SetServoAngle(bottomServo, angles[bottomServo], 0);
					}

					break;
				default:
					break;
			}
			break;
		case 2:
		case 4:
			switch(movement)
			{
				case 'U':
					if (angles[middleServo] > MIN_ANGLE && angles[topServo] > MIN_ANGLE)
					{
						angles[middleServo] -= step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
						angles[topServo] += 5;
						PCA9685_SetServoAngle(topServo, angles[topServo], 0);
					}
					else if (angles[middleServo] > MAX_ANGLE)
					{
						angles[middleServo] -= step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
					}
					else
					{
						angles[topServo] += 5;
						PCA9685_SetServoAngle(topServo, angles[topServo], 0);
					}

					break;
				case 'D':
					if (angles[middleServo] < MAX_ANGLE && angles[topServo] < MAX_ANGLE)
					{
						angles[middleServo] += step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
						angles[topServo] -= 5;
						PCA9685_SetServoAngle(topServo, angles[topServo], 0);

					}
					else if (angles[middleServo] < MAX_ANGLE)
					{
						angles[middleServo] += step;
						PCA9685_SetServoAngle(middleServo, angles[middleServo], 0);
					}
					else
					{
						angles[topServo] -= 5;
						PCA9685_SetServoAngle(topServo, angles[topServo], 0);
					}

					break;
				case 'R':
					if (angles[bottomServo] > MIN_ANGLE)
					{
						angles[bottomServo] -= step;
						PCA9685_SetServoAngle(bottomServo, angles[bottomServo], 0);
					}

					break;
				case 'L':
					if (angles[bottomServo] < MAX_ANGLE)
					{
						angles[bottomServo] += step;
						PCA9685_SetServoAngle(bottomServo, angles[bottomServo], 0);
					}

					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

//	Returns the angle in degrees of a given servo or joint
uint16_t getAngle(uint8_t servo, uint16_t *angles)
{
	return angles[servo];
}

void printPosition()
{
	int i, j;
	char USART_TxStream[300];
	char axes[4] = {'X', 'Y', 'Z', '\0'};
	char temp[40];
	memset(&USART_TxStream[0], 0, sizeof(USART_TxStream));
	for (i = 0; i < LEG_COUNT; i++)
	{
		memset(&temp[0], 0, sizeof(temp));
		sprintf(temp, "Leg %d:\n\r", i);
		strcat(USART_TxStream, temp);
		for (j = 0; j < 3; j++)
		{
			memset(&temp[0], 0, sizeof(temp));
			sprintf(temp, "Position %c: %.2f\n\r", axes[j], actualPos[i][j]);
			strcat(USART_TxStream, temp);
		}
	}
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) USART_TxStream, (uint16_t)(sizeof(USART_TxStream)-1));
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
}
