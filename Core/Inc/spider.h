/*
 * spider.h
 *
 *  Created on: Dec 9, 2020
 *      Author: acapone13
 */

#ifndef __SPIDER_H_
#define __SPIDER_H_
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define SERVO_COUNT	12
#define LEG_COUNT	4

/* Size of the robot */
#define lengthA				27.5
#define lengthB				55
#define lengthC				77.5
#define absoluteStep		20.0
#define tolerance			0.5

 /* Positions in cartesian space */
#define xRef		0
#define yRef		0
#define zRef		0

#define xMid		58.33
#define yMid		58.33

#define xMax		82.5
#define yMax		82.5
#define zMax		10

#define xmidMin		41.37
#define ymidMin		41.37
#define xmidMax		71.37
#define ymidMax		71.37
#define zMin		-77.5

/* variables for movement */
volatile float actualPos[4][3];    //real-time coordinates of the end of each leg
volatile float expectedPos[4][3]; //expected coordinates of the end of each leg

typedef enum {legSpeed, bodySpeed}Speed;
Speed speed;
//define PI for calculation
#define PI  3.1415926
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
//void homing(uint16_t *angles, uint16_t *refAngles);
void homing(float pos[4][3]);

void forward_walk();

void backward_walk();

void right_walk();

void left_walk();

void turn_left();

void turn_right();

void stop();

void sit();

void stand();

void servoControl(uint16_t *angles, uint8_t servo, uint16_t angle);

void legControl(uint16_t *angles, uint8_t leg, char movement, uint16_t step);

void inverseCinematic(float *alpha, float *beta, float *gamma, float x, float y, float z);

void pos2Angle(float position[4][3], int homing);

void servoMovement();

void setPosition(int leg, float x, float y, float z);

void setServo(uint16_t *angles);

void setStep(int leg);

void waitReach();

void waitAllReach();

void checkLimits(uint16_t *angles);

void printPosition();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif


#endif /* __SPIDER_H_ */
