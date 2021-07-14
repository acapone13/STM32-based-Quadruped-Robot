/*
 * pca9685.h
 *
 *  Created on: Dec 8, 2020
 *      Author: acapone13
 */

#ifndef __PCA9685_H_
#define __PCA9685_H_
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
//	Enable Servo Control Mode
#define PCA9685_SERVO_MODE

#ifdef PCA9685_SERVO_MODE
//	Servo min and max values for TG9e Servos
#define SERVO_MIN	110
#define SERVO_MAX	500
#define	MIN_ANGLE 	0
#define MAX_ANGLE 	180
#endif

//	Adjustable address 0x80 - 0xFE
#define PCA9685_ADDRESS			0x80

//	Registers
#define PCA9685_SUBADDR1		0x2
#define PCA9685_SUBADDR2		0x3
#define PCA9685_SUBADDR3		0x4

#define PCA9685_MODE1			0x0
#define PCA9685_PRESCALE		0xFE

#define PCA9685_LED0_ON_L		0x6
#define PCA9685_LED0_ON_H		0x7
#define PCA9685_LED0_OFF_L		0x8
#define	PCA9685_LED0_OFF_H		0x9

#define PCA9685_ALLLED_ON_L		0xFA
#define PCA9685_ALLLED_ON_H		0xFB
#define PCA9685_ALLLED_OFF_L	0xFC
#define PCA9685_ALLLED_OFF_H	0xFD

#define PCA9685_MODE1_ALLCALL_BIT	0
#define PCA9685_MODE1_SLEEP_BIT		4
#define PCA9685_MODE1_AI_BIT		5
#define PCA9685_MODE1_EXTCLK_BIT	6
#define PCA9685_MODE1_RESTART_BIT	7

typedef enum
{
	PCA9685_MODE1_SUB1_BIT	= 3,
	PCA9685_MODE1_SUB2_BIT	= 2,
	PCA9685_MODE1_SUB3_BIT	= 1
}SubaddressBit;
//	Servo Controller return Status
typedef enum
{
	PCA9685_OK		= 0,
	PCA9685_ERROR	= 1
}PCA9685_STATUS;
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
PCA9685_STATUS PCA9685_SoftwareReset();
PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable);
PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable);
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable);

#ifndef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetPWMFrequency(uint16_t Frequency);
#endif

PCA9685_STATUS PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert);
#ifdef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetServoAngle(uint8_t Channel, uint16_t Angle, uint8_t Invert);
#endif

void PCA9685_Init(I2C_HandleTypeDef *hi2c);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif


#endif /* __PCA9685_H_ */
