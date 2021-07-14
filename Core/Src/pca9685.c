/*
 * pca9685.c
 *
 *  Created on: Dec 8, 2020
 *      Author: acapone13
 */

/* Includes ------------------------------------------------------------------*/
#include "pca9685.h"
#include "i2c.h"


I2C_HandleTypeDef *pca9685_i2c;

PCA9685_STATUS PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
	if (Value) Value =1;

	if (HAL_OK != HAL_I2C_Mem_Read(pca9685_i2c, PCA9685_ADDRESS, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}
	tmp &= ~((1 << PCA9685_MODE1_RESTART_BIT) | (1 << Bit));
	tmp |= (Value&1) << Bit;

	if (HAL_OK != HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_ADDRESS, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}


PCA9685_STATUS PCA9685_SoftwareReset()
{
	uint8_t cmd = 0x6;
	if (HAL_OK != HAL_I2C_Master_Transmit(pca9685_i2c, 0x00, &cmd, 1, 10))
	{
		return PCA9685_ERROR;
	}
	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, Enable);
}

PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, Enable);
}

PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, Enable);
}

PCA9685_STATUS PCA9685_SubaddressResponde(SubaddressBit Subaddress, uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, Subaddress, Enable);
}

PCA9685_STATUS PCA9685_AllCallRespond(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_ALLCALL_BIT, Enable);
}

//	Frequency
PCA9685_STATUS PCA9685_SetPWMFrequency(uint16_t Freq)
{
	float PrescalerVal;
	uint8_t Prescale;

	if (Freq >= 1526)
	{
		Prescale = 0x03;
	}
	else if (Freq <= 24)
	{
		Prescale = 0xFF;
	}
	else
	{
		PrescalerVal = (25000000 / (4096.0 * (float)Freq)) - 1;
		Prescale = floor(PrescalerVal + 0.5);
	}

	//	To change the frequency, PCA9685 has to be in Sleep Mode
	PCA9685_SleepMode(1);
	//	Send Prescale Value to Servo Controller
	HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_ADDRESS, PCA9685_PRESCALE, 1, &Prescale, 1, 10);
	PCA9685_SleepMode(0);
	PCA9685_RestartMode(1);

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
	uint8_t RegisterAddress;
	uint8_t Message[4];

	RegisterAddress = PCA9685_LED0_ON_L + (4 * Channel);
	Message[0] = OnTime & 0xFF;
	Message[1] = OnTime >> 8;
	Message[2] = OffTime & 0xFF;
	Message[3] = OffTime >> 8;

	if (HAL_OK != HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_ADDRESS, RegisterAddress, 1, Message, 4, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert)
{
	if (Value > 4095) Value = 4095;

	if (Invert) {
		if (Value == 0) {
			//	Special value for signal fully on
			return PCA9685_SetPWM(Channel, 4096, 0);
		}
		else if (Value == 4095)
		{
			//	Special value for signal fully off
			return PCA9685_SetPWM(Channel, 0, 4096);
		}
		else
		{
			return PCA9685_SetPWM(Channel, 0, 4095-Value);
		}
	}
	else
	{
		if (Value == 4095) {
			//	Special value for signal fully on
			return PCA9685_SetPWM(Channel, 4096, 0);
		}
		else if (Value == 0)
		{
			//	Special value for signal fully off
			return PCA9685_SetPWM(Channel, 0, 4096);
		}
		else
		{
			return PCA9685_SetPWM(Channel, 0, Value);
		}
	}
	return PCA9685_ERROR;
}

#ifdef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetServoAngle(uint8_t Channel, uint16_t Angle, uint8_t Invert)
{
	float Value;
	if (Angle < MIN_ANGLE) Angle = MIN_ANGLE;
	if (Angle > MAX_ANGLE) Angle = MAX_ANGLE;

	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;

	return PCA9685_SetPin(Channel, (uint16_t)Value, Invert);
}
#endif

void PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
	pca9685_i2c = hi2c;

	PCA9685_STATUS Status;

	Status = PCA9685_SoftwareReset();
#ifdef PCA9685_SERVO_MODE
	Status = PCA9685_SetPWMFrequency(48);
#else
	PCA9685_SetPWMFrequency(1000);
#endif
	//	Set AutoIncrement
	Status = PCA9685_AutoIncrement(1);
	if (Status != PCA9685_OK)
	{
		Error_Handler();
	}
}
