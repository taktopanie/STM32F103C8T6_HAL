/*
 * ULN2003_motor.c
 *
 *  Created on: Feb 9, 2024
 *      Author: maciej
 */

#include "MY_motor.h"


///////////////////// STEPPER MOTOR WITH ULN2003 DRIVER /////////////////////

uint8_t _motor_position_ULN2003 = 0;

void MOTOR_set_position_ULN2003 (uint8_t pos)
{

	switch (pos)
	{
		case 0:
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, DISABLE);
			break;
		case 1:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, DISABLE);
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, ENABLE);
			break;
		}
		case 3:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, ENABLE);
			break;
		}
		case 99:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, DISABLE);
			break;
		}
		default:
		{
			break;
		}

	}
}

void MOTOR_left_ULN2003 (void)
{
	MOTOR_set_position_ULN2003(_motor_position_ULN2003);
	if(_motor_position_ULN2003 == 0)
	{
		_motor_position_ULN2003 = 3;
	}else
	{
		_motor_position_ULN2003--;
	}
}

void MOTOR_right_ULN2003 (void)
{
	MOTOR_set_position_ULN2003(_motor_position_ULN2003++);
	if(_motor_position_ULN2003 == 4)
	{
		_motor_position_ULN2003 = 0;
	}
}


/////////////////////////////////////////////////////////////////////////////

/*
 * CD MOTOR STEERING
 */

uint8_t _motor_position_CD = 0;

void MOTOR_set_position_CD (uint8_t pos)
{

	switch (pos)
	{
		case 0:
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, DISABLE);
			break;
		case 1:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, DISABLE);
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, ENABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, DISABLE);
			break;
		}
		case 3:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, ENABLE);
			break;
		}
		case 99:
		{
			HAL_GPIO_WritePin(MOTOR_PORT_0_ULN2003, MOTOR_PIN_0_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_1_ULN2003, MOTOR_PIN_1_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_2_ULN2003, MOTOR_PIN_2_ULN2003, DISABLE);
			HAL_GPIO_WritePin(MOTOR_PORT_3_ULN2003, MOTOR_PIN_3_ULN2003, DISABLE);
			break;
		}
		default:
		{
			break;
		}

	}
}

void MOTOR_left_CD (void)
{
	MOTOR_set_position_CD(_motor_position_CD);
	if(_motor_position_CD == 0)
	{
		_motor_position_CD = 3;
	}else
	{
		_motor_position_CD--;
	}
}

void MOTOR_right_CD (void)
{
	MOTOR_set_position_CD(_motor_position_CD++);
	if(_motor_position_CD == 4)
	{
		_motor_position_CD = 0;
	}
}
