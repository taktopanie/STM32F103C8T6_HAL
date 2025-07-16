/*
 * ULN2003_motor.h
 *
 *  Created on: Feb 9, 2024
 *      Author: maciej
 */

#ifndef MY_MOTOR_H_
#define MY_MOTOR_H_

#include <stdint.h>
#include <stm32f1xx.h>

///////////////////// STEPPER MOTOR WITH ULN2003 DRIVER /////////////////////
/*
 * MOTOR PIN DEFINITION ULN2003
 */
#define MOTOR_PIN_0_ULN2003			GPIO_PIN_4
#define MOTOR_PORT_0_ULN2003		GPIOA
#define MOTOR_PIN_1_ULN2003			GPIO_PIN_5
#define MOTOR_PORT_1_ULN2003		GPIOA
#define MOTOR_PIN_2_ULN2003			GPIO_PIN_6
#define MOTOR_PORT_2_ULN2003		GPIOA
#define MOTOR_PIN_3_ULN2003			GPIO_PIN_7
#define MOTOR_PORT_3_ULN2003		GPIOA

/*
 * MOTOR PIN DEFINITION CD
 */
#define MOTOR_PIN_0_CD				GPIO_PIN_4
#define MOTOR_PORT_0_CD				GPIOA
#define MOTOR_PIN_1_CD				GPIO_PIN_5
#define MOTOR_PORT_1_CD				GPIOA
#define MOTOR_PIN_2_CD				GPIO_PIN_6
#define MOTOR_PORT_2_CD				GPIOA
#define MOTOR_PIN_3_CD				GPIO_PIN_7
#define MOTOR_PORT_3_CD				GPIOA

/*
 * STEPPER MOTOR FUNCTION DEFINITIONS
 */
void MOTOR_set_position_ULN2003(uint8_t pos);
void MOTOR_left_ULN2003 (void);
void MOTOR_right_ULN2003 (void);

/*
 * CD MOTOR FUNCTION DEFINITIONS
 */

void MOTOR_set_position_CD (uint8_t pos);
void MOTOR_left_CD (void);
void MOTOR_right_CD (void);

/////////////////////////////////////////////////////////////////////////////

#endif /* MY_MOTOR_H_ */
