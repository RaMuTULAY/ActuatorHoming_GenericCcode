/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : actuator_xy.h
  * @brief          : definitions for actuator_xy.c
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACTUATOR_XY_H
#define __ACTUATOR_XY_H
#endif /* */

/* Includes ------------------------------------------------------------------*/
#include "main.h"					//for getting pin definitions
#include "stdbool.h"
/*
#define LIMSW_X_Pin GPIO_PIN_0
#define LIMSW_X_GPIO_Port GPIOC

#define LIMSW_Y_Pin GPIO_PIN_1
#define LIMSW_Y_GPIO_Port GPIOC

#define DIR_XtoY_Pin GPIO_PIN_1
#define DIR_XtoY_GPIO_Port GPIOA

#define DIR_YtoX_Pin GPIO_PIN_4
#define DIR_YtoX_GPIO_Port GPIOA
*/

void ActuatorHomingIdleStart();
uint8_t ActuatorHomingProcess(uint32_t *);



//Below times are multiples of "ActuatorHoming" function call intervals, eg;
//if superloop calls "ActuatorHoming" function every 10ms and for example #define value is 5
//it will take 5*10ms=50 ms as a real execution time.
//These times will trigger the MIN and MAX time errors
#define		ACT_XtoY_TRAVELTIME_MIN		0x0001UL	//how short could ACTUATOR movement take to travel from X to Y,
#define		ACT_XtoY_TRAVELTIME_MAX		0x08FFUL	//how long  could ACTUATOR movement take to travel from X to Y,

#define		ACT_YtoX_TRAVELTIME_MIN		0x0001UL	//how short could ACTUATOR movement take to travel from Y to X,
#define		ACT_YtoX_TRAVELTIME_MAX		0x0AFFUL	//how long  could ACTUATOR movement take to travel from Y to X,


#define		LIMSW_X_CLEARENCE_TIME_MIN	0x0001UL	/*how short could ACTUATOR movement take to pass limit switch X,
													if limit switch state changes shorter than this time it will cause error*/
#define		LIMSW_X_CLEARENCE_TIME_MAX	0x03FFUL	/*how  long could ACTUATOR movement take to pass limit switch X,
													if limit switch state does not change in this time it will cause error*/

#define		LIMSW_Y_CLEARENCE_TIME_MIN	0x0001UL	/*how short could ACTUATOR movement take to pass limit switch Y,
													if limit switch state changes shorter than this time it will cause error*/
#define		LIMSW_Y_CLEARENCE_TIME_MAX	0x03FFUL	/*how  long could ACTUATOR movement take to pass limit switch Y,
													if limit switch state does not change in this time it will cause error*/

#define		LIMSW_DEBOUNCE_TIME_MAX		0x000FUL	//must be bigger than "LIMSW_SETTLE_CNT"
#define 	LIMSW_SETTLE_CNT 			0x0003UL	/*how many consecutive times of reading limit switch changed state
													will be considered as a settled limit switch state*/

//Hardware PINs
#define MoveActuatorXtoY() {\
            HAL_GPIO_WritePin(DIR_XtoY_GPIO_Port, DIR_XtoY_Pin, GPIO_PIN_SET);\
            HAL_GPIO_WritePin(DIR_YtoX_GPIO_Port, DIR_YtoX_Pin, GPIO_PIN_RESET);\
           }

#define MoveActuatorYtoX() {\
            HAL_GPIO_WritePin(DIR_XtoY_GPIO_Port, DIR_XtoY_Pin, GPIO_PIN_RESET);\
            HAL_GPIO_WritePin(DIR_YtoX_GPIO_Port, DIR_YtoX_Pin, GPIO_PIN_SET);\
           }
#define StopActuator() {\
            HAL_GPIO_WritePin(DIR_XtoY_GPIO_Port, DIR_XtoY_Pin, GPIO_PIN_RESET);\
            HAL_GPIO_WritePin(DIR_YtoX_GPIO_Port, DIR_YtoX_Pin, GPIO_PIN_RESET);\
           }

#define ReadLimitSwitchX_GPIO()	HAL_GPIO_ReadPin(LIMSW_X_GPIO_Port, LIMSW_X_Pin)
#define ReadLimitSwitchY_GPIO()	HAL_GPIO_ReadPin(LIMSW_Y_GPIO_Port, LIMSW_Y_Pin)
