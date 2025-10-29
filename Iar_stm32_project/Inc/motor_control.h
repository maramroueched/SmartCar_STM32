/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   Header file for motor control with temperature safety monitoring
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_EMERGENCY_STOP
} MotorState_t;

typedef enum {
    MOTOR_TEMP_NORMAL = 0,
    MOTOR_TEMP_WARNING,
    MOTOR_TEMP_CRITICAL
} MotorTempStatus_t;

/* Exported constants --------------------------------------------------------*/
#define MOTOR_TEMP_WARNING_THRESHOLD    75.0f   /* Temperature warning threshold (°C) */
#define MOTOR_TEMP_CRITICAL_THRESHOLD   85.0f   /* Temperature critical threshold (°C) */
#define MOTOR_SLOW_SPEED                30      /* Slow speed percentage (0-100) */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Init(void);
void Motor_MoveRightSlowly(void);
void Motor_Stop(void);
void Motor_EmergencyStop(void);
void Motor_UpdateTemperature(float temperature);
void Motor_CheckTemperatureAndControl(void);
MotorState_t Motor_GetState(void);
MotorTempStatus_t Motor_GetTemperatureStatus(void);
float Motor_GetCurrentTemperature(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
