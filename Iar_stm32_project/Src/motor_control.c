/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   Motor control implementation with temperature safety monitoring
  *          Controls the car motor to move right slowly and stops if 
  *          temperature becomes dangerously high
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PWM_PERIOD              1000    /* PWM period value */
#define MOTOR_RIGHT_TURN_DUTY   70      /* Right motor duty cycle percentage for turning */
#define MOTOR_LEFT_TURN_DUTY    30      /* Left motor duty cycle percentage for turning */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static MotorState_t motorState = MOTOR_STATE_STOPPED;
static MotorTempStatus_t tempStatus = MOTOR_TEMP_NORMAL;
static float currentTemperature = 25.0f;  /* Default ambient temperature */
static bool temperatureOverrideActive = false;

/* Private function prototypes -----------------------------------------------*/
static void Motor_SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed);
static void Motor_UpdateTemperatureStatus(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Updates the temperature status based on current temperature
  * @retval None
  */
static void Motor_UpdateTemperatureStatus(void)
{
    if (currentTemperature >= MOTOR_TEMP_CRITICAL_THRESHOLD) {
        tempStatus = MOTOR_TEMP_CRITICAL;
    } else if (currentTemperature >= MOTOR_TEMP_WARNING_THRESHOLD) {
        tempStatus = MOTOR_TEMP_WARNING;
    } else {
        tempStatus = MOTOR_TEMP_NORMAL;
    }
}

/**
  * @brief  Sets the motor speed for left and right motors
  * @param  leftSpeed: Speed percentage for left motor (0-100)
  * @param  rightSpeed: Speed percentage for right motor (0-100)
  * @retval None
  */
static void Motor_SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed)
{
    /* Calculate PWM duty cycles */
    uint16_t leftDuty = (leftSpeed * PWM_PERIOD) / 100;
    uint16_t rightDuty = (rightSpeed * PWM_PERIOD) / 100;
    
    /* TODO: Set actual PWM values to motor control pins
     * Example (adjust according to your hardware configuration):
     * __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, leftDuty);
     * __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, rightDuty);
     */
    
    /* For now, this is a placeholder for the actual PWM control */
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initializes the motor control system
  * @retval None
  */
void Motor_Init(void)
{
    motorState = MOTOR_STATE_STOPPED;
    tempStatus = MOTOR_TEMP_NORMAL;
    currentTemperature = 25.0f;
    temperatureOverrideActive = false;
    
    /* TODO: Initialize GPIO pins for motor control
     * Example:
     * - Configure PWM timer
     * - Configure motor direction pins
     * - Configure enable pins
     */
    
    Motor_Stop();
}

/**
  * @brief  Commands the car to move right slowly
  *         Only executes if temperature is within safe limits
  * @retval None
  */
void Motor_MoveRightSlowly(void)
{
    /* Check if temperature override is active */
    if (temperatureOverrideActive) {
        Motor_EmergencyStop();
        return;
    }
    
    /* Check temperature before allowing movement */
    if (tempStatus == MOTOR_TEMP_CRITICAL) {
        Motor_EmergencyStop();
        return;
    }
    
    /* Set motor state */
    motorState = MOTOR_STATE_RUNNING;
    
    /* To turn right slowly:
     * - Left motor runs at higher speed
     * - Right motor runs at lower speed
     * - Both at slow overall speed
     */
    uint8_t leftMotorSpeed = (MOTOR_SLOW_SPEED * MOTOR_RIGHT_TURN_DUTY) / 100;
    uint8_t rightMotorSpeed = (MOTOR_SLOW_SPEED * MOTOR_LEFT_TURN_DUTY) / 100;
    
    /* Set the motor speeds */
    Motor_SetSpeed(leftMotorSpeed, rightMotorSpeed);
    
    /* TODO: Set direction pins for forward movement
     * Example:
     * HAL_GPIO_WritePin(MOTOR_LEFT_DIR_GPIO_Port, MOTOR_LEFT_DIR_Pin, GPIO_PIN_SET);
     * HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_GPIO_Port, MOTOR_RIGHT_DIR_Pin, GPIO_PIN_SET);
     */
}

/**
  * @brief  Stops the motor normally
  * @retval None
  */
void Motor_Stop(void)
{
    motorState = MOTOR_STATE_STOPPED;
    Motor_SetSpeed(0, 0);
    
    /* TODO: Disable motor enable pins if needed
     * Example:
     * HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
     */
}

/**
  * @brief  Emergency stop - immediately stops the motor due to critical temperature
  * @retval None
  */
void Motor_EmergencyStop(void)
{
    motorState = MOTOR_STATE_EMERGENCY_STOP;
    temperatureOverrideActive = true;
    Motor_SetSpeed(0, 0);
    
    /* TODO: Additional emergency stop actions
     * Example:
     * - Activate warning LED
     * - Sound buzzer
     * - Send alert message
     */
}

/**
  * @brief  Updates the current motor temperature reading
  * @param  temperature: Temperature in degrees Celsius
  * @retval None
  */
void Motor_UpdateTemperature(float temperature)
{
    currentTemperature = temperature;
    Motor_UpdateTemperatureStatus();
}

/**
  * @brief  Checks temperature and controls motor accordingly
  *         Should be called periodically in main loop
  * @retval None
  */
void Motor_CheckTemperatureAndControl(void)
{
    Motor_UpdateTemperatureStatus();
    
    /* If temperature reaches critical level, emergency stop */
    if (tempStatus == MOTOR_TEMP_CRITICAL) {
        if (motorState == MOTOR_STATE_RUNNING) {
            Motor_EmergencyStop();
        }
    }
    /* If temperature drops back to normal, clear override */
    else if (tempStatus == MOTOR_TEMP_NORMAL) {
        temperatureOverrideActive = false;
        
        /* If in emergency stop and temperature is normal, go to normal stop */
        if (motorState == MOTOR_STATE_EMERGENCY_STOP) {
            motorState = MOTOR_STATE_STOPPED;
        }
    }
}

/**
  * @brief  Gets the current motor state
  * @retval Current motor state
  */
MotorState_t Motor_GetState(void)
{
    return motorState;
}

/**
  * @brief  Gets the current temperature status
  * @retval Current temperature status
  */
MotorTempStatus_t Motor_GetTemperatureStatus(void)
{
    return tempStatus;
}

/**
  * @brief  Gets the current motor temperature
  * @retval Current temperature in degrees Celsius
  */
float Motor_GetCurrentTemperature(void)
{
    return currentTemperature;
}
