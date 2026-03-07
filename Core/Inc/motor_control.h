/*
 * motor_control.h
 *
 *  Created on: Mar 6, 2026
 *      Author: Ekemini Udofia
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"
#include <stdint.h>

#define MOTOR_COUNT 4      // Four motors
#define MOTOR_DUTY_MIN 0.0f
#define MOTOR_DUTY_MAX 100.0f

// Initialize timers and PWM channels for all motors
void Motor_Init(void);

// Arm/disarm motors (gradual ramp-up or immediate stop)
void Motor_Arm(void);
void Motor_Disarm(void);

// Set individual motor speed (0-100%)
void Motor_SetSpeed(uint8_t motor_id, float duty);

// Set all motors at once
void Motor_SetAllSpeeds(const float duty[MOTOR_COUNT]);

// Make micro-adjustments to a motor (used by PID)
void Motor_AdjustSpeed(uint8_t motor_id, float delta);

// Gradually ramp motor to a target speed
void Motor_RampTo(uint8_t motor_id, float target_duty, float step);
void Motor_RampAllTo(const float target_duty[MOTOR_COUNT], float step);

// Query current motor state (optional, useful for logging/PID)
float Motor_GetSpeed(uint8_t motor_id);
uint8_t Motor_IsArmed(void);


#endif /* INC_MOTOR_CONTROL_H_ */
