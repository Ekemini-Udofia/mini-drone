#include "motor_control.h"

// Storage for motor speeds and armed state
static float motor_speeds[MOTOR_COUNT] = {0};
static uint8_t motors_armed = 0; // Can be 0 or 1
static float MOTOR_ARMED_SPEED = 5.0F; // Set this after callibration
static float MOTOR_DISARMED_SPEED = 0.0F; // Set this after callibration
static uint8_t MOTOR_RAMP_DELAY_MS = 10; // Set this after Callibration

// The Timer is declared elsewhere
extern TIM_HandleTypeDef htim1;
// Remeber to do logging functions for errors

// Map motor index to TIM1 channel
static const uint32_t motor_channels[MOTOR_COUNT] = {
		TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};


// Initialize timers and PWM channels for all motors
void Motor_Init(void) {
	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		HAL_TIM_PWM_Start(&htim1, motor_channels[i]);
		__HAL_TIM_SET_COMPARE(&htim1, motor_channels[i], 0); // Set PWM TO 0
		motor_speeds[i] = 0;
	}
	motors_armed = 0;
}

// Arm/disarm motors (gradual ramp-up or immediate stop)
void Motor_Arm(void) {
	motors_armed = 1;
	// Ramp all motors to idle slowly
	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		motor_speeds[i] = MOTOR_ARMED_SPEED;
		__HAL_TIM_SET_COMPARE(&htim1, motor_channels[i], (uint32_t)((motor_speeds[i]/100.0f)*4199) );
	}
}

void Motor_Disarm(void) {
	motors_armed = 0;
	// Ramp all motors to idle slowly
	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		motor_speeds[i] = MOTOR_DISARMED_SPEED;
		__HAL_TIM_SET_COMPARE(&htim1, motor_channels[i], (uint32_t)((motor_speeds[i]/100.0f)*4199) );
	}
}

// Set individual motor speed (0-100%)
void Motor_SetSpeed(uint8_t motor_id, float duty) {
	if (motor_id >= MOTOR_COUNT) return; // and log, do that later
	if (duty < MOTOR_DUTY_MIN) duty = MOTOR_DUTY_MIN;
	if (duty > MOTOR_DUTY_MAX) duty = MOTOR_DUTY_MAX;
	motor_speeds[motor_id] = duty;
	__HAL_TIM_SET_COMPARE(&htim1, motor_channels[motor_id], (uint32_t)((duty/100.0f)*4199) );
}

// Set all motors at once
void Motor_SetAllSpeeds(const float duty[MOTOR_COUNT]) {
	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		Motor_SetSpeed(i, duty[i]);
	}
}

// Make micro-adjustments to a motor (used by PID)
void Motor_AdjustSpeed(uint8_t motor_id, float delta) {
	if (motor_id >= MOTOR_COUNT) return; // and log
	Motor_SetSpeed(motor_id, motor_speeds[motor_id] + delta);
}

// Gradually ramp motor to a target speed
void Motor_RampTo(uint8_t motor_id, float target_duty, float step) {
	if (motor_id >= MOTOR_COUNT) return; // and log

	float current_speed = motor_speeds[motor_id];

	if (current_speed < target_duty) {
		while (current_speed < target_duty) {
			current_speed += step;
			if (current_speed > target_duty) current_speed = target_duty;
			Motor_SetSpeed(motor_id, current_speed);
			HAL_Delay(MOTOR_RAMP_DELAY_MS);
		}
	} else {
		while (current_speed > target_duty) {
			current_speed -= step;
			if (current_speed < target_duty) current_speed = target_duty;
			Motor_SetSpeed(motor_id, current_speed);
			HAL_Delay(MOTOR_RAMP_DELAY_MS);
		}
	}
}

void Motor_RampAllTo(const float target_duty[MOTOR_COUNT], float step) {
	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		Motor_RampTo(i, target_duty[i], step);
	}
}

// Query current motor state
float Motor_GetSpeed(uint8_t motor_id) {
	if (motor_id >= MOTOR_COUNT) return 0.0f; // and log
	return motor_speeds[motor_id];
}

uint8_t Motor_IsArmed(void) {
	return motors_armed;
}
