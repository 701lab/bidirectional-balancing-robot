#ifndef MOTORS_H_
#define MOTORS_H_

#include "interfacing.h"





typedef struct{

	float kp;
	float target_position;
	float current_position;
	float regulator_control_signal;
	float position_precision;
	int16_t previous_encoder_counter_value;

} motor_position_controler;


typedef struct{

	float current_integral;
	float kp;
	float ki;
	float target_speed;		// Revolutions of reducer shaft per second
	float current_speed;
	float previous_speed_mistake; // Used in trapezoidal integration
	float regulator_control_signal;
	float controller_output_limitat;

	int16_t previous_encoder_counter_value;

} motor_speed_controler;


typedef struct{

	float torque_coeficient;	// Cm

}torque_control;

//****** Motor definition for control ******//
/*
	@ brief

	@Calculations

	encoder_constant = 4 * encoder_PPR * reducer_reduction_ratio [pulses per reducer shaft revolution]

	reducer_reduction_ration = 1 if reduces isn't used

								 encoder_counter_value
	reducer_shaft_revolutions = -----------------------
								   encoder_constant

	where 4 is used because of the algorithm
 */
typedef struct{

	motor_position_controler *position_controller;
	motor_speed_controler *speed_controller;

	void (*enable)(void);
	void (*disable)(void);
	uint32_t (*set_pwm_duty_cycle)(const int32_t duty_cycle_coefficient);
	int16_t	(*get_encoder_counter_val)(void);

	const float encoder_constant;

	const uint32_t max_duty_cycle;

} motor;

/*
	@brief Checks if given motor shaft rotates in the same direction as encoder counts. Returns mistake code if rotation and counting are in different directions, or encoder counted 0.
 */
uint32_t motors_rotation_deiraction_test (motor *motor_instance);

float motors_get_position(motor *motor_instance);
float motor_get_speed_by_incements(const motor *motor_instance, const float time_increment);

float motors_position_controller_handler(motor *motor_instance); // returns speed task
float motors_speed_controller_handler(const motor *motor_instance, const float time_increment); // ideally should return current controller task (float)

float motors_combined_position_controller_nadler(motor *motor_instance);


void motor_reset(motor *motor_instance);

#endif /* MOTORS_H_ */
