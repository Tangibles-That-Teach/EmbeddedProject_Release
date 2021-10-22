/** ----------------------------------------------------------------------------
Copyright 2020-2021 Tangibles That Teach, LLC, 4617 Chattahoochee Crossing SE,
Marietta, GA 30067, U.S.A. All rights reserved.

Tangibles That Teach, LLC has intellectual property rights relating to
technology embodied in this product. In particular, and without
limitation, these intellectual property rights may include one or more
of U.S. patents or pending patent applications in the U.S. and/or other countries.

This product is distributed under licenses restricting its use, copying
distribution, and decompilation. No part of this product may be
reproduced in any form by any means without prior written authorization
of Tangibles That Teach, LLC.

Contact Info:
Email: contact@tangiblesthatteach.com

@brief: Main program for the T-RECS running on the Raspberry Pi Pico board
@author: Joel Dunham <joel.dunham@tangiblesthatteach.com>
@date: 2021/04/07
------------------------------------------------------------------------------ **/

#include <t3Library.h>
#include <algorithm>
#include <Arduino.h>

T3Library::T3Library t3Library(false, T3Library::T3Library::TRECS_PROJECT);  // Simulated for testing, example for a T-RECS project

// Set up the TRECS control and sensors
// Controller parameters
#define P_GAIN          (2.3)     // Proportional gain
#define I_GAIN          (4.8)     // Integral gain
#define D_GAIN          (0.4)     // Derivative gain
#define MIN_I_TERM      (-250.0)  // Minimum Contribution of iTerm in PI controller
#define MAX_I_TERM      (250.0)   // Maximum Contribution of iTerm in PI controller

#define MOTOR_CONTROL_PIN (2)

// Sensor parameters
#define MIN_ANGLE_DEG   (-65.0)
#define MAX_ANGLE_DEG   (30.0)
static const uint8_t SENSOR_PIN{26};  // Pin for reading sensor values (ADC0)
#define SENSOR_SLOPE    (-0.3656) // Sensor slope (deg)
#define SENSOR_INTERCEPT (185.64) // Sensor intercept (deg)

/* Local controller struct */
typedef struct {
	int8_t  Ki_id;            // ID of the integral gain
	int8_t  Kp_id;            // ID of the proportional gain
	int8_t  Kd_id;            // ID of the derivative gain
	int8_t  max_i_term_id;    // ID of the maximum integral value
	int8_t  min_i_term_id;    // ID of the minimum integral value
	float   old_error;        // Last error value
	float   integrator_state; // Integrator state (accumulated error)
	uint64_t last_time_ms;    // Last time the controller was called
} PID_t;
PID_t pid_controller;

// Sensor
int8_t angle_id;


/* Filter buffer size */
const uint8_t FILTER_BUFFER_SIZE = 3;
float filterBuffer[FILTER_BUFFER_SIZE];  // Array for moving average filter
float filteredVal;                		 // Current filtered valued
int filterIndex;                         // Current index of filterBuffer


// ================================================================
// ===                    HELPER FUNCTIONS                      ===
// ================================================================

/*
 * Lowpass moving average filter to smooth analog sensor readings
 */
float filter_sensor(float sensedValue, uint64_t time_ms, bool reset) {
	// Handle if the filter is being reset
	uint8_t max_steps = (reset == true) ? FILTER_BUFFER_SIZE : 1;
	for(uint8_t counter = 0; counter < max_steps; counter++) {
		// Remove oldest value from moving average
		filteredVal -= filterBuffer[filterIndex] / (float)FILTER_BUFFER_SIZE;

		// Add new value to buffer and incrememnt index
		filterBuffer[filterIndex++] = sensedValue;

		// Add new value to moving average
		filteredVal += sensedValue / (float)FILTER_BUFFER_SIZE;

		// Prevent index out of bounds errors
		filterIndex %= FILTER_BUFFER_SIZE;
	}

	return filteredVal;
}

/*
 * Update PID output signal using current system state
 */
uint16_t updatePID(float goalValue, uint64_t time_ms, bool reset) {
	// P, I, & D terms
	float pTerm, iTerm, dTerm;

	// Get the sensed value
	float sensedValue = 0.0;
	t3Library.getFilteredSensorById(angle_id, &sensedValue);

	// Reset any state values if required.
	if(reset == true) {
		pid_controller.old_error = 0.0;
		pid_controller.integrator_state = 0.0;
		pid_controller.last_time_ms = time_ms;  // So the integrator does not jump
	}

	// Controller error is difference between goal and current state
	float error = goalValue - sensedValue;

	// Delta time
	float dt = (float)(time_ms - pid_controller.last_time_ms) / 1000.0;

	// Get controller terms
	float ki = 0.0;
	float kp = 0.0;
	float kd = 0.0;
	float max_i_term = 0.0;
	float min_i_term = 0.0;
	t3Library.getParameterById(pid_controller.Ki_id, &ki);
	t3Library.getParameterById(pid_controller.Kp_id, &kp);
	t3Library.getParameterById(pid_controller.Kd_id, &kd);
	t3Library.getParameterById(pid_controller.max_i_term_id, &max_i_term);
	t3Library.getParameterById(pid_controller.min_i_term_id, &min_i_term);

	// Calculate the proportional term
	pTerm = kp * error;

	// Calculate the integral state with appropriate min/max constraints
	pid_controller.integrator_state += error * dt;
	pid_controller.integrator_state =
		std::max(std::min(pid_controller.integrator_state, max_i_term / ki), min_i_term / ki);

	// Calculate the integral term
	iTerm  = ki * pid_controller.integrator_state;

	// Calculate the derivative term
	dTerm  = (dt > 0.001) ? kd * ((error - pid_controller.old_error) / dt) : 0.0;  // Handle the reset/zero dt case

	// Update the state of the controller
	pid_controller.old_error = error;
	pid_controller.last_time_ms = time_ms;

	// Add PID terms to get new drive signal (0-MAX_PWM scale)
	return uint16_t(std::max(0.0f, pTerm + iTerm + dTerm));  // Do not allow negatives to avoid any wrapping issues
}

void setup() {
	// Local setup
	pid_controller.last_time_ms = 0;

	// Define the command that is driven and that is modifiable through the front end.
	t3Library.defineCommand(0.0, MIN_ANGLE_DEG, MAX_ANGLE_DEG, "Angle(deg)");

	// Define controller parameters that are modifiable through the front end.
	// Define default values if the parameters are not already stored in the EEPROM.
	pid_controller.Kp_id = t3Library.addParameter(P_GAIN, true, 0.0, false, 0.0, "Kp");
	pid_controller.Ki_id = t3Library.addParameter(I_GAIN, true, 0.0, false, 0.0, "Ki");
	pid_controller.Kd_id = t3Library.addParameter(D_GAIN, true, 0.0, false, 0.0, "Kd");
	pid_controller.min_i_term_id = t3Library.addParameter(MIN_I_TERM, false, 0.0, false, 0.0, "Min I");
	pid_controller.max_i_term_id = t3Library.addParameter(MAX_I_TERM, false, 0.0, false, 0.0, "Max I");

	// Define sensor parameters that are available to the front end
	// Define the translated limits to enable auto-translation after calibration.
	// Angle should ALWAYS be the first sensor for the T-RECS.
	angle_id = t3Library.addSensor(SENSOR_PIN, true, MIN_ANGLE_DEG, MAX_ANGLE_DEG, true, filter_sensor, "Angle(deg)");
	t3Library.setSensorLinearModel(angle_id, SENSOR_INTERCEPT, SENSOR_SLOPE);

	// Set the external function for running the controller
	// If this functions is not set, then this step is simply skipped.
	t3Library.setControllerFunction(updatePID);
	t3Library.setMotorControlPin(MOTOR_CONTROL_PIN);  // Update for RPi Pico

	// Setup all required functions that are kept internal/not modifiable
	// This must happen last after all setup of parameters and sensors and after adding the controller function
	t3Library.setup();

	// Put any additional local setup steps here
}

void loop() {
	// Update all internal/non-modifiable functions
	t3Library.update();

	// Any updates here should be kept to a minimum because it may affect the timing of the sensors and controller
}
