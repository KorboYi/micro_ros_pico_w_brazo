/*
 * Brazo.h
 *
 *  Created on: 19 Aug 2025
 *      Author: korboyi
 */

#pragma once

#include "pico/stdlib.h"
#include "Servo.h"

 // Simple wrapper for a 4-servo robotic arm (brazo = arm)
class Brazo {
public:
	// Construct with the 4 GPIO pins used by each servo (order: 1..4)
	Brazo(uint8_t gp1, uint8_t gp2, uint8_t gp3, uint8_t gp4);

	// Initialize PWM and pins for all servos
	void init();

	// Move a single servo by index (0..3) to degree [0..180]
	void goDegree(uint8_t index, float degree);

	// Move all servos to the provided degrees
	void goDegrees(float d1, float d2, float d3, float d4);

	// Convenience: move all servos to the same degree
	void goAllTo(float degree);

	// Get all angles of the servos
	void getAngles(float* angles);

	void setSpeed(uint8_t index, float speed);

	void getSpeeds(float* speeds);

private:
	// The four servos
	Servo s1;
	Servo s2;
	Servo s3;
	Servo s4;
};
