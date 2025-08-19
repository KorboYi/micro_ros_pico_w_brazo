/*
 * Brazo.h
 *
 *  Created on: 19 Aug 2025
 *      Author: automated
 */

#ifndef BRAZO_H_
#define BRAZO_H_

#include "pico/stdlib.h"
#include "Servo.h"

#define SERVO1_GP 27
#define SERVO2_GP 26
#define SERVO3_GP 22
#define SERVO4_GP 21

// Simple wrapper for a 4-servo robotic arm (brazo = arm)
class Brazo {
public:
    // Default constructor using the project's servo pins
    Brazo();
	// Construct with the 4 GPIO pins used by each servo (order: 1..4)
	Brazo(uint8_t gp1, uint8_t gp2, uint8_t gp3, uint8_t gp4);
	virtual ~Brazo();

	// Initialize PWM and pins for all servos
	void init();

	// Move a single servo by index (0..3) to degree [0..180]
	void goDegree(uint8_t index, float degree);

	// Move all servos to the provided degrees
	void goDegrees(float d1, float d2, float d3, float d4);

	// Convenience: move all servos to the same degree
	void goAllTo(float degree);

private:
	// The four servos
	Servo s1;
	Servo s2;
	Servo s3;
	Servo s4;
};

#endif /* BRAZO_H_ */
