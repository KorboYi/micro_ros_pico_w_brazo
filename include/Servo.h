/*
 * Servo.h
 *
 *  Created on: 1 Jun 2023
 *      Author: jondurrant
 */

#ifndef SERVO_SRC_SERVO_H_
#define SERVO_SRC_SERVO_H_

#include "pico/stdlib.h"

class Servo {
public:
	Servo(uint8_t gp);
	virtual ~Servo();

	void init();

	/**
	 * move to angle: 0 to 180.
	 * @param degree
	 */
	void goDegree(float degree);

	float getAngle();

private:
	uint8_t xGP = 0;
	float angle = 0.0f;
};

#endif /* SERVO_SRC_SERVO_H_ */