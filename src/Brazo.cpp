#include "Brazo.h"

Brazo::Brazo()
	: s1(SERVO1_GP), s2(SERVO2_GP), s3(SERVO3_GP), s4(SERVO4_GP) {
}

Brazo::Brazo(uint8_t gp1, uint8_t gp2, uint8_t gp3, uint8_t gp4)
	: s1(gp1), s2(gp2), s3(gp3), s4(gp4) {
}

Brazo::~Brazo() {
}

void Brazo::init() {
	s1.init();
	s2.init();
	s3.init();
	s4.init();

	goDegrees(90, 0, 0, 0);
}

void Brazo::goDegree(uint8_t index, float degree) {
	switch (index) {
	case 1: s1.goDegree(degree); break;
	case 2: s2.goDegree(degree); break;
	case 3: s3.goDegree(degree); break;
	case 4: s4.goDegree(degree); break;
	default: /* ignore invalid index */ break;
	}
}

void Brazo::goDegrees(float d1, float d2, float d3, float d4) {
	s1.goDegree(d1);
	s2.goDegree(d2);
	s3.goDegree(d3);
	s4.goDegree(d4);
}

void Brazo::goAllTo(float degree) {
	goDegrees(degree, degree, degree, degree);
}

void Brazo::getAngles(float* angles) {
	if (angles == nullptr) {
		return;
	}
	angles[0] = s1.getAngle();
	angles[1] = s2.getAngle();
	angles[2] = s3.getAngle();
	angles[3] = s4.getAngle();
}
