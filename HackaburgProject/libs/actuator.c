/*
 * actuator.c
 *
 *  Created on: May 26, 2018
 *      Author: virpwa
 */

#include "actuator.h"

int send_actuator(int fdactor, int speed, int steer) {
//	int ret1 = get_sensor_arduino(fdactor, ARD_ACTOR);
//	if (ret1 == -1) {
//		return -1;
//	}
	struct timeval tv;
	gettimeofday(&tv, NULL);
	unsigned long micros = 1000000 * tv.tv_sec + tv.tv_usec;
	uint32_t send_micros = micros;
	uint8_t one = 1;

	send_actor_frame(fdactor, ID_ARD_ACT_SPEED_CONTR, one, send_micros, speed);

	int ret2 = send_actor_frame(fdactor, ID_ARD_ACT_WATCHDOG, one, send_micros,
			one);
	if (ret2 == -1) {
		return -1;
	}
	int ret3 = send_actor_frame(fdactor, ID_ARD_ACT_STEER_SERVO, one,
			send_micros, steer);
	if (ret3 == -1) {
		return -1;
	}

	return 1;

}
