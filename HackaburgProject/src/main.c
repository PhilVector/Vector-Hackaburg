/*
 * main.c
 *
 *  Created on: May 26, 2018
 *      Author: virpwa
 */

#include <pthread.h>
#include <sched.h>
#include <semaphore.h>

#include "arduino_utilities.h"
#include "actuator.h"
#include "remote.h"


#define MAX_PRIO (sched_get_priority_min(SCHED_FIFO) + 3)



int main(int argc, const char *argv[]) {
	int fdremote = scan_ttyACM(ARD_REMOTE);
	int fdfront = scan_ttyACM(ARD_FRONT);
	int fdrearimu = scan_ttyACM(ARD_REAR_IMU);
	int fdactor = scan_ttyACM(ARD_ACTOR);

//	while(true){

	sleep(1);
	int speed = 90;
	int steer = 90;


	if (fdremote != -1) {
		int ret = read_remote(fdremote, &speed, &steer);
		if (ret == -1) {
			perror("Error");
		}
	}

	if (fdactor != -1) {
		int ret = send_actuator(fdactor, speed, steer);
		if (ret == -1) {
			perror("Error");
		}
	}

	while(true){
	if (fdfront != -1) {
		int ret = read_front(fdfront);
		if (ret == -1) {
			perror("Error");
		}
	}
	}

//}
	return 0;
}

