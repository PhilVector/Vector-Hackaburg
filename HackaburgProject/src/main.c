/*
 * main.c
 *
 *  Created on: May 26, 2018
 *      Author: virpwa
 */

#include "arduino_utilities.h"
#include "actuator.h"
int main(int argc, const char *argv[]) {
	int fdremote = scan_ttyACM(ARD_REMOTE);
	int fdactor = scan_ttyACM(ARD_ACTOR);

//	while(true){

	sleep(1);
	int speed = 90;
	int steer = 90;


	if (fdremote != -1) {
		int ret = read_remote(fdremote, &speed, &steer);

	}

	if (fdremote != -1) {
		int ret = send_actuator(fdactor, speed, steer);
		if (ret == -1) {
			perror("Error");
		}
	}
//}
	return 0;
}

