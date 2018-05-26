/*
 * front.c
 *
 *  Created on: May 26, 2018
 *      Author: hyp
 */
#include "front.h"


int read_front(int fdfront){
	if(fdfront != -1){
		tFrontPacket frontPacket;
		frontPacket = read_front_frame(fdfront, frontPacket);
		if(frontPacket.error == -1){
			perror("Read Front ERROR");
		} else {
			uint16_t us_front_left = frontPacket.us_front_center_left.ui16Distance;
			uint16_t us_front_center_left = frontPacket.us_front_center_left.ui16Distance;
			uint16_t us_front_center = frontPacket.us_front_center.ui16Distance;
			uint16_t us_front_center_right = frontPacket.us_front_center_right.ui16Distance;
			uint16_t us_front_right = frontPacket.us_front_right.ui16Distance;
			printf("%u\t", us_front_left);
			printf("%u\t", us_front_center_left);
			printf("%u\t", us_front_center);
			printf("%u\t", us_front_center_right);
			printf("%u\n", us_front_right);

		}
		return 1;
	}
	return -1;

}

