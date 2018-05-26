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
		return 1;
	}
	return -1;

}

