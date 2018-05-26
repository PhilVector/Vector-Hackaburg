/*
 * rear_imu.c
 *
 *  Created on: May 26, 2018
 *      Author: hyp
 */
#include "rear_imu.h"


int read_rear_imu(int fdrearimu){
	tRearImuPacket rearimupacket = read_rear_imu_frame(fdrearimu, rearimupacket);
	if(rearimupacket.error == -1){
		perror("")
	}else{
		uint32_t wheel_right_tacho = rearimupacket.wheel_right.ui32WheelTach;
		int8_t wheel_right_direction = rearimupacket.wheel_right.i8WheelDir;
		uint32_t wheel_left_tacho = rearimupacket.wheel_left.ui32WheelTach;
		int8_t wheel_left_direction = rearimupacket.wheel_left.i8WheelDir;
		return 1;
	}
	return -1;
}
