/*
 * remote.c
 *
 *  Created on: May 26, 2018
 *      Author: virpwa
 */

#include "remote.h"

int read_remote(int fdremote, int *speed, int *steer) {
	tRemotePacket remote_packet;
	remote_packet = read_remote_frame(fdremote, remote_packet);
	if (remote_packet.error == -1) {
		perror("Read REMOTE");
		return -1;
	} else {
		uint16_t raw_speed = remote_packet.remote.ui16Speed;
		uint16_t raw_steer = remote_packet.remote.ui16Steer;
		speed = (int) (((float) raw_speed - 1500) * (90.0 / 500.0) + 90);
		steer = (int) (((float) raw_steer - 1500) * (90.0 / 500.0) + 90);
		return 1;
	}
}
