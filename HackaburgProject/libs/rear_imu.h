/*
 * rear_imu.h
 *
 *  Created on: May 26, 2018
 *      Author: hyp
 */

#ifndef LIBS_REAR_IMU_H_
#define LIBS_REAR_IMU_H_
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <libudev.h>
#include "arduino_utilities.h"

int read_rear_imu(int fdrearimu);


#endif /* LIBS_REAR_IMU_H_ */
