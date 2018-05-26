/*
 * remote.h
 *
 *  Created on: May 26, 2018
 *      Author: virpwa
 */

#ifndef LIBS_REMOTE_H_
#define LIBS_REMOTE_H_
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

int read_remote(int fdremote, int *speed, int *steer);



#endif /* LIBS_REMOTE_H_ */
