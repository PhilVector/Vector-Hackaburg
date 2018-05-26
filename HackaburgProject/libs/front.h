/*
 * front.h
 *
 *  Created on: May 26, 2018
 *      Author: hyp
 */

#ifndef LIBS_FRONT_H_
#define LIBS_FRONT_H_
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

int read_front(int fdfront);

#endif /* LIBS_FRONT_H_ */
