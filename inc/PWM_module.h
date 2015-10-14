/*
 * PWMmodule.h
 *
 *  Created on: 23 Sep 2013
 *      Author: yashren
 */

#ifndef PWMMODULE_H_
#define PWMMODULE_H_

#include <iostream>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>
#include <netdb.h>
#include <sys/time.h>
#include <fstream>
#include <cmath>

#define PWM_MODULE_ADDRESS (0b01001010)
#define PWM_LENGTH_MAX 		(20)

// Specs on PWM module in us
#define PWM_WIDTH_MAX       (1998)
#define PWM_WIDTH_MIN       (900)
#define PWM_MAX_OUTPUT      (255)
#define PWM_INPUT_DIFF      (PWM_WIDTH_MAX - PWM_WIDTH_MIN)
#define PWM_MAP_GAIN        ((float)PWM_MAX_OUTPUT/PWM_INPUT_DIFF)

#define PWM_RIGHT_SIGN		(1)
#define PWM_LEFT_SIGN		(-1)
#define PWM_FRONT_SIGN		(-1)

#define PWM_RIGHT_CHANNEL   5
#define PWM_LEFT_CHANNEL	1
#define PWM_FRONT_CHANNEL	2

typedef struct{
	char length;
	char data[PWM_LENGTH_MAX];
} PWM_packet_struct;

class PWM_module {
public:
	PWM_module(void);
	bool sendPacket(void);
	bool sendData(void);
	bool writeI2C(int fds, char * buffer, int len);
	bool readI2C(int fds, char * buffer, int len);
	char pulsewidth2byte(int pulsewidth);
	bool plantInputs(float esc, float lateral, float longitudinal, float collective, float delta_rudder);
	void updatePacket(int index, char data);
	virtual ~PWM_module();

private:
	int fds_pwm_module;
	PWM_packet_struct pwm_packet;
	char PWM_packet[10];
};

#endif /* PWMMODULE_H_ */
