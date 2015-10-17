/*
 * BatteryMonitor.h
 *
 *  Created on: 15 Oct 2015
 *      Author: yashren
 */

#ifndef BATTERYMONITOR_H_
#define BATTERYMONITOR_H_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <tr1/stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <locale.h>
#include <bitset>
#include <climits>

#define BATTERY_ADC_PATH "/sys/devices/ocp.3/44e0d000.tscadc/tiadc/iio:device0/in_voltage0_raw"
#define BATTERY_MONITOR_PERIOD	5

class BatteryMonitor {
public:
	BatteryMonitor();
	virtual void process(void);
	virtual ~BatteryMonitor();
private:
	int batt_fd;
	float battery_voltage;

    bool quit_rx_thread;
    pthread_t receive_thread;
    char rx_buffer[10];
};

#endif /* BATTERYMONITOR_H_ */
