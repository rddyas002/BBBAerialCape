/*
 * gps.h
 *
 *  Created on: 14 Oct 2015
 *      Author: yashren
 */

#ifndef GPS_H_
#define GPS_H_

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

using namespace std;

#define UART_BAUD B57600
#define UART_DATABITS CS8
#define UART_STOPBITS 0
#define UART_PARITY 0
#define UART_PARITYON 0
#define UART_MODEMDEVICE "/dev/ttyO2"
#define _POSIX_SOURCE 1         //POSIX compliant source

class GPS {
public:
	GPS();
	void stopReceiveThread(void);
	void closeUART(void);

	virtual void receiveThread(void);
	virtual ~GPS();
private:
    int uart_fd;
    struct termios options;       //place for old and new port settings for serial port
    struct sigaction saio;               //definition of signal action
    char devicename[80];

    bool quit_rx_thread;
    pthread_t receive_thread;

    char rx_buffer[256];
};

#endif /* GPS_H_ */
