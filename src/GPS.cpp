/*
 * GPS.cpp
 *
 *  Created on: 14 Oct 2015
 *      Author: yashren
 */

#include "gps.h"

extern "C"{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_UART_rx(void* arg){
        GPS* t = static_cast<GPS*>(arg);
        t->receiveThread();
        return 0;
    }
}

GPS::GPS() {
    sprintf(devicename, "%s", UART_MODEMDEVICE);
    uart_fd = open(devicename, O_RDWR | O_NOCTTY);

    if (uart_fd < 0)
        perror(devicename);

    tcgetattr(uart_fd, &options);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~(ONLCR | OCRNL);
    // set baud rate
    cfsetispeed(&options, UART_BAUD);
    cfsetospeed(&options, UART_BAUD);
    tcsetattr(uart_fd, TCSANOW, &options);

    // create a read thread
    quit_rx_thread = false;
    if (pthread_create(&receive_thread, 0, &thread_catch_UART_rx, this) != 0){
        perror("Thread creation");
    }

    setlocale(LC_ALL,"C");
}

void GPS::receiveThread(void){
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = uart_fd;
    fds.events = POLLIN;

    int read_timeout = 3000;

    std::cout << "Entering GPS RX thread" << std::endl;
    // Flush read buffer
    tcflush(uart_fd, TCIFLUSH);

    while(!quit_rx_thread){
        int poll_ret = poll(&fds, 1, read_timeout);
        if(poll_ret > 0){
            int n = read(uart_fd, rx_buffer,256);
            rx_buffer[n] = '\0';
            printf("%s",rx_buffer);
        }
        else if (poll_ret == 0){
        	std::cout << "GPS timeout" << std::endl;
        }
    }

    std::cout << "Exiting GPS RX thread" << std::endl;
}

void GPS::stopReceiveThread(void){
    if (!quit_rx_thread){
        quit_rx_thread = true;
        // wait for thread to end
        pthread_join(receive_thread, NULL);
    }
}

void GPS::closeUART(void){
    stopReceiveThread();
    if (!(uart_fd < 0)){
        close(uart_fd);
    }
}

GPS::~GPS() {
	closeUART();
}
