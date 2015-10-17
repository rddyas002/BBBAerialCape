/*
 * BatteryMonitor.cpp
 *
 *  Created on: 15 Oct 2015
 *      Author: yashren
 */

#include "BatteryMonitor.h"

extern "C"{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_battery(void* arg)
    {
    	BatteryMonitor* t = static_cast<BatteryMonitor*>(arg);
        t->process();
        return 0;
    }
}

BatteryMonitor::BatteryMonitor() {
	batt_fd = open(BATTERY_ADC_PATH, O_RDONLY);

	if (batt_fd > 0){
	    // create a read thread
	    quit_rx_thread = false;
	    if (pthread_create(&receive_thread, 0, &thread_catch_battery, this) != 0){
	        perror("Battery monitor thread creation");
	    }
	}
	else{
		perror("Battery monitor file open error");
	}

}

void BatteryMonitor::process(void){
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = batt_fd;
    fds.events = POLLIN;

    int read_timeout = 3000;

    std::cout << "Entering Battery Monitor thread" << std::endl;

    while(!quit_rx_thread){
        int poll_ret = poll(&fds, 1, read_timeout);
        if(poll_ret > 0){
            int n = read(batt_fd, rx_buffer, 10);
            rx_buffer[n] = '\0';
        	if (n != -1){
        		rx_buffer[n] = '\0';
        		int value = atoi(rx_buffer);
        		battery_voltage = (float)value*(1.8/4095)*(10470/470);
        		printf("Battery voltage is %.2fV\r\n", battery_voltage);
        		if (battery_voltage < 11.1){
        			printf("WARNING! Battery voltage is %.2fV\r\n", battery_voltage);
        		}
        		lseek(batt_fd, 0, 0);
        	}
        }
        else if (poll_ret == 0){
        	std::cout << "Battery Monitor timeout" << std::endl;
        }
        sleep(BATTERY_MONITOR_PERIOD);
    }

    std::cout << "Exiting Battery Monitor RX thread" << std::endl;
}

BatteryMonitor::~BatteryMonitor() {
	if (batt_fd > 0)
		close(batt_fd);
}

