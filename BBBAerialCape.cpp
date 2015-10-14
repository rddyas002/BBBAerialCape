/*
 * BBBAerialCape.cpp
 *
 *  Created on: 13 Oct 2015
 *      Author: yashren
 */

#include <iostream>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mpu9150.h"
#include "SpektrumRX.h"
#include "PWM_module.h"
#include "gps.h"

#define THREAD_PRIORITY 85
#define SAMPLE_FREQ 50

using namespace std;

double time_t0 = 0.0;

const char AIN_DEV[] = "/sys/devices/ocp.2/44e0d000.tscadc/tiadc/iio:device0/in_voltage0_raw";
int batt_fd = -1;

double timeNow(void){
	struct timespec tv;

	if (time_t0 == 0.0){
		clock_gettime(CLOCK_MONOTONIC, &tv);
		time_t0 = (tv.tv_sec)*1e6 + (tv.tv_nsec)/1e3;
		return 0.0;
	}
	else{
		clock_gettime(CLOCK_MONOTONIC, &tv);
		return ((tv.tv_sec)*1e6 + (tv.tv_nsec)/1e3 - time_t0);
	}
}

float getBatteryVoltage(void){
	char buffer[10];
	int ret = read(batt_fd, buffer, 4);
	if (ret != -1){
		buffer[ret] = '\0';
		int value = atoi(buffer);
		printf("Battery voltage: %.2f volts\n", (float)value*10.470/470);
		lseek(batt_fd, 0, 0);
		return (float)value*10.470/470;
	}
	return -1;
}

int main (int argc, char * arg[]){
	// test functions
//	batt_fd = open(AIN_DEV, O_RDONLY);
//	GPS gps_sensor = GPS();
//	while(1);
//	while(1){
//		getBatteryVoltage();
//		sleep(2);
//	}
	//PWM_module PWMmodule;
	float frequency = 1;
	PWM_module * PWMmodule;
	PWMmodule = new PWM_module();

	timeNow();
	SpektrumRX spektrumRx = SpektrumRX(time_t0, PWMmodule);
	while(1){


	}

	mpu9150 IMUSensor = mpu9150(time_t0);



	IMUSensor.autoSample(THREAD_PRIORITY, SAMPLE_FREQ);

	char buffer[20];

	int c=0;
	char data[6] = {127};


	while(c++ < 200){
//		float temp = 300*sin(2*M_PI*frequency*(timeNow()/1e6));
//		bool ret = PWMmodule.plantInputs(900,0,0,temp,0);

//		printf("Packet %d, Data received %4.2f %9.0fus\r\n", c, temp, timeNow());
//		fflush(stdout);


		system("clear");

		printf("%-18d%15s%15s%15s\n",
				c,
				"x axis",
				"y axis",
				"z axis");
		printf("%-18s%15.2f%15.2f%15.2f\n",
				"Accelerometer",
				IMUSensor.getAccelX(),
				IMUSensor.getAccelY(),
				IMUSensor.getAccelZ());

		printf("%-18s%15.2f%15.2f%15.2f\n",
				"Gyroscope",
				IMUSensor.getGyroX(),
				IMUSensor.getGyroY(),
				IMUSensor.getGyroZ());

//		printf("%-18s%15.2f%15.2f%15.2f\n",
//				"Magnetometer",
//				IMUSensor.getMagnetox(),
//				IMUSensor.getMagnetoy(),
//				IMUSensor.getMagnetoz());

		printf("%-18s%15.2f\n",
				"Temperature",
				IMUSensor.getGyroTemp());

		fflush(stdout);
		usleep(100000);
	}

//	int c=0;
//	while(c++ < 1000){
//		usleep(10000);
//	}
    return 0;
}

