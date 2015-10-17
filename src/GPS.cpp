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

ofstream GPS::logFileGPS;
//char GPS::write_buffer[512] = {0};

GPS::GPS(double t0) {
	time_t0 = t0;
	openLogFile();
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

//$GPGGA,045956.742,,,,,0,0,,,M,,M,,*42
//$GPGLL,,,,,045956.742,V,N*70
//$GPGSA,A,1,,,,,,,,,,,,,,,*1E
//$GPGSV,1,1,00*79
//$GPRMC,045956.742,V,,,,,0.00,0.00,161015,,,N*45
//$GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32

void GPS::test(void){
    const char *buff[] = {
        "$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n",
        "$GPGGA,111609.14,5001.27,N,3613.06,E,3,08,0.0,10.2,M,0.0,M,0.0,0000*70\r\n",
        "$GPGSV,2,1,08,01,05,005,80,02,05,050,80,03,05,095,80,04,05,140,80*7f\r\n",
        "$GPGSV,2,2,08,05,05,185,80,06,05,230,80,07,05,275,80,08,05,320,80*71\r\n",
        "$GPGSA,A,3,01,02,03,04,05,06,07,08,00,00,00,00,0.0,0.0,0.0*3a\r\n",
        "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n",
        "$GPVTG,217.5,T,208.8,M,000.00,N,000.01,K*4C\r\n"
    };
    int it;
    nmeaINFO info;
    nmeaPARSER parser;

    nmea_zero_INFO(&info);
    nmea_parser_init(&parser);

    for(it = 0; it < 6; ++it)
        nmea_parse(&parser, buff[it], (int)strlen(buff[it]), &info);

	printf("lat: %f lon: %f elv: %f\r\n", info.lat, info.lon, info.elv);

    nmea_parser_destroy(&parser);
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
    gps_index = 0;
    int i;

    while(!quit_rx_thread){
        int poll_ret = poll(&fds, 1, read_timeout);
        if(poll_ret > 0){
            int n = read(uart_fd, rx_buffer, 256);
            rx_buffer[n] = '\0';
            printf("%s", rx_buffer);
    		// Log raw data
            logFileGPS << rx_buffer;
//            printf("%d \r\n",n);
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

void GPS::openLogFile(void){
	logFileGPS.open("gpsLog.txt");
}

void GPS::closeLogFile(void){
	logFileGPS.close();
}

bool GPS::writeLogFile(void){
	if (logFileGPS.is_open()){
		logFileGPS << log_buffer;
		return true;
	}
	else
		return false;
}

void GPS::closeUART(void){
    stopReceiveThread();
    if (!(uart_fd < 0)){
        close(uart_fd);
    }
}

GPS::~GPS() {
	closeUART();
	if (logFileGPS.is_open())
		closeLogFile();
}
