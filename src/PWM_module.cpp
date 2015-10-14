/*
 * PWMmodule.cpp
 *
 *  Created on: 23 Sep 2013
 *      Author: yashren
 */

#include "../inc/PWM_module.h"

PWM_module::PWM_module(void) {
    char filename[20];
    sprintf(filename,"/dev/i2c-1");

    // Get fd of module
    if ((fds_pwm_module = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the I2C bus for PWM_module.");
        exit(1);
    }
    if (ioctl(fds_pwm_module,I2C_SLAVE,PWM_MODULE_ADDRESS) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        exit(1);
    }

    PWM_packet[0] = pulsewidth2byte(900);
    PWM_packet[1] = pulsewidth2byte(1500);
    PWM_packet[2] = pulsewidth2byte(1500);
    PWM_packet[3] = pulsewidth2byte(1300);
    PWM_packet[4] = pulsewidth2byte(1300);
    PWM_packet[5] = pulsewidth2byte(1500);
}

bool PWM_module::plantInputs(float esc, float lateral, float longitudinal, float collective, float delta_rudder){
	    /*
	     * Plant to servo input mapping
	     * ----------------------------
	     * S = [dSrt dSrr dSlt]'
	     * U = [theta A B]
	     * U = A*S
	     *       1/3   1/3   1/3
	     * A =     1     0    -1
	     *        -1     2    -1
	     *
	     *            1   1/2  -1/6
	     * A^-1 =     1     0   1/3
	     *            1  -1/2  -1/6
	     *
	     * Hard saturation limits
	     * Collective = +-500
	     * Lateral    = +- 1000
	     * Longitudinal = +- 2000
	     */

	    float delta_right = collective + lateral/2 - longitudinal/6;
	    float delta_front = collective + longitudinal/3;
	    float delta_left = collective - lateral/2 - longitudinal/6 ;

	    PWM_packet[0] = pulsewidth2byte((int)esc);
	    PWM_packet[1] = pulsewidth2byte((int)(1500 + PWM_LEFT_SIGN*delta_left));
	    PWM_packet[2] = pulsewidth2byte((int)(1500 + PWM_FRONT_SIGN*delta_front));
	    PWM_packet[3] = pulsewidth2byte((int)(1300 + delta_rudder));
	    PWM_packet[4] = pulsewidth2byte(1300);
	    PWM_packet[5] = pulsewidth2byte((int)(1500 + PWM_RIGHT_SIGN*delta_right));

	    return sendPacket();
}

void PWM_module::updatePacket(int index, char data){
		PWM_packet[index] = data;
}

char PWM_module::pulsewidth2byte(int pulsewidth){
    float temp = (((float)pulsewidth - PWM_WIDTH_MIN)*(float)PWM_MAP_GAIN);

    if (temp < 0)
        return 0;

    if (temp > PWM_MAX_OUTPUT)
        return PWM_MAX_OUTPUT;

    return (char)temp;
}

bool PWM_module::sendPacket(void){
	int i = 0;
	char checksum = 0;
	char len = 6;

	pwm_packet.data[0] = 0x01;
	checksum ^= 0x01;
	for (i = 0; i < len; i++){
		pwm_packet.data[i+1] = PWM_packet[i];
		checksum ^= pwm_packet.data[i+1];
	}
	pwm_packet.data[i+1] = checksum;
	pwm_packet.length = len + 2;

	return sendData();
}

bool PWM_module::sendData(void){
	char read_data = 0;

	writeI2C(fds_pwm_module,&pwm_packet.data[0],pwm_packet.length);
	readI2C(fds_pwm_module,&read_data,1);

	if (read_data == 'A')
		return true;
	else
		return false;
}

bool PWM_module::writeI2C(int fds, char * buffer, int len)
{
	char * error_buf;
	if (write(fds, buffer, len) != len)
    {
        // ERROR HANDLING: i2c transaction failed
        printf("Failed to write to the i2c bus.\n");
        error_buf = strerror(errno);
        printf("%s\n\n",error_buf);
        return false;
    }
    else
		return true;
}

bool PWM_module::readI2C(int fds, char * buffer, int len)
{
	char * error_buf;

	if (read(fds, buffer, len) != len)
    {
        // ERROR HANDLING: i2c transaction failed
	    printf("Failed to read from the i2c bus.\n");
        error_buf = strerror(errno);
        printf("%s\n\n",error_buf);
        return false;
    }
    else
		return true;
}

PWM_module::~PWM_module() {
	close(fds_pwm_module);
	std::cout << "PWM_module has been killed." << std::endl;
}

