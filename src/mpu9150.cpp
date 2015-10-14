#include "../inc/mpu9150.h"

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_imu(void* arg)
    {
    	mpu9150* t = static_cast<mpu9150*>(arg);
        t->process();
        return 0;
    }
}

ofstream mpu9150::logFileMPU;
char mpu9150::write_buffer[512] = {0};

mpu9150::mpu9150(double t0) {
    char filename[20];
    sprintf(filename,"/dev/i2c-2");

    time_t0 = t0;

    // Get fd of mpu
    if ((fd = open(filename,O_RDWR)) < 0) {
        printf("Failed to open I2C1 the bus for MPU9150.\r\n");
        exit(1);
    }
    // Bind address to file descriptor
    if (ioctl(fd,I2C_SLAVE,MPU_ADDRESS) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        exit(1);
    }

    setup();

    accelerometer.x_axis = 0.0;
    accelerometer.y_axis = 0.0;
    accelerometer.z_axis = 0.0;
    accelerometer.x_mean = 0.0;
    accelerometer.y_mean = 0.0;
    accelerometer.z_mean = 0.0;
    gyroscope.temperature = 0;
    gyroscope.x_axis = 0.0;
    gyroscope.y_axis = 0.0;
    gyroscope.z_axis = 0.0;
    gyroscope.x_mean = 0.0;
    gyroscope.y_mean = 0.0;
    gyroscope.z_mean = 0.0;

    // Find biases, assuming level
    int iter = 0;
	float init_bias_x_magneto = 0, init_bias_y_magneto = 0, init_bias_z_magneto = 0;
	float init_bias_x_gyro = 0, init_bias_y_gyro = 0, init_bias_z_gyro = 0;
	float init_bias_x_accel = 0, init_bias_y_accel = 0, init_bias_z_accel = 0;
	float gyroscope_sample[CALIB_ITERATION][3] = {{0},{0}};
	float accelerometer_sample[CALIB_ITERATION][3] = {{0},{0}};
	float magnetometer_sample[CALIB_ITERATION][3] = {{0},{0}};

    for (iter = 0; iter < CALIB_ITERATION; iter++){
    	getIMUdata();

		gyroscope_sample[iter][0] = gyroscope.x_axis;
		gyroscope_sample[iter][1] = gyroscope.y_axis;
		gyroscope_sample[iter][2] = gyroscope.z_axis;

		accelerometer_sample[iter][0] = accelerometer.x_axis;
		accelerometer_sample[iter][1] = accelerometer.y_axis;
		accelerometer_sample[iter][2] = accelerometer.z_axis;

		init_bias_x_gyro += gyroscope.x_axis;
		init_bias_y_gyro += gyroscope.y_axis;
		init_bias_z_gyro += gyroscope.z_axis;

		init_bias_x_accel += accelerometer.x_axis;
		init_bias_y_accel += accelerometer.y_axis;
		init_bias_z_accel += accelerometer.z_axis;

		usleep(10000);
    }

    // Calculate variance
    for (iter = 0; iter < CALIB_ITERATION; iter++){
    	gyroscope.x_err_var += pow(gyroscope_sample[iter][0],2);
    	gyroscope.y_err_var += pow(gyroscope_sample[iter][1],2);
    	gyroscope.z_err_var += pow(gyroscope_sample[iter][2],2);

    	accelerometer.x_err_var += pow(accelerometer_sample[iter][0],2);
    	accelerometer.y_err_var += pow(accelerometer_sample[iter][1],2);
    	accelerometer.z_err_var += pow(accelerometer_sample[iter][2],2);
    }

	gyroscope.x_mean = init_bias_x_gyro/CALIB_ITERATION;
	gyroscope.y_mean = init_bias_y_gyro/CALIB_ITERATION;
	gyroscope.z_mean = init_bias_z_gyro/CALIB_ITERATION;

	accelerometer.x_mean = init_bias_x_accel/CALIB_ITERATION;
	accelerometer.y_mean = init_bias_y_accel/CALIB_ITERATION;
	accelerometer.z_mean = init_bias_z_accel/CALIB_ITERATION;

	gyroscope.x_err_var = gyroscope.x_err_var/CALIB_ITERATION - pow(gyroscope.x_mean,2);
	gyroscope.y_err_var = gyroscope.y_err_var/CALIB_ITERATION - pow(gyroscope.y_mean,2);
	gyroscope.z_err_var = gyroscope.z_err_var/CALIB_ITERATION - pow(gyroscope.z_mean,2);

	accelerometer.x_err_var = accelerometer.x_err_var/CALIB_ITERATION - pow(accelerometer.x_mean,2);
	accelerometer.y_err_var = accelerometer.y_err_var/CALIB_ITERATION - pow(accelerometer.y_mean,2);
	accelerometer.z_err_var = accelerometer.z_err_var/CALIB_ITERATION - pow(accelerometer.z_mean,2);

    autoSampleDelay = 100000;	// usec default
    count_mutex = PTHREAD_MUTEX_INITIALIZER;

	sprintf(write_buffer, "%s %f,%f,%f\r\n%s %f,%f,%f\r\n%s %f,%f,%f\r\n%s %f,%f,%f\r\n\r\n",
			"Accelerometer variance:",accelerometer.x_err_var, accelerometer.y_err_var, accelerometer.z_err_var,
			"Accelerometer mean:",accelerometer.x_mean, accelerometer.y_mean, accelerometer.z_mean,
			"Gyroscope variance:",gyroscope.x_err_var,gyroscope.y_err_var,gyroscope.z_err_var,
			"Gyroscope mean:",gyroscope.x_mean, gyroscope.y_mean, gyroscope.z_mean);

	openLogFile();
}

void mpu9150::setup(){
	buffer[0] = MPU9150_PWR_MGMT_1;
	// Sleep mode off, use gyro x as PLL
	buffer[1] = 0b00000001;
    if (writeI2C(fd,&buffer[0],2))
		printf("Gyroscope sleep-mode off.\r\n");

	// Set gyro range to 2000deg/s
	buffer[0] = MPU9150_GYRO_CONFIG;
	buffer[1] = 0b00011000;
    if (writeI2C(fd,&buffer[0],2))
		printf("Gyroscope range set.\r\n");

	// Set accel range to 16g
	buffer[0] = MPU9150_ACCEL_CONFIG;
	buffer[1] = 0b00011000;
    if (writeI2C(fd,&buffer[0],2))
		printf("Accelerometer range set.\r\n");

	// Set DLPF to 0
	buffer[0] = MPU9150_CONFIG;
	buffer[1] = 0b00000000;
    if (writeI2C(fd,&buffer[0],2))
		printf("Internal sampling of 1kHz and 8kHz of accelerometer and gyroscope.\r\n");

	// Set sample update rate
	buffer[0] = MPU9150_SMPLRT_DIV;
	buffer[1] = 31;
    if (writeI2C(fd,&buffer[0],2))
		printf("Set register update rate.\r\n");
}

void mpu9150::autoSample(int threadPriority, int rate){
	autoSampleDelay = 1e6/rate;
	pthread_create(&autoSample_thread, 0, &thread_catch_imu, this);

    // Make thread priority high
    struct sched_param sp;
    sp.sched_priority = threadPriority;
    pthread_setschedparam(autoSample_thread, SCHED_FIFO, &sp);
}

void mpu9150::process(void){
	double elapsedTime, start_time, end_time;
	int remainderDelay = 0;

	autoSampleThreadRun = true;

	while(autoSampleThreadRun){
		start_time = timeSinceStart();
		getIMUdata();

		// Log raw data
        sprintf(write_buffer, "%f,%f,%f,%f,%f,%f,%f,%.0f\n",
        		gyroscope.x_axis,gyroscope.y_axis,gyroscope.z_axis,
        		accelerometer.x_axis,accelerometer.y_axis,accelerometer.z_axis,
        		gyroscope.temperature,
        		timeSinceStart());
        writeLogFile();

        end_time = timeSinceStart();

	    elapsedTime = start_time - end_time;

	    remainderDelay = autoSampleDelay - elapsedTime;
	    if (remainderDelay < 0){
	    	remainderDelay = 1000;
	    	std::cout << "miniIMU Err: remainderDelay less than zero." << std::endl;
	    }

		usleep(remainderDelay);
	}
}

void mpu9150::stopAutoSample(void){
	pthread_mutex_lock( &count_mutex );
	autoSampleThreadRun = false;
	pthread_mutex_unlock( &count_mutex );
}

double mpu9150::timeSinceStart(void){
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return ((tv.tv_sec)*1e6 + (tv.tv_nsec)/1e3 - time_t0);
}

bool mpu9150::writeI2C(int fds, char * buffer, int len){
	char * error_buf;

	if (write(fds, buffer, len) != len)
    {
        // ERROR HANDLING: i2c transaction failed
        printf("Failed to write to the I2C1 bus.\r\n");
        error_buf = strerror(errno);
        printf(error_buf);
        printf("\n\n");
        return false;
    }
    else
		return true;
}

bool mpu9150::readI2C(int fds, char * buffer, int len){
	char * error_buf;

	if (read(fds, buffer, len) != len)
    {
        // ERROR HANDLING: i2c transaction failed
	    printf("Failed to read from the I2C1 bus.\r\n");
        error_buf = strerror(errno);
        printf(error_buf);
        printf("\n\n");
        return false;
    }
    else
		return true;
}

void mpu9150::getGyroData(void){
	// Write the address to read from
	buffer[0] = MPU9150_TEMP_OUT_H;

	// Write address that should be read
	writeI2C(fd, &buffer[0], 1);

	if (readI2C(fd,&buffer[0],8))
	{
		gyroscope.temperature = (float)((signed short int)(((unsigned short int)(buffer[0] << 8)) | buffer[1]))/340 + 35;
		gyroscope.x_axis = (float)((signed short int)(((unsigned short int)(buffer[2] << 8))| buffer[3]))*MPU9150_LSB2DEGREES - gyroscope.x_mean;
		gyroscope.y_axis = -(float)((signed short int)(((unsigned short int)(buffer[4] << 8))| buffer[5]))*MPU9150_LSB2DEGREES - gyroscope.y_mean;
		gyroscope.z_axis = -(float)((signed short int)(((unsigned short int)(buffer[6] << 8))| buffer[7]))*MPU9150_LSB2DEGREES - gyroscope.z_mean;
	}
}

void mpu9150::getAccelData(void){
	// Write the address to read from
	buffer[0] = MPU9150_ACCEL_XOUT_H;

	// Write address that should be read
	writeI2C(fd, &buffer[0], 1);

	if (readI2C(fd,&buffer[0],6))
	{
		accelerometer.x_axis = (float)((signed short int)(((unsigned short int)(buffer[0] << 8)) | buffer[1]))*MPU9150_LSB2GS;
		accelerometer.y_axis = -(float)((signed short int)(((unsigned short int)(buffer[2] << 8)) | buffer[3]))*MPU9150_LSB2GS;
		accelerometer.z_axis = -(float)((signed short int)(((unsigned short int)(buffer[4] << 8)) | buffer[5]))*MPU9150_LSB2GS;
	}
}

void mpu9150::getIMUdata(void){
	// Write the address to read from
	buffer[0] = MPU9150_ACCEL_XOUT_H;

	// Write address that should be read
	writeI2C(fd, &buffer[0], 1);

	if (readI2C(fd,&buffer[0],14))
	{
		accelerometer.x_axis = (float)((signed short int)(((unsigned short int)(buffer[0] << 8)) | buffer[1]))*MPU9150_LSB2GS;
		accelerometer.y_axis = -(float)((signed short int)(((unsigned short int)(buffer[2] << 8)) | buffer[3]))*MPU9150_LSB2GS;
		accelerometer.z_axis = -(float)((signed short int)(((unsigned short int)(buffer[4] << 8)) | buffer[5]))*MPU9150_LSB2GS;
		gyroscope.temperature = (float)((signed short int)(((unsigned short int)(buffer[6] << 8)) | buffer[7]))/340 + 35;
		gyroscope.x_axis = (float)((signed short int)(((unsigned short int)(buffer[8] << 8))| buffer[9]))*MPU9150_LSB2DEGREES - gyroscope.x_mean;
		gyroscope.y_axis = -(float)((signed short int)(((unsigned short int)(buffer[10] << 8))| buffer[11]))*MPU9150_LSB2DEGREES - gyroscope.y_mean;
		gyroscope.z_axis = -(float)((signed short int)(((unsigned short int)(buffer[12] << 8))| buffer[13]))*MPU9150_LSB2DEGREES - gyroscope.z_mean;
	}
}

float mpu9150::getGyroX(void){
	return gyroscope.x_axis;
}

float mpu9150::getGyroY(void){
	return gyroscope.y_axis;
}

float mpu9150::getGyroZ(void){
	return gyroscope.z_axis;
}

float mpu9150::getGyroTemp(void){
	return gyroscope.temperature;
}

float mpu9150::getAccelX(void){
	return accelerometer.x_axis;
}

float mpu9150::getAccelY(void){
	return accelerometer.y_axis;
}

float mpu9150::getAccelZ(void){
	return accelerometer.z_axis;
}

void mpu9150::openLogFile(void)
{
	logFileMPU.open("mpuLog.txt");

	writeLogFile();
}

void mpu9150::closeLogFile(void)
{
	logFileMPU.close();
}

bool mpu9150::writeLogFile(void)
{
	if (logFileMPU.is_open())
	{
		logFileMPU << write_buffer;
		return true;
	}
	else
		return false;
}

mpu9150::~mpu9150() {
	if (logFileMPU.is_open())
		closeLogFile();

	stopAutoSample();
	// Wait for thread to end
	pthread_join(autoSample_thread, NULL);

	close(fd);
	std::cout << "mpu9150 has been killed." << std::endl;
}

