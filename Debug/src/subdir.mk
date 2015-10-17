################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BatteryMonitor.cpp \
../src/GPS.cpp \
../src/PWM_module.cpp \
../src/SpektrumRX.cpp \
../src/mpu9150.cpp 

OBJS += \
./src/BatteryMonitor.o \
./src/GPS.o \
./src/PWM_module.o \
./src/SpektrumRX.o \
./src/mpu9150.o 

CPP_DEPS += \
./src/BatteryMonitor.d \
./src/GPS.d \
./src/PWM_module.d \
./src/SpektrumRX.d \
./src/mpu9150.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -I/home/yashren/Academia/BeagleBone/gcc-linaro-arm-linux-gnueabihf-4.8-2014.03_linux/arm-linux-gnueabihf/libc/usr/include -I../inc/ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


