################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/GPS.cpp \
../src/PWM_module.cpp \
../src/SpektrumRX.cpp \
../src/mpu9150.cpp 

OBJS += \
./src/GPS.o \
./src/PWM_module.o \
./src/SpektrumRX.o \
./src/mpu9150.o 

CPP_DEPS += \
./src/GPS.d \
./src/PWM_module.d \
./src/SpektrumRX.d \
./src/mpu9150.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabi-g++ -I/usr/arm-linux-gnueabi/include -I../inc/ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


