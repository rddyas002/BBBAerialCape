################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../BBBAerialCape.cpp 

OBJS += \
./BBBAerialCape.o 

CPP_DEPS += \
./BBBAerialCape.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -I/home/yashren/Academia/BeagleBone/gcc-linaro-arm-linux-gnueabihf-4.8-2014.03_linux/arm-linux-gnueabihf/libc/usr/include -I../inc/ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


