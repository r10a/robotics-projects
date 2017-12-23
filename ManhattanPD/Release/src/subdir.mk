################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/ManhattanVariable.cpp 

OBJS += \
./src/ManhattanVariable.o 

CPP_DEPS += \
./src/ManhattanVariable.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-linux-gnueabi-g++ -mcpu=arm9 -marm -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -I/usr/local/EV3-API/API -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


