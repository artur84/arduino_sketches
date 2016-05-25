################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
INO_SRCS += \
../MyBluetooth.ino 

INO_DEPS += \
./MyBluetooth.ino.d 


# Each subdirectory must supply rules for building sources it contributes
MyBluetooth.o: ../MyBluetooth.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/arturo/ActiveSoftware/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR     -I"/home/arturo/ActiveSoftware/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/arturo/ActiveSoftware/arduino-1.6.5-r5/hardware/arduino/avr/variants/standard" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '


