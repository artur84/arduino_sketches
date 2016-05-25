################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
INO_SRCS += \
../MyBluetooth.ino 

CPP_SRCS += \
../.ino.cpp 

LINK_OBJ += \
./.ino.cpp.o 

INO_DEPS += \
./MyBluetooth.ino.d 

CPP_DEPS += \
./.ino.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
.ino.cpp.o: ../.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/tools/arduino/avr-gcc/4.8.1-arduino5/bin/avr-g++" -c -g -Os -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10606 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR     -I"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/packages/arduino/hardware/avr/1.6.11/cores/arduino" -I"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/packages/arduino/hardware/avr/1.6.11/variants/standard" -I"/home/arturo/arduino_sketches/libraries/ros_lib" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

MyBluetooth.o: ../MyBluetooth.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/tools/arduino/avr-gcc/4.8.1-arduino5/bin/avr-g++" -c -g -Os -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10606 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR     -I"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/packages/arduino/hardware/avr/1.6.11/cores/arduino" -I"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/packages/arduino/hardware/avr/1.6.11/variants/standard" -I"/home/arturo/arduino_sketches/libraries/ros_lib" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '


