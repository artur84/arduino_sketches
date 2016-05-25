################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/arturo/arduino_sketches/sketchbook_eclipse/libraries/ros_lib/duration.cpp \
/home/arturo/arduino_sketches/sketchbook_eclipse/libraries/ros_lib/time.cpp 

CPP_DEPS += \
./Libraries/ros_lib/duration.cpp.d \
./Libraries/ros_lib/time.cpp.d 

LINK_OBJ += \
./Libraries/ros_lib/duration.cpp.o \
./Libraries/ros_lib/time.cpp.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/ros_lib/duration.cpp.o: /home/arturo/arduino_sketches/sketchbook_eclipse/libraries/ros_lib/duration.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10601 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR     -I"/home/arturo/ActiveSoftware/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/arturo/ActiveSoftware/arduino-1.6.5-r5/hardware/arduino/avr/variants/standard" -I"/home/arturo/arduino_sketches/sketchbook_eclipse/libraries/ros_lib" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

Libraries/ros_lib/time.cpp.o: /home/arturo/arduino_sketches/sketchbook_eclipse/libraries/ros_lib/time.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10601 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR     -I"/home/arturo/ActiveSoftware/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/arturo/ActiveSoftware/arduino-1.6.5-r5/hardware/arduino/avr/variants/standard" -I"/home/arturo/arduino_sketches/sketchbook_eclipse/libraries/ros_lib" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '


