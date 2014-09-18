# REF: http://www.ashleymills.com/node/327
# TODO @tttor: ISSUE: can not link the libsensor-core so t
# NOTE: -c for avr-gcc tells avr-gcc not to link the object file.
# NOTE: -c to force compilation only.
# WORKAROUND: compile the encoder source files together with the main.cpp

# REF:
#http://forums.adafruit.com/viewtopic.php?f=25&p=161974

MCU=-mmcu=atmega2560 # atmega328p
CPU_SPEED=-DF_CPU=16000000UL

INCDIR=../../include/sensor
LIBDIR=../../lib

CXX_COMPILER= avr-gcc
CFLAGS= $(MCU) $(CPU_SPEED) -Os -w
LIBNAME= libsensor.a

#INCLUDE_2= -I ../../include
#INCLUDE_3= -I ../../include/arduino-core # Because some files from arduino-core include headers using #include <new.h> instead of #include <arduino/new>
#INCLUDE_4= -I ../common/macro/include
#LIBS= -L $(LIBDIR) -larduino-core -lm # _must_ be in this order, see: http://stackoverflow.com/questions/11111966/eclipse-arduino-linker-trouble
#LIBS= -Wl,--start-group $(LIBDIR)/libsensor-core.a -Wl,--end-group, # see: http://stackoverflow.com/questions/11946536/undefined-reference-when-linking-static-libraries

OBJECTS= chr_um6_packet.cpp chr_um6.cpp two_phase_incremental_encoder.cpp

default:
	#$(CXX_COMPILER) $(INCLUDE) $(INCLUDE_2) $(INCLUDE_3) $(INCLUDE_4) $(OBJECTS) $(CFLAGS) -o encoder.o $(LIBS)
	#avr-ar rcs $(LIBNAME) sensor.o

install:
	mkdir -p $(INCDIR)
	#mkdir -p $(LIBDIR)
	cp *.hpp $(INCDIR)
	#mv $(LIBNAME) $(LIBDIR)
	#rm *.o

clean:
	$(shell rm $(LIBDIR)/$(LIBNAME) 2> /dev/null)
	$(shell rm $(INCDIR)/*.hpp 2> /dev/null)
