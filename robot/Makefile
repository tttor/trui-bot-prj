# REF: http://www.ashleymills.com/node/327
CC=avr-gcc
CXX=avr-g++

MASTER_BOARD=arduino
MASTER_MCU=-mmcu=atmega2560 #atmega328p # atmega328p, 
MASTER_CPU_SPEED=-DF_CPU=16000000UL
MASTER_CFLAGS=$(MASTER_MCU) $(MASTER_CPU_SPEED) -Os -w
MASTER_LIBNAME=lib$(MASTER_BOARD)-master.a

SLAVE_BOARD=arduino
SLAVE_MCU=-mmcu=atmega328p # atmega328p, #atmega2560 
SLAVE_CPU_SPEED=-DF_CPU=16000000UL
SLAVE_CFLAGS=$(SLAVE_MCU) $(SLAVE_CPU_SPEED) -Os -w
SLAVE_LIBNAME=lib$(SLAVE_BOARD)-slave.a

TARGET_INCDIR=./include 
TARGET_LIBDIR=./lib
TARGET_BUILDDIR=./build

ARDUINO_SRC= ./src/arduino/
ARDUINO_INC=-I $(ARDUINO_SRC)
ARDUINO_OBJ=  $(ARDUINO_SRC)pins_arduino.o $(ARDUINO_SRC)wiring.o $(ARDUINO_SRC)wiring_analog.o $(ARDUINO_SRC)wiring_digital.o \
				 $(ARDUINO_SRC)wiring_pulse.o $(ARDUINO_SRC)wiring_shift.o $(ARDUINO_SRC)HardwareSerial.o  $(ARDUINO_SRC)Print.o   \
		 		 $(ARDUINO_SRC)Tone.o $(ARDUINO_SRC)WMath.o $(ARDUINO_SRC)WString.o $(ARDUINO_SRC)WInterrupts.o $(ARDUINO_SRC)new.o 

default: master_main

master_main:

slave_main:

arduino_master: $(ARDUINO_OBJ)
	mkdir -p $(TARGET_BUILDDIR)
	avr-ar rcs $(TARGET_BUILDDIR)/$(MASTER_LIBNAME) $^
	mkdir -p $(TARGET_INCDIR)
	mkdir -p $(TARGET_LIBDIR)
	cp $(ARDUINO_SRC)*.h $(TARGET_INCDIR)
	cp $(TARGET_BUILDDIR)/$(MASTER_LIBNAME) $(TARGET_LIBDIR)
	rm $(ARDUINO_SRC)*.o

arduino_slave: $(ARDUINO_OBJ)
	mkdir -p $(TARGET_BUILDDIR)
	avr-ar rcs $(TARGET_BUILDDIR)/$(SLAVE_LIBNAME) $^
	mkdir -p $(TARGET_INCDIR)
	mkdir -p $(TARGET_LIBDIR)
	cp $(ARDUINO_SRC)*.h $(TARGET_INCDIR)
	cp $(TARGET_BUILDDIR)/$(SLAVE_LIBNAME) $(TARGET_LIBDIR)
	rm $(ARDUINO_SRC)*.o
	
clean:
	$(shell rm $(TARGET_LIBDIR)/$(MASTER_LIBNAME) 2> /dev/null)
	$(shell rm $(TARGET_INCDIR)/*.h 2> /dev/null)
	rm -rf $(TARGET_BUILDDIR)

%.o : %.c
	$(CC) $< $(MASTER_CFLAGS) $(ARDUINO_INC) -c -o $@

%.o : %.cpp
	$(CXX) $< $(MASTER_CFLAGS) $(ARDUINO_INC) -c -o $@
