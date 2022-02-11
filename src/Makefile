# Makefile for Epson Accelerometer User Space Example
CC=g++

DEFINES= -D$(MODEL) -D$(IF) -D$(PLATFORM)
INCLUDES= .
CFLAGS= -I$(INCLUDES) $(DEFINES) -Wall
LIBS=
DEPS= hcl.h hcl_gpio.h accel_epsonCommon.h main_helper.h
_OBJ= accel_epsonCommon.o main_helper.o

# If no MODEL macro is defined when calling make it defaults to A352
MODEL ?= A352

# defaults Interface to UART
IF ?= UART

# defaults to NO PLATFORM (UART)
PLATFORM ?= NONE

####### Adding MODEL Specific Files 
ifeq ($(MODEL), A352)
	_OBJ+= accel_epsonA352.o
	DEPS+= accel_epsonA352.h
endif

####### Adding Interface Specific Files
# SPI interface is only supported for RaspberryPi
# UART interface is supported on any Linux system with tty
ifeq ($(IF), SPI)
	_OBJ+= accel_epsonSpi.o
	DEPS+= hcl_spi.h
	PLATFORM= RPI
else
	_OBJ+= hcl_gpio.o hcl_linux.o hcl_uart.o accel_epsonUart.o
	DEPS+= hcl_uart.h
endif

####### Adding PLATFORM Specific Files 
ifeq ($(PLATFORM), RPI)
	_OBJ+= hcl_rpi.o hcl_spi_rpi.o hcl_gpio_rpi.o
	LIBS+= -lwiringPi -lpthread -lcrypt -lrt
endif

OBJ_SCREEN= main_screen.o $(_OBJ)
OBJ_CSV= main_csvlogger.o $(_OBJ)
OBJ_REG= main_regdump.o $(_OBJ)

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

screen: $(OBJ_SCREEN)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

csvlogger: $(OBJ_CSV)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

regdump: $(OBJ_REG)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean all tar help

clean:
	rm -f *.o *~ core *~
	rm -f csvlogger screen regdump

all: screen csvlogger regdump

tar:
	tar cvzf archive.tar.gz *.c *.h README_src.md Makefile

help:
	@echo "supported make commands are:"
	@echo "\tmake clean"
	@echo "\tmake <targets> MODEL=<model>\n"
	@echo "valid <targets> are: csvlogger screen or regdump\n"
	@echo "valid <models> are:"
	@echo "\tA352(default) future models TBD\n"
	@echo "example:\n\tmake csvlogger MODEL=A352"
	@echo "\tmake screen (defaults to MODEL=A352)"
