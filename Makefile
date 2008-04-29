AVRDUDE_PORT=/dev/ttyUSB0
MCU = attiny45

F_CPU = 8000000

TARGET = pir-lite

SRC = pir-lite.c

include ../avr-tmpl.mk


