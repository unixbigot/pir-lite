MCU = attiny45
F_CPU = 8000000

TARGET = pir-lite
SRC = pir-lite.c

include avr-tmpl.mk
AVRDUDE_PORT=/dev/ttyUSB1


