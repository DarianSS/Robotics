CC=gcc
CFLAGS=-g -Wall -lm
all:	calibration
calibration:	calibration.o picomms.o
	$(CC) $(CFLAGS) -o calibration calibration.o picomms.o