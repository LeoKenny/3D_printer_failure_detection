CC = gcc
CFLAGS = -Wall -pthread
LIBS = -L /usr/local/include -lpigpio -lrt
INSTALL = `which install`

all: packages calibration

install: calibration
	$(INSTALL) ./calibration /usr/local/bin/calibration

clean:
	rm -f calibration

packages:
	if ! dpkg-query -W -f='$${Status}' pigpio | grep "ok installed"; then apt-get -y install pigpio; fi

calibration: calibration.c
	$(CC) $(CFLAGS) calibration.c $(LIBS) -o calibration
