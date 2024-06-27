CC = gcc
CFLAGS = -Wall -pthread
LIBS = -L /usr/local/include -lpigpio -lrt
INSTALL = `which install`

all: packages acquisition

install: acquisition
	$(INSTALL) ./acquisition /usr/local/bin/acquisition

clean:
	rm -f acquisition

packages:
	if ! dpkg-query -W -f='$${Status}' pigpio | grep "ok installed"; then apt-get -y install pigpio; fi

acquisition: acquisition.c
	$(CC) $(CFLAGS) acquisition.c $(LIBS) -o acquisition
