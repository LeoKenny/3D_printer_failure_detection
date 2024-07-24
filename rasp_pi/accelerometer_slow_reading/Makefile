CC = gcc
CFLAGS = -Wall -pthread
LIBS = -L /usr/local/include -lpigpio -lrt
INSTALL = `which install`

all: packages reading

install: reading
	$(INSTALL) ./reading /usr/local/bin/reading

clean:
	rm -f reading

packages:
	if ! dpkg-query -W -f='$${Status}' pigpio | grep "ok installed"; then apt-get -y install pigpio; fi

reading: reading.c
	$(CC) $(CFLAGS) reading.c $(LIBS) -o reading
