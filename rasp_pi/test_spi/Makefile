CC = gcc
CFLAGS = -Wall -pthread
LIBS = -L /usr/local/include -lpigpio -lrt
INSTALL = `which install`

all: packages test_spi

install: test_spi
	$(INSTALL) ./test_spi /usr/local/bin/test_spi

clean:
	rm -f test_spi

packages:
	if ! dpkg-query -W -f='$${Status}' pigpio | grep "ok installed"; then apt-get -y install pigpio; fi

test_spi: test_spi.c
	$(CC) $(CFLAGS) test_spi.c $(LIBS) -o test_spi
