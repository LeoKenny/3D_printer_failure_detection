#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <pigpio.h>
#define BUFFER_SIZE 8

const double conversion_const = (2.0*16.0)/8192;       // +-16g for 13 bits, pg 27

// SPI comm configuration
const int spi_speed = 2000000;                      // SPI communication speed, bps
const int spi_channel = 0;                          // SPI communication channel
const int spi_buffer_size = 8;                      // Communication buffer size
const double delay_read = 0.006;                    // Delay for reading FIFO update

enum { NS_PER_SECOND = 1000000000,
       MS_PER_NS = 1000000,
       MS_PER_SECOND = 1000
};

typedef struct test_data{
    int16_t value1;
    int32_t value2;
} test_data;

void delay_ms(double ms){
    struct timespec delay;
    delay.tv_sec = (int)(ms/MS_PER_SECOND);
    delay.tv_nsec = (long int)((ms - (int)(ms/MS_PER_SECOND)*MS_PER_SECOND)*MS_PER_NS);
    nanosleep(&delay,NULL);
}

int spi_write_and_read(int spi_handle, uint8_t *tx_message, uint8_t *rx_message, int count){
    return spiXfer(spi_handle, (char *)tx_message, (char *)rx_message, count);
}

int spi_read(int spi_handle, uint8_t *buffer, int count){
    return spiRead(spi_handle, (char *)buffer,count);
}

int spi_write(int spi_handle, uint8_t *tx_message, int count){
    return spiWrite(spi_handle, (char *)tx_message, count);
}

int verify_spi(int spi_handle){
    if(spi_handle < 0){
        switch(spi_handle){
            case(PI_BAD_SPI_CHANNEL):
                printf("Wrong SPI Channel.");
                break;
            case(PI_BAD_SPI_SPEED):
                printf("Wrong SPI Baud Rate.");
                break;
            case(PI_BAD_FLAGS):
                printf("Bad Flags.");
                break;
            case(PI_NO_AUX_SPI):
                printf("No Auxiliary SPI.");
                break;
            case(PI_SPI_OPEN_FAILED):
                printf("Failed to open SPI.");
                break;
        }
    }
    return spi_handle;
}

void convert_to_buffer(uint8_t* tx_buf, test_data data_tx){
    memcpy((void*)tx_buf, (void*)&data_tx.value1, sizeof(data_tx.value1));
    memcpy((void*)&tx_buf[2], (void*)&data_tx.value2, sizeof(data_tx.value2));
}

void convert_to_data(uint8_t* rx_buf, test_data *data_rx){
    memcpy((void*)&(data_rx->value1), (void*)rx_buf, sizeof(data_rx->value1));
    memcpy((void*)&(data_rx->value2), (void*)&rx_buf[2], sizeof(data_rx->value2));
}

int main(int argc, char *argv[]){
    uint8_t rx_buf[BUFFER_SIZE] = {0};
    uint8_t tx_buf[BUFFER_SIZE] = {0};
    int spi_handle;
    test_data data_tx;
    test_data data_rx;

    data_tx.value1 = -1;
    data_tx.value2 = 70000;

   // Starting GPIO
    if (gpioInitialise() == PI_INIT_FAILED){      // pigpio initialisation failed.
        printf("GPIO initialization failed.");
        return 1;
    }

    // Starting SPI
    spi_handle = spiOpen(spi_channel,spi_speed,0);
    if(verify_spi(spi_handle) < 0){ return 1; }

    convert_to_buffer(tx_buf, data_tx);
    delay_ms(2*MS_PER_SECOND);
    spi_write_and_read(spi_handle,tx_buf,rx_buf,BUFFER_SIZE);
    convert_to_data(rx_buf, &data_rx);
    convert_to_data(tx_buf, &data_tx);

    printf("TX: \tvalue 1: %d - value 2: %d\n", data_tx.value1,data_tx.value2);
    for (int i=0; i<BUFFER_SIZE; i++){
        printf("%d ", tx_buf[i]);
    }
    printf("\n");

    printf("RX: \tvalue 1: %d - value 2: %d\n", data_rx.value1,data_rx.value2);
    for (int i=0; i<BUFFER_SIZE; i++){
        printf("%d ", rx_buf[i]);
    }
    printf("\n");

    spiClose(spi_handle);
    return 0;
}
