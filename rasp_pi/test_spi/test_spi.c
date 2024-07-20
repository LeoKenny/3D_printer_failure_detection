#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <pigpio.h>
#define BUFFER_SIZE 8
#define FIFO_SIZE 1
#define HEADER_SIZE 8

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

typedef struct fifo_accel{
  int16_t accel_x[FIFO_SIZE];
  int16_t accel_y[FIFO_SIZE];
  int16_t accel_z[FIFO_SIZE];
  uint8_t count;
  uint32_t block;
  uint8_t overrun;
  uint16_t queue_state;
} fifo_accel;

void convert_to_header(uint8_t* tx_header, fifo_accel tx_data){
    memset(tx_header, 0, HEADER_SIZE);
    memcpy((void*)tx_header,(void*)&(tx_data.count), sizeof(tx_data.count));
    memcpy((void*)&(tx_header[1]),(void*)&(tx_data.block), sizeof(tx_data.block));
    memcpy((void*)&(tx_header[5]),(void*)&(tx_data.overrun), sizeof(tx_data.overrun));
    memcpy((void*)&(tx_header[6]),(void*)&(tx_data.queue_state), sizeof(tx_data.queue_state));
}

void convert_to_data(uint8_t* rx_header, fifo_accel* rx_data){
  rx_data->accel_x[0] = 0;
  rx_data->accel_y[0] = 0;
  rx_data->accel_z[0] = 0;
  memcpy((void*)&(rx_data->count), (void*)rx_header, sizeof(rx_data->count));
  memcpy((void*)&(rx_data->block), (void*)&(rx_header[1]), sizeof(rx_data->block));
  memcpy((void*)&(rx_data->overrun), (void*)&(rx_header[5]), sizeof(rx_data->overrun));
  memcpy((void*)&(rx_data->queue_state), (void*)&(rx_header[6]), sizeof(rx_data->queue_state));
}

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

int main(int argc, char *argv[]){
    uint8_t rx_buf[HEADER_SIZE] = {0};
    uint8_t tx_buf[HEADER_SIZE] = {0};
    int spi_handle;
    fifo_accel data_tx;
    fifo_accel data_rx;

    data_tx.accel_x[0] = 6;
    data_tx.accel_y[0] = 5;
    data_tx.accel_z[0] = 4;
    data_tx.count = 6;
    data_tx.block = 7;
    data_tx.overrun = 1;
    data_tx.queue_state = 8;


    data_rx.accel_x[0] = 0;
    data_rx.accel_y[0] = 1;
    data_rx.accel_z[0] = 2;
    data_rx.count = 2;
    data_rx.block = 3;
    data_rx.overrun = 1;
    data_rx.queue_state = 5;

   // Starting GPIO
    if (gpioInitialise() == PI_INIT_FAILED){      // pigpio initialisation failed.
        printf("GPIO initialization failed.");
        return 1;
    }

    // Starting SPI
    spi_handle = spiOpen(spi_channel,spi_speed,0);
    if(verify_spi(spi_handle) < 0){ return 1; }

    convert_to_header(tx_buf, data_tx);
    delay_ms(2*MS_PER_SECOND);
    spi_write_and_read(spi_handle,tx_buf,rx_buf,HEADER_SIZE);
    convert_to_data(tx_buf, &data_tx);

    printf("tx_header: \tcount: ");
    printf("%d",data_tx.count);
    printf(" - block: ");
    printf("%d",data_tx.block);
    printf(" - overrun: ");
    printf("%d",data_tx.overrun);
    printf(" - queue_state: ");
    printf("%d",data_tx.queue_state);
    printf("\n");
    for(int i=0; i<HEADER_SIZE;i++){
      printf("%d ",tx_buf[i]);
    }
    printf("\n");

    convert_to_data(rx_buf, &data_rx);
    printf("rx_header: \tcount: ");
    printf("%d",data_rx.count);
    printf(" - block: ");
    printf("%d",data_rx.block);
    printf(" - overrun: ");
    printf("%d",data_rx.overrun);
    printf(" - queue_state: ");
    printf("%d",data_rx.queue_state);
    printf("\n");
    for(int i=0; i<HEADER_SIZE;i++){
      printf("%d ",rx_buf[i]);
    }
    printf("\n");

    spiClose(spi_handle);
    return 0;
}
