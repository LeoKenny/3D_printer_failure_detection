#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <pigpio.h>

#define FIFO_SIZE 32
#define BUFFER_SIZE (FIFO_SIZE*2*3)
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
    uint8_t size_count = sizeof(tx_data.count);
    uint8_t size_block = sizeof(tx_data.block);
    uint8_t size_overrun = sizeof(tx_data.overrun);
    uint8_t size_queue_state = sizeof(tx_data.queue_state);
    memcpy((void*)tx_header,(void*)&(tx_data.count), size_count);
    memcpy((void*)(tx_header+size_count),(void*)&(tx_data.block), size_block);
    memcpy((void*)(tx_header+size_count+size_block),(void*)&(tx_data.overrun), size_overrun);
    memcpy((void*)(tx_header+size_count+size_block+size_overrun),(void*)&(tx_data.queue_state), size_queue_state);
}

void convert_to_info(uint8_t* rx_header, fifo_accel* rx_data){
    uint8_t size_count = sizeof(rx_data->count);
    uint8_t size_block = sizeof(rx_data->block);
    uint8_t size_overrun = sizeof(rx_data->overrun);
    uint8_t size_queue_state = sizeof(rx_data->queue_state);
    memcpy((void*)&(rx_data->count), (void*)rx_header, size_count);
    memcpy((void*)&(rx_data->block), (void*)(rx_header+size_count), size_block);
    memcpy((void*)&(rx_data->overrun), (void*)(rx_header+size_count+size_block), size_overrun);
    memcpy((void*)&(rx_data->queue_state), (void*)(rx_header+size_count+size_block+size_overrun), size_queue_state);
}

void convert_to_buffer(uint8_t* tx_buf, fifo_accel tx_data){
  memset(tx_buf, 0, BUFFER_SIZE);
  uint8_t size = sizeof(tx_data.accel_x);
  memcpy((void*)tx_buf,(void*)tx_data.accel_x, size);
  memcpy((void*)tx_buf + size,(void*)tx_data.accel_y, size);
  memcpy((void*)tx_buf + 2*size,(void*)tx_data.accel_z, size);
}

void convert_to_data(uint8_t* rx_buf, fifo_accel* rx_data){
  uint8_t size = sizeof(rx_data->accel_x);
  memcpy((void*)(rx_data->accel_x), (void*)rx_buf, size);
  memcpy((void*)(rx_data->accel_y), (void*)rx_buf + size, size);
  memcpy((void*)(rx_data->accel_z), (void*)rx_buf + 2*size, size);
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

int verify_message(int message_status){
    if (message_status<0){
        switch(message_status){
            case(PI_BAD_HANDLE):
                printf("PI BAD HANDLE");
                break;
            case(PI_BAD_SPI_COUNT):
                printf("PI_BAD_SPI_COUNT");
                break;
            case(PI_SPI_XFER_FAILED):
                printf("PI_SPI_XFER_FAILED");
                break;
        }
        return 0;
    }
    return 1;
}

int main(int argc, char *argv[]){
    uint8_t rx_header[HEADER_SIZE] = {0};
    uint8_t tx_header[HEADER_SIZE] = {0};
    uint8_t rx_buf[BUFFER_SIZE] = {0};
    uint8_t tx_buf[BUFFER_SIZE] = {0};
    int spi_handle;
    int message_handle;
    fifo_accel data_tx;
    fifo_accel data_rx;

    for (uint8_t i=0; i<FIFO_SIZE; i++){
        data_tx.accel_x[i] = 9 - (i%10);
        data_tx.accel_y[i] = 9 - (i%10);
        data_tx.accel_z[i] = 9 - (i%10);
    }
    data_tx.count = 6;
    data_tx.block = 7;
    data_tx.overrun = 1;
    data_tx.queue_state = 8;
    
    memset(data_rx.accel_x, 0, FIFO_SIZE*2);
    memset(data_rx.accel_y, 0, FIFO_SIZE*2);
    memset(data_rx.accel_z, 0, FIFO_SIZE*2);
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

    convert_to_header(tx_header, data_tx);
    message_handle = spi_write_and_read(spi_handle,tx_header,rx_header,HEADER_SIZE);
    printf("Message Handle header: %d\n", message_handle);
    convert_to_info(tx_header, &data_tx);

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
      printf("%d ",tx_header[i]);
    }
    printf("\n");

    convert_to_info(rx_header, &data_rx);
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
      printf("%d ",rx_header[i]);
    }
    printf("\n");

    convert_to_buffer(tx_buf,data_tx);
    delay_ms(MS_PER_SECOND*0.001);
    message_handle = spi_write_and_read(spi_handle,tx_buf,rx_buf,BUFFER_SIZE);
    printf("Message Handle buffer: %d\n", message_handle);

    /******************** TX data buff **************/
    convert_to_data(tx_buf, &data_tx);
    printf("tx_buf: \t[ ");
    for(int i=0; i<BUFFER_SIZE;i++){
      printf("%d ",tx_buf[i]);
    }
    printf("]\n");

    printf("accel_x: \t[ ");
    for(int i=0; i<FIFO_SIZE;i++){
      printf("%d ", data_tx.accel_x[i]);
    }
    printf("]\n");

    printf("accel_y: \t[ ");
    for(int i=0; i<FIFO_SIZE;i++){
      printf("%d ", data_tx.accel_y[i]);
    }
    printf("]\n");

    printf("accel_z: \t[ ");
    for(int i=0; i<FIFO_SIZE;i++){
      printf("%d ", data_tx.accel_z[i]);
    }
    printf("]\n");

    /******************** RX data buff **************/
    convert_to_data(rx_buf, &data_rx);
    printf("rx_buf: \t[ ");
    for(int i=0; i<BUFFER_SIZE;i++){
      printf("%d ",rx_buf[i]);
    }
    printf("]\n");

    printf("accel_x: \t[ ");
    for(int i=0; i<FIFO_SIZE;i++){
      printf("%d ", data_rx.accel_x[i]);
    }
    printf("]\n");

    printf("accel_y: \t[ ");
    for(int i=0; i<FIFO_SIZE;i++){
      printf("%d ", data_rx.accel_y[i]);
    }
    printf("]\n");

    printf("accel_z: \t[ ");
    for(int i=0; i<FIFO_SIZE;i++){
      printf("%d ", data_rx.accel_z[i]);
    }
    printf("]\n");


    spiClose(spi_handle);
    return 0;
}
