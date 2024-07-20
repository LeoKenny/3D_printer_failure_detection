#include <ESP32SPISlave.h>
#include "helper.h"

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   26

ESP32SPISlave slave;
static constexpr size_t BUFFER_SIZE = 8;
static constexpr size_t HEADER_SIZE = 8;
static constexpr size_t QUEUE_SIZE = 1;
static constexpr size_t FIFO_SIZE = 1;
uint8_t tx_buf[BUFFER_SIZE] {3, 4, 5, 6, 7, 8, 9, 10};
uint8_t rx_buf[BUFFER_SIZE] {0};
uint8_t tx_header[BUFFER_SIZE] {0};
uint8_t rx_header[BUFFER_SIZE] {0};
size_t offset = 0;

typedef struct fifo_accel{
  int16_t accel_x[FIFO_SIZE];
  int16_t accel_y[FIFO_SIZE];
  int16_t accel_z[FIFO_SIZE];
  uint8_t count;
  uint32_t block;
  bool overrun;
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
  memcpy((void*)&(rx_data->count), (void*)rx_header, sizeof(rx_data->count));
  memcpy((void*)&(rx_data->block), (void*)&(rx_header[1]), sizeof(rx_data->block));
  memcpy((void*)&(rx_data->overrun), (void*)&(rx_header[5]), sizeof(rx_data->overrun));
  memcpy((void*)&(rx_data->queue_state), (void*)&(rx_header[6]), sizeof(rx_data->queue_state));
  rx_data->accel_x[0] = 0;
  rx_data->accel_y[0] = 0;
  rx_data->accel_z[0] = 0;
}

fifo_accel fifo_data;
fifo_accel fifo_data_rx;

struct test_data{
  int16_t value1;
  int32_t value2;
};

void setup()
{
    Serial.begin(115200);

    delay(2000);
    slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
    slave.setQueueSize(QUEUE_SIZE); // default: 1
    slave.begin(HSPI, HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);

    fifo_data.accel_x[0] = 1;
    fifo_data.accel_y[0] = 2;
    fifo_data.accel_z[0] = 3;
    fifo_data.count = 4;
    fifo_data.block = 80000;
    fifo_data.overrun = 1;
    fifo_data.queue_state = 5;

    convert_to_header(tx_header, fifo_data);
    Serial.println("start spi slave");
    for(int i=0; i<HEADER_SIZE;i++){
      Serial.print(tx_header[i]);
      Serial.print(" ");
    }
    Serial.println();
}

void loop()
{
    const size_t received_bytes = slave.transfer(tx_header, rx_header, HEADER_SIZE);
  
    Serial.print("tx_header: \tcount: ");
    Serial.print(fifo_data.count);
    Serial.print(" - block: ");
    Serial.print(fifo_data.block);
    Serial.print(" - overrun: ");
    Serial.print(fifo_data.overrun);
    Serial.print(" - queue_state: ");
    Serial.println(fifo_data.queue_state);
    for(int i=0; i<BUFFER_SIZE;i++){
      Serial.print(tx_header[i]);
      Serial.print(" ");
    }
    Serial.println();
    convert_to_data(rx_header, &fifo_data_rx);
    Serial.print("rx_header: \tcount: ");
    Serial.print(fifo_data_rx.count);
    Serial.print(" - block: ");
    Serial.print(fifo_data_rx.block);
    Serial.print(" - overrun: ");
    Serial.print(fifo_data_rx.overrun);
    Serial.print(" - queue_state: ");
    Serial.println(fifo_data_rx.queue_state);
    
    for(int i=0; i<BUFFER_SIZE;i++){
      Serial.print(rx_header[i]);
      Serial.print(" ");
    }
    Serial.println();

}
