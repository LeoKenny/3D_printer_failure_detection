#include <ESP32SPISlave.h>
#include "helper.h"

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   26

ESP32SPISlave slave;
static constexpr size_t BUFFER_SIZE = 8;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t tx_buf[BUFFER_SIZE] {3, 4, 5, 6, 7, 8, 9, 10};
uint8_t rx_buf[BUFFER_SIZE] {0, 0, 0, 0, 0, 0, 0, 0};
size_t offset = 0;

struct test_data{
  int16_t value1;
  int32_t value2;
};
test_data data_tx;
test_data data_rx;

void convert_to_buffer(uint8_t* tx_buf, test_data data_tx){
    memcpy((void*)tx_buf,(void*)&data_tx.value1, sizeof(data_tx.value1));
    memcpy((void*)&tx_buf[2],(void*)&data_tx.value2, sizeof(data_tx.value2));
}
void convert_to_data(uint8_t* rx_buf, test_data *data_rx){
    memcpy((void*)&(data_rx->value1), (void*)rx_buf, sizeof(data_rx->value1));
    memcpy((void*)&(data_rx->value2), (void*)&rx_buf[2], sizeof(data_rx->value2));
}

void setup()
{
    Serial.begin(115200);

    delay(2000);
    slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
    slave.setQueueSize(QUEUE_SIZE); // default: 1
    slave.begin(HSPI, HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);

    data_tx.value1 = -2;
    data_tx.value2 = 80000;
    convert_to_buffer(tx_buf, data_tx);
    Serial.println("start spi slave");
    for(int i=0; i<BUFFER_SIZE;i++){
      Serial.print(tx_buf[i]);
    }
    Serial.println();
}

void loop()
{
    const size_t received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);
  
    Serial.print("tx_buff: \tValue 1: ");
    Serial.print(data_tx.value1);
    Serial.print(" - Value 2: ");
    Serial.println(data_tx.value2);
    for(int i=0; i<BUFFER_SIZE;i++){
      Serial.print(tx_buf[i]);
    }
    convert_to_data(rx_buf, &data_rx);
    Serial.println();
    Serial.print("rx_buff: \tValue 1: ");
    Serial.print(data_rx.value1);
    Serial.print(" - Value 2: ");
    Serial.println(data_rx.value2);
    
    for(int i=0; i<BUFFER_SIZE;i++){
      Serial.print(rx_buf[i]);
    }
    Serial.println();

}
