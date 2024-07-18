#include <ESP32SPISlave.h>
#include "helper.h"

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   26

ESP32SPISlave slave;
static constexpr size_t BUFFER_SIZE = 8;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t tx_buf[BUFFER_SIZE] {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t rx_buf[BUFFER_SIZE] {0, 0, 0, 0, 0, 0, 0, 0};
size_t offset = 0;


void setup()
{
    Serial.begin(115200);

    delay(2000);
    slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
    slave.setQueueSize(QUEUE_SIZE); // default: 1

    // begin() after setting
    slave.begin(HSPI, HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);

    Serial.println("start spi slave");
    for(int i=0; i<BUFFER_SIZE;i++){
      Serial.print(tx_buf[i]);
    }
    Serial.println();
}

void loop()
{
    // start and wait to complete one BIG transaction (same data will be received from slave)
    const size_t received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

    // verify and dump difference with received data
    if (verifyAndDumpDifference("slave", tx_buf, BUFFER_SIZE, "master", rx_buf, received_bytes)) {
        Serial.println("successfully received expected data from master");
    } else {
        Serial.println("unexpected difference found between master/slave data");
    }
    for(int i=0; i<BUFFER_SIZE;i++){
      Serial.print(tx_buf[i]);
    }
    Serial.println();
}
