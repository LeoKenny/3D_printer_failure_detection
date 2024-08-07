#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   26

static constexpr size_t HEADER_SIZE = 8;
static constexpr size_t QUEUE_SIZE = 2;
static constexpr size_t FIFO_SIZE = 32;
static constexpr size_t BUFFER_SIZE = (FIFO_SIZE*2*3);

typedef struct fifo_accel{
  int16_t accel_x[FIFO_SIZE];
  int16_t accel_y[FIFO_SIZE];
  int16_t accel_z[FIFO_SIZE];
  uint8_t count;
  uint32_t block;
  bool overrun;
  uint16_t queue_state;
} fifo_accel;

uint8_t *tx_header;
uint8_t *rx_header;
uint8_t *tx_buf;
uint8_t *rx_buf;

fifo_accel fifo_data;
fifo_accel fifo_data_rx;

void convert_to_header(uint8_t* tx_header, fifo_accel tx_data){
  memset(tx_header, 0, HEADER_SIZE);
  memcpy((void*)tx_header,(void*)&(tx_data.count), sizeof(tx_data.count));
  memcpy((void*)&(tx_header[1]),(void*)&(tx_data.block), sizeof(tx_data.block));
  memcpy((void*)&(tx_header[5]),(void*)&(tx_data.overrun), sizeof(tx_data.overrun));
  memcpy((void*)&(tx_header[6]),(void*)&(tx_data.queue_state), sizeof(tx_data.queue_state));
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

void setup()
{
  Serial.begin(115200);

  tx_header = slave.allocDMABuffer(HEADER_SIZE);
  rx_header = slave.allocDMABuffer(HEADER_SIZE);
  tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  delay(1000);
  slave.setDataMode(SPI_MODE0);           // default: SPI_MODE0
  slave.setMaxTransferSize(2*BUFFER_SIZE);  // default: 4092 bytes
  slave.setQueueSize(QUEUE_SIZE);         // default: 1

  slave.begin(HSPI, HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);

  for (uint8_t i=0; i<FIFO_SIZE; i++){
    fifo_data.accel_x[i] = i%10;
    fifo_data.accel_y[i] = i%10;
    fifo_data.accel_z[i] = i%10;
  }
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
  convert_to_buffer(tx_buf, fifo_data);
  for(int i=0; i<BUFFER_SIZE;i++){
    Serial.print(tx_buf[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void loop()
{
  size_t received_bytes = 0;
  while(received_bytes<=0){
    Serial.println("Completed transactions");
    Serial.print("received_bytes: ");
    Serial.println(received_bytes);
    received_bytes = slave.transfer(tx_header, rx_header, HEADER_SIZE);
  }

  const size_t received_bytes2 = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);
  Serial.println("Completed transactions");
  Serial.print("received_bytes: ");
  Serial.println(received_bytes2);

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
  convert_to_info(rx_header, &fifo_data_rx);
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

  /********************* TX BUFF *******************/
  convert_to_data(tx_buf,&fifo_data);
  Serial.print("tx_buf: \t[ ");
  for(uint16_t i=0; i<BUFFER_SIZE;i++){
    Serial.print(tx_buf[i]);
    Serial.print(" ");
  }
  Serial.println("]");

  Serial.print("accel_x: \t[ ");
  for(uint8_t i=0; i<FIFO_SIZE;i++){
    Serial.print(fifo_data.accel_x[i]);
    Serial.print(" ");
  }
  Serial.println("]");

  Serial.print("accel_y: \t[ ");
  for(uint8_t i=0; i<FIFO_SIZE;i++){
    Serial.print(fifo_data.accel_y[i]);
    Serial.print(" ");
  }
  Serial.println("]");

  Serial.print("accel_z: \t[ ");
  for(uint8_t i=0; i<FIFO_SIZE;i++){
    Serial.print(fifo_data.accel_z[i]);
    Serial.print(" ");
  }
  Serial.println("]");

  /********************* RX BUFF *******************/
  convert_to_data(rx_buf,&fifo_data_rx);
  Serial.print("rx_buf: \t[ ");
  for(uint16_t i=0; i<BUFFER_SIZE;i++){
    Serial.print(rx_buf[i]);
    Serial.print(" ");
  }
  Serial.println("]");

  Serial.print("accel_x: \t[ ");
  for(uint8_t i=0; i<FIFO_SIZE;i++){
    Serial.print(fifo_data_rx.accel_x[i]);
    Serial.print(" ");
  }
  Serial.println("]");

  Serial.print("accel_y: \t[ ");
  for(uint8_t i=0; i<FIFO_SIZE;i++){
    Serial.print(fifo_data_rx.accel_y[i]);
    Serial.print(" ");
  }
  Serial.println("]");

  Serial.print("accel_z: \t[ ");
  for(uint8_t i=0; i<FIFO_SIZE;i++){
    Serial.print(fifo_data_rx.accel_z[i]);
    Serial.print(" ");
  }
  Serial.println("]");

}
