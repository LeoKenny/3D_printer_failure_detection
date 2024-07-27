#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   26

static constexpr size_t HEADER_SIZE = 8;
static constexpr size_t QUEUE_SIZE = 2;
static constexpr size_t FIFO_SIZE = 32;
static constexpr size_t BUFFER_SIZE = (FIFO_SIZE*2*3)+HEADER_SIZE;

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

void convert_to_buffer(uint8_t* tx_buf, fifo_accel tx_data){
  memset(tx_buf, 0, BUFFER_SIZE);
  uint8_t size_count = sizeof(tx_data.count);
  uint8_t size_block = sizeof(tx_data.block);
  uint8_t size_overrun = sizeof(tx_data.overrun);
  uint8_t size_queue_state = sizeof(tx_data.queue_state);
  uint8_t size_header = size_count+size_block+size_overrun+size_queue_state;
  uint8_t size = sizeof(tx_data.accel_x);
  // count, block, overrun, queue_state, accel_x, accel_y, accel_z
  memcpy((void*)tx_buf,(void*)&(tx_data.count), size_count);
  memcpy((void*)(tx_buf+size_count),(void*)&(tx_data.block), size_block);
  memcpy((void*)(tx_buf+size_count+size_block),(void*)&(tx_data.overrun), size_overrun);
  memcpy((void*)(tx_buf+size_count+size_block+size_overrun),(void*)&(tx_data.queue_state), size_queue_state);
  memcpy((void*)(tx_buf+size_header),(void*)tx_data.accel_x, size);
  memcpy((void*)(tx_buf+size_header+size),(void*)tx_data.accel_y, size);
  memcpy((void*)(tx_buf+size_header+(2*size)),(void*)tx_data.accel_z, size);
}

void convert_to_data(uint8_t* rx_buf, fifo_accel* rx_data){
  uint8_t size_count = sizeof(rx_data->count);
  uint8_t size_block = sizeof(rx_data->block);
  uint8_t size_overrun = sizeof(rx_data->overrun);
  uint8_t size_queue_state = sizeof(rx_data->queue_state);
  uint8_t size_header = size_count+size_block+size_overrun+size_queue_state;
  uint8_t size = sizeof(rx_data->accel_x);
  // count, block, overrun, queue_state, accel_x, accel_y, accel_z
  memcpy((void*)&(rx_data->count), (void*)rx_buf, size_count);
  memcpy((void*)&(rx_data->block), (void*)(rx_buf+size_count), size_block);
  memcpy((void*)&(rx_data->overrun), (void*)(rx_buf+size_count+size_block), size_overrun);
  memcpy((void*)&(rx_data->queue_state), (void*)(rx_buf+size_count+size_block+size_overrun), size_queue_state);
  memcpy((void*)(rx_data->accel_x), (void*)(rx_buf+size_header), size);
  memcpy((void*)(rx_data->accel_y), (void*)(rx_buf+size_header+size), size);
  memcpy((void*)(rx_data->accel_z), (void*)(rx_buf+size_header+(2*size)), size);
}

void setup()
{
  Serial.begin(115200);

  tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  delay(1000);
  slave.setDataMode(SPI_MODE0);             // default: SPI_MODE0
  slave.setMaxTransferSize(2*BUFFER_SIZE);  // default: 4092 bytes
  slave.setQueueSize(QUEUE_SIZE);           // default: 1

  slave.begin(HSPI, HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);

  for (uint8_t i=0; i<FIFO_SIZE; i++){
    fifo_data.accel_x[i] = i%10;
    fifo_data.accel_y[i] = i%10;
    fifo_data.accel_z[i] = i%10;
    fifo_data.count = FIFO_SIZE;
  }
  fifo_data.block = 0;
  fifo_data.overrun = 0;
  fifo_data.queue_state = 10;

  Serial.println("start spi slave");
}

void loop()
{
  size_t received_bytes = 0;

  convert_to_buffer(tx_buf, fifo_data);
  received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);
  convert_to_data(tx_buf, &fifo_data_rx);
  Serial.println(fifo_data_rx.count);

  if(fifo_data.queue_state == 0){
    Serial.println(fifo_data.queue_state);
    Serial.println("End of queue");
    while (1);
  }

  fifo_data.block++;
  fifo_data.queue_state--;
}
