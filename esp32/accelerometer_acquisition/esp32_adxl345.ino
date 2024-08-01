#include <ESP32DMASPISlave.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#include <SPI.h>

// SPI with Raspberry pi
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   26

// SPI with ADXL345
#define ADXL345_CS   5
#define ADXL345_SCK  18
#define ADXL345_MOSI 23
#define ADXL345_MISO 19
#define GPIO_WATERMARK 21
#define GPIO_OVERRUN 22

// ADXL345 communication modes
#define WRITEBYTE 0x00
#define READBYTE 0x80
#define MULTIBYTE 0x40

// ADXL345 registers
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAY0 0x34
#define DATAZ0 0x36
#define FIFO_CTL 0x38
#define FIFO_STATUS 0x39
#define COMM_ERROR_BIT 1

// Tasks constants
#define SAVE_STACK_SIZE 4096
#define ACQUISITION_STACK_SIZE 4096
#define WATERMARK_SIZE 16

const TickType_t xFrequencyRead = 2/portTICK_PERIOD_MS;
const TickType_t xFrequencySave = 15/portTICK_PERIOD_MS;
const TickType_t xFrequencyQueue = 100/portTICK_PERIOD_MS;

static constexpr size_t HEADER_SIZE = 8;
static constexpr size_t QUEUE_SIZE = 550;
static constexpr size_t FIFO_SIZE = 32;
static constexpr size_t BUFFER_SIZE = (FIFO_SIZE*2*3)+HEADER_SIZE;

typedef struct fifo_accel{
  int16_t accel_x[FIFO_SIZE];
  int16_t accel_y[FIFO_SIZE];
  int16_t accel_z[FIFO_SIZE];
  uint8_t count;
  uint32_t block;
  uint8_t overrun;
  uint16_t queue_state;
} fifo_accel;

uint8_t *tx_buf;
uint8_t *rx_buf;

volatile bool watermark_interrupt = false;
volatile bool overrun_interrupt = false;

ESP32DMASPI::Slave slave;
QueueHandle_t xQueueSave;

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



void initialise_comm() {
  Serial.begin(115200);
  while (!Serial)
    ;

  SPI.begin(ADXL345_SCK, ADXL345_MISO, ADXL345_MOSI, ADXL345_CS);
  pinMode(ADXL345_CS, OUTPUT);
  digitalWrite(ADXL345_CS, HIGH);

  slave.setDataMode(SPI_MODE0);             // default: SPI_MODE0
  slave.setMaxTransferSize(2*BUFFER_SIZE);  // default: 4092 bytes
  slave.setQueueSize(QUEUE_SIZE);           // default: 1
  slave.begin(HSPI, HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
}

void registerWrite(byte Address, byte data) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(Address | WRITEBYTE);
  SPI.transfer(data);
  digitalWrite(ADXL345_CS, HIGH);
  SPI.endTransaction();
}

uint8_t registerRead(byte Address) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(Address | READBYTE);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(ADXL345_CS, HIGH);
  SPI.endTransaction();
  return data;
}

void multipleRegisterRead(byte Address, int8_t *data, byte size) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(Address | READBYTE | MULTIBYTE);
  for(byte i=0; i<size;i++){
    data[i] = SPI.transfer(0);
  }
  digitalWrite(ADXL345_CS, HIGH);
  SPI.endTransaction();
}

void acceleration(int16_t* accel_x, int16_t *accel_y, int16_t *accel_z) {
  int8_t data[6];
  multipleRegisterRead(DATAX0, data, 6);
  *accel_x = (data[1] << 5) | (data[0] >> 3);
  *accel_y = (data[3] << 5) | (data[2] >> 3);
  *accel_z = (data[5] << 5) | (data[4] >> 3);
}

void IRAM_ATTR ISR_watermark(){ watermark_interrupt = true; }
void IRAM_ATTR ISR_overrun(){ overrun_interrupt = true; }

void initialize_ADXL(){
  pinMode(GPIO_WATERMARK, INPUT_PULLDOWN);                    // Config INT1 PIN as input
  attachInterrupt(digitalPinToInterrupt(GPIO_WATERMARK),      // Pin for the interruption
                  ISR_watermark,                              // Function for ISR 
                  RISING);                                    // Config to rising signal

  pinMode(GPIO_OVERRUN, INPUT_PULLDOWN);                      // Config INT2 PIN as input
  attachInterrupt(digitalPinToInterrupt(GPIO_OVERRUN),        // Pin for the interruption
                  ISR_overrun,                                // Function for ISR 
                  RISING);                                    // Config to rising signal

  registerWrite(POWER_CTL, 0x00);       // Activate standby mode
  registerWrite(DATA_FORMAT, 0x0F);     // Enter 4-wire SPI mode, left justified, full resolution, 16g+- range
  registerWrite(BW_RATE, 0x0F);         // Hz:6.25 - Power Mode: Normal
  registerWrite(INT_MAP, 0x01);         // Watermark in INT1 and Overrun in INT2
  registerWrite(FIFO_CTL, 0x00);        // Clearing FIFO
  delay(10);
  registerWrite(FIFO_CTL, 0x58);        // FIFO mode, 24 samples
  registerRead(DATAX0);                 // Clearing overrun int flag
  registerWrite(POWER_CTL, 0x08);       // Exit standby mode
  registerWrite(INT_ENABLE, 0x03);      // Enable watermark and overrun interrupts
}

void read_fifo(fifo_accel *fifo, byte size){
  if (size == 0)  size = WATERMARK_SIZE;

  for(byte i=0; i<size; i++){
    acceleration(&(fifo->accel_x[i]), &(fifo->accel_y[i]), &(fifo->accel_z[i]));
  }
  fifo->count = size;
}

void setup() {
  tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  initialise_comm();
  initialize_ADXL();
  watermark_interrupt = 0;
  overrun_interrupt = 0;

  delay(500);

  xTaskCreatePinnedToCore(
    vTaskSave,             // Task run the serial communication function
    "Save Task",           // Name of the task
    SAVE_STACK_SIZE,       // Size of the stack to be used in the function
    NULL,                  // No parameters passed to the task
    10,                    // Low Priority task
    NULL,                  // Task handle, not used
    1                      // Affinity with core 1
  );

  xQueueSave = xQueueCreate(    // Creating queue to send data from fifo to Raspberry
  QUEUE_SIZE,                   // Number of simultaneous items that the fifo can allocate
  sizeof(fifo_accel));          // Size of the items that the fifo will allocate

  xTaskCreatePinnedToCore(
    vTaskAcquisition,        // Task run the acquisition function
    "Acq. task",             // Name of the task
    ACQUISITION_STACK_SIZE,  // Size of the stack to be used in the function
    NULL,                    // No parameters passed to the task
    11,                      // High Priority task
    NULL,                    // Task handle, not used
    0                        // Affinity with core 0
  );
}

void vTaskSave(void * pvParams){
  size_t received_bytes = 0;
  uint16_t messages = 0;
  fifo_accel fifo_data;
  fifo_accel fifo_data_old;
  fifo_accel fifo_data_rx;

  // task timing
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("Starting save task");

  while(1){
    while(messages == 0){
      vTaskDelayUntil( &xLastWakeTime, xFrequencySave );
      messages = uxQueueMessagesWaiting(xQueueSave);
    }
    xQueueReceive(xQueueSave, &fifo_data, xFrequencyQueue);
    fifo_data.queue_state = messages - 1;

    convert_to_buffer(tx_buf, fifo_data);
    received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

    convert_to_data(rx_buf, &fifo_data_rx);
    if(fifo_data_old.block != fifo_data_rx.block){
      Serial.println(fifo_data_rx.block);
      fifo_data_old.overrun |= 1 << COMM_ERROR_BIT;
      xQueueSendToFront(xQueueSave, &fifo_data_old, xFrequencyRead);
    }

    memcpy((void*)&(fifo_data_old.count), (void*)&(fifo_data.count), sizeof(fifo_data.count));
    memcpy((void*)&(fifo_data_old.block), (void*)&(fifo_data.block), sizeof(fifo_data.block));
    memcpy((void*)&(fifo_data_old.overrun), (void*)&(fifo_data.overrun), sizeof(fifo_data.overrun));
    memcpy((void*)&(fifo_data_old.queue_state), (void*)&(fifo_data.queue_state), sizeof(fifo_data.queue_state));
    memcpy((void*)(fifo_data_old.accel_x), (void*)(fifo_data.accel_x), FIFO_SIZE*2);
    memcpy((void*)(fifo_data_old.accel_y), (void*)(fifo_data.accel_y), FIFO_SIZE*2);
    memcpy((void*)(fifo_data_old.accel_z), (void*)(fifo_data.accel_z), FIFO_SIZE*2);
    messages = uxQueueMessagesWaiting(xQueueSave);
  }
}

void vTaskAcquisition(void * pvParams){
  // task variables
  fifo_accel fifo;
  fifo.queue_state = 0;
  uint32_t block = 0;
  int16_t size = 0;
  bool sent = true;

  // task timing
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  Serial.println("Starting acquisition task");

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequencyRead );

    if (overrun_interrupt | watermark_interrupt){
      memset(fifo.accel_x, 0, FIFO_SIZE*2);
      memset(fifo.accel_y, 0, FIFO_SIZE*2);
      memset(fifo.accel_z, 0, FIFO_SIZE*2);

      if (overrun_interrupt){
        read_fifo(&fifo,FIFO_SIZE);
        size = FIFO_SIZE;
        fifo.overrun = true;
      }
      else {
        size = registerRead(FIFO_STATUS) & 0x3F;
        read_fifo(&fifo,size);
        fifo.overrun = false;
      }
      fifo.block = block;
      fifo.count = size;
      overrun_interrupt = false;
      watermark_interrupt = false;

      sent = xQueueSend(xQueueSave,(void *) &fifo, xFrequencyRead);
      // Make something when message was not able to be saved over queue??? [flag?]
      block++;
    }
  }
}

void loop(){}
