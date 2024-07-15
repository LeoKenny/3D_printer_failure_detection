#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#include <SPI.h>

#define WRITEBYTE 0x00
#define READBYTE 0x80
#define MULTIBYTE 0x40

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

#define GPIO_WATERMARK 21
#define GPIO_OVERRUN 22

#define WATERMARK_SIZE 16
#define FIFO_SIZE 32
#define ACQUISITION_STACK_SIZE 4096
#define SERIAL_STACK_SIZE 4096

// Initialize SPI pins
#define ADXL345_CS          5
#define ADXL345_SCK         18
#define ADXL345_MOSI        23
#define ADXL345_MISO        19

// ********************** Global variables ********************** // 

volatile bool watermark_interrupt = false;
volatile bool overrun_interrupt = false;

typedef struct fifo_accel{
  int16_t accel_x[FIFO_SIZE];
  int16_t accel_y[FIFO_SIZE];
  int16_t accel_z[FIFO_SIZE];
  uint32_t block;
  byte count;
  bool overrun;
  uint16_t queue_state;
} fifo_accel;

// ********************** Prototypes ********************** // 

void vTaskAcquisition(void * pvParams);
void vTaskSerial(void * pvParams);
QueueHandle_t xQueueSerial;

// ********************** ISR ********************** // 
void IRAM_ATTR ISR_watermark(){ watermark_interrupt = true; }
void IRAM_ATTR ISR_overrun(){ overrun_interrupt = true; }

// ********************** ADXL Library ********************** // 

void initialize_comm() {
  Serial.begin(115200);
  SPI.begin();
  SPI.begin(ADXL345_SCK, ADXL345_MISO, ADXL345_MOSI, ADXL345_CS);
  pinMode(ADXL345_CS, OUTPUT);
  digitalWrite(ADXL345_CS, HIGH);
  while (!Serial)
    ;
  Serial.println("Initialized");
}

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
  registerWrite(BW_RATE, 0x0C);         // Hz:6.25 - Power Mode: Normal
  registerWrite(INT_MAP, 0x01);         // Watermark in INT1 and Overrun in INT2
  registerWrite(FIFO_CTL, 0x50);        // FIFO mode, 16 samples
  registerWrite(POWER_CTL, 0x08);       // Exit standby mode
  delay(50);
  registerWrite(INT_ENABLE, 0x03);      // Enable watermark and overrun interrupts
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

void read_fifo(fifo_accel *fifo, byte size){
  fifo->count = 0;
  if (size == 0){
    size = WATERMARK_SIZE;
  }
  for(byte i=0; i<size; i++){
    acceleration(&(fifo->accel_x[i]), &(fifo->accel_y[i]), &(fifo->accel_z[i]));
    fifo->count++;
  }
}

// ********************** RTOS Tasks ********************** // 

void vTaskAcquisition(void * pvParams)
{
  // task variables
  fifo_accel fifo;
  fifo.queue_state = 0;
  uint32_t block = 0;
  int16_t size = 0;
  bool sent = false;

  // task timing
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5/portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    if (overrun_interrupt){
      read_fifo(&fifo,FIFO_SIZE);
      fifo.overrun = true;
      fifo.block = block;
      fifo.queue_state = uxQueueMessagesWaiting(xQueueSerial);
      while (!xQueueSend(xQueueSerial,(void *) &fifo, xFrequency))
        ;
      overrun_interrupt = false;
      watermark_interrupt = false;
      block++;
    }

    if (watermark_interrupt){
      size = registerRead(FIFO_STATUS) & 0x3F;
      read_fifo(&fifo,size);
      fifo.block = block;
      fifo.overrun = false;
      fifo.queue_state = uxQueueMessagesWaiting(xQueueSerial);
      while (!xQueueSend(xQueueSerial,(void *) &fifo, xFrequency))
        ;
      watermark_interrupt = false;
      overrun_interrupt = false;
      block++;
    }
  }
}

void vTaskSerial(void * pvParams)
{
  // task variables
  fifo_accel data;
  uint8_t messages;

  // task timing
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  const TickType_t xFrequencySerial = 5/portTICK_PERIOD_MS;

  while(1)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    messages = uxQueueMessagesWaiting(xQueueSerial);
    while (messages){
      if(xQueueReceive(xQueueSerial ,&data ,xFrequencySerial)){
        for (uint8_t i=0; i < data.count; i++){
          Serial.println(
            String(data.accel_x[i]) +
            "," +
            String(data.accel_y[i]) +
            "," +
            String(data.accel_z[i]) +
            "," +
            String(data.block) +
            "," +
            String(data.overrun) +
            "," +
            String(data.count) +
            "," +
            String(data.queue_state) +
            "," +
            String(i)
          );
        }
      }
      messages = uxQueueMessagesWaiting(xQueueSerial);
    }
  }
}

// ********************** Arduino task ********************** // 

void setup() {
    initialize_comm();
    initialize_ADXL();

    xQueueSerial = xQueueCreate(        // Creating queue to send data from fifo to serial
    400,                                // Number of simultaneous items that the fifo can allocate
    sizeof(fifo_accel));                // Size of the items that the fifo will allocate

    xTaskCreatePinnedToCore(
    vTaskAcquisition                    // Task run the acquisition function
    ,  "Acq. task"                      // Name of the task
    ,  ACQUISITION_STACK_SIZE           // Size of the stack to be used in the function
    ,  NULL                             // No parameters passed to the task
    ,  3                                // High Priority task
    ,  NULL                             // Task handle, not used
    , 0 );                              // Affinity with core 0

    xTaskCreatePinnedToCore(
    vTaskSerial                         // Task run the serial communication function
    ,  "Serial task"                    // Name of the task
    ,  SERIAL_STACK_SIZE                // Size of the stack to be used in the function
    ,  NULL                             // No parameters passed to the task
    ,  3                                // Low Priority task
    ,  NULL                             // Task handle, not used
    , 1 );                              // Affinity with core 1
}

void loop() {}
