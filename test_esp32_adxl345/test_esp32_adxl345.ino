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

// Initialize SPI pins
#define ADXL345_CS          5
#define ADXL345_SCK         18
#define ADXL345_MOSI        23
#define ADXL345_MISO        19

// ********************** Global variables ********************** // 

volatile bool watermark_interrupt = false;
volatile bool overrun_interrupt = false;

// ********************** ISR ********************** // 
void IRAM_ATTR ISR_watermark(){ watermark_interrupt = true; }
void IRAM_ATTR ISR_overrun(){ overrun_interrupt = true; }

// ********************** ADXL Library ********************** // 

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
  registerWrite(BW_RATE, 0x05);         // Hz:3.13 - Power Mode: Normal
  registerWrite(INT_MAP, 0x01);         // Watermark in INT1 and Overrun in INT2
  registerWrite(FIFO_CTL, 0x00);        // Clearing FIFO
  delay(10);
  registerWrite(FIFO_CTL, 0x50);        // FIFO mode, 16 samples
  registerRead(DATAX0);                 // Clearing overrun int flag
  registerWrite(POWER_CTL, 0x08);       // Exit standby mode
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

// ********************** Arduino task ********************** // 

bool watermark_notify = true;
bool overrun_notify = true;

void setup() {
  Serial.begin(115200);
  SPI.begin(ADXL345_SCK, ADXL345_MISO, ADXL345_MOSI, ADXL345_CS);
  pinMode(ADXL345_CS, OUTPUT);
  digitalWrite(ADXL345_CS, HIGH);
  while (!Serial)
    ;
  Serial.println("Initialized");

  initialize_ADXL();
  
  watermark_notify = true;
  overrun_notify = true;
  watermark_interrupt = false;
  overrun_interrupt = false;
}

void fifo_status(uint8_t int_source, uint8_t source, uint8_t size){
  Serial.print("Interrupt register source: ");
  Serial.println(int_source & source);
  Serial.print("Fifo size: ");
  Serial.println(size);
}

void loop() {
  uint8_t int_source;
  uint8_t size;

  Serial.println("Starting loop...");

  while (1)
  {

    int_source = registerRead(INT_SOURCE);
    size = registerRead(FIFO_STATUS) & 0x3F;

    if (overrun_interrupt){
      Serial.println("Overrun interrupt pin is on.");
      fifo_status(int_source,0x01,size);
      overrun_notify = false;
      overrun_interrupt = false;
    }
    if (overrun_notify & (int_source & 0x01)){
      overrun_notify = false;
      Serial.println("Overrun interrupt should be notified.");
      fifo_status(int_source,0x01,size);
      overrun_notify = false;
      overrun_interrupt = false;
    }

    if (watermark_interrupt){
      Serial.println("Watermark interrupt pin is on.");
      fifo_status(int_source,0x02,size);
      watermark_notify = false;
      watermark_interrupt = false;
    }
    if (watermark_notify & ((int_source >> 1) & 0x01)){
      Serial.println("Watermark interrupt should be notified.");
      fifo_status(int_source,0x02,size);
      watermark_notify = false;
      watermark_interrupt = false;
    }
    delay(100);
  }
}
