#include <SPI.h>
#define CS 5
#define WRITEBYTE 0x00
#define READBYTE 0x80
#define MULTIBYTE 0x40
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define GPIO_INT1 2
#define GPIO_INT2 4
#define WATERMARK_SIZE 16

bool watermark_interrupt = false;
bool overflow_interrupt = false;

void initialize_comm() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  while (!Serial)
    ;
  Serial.println("Initialized");
}

void initialize_ADXL(){
  registerWrite(0x31, 0x0F);            // Enter 4-wire SPI mode, left justified, full resolution, 16g+- range
  registerWrite(0x2C, 0x06);            // Hz:6.25 - Power Mode: Normal
  registerWrite(0x2D, 0x08);            // Exit standby mode
}

void registerWrite(byte Address, byte data) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS, LOW);
  SPI.transfer(Address | WRITEBYTE);
  SPI.transfer(data);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

uint8_t registerRead(byte Address) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS, LOW);
  SPI.transfer(Address | READBYTE);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
  return data;
}

void multipleRegisterRead(byte Address, int8_t *data, byte size) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS, LOW);
  SPI.transfer(Address | READBYTE | MULTIBYTE);
  for(byte i=0; i<size;i++){
    data[i] = SPI.transfer(0);
  }
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void IRAM_ATTR ISR_watermark(){
  watermark_interrupt = true;
}

void IRAM_ATTR ISR_overflow(){
  overflow_interrupt = true;
}

void setup() {
  pinMode(GPIO_INT1, INPUT);            // Config INT1 PIN as input
  attachInterrupt(GPIO_INT1,            // Pin for the interruption
                  ISR_watermark,        // Function for ISR 
                  RISING);              // Config to rising signal

  pinMode(GPIO_INT2, INPUT);            // Config INT2 PIN as input
  attachInterrupt(GPIO_INT2,            // Pin for the interruption
                  ISR_overflow,         // Function for ISR 
                  RISING);

  initialize_comm();
  initialize_ADXL();
}

void acceleration(int16_t* accel_x, int16_t *accel_y, int16_t *accel_z) {
  int8_t data[6];
  multipleRegisterRead(DATAX0, data, 6);
  *accel_x = (data[1] << 5) | (data[0] >> 3);
  *accel_y = (data[3] << 5) | (data[2] >> 3);
  *accel_z = (data[5] << 5) | (data[4] >> 3);
}

void loop() {
  int16_t accel_x=0;
  int16_t accel_y=0;
  int16_t accel_z=0;
  if (watermark_interrupt){
    acceleration(&accel_x, &accel_y, &accel_z);
    Serial.println(accel_x);
    watermark_interrupt = false;
  }
  delay(500);
}
