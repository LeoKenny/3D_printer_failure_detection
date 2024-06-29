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
#define DATAZ1 0x37

void initialise() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  while (!Serial)
    ;
  Serial.println("Initialized");
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

void setup() {
  initialise();
  registerWrite(0x31, 0x0F);            // Enter 4-wire SPI mode, left justified, full resolution, 16g+- range
  registerWrite(0x2C, 0x06);            // Hz:6.25 - Power Mode: Normal
  registerWrite(0x2D, 0x08);            // Exit standby mode
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
  acceleration(&accel_x, &accel_y, &accel_z);
  Serial.print(accel_x/256.0 * 9.807);
  Serial.print(" - ");
  Serial.print(accel_y/256.0 * 9.807);
  Serial.print(" - ");
  Serial.println(accel_z/256.0 * 9.807);
  delay(100);
}
