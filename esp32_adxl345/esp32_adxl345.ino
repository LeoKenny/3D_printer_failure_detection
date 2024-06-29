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

void multipleRegisterRead(byte Address, uint8_t *data) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS, LOW);
  SPI.transfer(Address | READBYTE | MULTIBYTE);
  data[0] = SPI.transfer(0);
  data[1] = SPI.transfer(0);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void setup() {
  initialise();
  registerWrite(0x31, 0x0F);            // Enter 4-wire SPI mode, left justified, full resolution, 16g+- range
  registerWrite(0x2C, 0x06);            // Hz:6.25 - Power Mode: Normal
  registerWrite(0x2D, 0x08);            // Exit standby mode
}

float acceleration(byte Address) {
  uint8_t data[2];
  multipleRegisterRead(Address, data);
  int16_t DATAn = data[1] << 8;  // Hi
  DATAn |= data[0];          // Lo
  DATAn = DATAn >> 3;
  Serial.print(data[0],BIN);
  Serial.print(" - ");
  Serial.print(data[1],BIN);
  Serial.print(" - ");
  Serial.print(DATAn);
  Serial.print(" - ");
  float nAccel = (float)DATAn / 256.0;
  nAccel = nAccel*9.807;
  return nAccel;
}

void loop() {
  Serial.println(acceleration(0x36));
  delay(1000);
}
