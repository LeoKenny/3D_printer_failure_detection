#include <SPI.h>
// Ctrl + T for autoformat
#define CS 5  // Chip select
// AND W ADDR
#define WRITEBYTE 0x00
#define READBYTE 0x80
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

void setup() {
  initialise();
  registerWrite(0x31, 0x0F);            // Enter 4-wire SPI mode, left justified, full resolution, 16g+- range
  Serial.print("Reg0x31: ");
  Serial.println(registerRead(0x31));
  Serial.print("DEVICE ID: ");
  Serial.println(registerRead(0x00));
  registerWrite(0x2C, 0x06);            // Hz:6.25 - Power Mode: Normal
  registerWrite(0x2D, 0x08);            // Exit standby mode
}

float acceleration(byte Address) {
  int16_t DATAn = registerRead(Address + 1) << 8;   // Hi
  DATAn |= registerRead(Address);                   // Lo
  DATAn = DATAn >> 3;
  return DATAn;
}

void loop() {
  Serial.println(acceleration(0x36));
  delay(1000);
}


// Shouldnt max accel be 1g?