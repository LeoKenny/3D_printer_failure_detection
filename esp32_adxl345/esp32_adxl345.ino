#include <SPI.h>

#define WRITEBYTE 0x00
#define READBYTE 0x80
#define MULTIBYTE 0x40

#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAY0 0x34
#define DATAZ0 0x36
#define FIFO_CTL 0x38
#define FIFO_STATUS 0x39

#define CS 5
#define GPIO_INT1 2
#define GPIO_INT2 4

#define WATERMARK_SIZE 16

bool watermark_interrupt = false;
bool overflow_interrupt = false;

typedef struct accel_data_base{
  int16_t accel_x[100];
  int16_t accel_y[100];
  int16_t accel_z[100];
  byte count;
} db_accel;

typedef struct accel_fifo{
  int16_t accel_x[32];
  int16_t accel_y[32];
  int16_t accel_z[32];
  byte count;
} fifo_accel;

db_accel database = {{0},{0},{0},0};
fifo_accel fifo = {{0},{0},{0},0};

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
  registerWrite(DATA_FORMAT, 0x0F);     // Enter 4-wire SPI mode, left justified, full resolution, 16g+- range
  registerWrite(BW_RATE, 0x06);         // Hz:6.25 - Power Mode: Normal
  registerWrite(INT_MAP, 0x02);         // Watermark in INT1 and Overrung in INT2
  registerWrite(FIFO_CTL, 0x50);        // FIFO mode, 16 samples
  registerWrite(POWER_CTL, 0x08);       // Exit standby mode
  registerWrite(INT_ENABLE, 0x03);      // Enable watermark and overrun interrupts
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

void acceleration(int16_t* accel_x, int16_t *accel_y, int16_t *accel_z) {
  int8_t data[6];
  multipleRegisterRead(DATAX0, data, 6);
  *accel_x = (data[1] << 5) | (data[0] >> 3);
  *accel_y = (data[3] << 5) | (data[2] >> 3);
  *accel_z = (data[5] << 5) | (data[4] >> 3);
}

void read_fifo(accel_fifo *fifo, byte size){
  fifo->count = 0;
  if (size == 0){
    size = WATERMARK_SIZE;
  }
  for(byte i=0; i<size; i++){
    acceleration(&(fifo->accel_x[i]), &(fifo->accel_y[i]), &(fifo->accel_z[i]));
    fifo->count++;
    delayMicroseconds(5);               // delay for FIFO register update
  }
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

void loop() {
  if (watermark_interrupt){
    uint8_t size = registerRead(FIFO_STATUS);
    read_fifo(&fifo,0);

    Serial.print(size);
    Serial.print(" - ");
    Serial.print(fifo.accel_x[0]);
    Serial.print(" - ");
    Serial.print(fifo.accel_y[0]);
    Serial.print(" - ");
    Serial.println(fifo.accel_z[0]);
    watermark_interrupt = false;
  }
  delay(500);
}
