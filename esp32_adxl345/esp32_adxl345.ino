// RTOS - added 09-07
#include <freertos/FreeRTOS.h>
//#include <freertos/queue.h>
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
#define ACCEL_DB_SIZE 400

#define SELECT_DB_A 1
#define SELECT_DB_B 0

// Initialize SPI pins
#define ADXL345_CS          5
#define ADXL345_SCK         18
#define ADXL345_MOSI        23
#define ADXL345_MISO        19

// ********************** Global variables ********************** // 

bool watermark_interrupt = false;
bool overrun_interrupt = false;

typedef struct accel_data_base{
  int16_t accel_x[ACCEL_DB_SIZE];
  int16_t accel_y[ACCEL_DB_SIZE];
  int16_t accel_z[ACCEL_DB_SIZE];
  bool overrun[ACCEL_DB_SIZE];
  byte count;
} db_accel;

typedef struct accel_fifo{
  int16_t accel_x[FIFO_SIZE];
  int16_t accel_y[FIFO_SIZE];
  int16_t accel_z[FIFO_SIZE];
  byte count;
  bool overrun;
} fifo_accel;

db_accel database_a = {{0},{0},{0},{0},0};
db_accel database_b = {{0},{0},{0},{0},0};
fifo_accel fifo = {{0},{0},{0},0,0};
uint8_t database_selection = false;

// ********************** Prototypes ********************** // 

void vTaskAcquisition(void * pvParams);
void vTaskCore1Example(void * pvParams);

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
  registerWrite(POWER_CTL, 0x00);       // Activate standby mode
  registerWrite(DATA_FORMAT, 0x0F);     // Enter 4-wire SPI mode, left justified, full resolution, 16g+- range
  registerWrite(BW_RATE, 0x06);         // Hz:6.25 - Power Mode: Normal
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

void read_fifo(accel_fifo *fifo, byte size){
  fifo->count = 0;
  if (size == 0){
    size = WATERMARK_SIZE;
  }
  for(byte i=0; i<size; i++){
    acceleration(&(fifo->accel_x[i]), &(fifo->accel_y[i]), &(fifo->accel_z[i]));
    fifo->count++;
    //delayMicroseconds(5);               // delay for FIFO register update
  }
}

void transfer_fifo_to_db(db_accel* db, fifo_accel* fifo){
  for(uint8_t i = 0; i<fifo->count; i++){
    db->accel_x[db->count] = fifo->accel_x[i];
    db->accel_y[db->count] = fifo->accel_y[i];
    db->accel_z[db->count] = fifo->accel_z[i];
    db->overrun[db->count] = fifo->overrun;
    db->count++;
  }
}


// ********************** ISR ********************** // 
void IRAM_ATTR ISR_watermark(){
  watermark_interrupt = true;
}

void IRAM_ATTR ISR_overrun(){
  overrun_interrupt = true;
}

// ********************** RTOS Tasks ********************** // 

void vTaskAcquisition(void * pvParams)
{
  pinMode(GPIO_WATERMARK, INPUT_PULLDOWN);                    // Config INT1 PIN as input
  attachInterrupt(digitalPinToInterrupt(GPIO_WATERMARK),      // Pin for the interruption
                  ISR_watermark,                              // Function for ISR 
                  RISING);                                    // Config to rising signal

  pinMode(GPIO_OVERRUN, INPUT_PULLDOWN);                      // Config INT2 PIN as input
  attachInterrupt(digitalPinToInterrupt(GPIO_OVERRUN),        // Pin for the interruption
                  ISR_overrun,                                // Function for ISR 
                  RISING);                                    // Config to rising signal

  initialize_comm();
  initialize_ADXL();
  Serial.println("Configurations Status");
  Serial.print("INT_MAP:");
  Serial.println(registerRead(INT_MAP),BIN);
  Serial.print("INT_ENABLE:");
  Serial.println(registerRead(INT_ENABLE),BIN);
  Serial.print("INT_SOURCE:");
  Serial.println(registerRead(INT_SOURCE),BIN);
  Serial.print("POWER_CTL:");
  Serial.println(registerRead(POWER_CTL),BIN);
  Serial.print("BW_RATE:");
  Serial.println(registerRead(BW_RATE),BIN);
  Serial.print("DATA_FORMAT:");
  Serial.println(registerRead(DATA_FORMAT),BIN);

  // task timing
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10/portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    // Place your actions below.

    byte interruption = registerRead(INT_SOURCE);

    if (overrun_interrupt ){
      int16_t size = registerRead(FIFO_STATUS) & 0x3F;
      read_fifo(&fifo,size);
      fifo.overrun = true;
      overrun_interrupt = false;
      watermark_interrupt = false;
      Serial.print("Overrun - ");
      Serial.print(size);
      Serial.print(" - ");
      Serial.println(interruption & 0x01);
    }

    if (watermark_interrupt){
      int16_t size = registerRead(FIFO_STATUS) & 0x3F;
      read_fifo(&fifo,size);
      fifo.overrun = false;
      watermark_interrupt = false;
      overrun_interrupt = false;
      Serial.println();
      Serial.print("Watermark - ");
      Serial.print(size);
      Serial.print(" - ");
      Serial.println(interruption & 0x02);
    }

    if (fifo.count){
      fifo.count = 0;
    }

  /*
    if (fifo.count){
    if (database_selection == SELECT_DB_A){
      if ((ACCEL_DB_SIZE - database_a.count)>=fifo.count){
        transfer_fifo_to_db(&database_a,&fifo);
        Serial.println("Loading database a");
      }
      else {
        database_selection = SELECT_DB_B;
        if((ACCEL_DB_SIZE - database_b.count)>=fifo.count){
          transfer_fifo_to_db(&database_b,&fifo);
        }
        else Serial.println("Both databases full");
      }
    }
    else{
      if ((ACCEL_DB_SIZE - database_b.count)>=fifo.count){
        transfer_fifo_to_db(&database_b,&fifo);
        Serial.println("Loading database b");
      }
      else {
        database_selection = SELECT_DB_A;
        if((ACCEL_DB_SIZE - database_a.count)>=fifo.count){
          transfer_fifo_to_db(&database_a,&fifo);
        }
        else Serial.println("Both databases full");
      }
    }
    }
    */
  }
}

void vTaskCore1Example(void * pvParams)
{
  while(1)
  {
    Serial.println("task core1 rodando");
    delay(200);
  }
}

// ********************** Arduino task ********************** // 

void setup() {
    // RTOS - added 09-07
    xTaskCreatePinnedToCore(
    vTaskAcquisition                    /* Funcao a qual esta implementado o que a tarefa deve fazer */
    ,  "Acq. task"                      /* Nome (para fins de debug, se necessário) */
    ,  1024                             /* Tamanho da stack (em words) reservada para essa tarefa */
    ,  NULL                             /* Parametros passados (nesse caso, não há) */
    ,  3                                /* Prioridade */
    ,  NULL                             /* Handle da tarefa, opcional (nesse caso, não há) */
    , 0 );                              /* Afinidade - Core 0*/

    xTaskCreatePinnedToCore(
    vTaskCore1Example                    /* Funcao a qual esta implementado o que a tarefa deve fazer */
    ,  "Acq. task"                      /* Nome (para fins de debug, se necessário) */
    ,  1024                             /* Tamanho da stack (em words) reservada para essa tarefa */
    ,  NULL                             /* Parametros passados (nesse caso, não há) */
    ,  3                                /* Prioridade */
    ,  NULL                             /* Handle da tarefa, opcional (nesse caso, não há) */
    , 1 );                              /* Afinidade - Core 1*/
}

void loop() {}
