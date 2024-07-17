#include "SerialTransfer.h"


SerialTransfer myTransfer;


void sendToPC(float data1, float data2, float data3, float data4)
{
  uint8_t sendLen = 0;
  
  myTransfer.txObj(data1, sizeof(data1), sendLen);
  sendLen += sizeof(data1);
  
  myTransfer.txObj(data2, sizeof(data2), sendLen);
  sendLen += sizeof(data2);
  
  myTransfer.txObj(data3, sizeof(data3), sendLen);
  sendLen += sizeof(data3);
  
  myTransfer.txObj(data4, sizeof(data4), sendLen);
  sendLen += sizeof(data4);
  
  myTransfer.sendData(sendLen);
}


void setup()
{
  Serial.begin(115200);
  myTransfer.begin(Serial);
}

void loop()
{
  float val1 = -999.00;
  float val2 = 1.1234;
  float val3 = 5.0;
  float val4 = 9.99;
  
  sendToPC(val1, val2, val3, val4);
}
