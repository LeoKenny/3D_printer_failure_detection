import serial

SerialPortObj = serial.Serial("/dev/ttyUSB0")
print("\nStatus -> ", SerialPortObj)

SerialPortObj.close()
