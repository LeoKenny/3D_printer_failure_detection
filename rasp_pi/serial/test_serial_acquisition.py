import serial
import time

SerialObj = serial.Serial("/dev/ttyUSB0", 115200)

time.sleep(3)

SerialObj.timeout = 3  # set the Read Timeout

while 1:
    ReceivedString = SerialObj.readline()  # readline reads a string terminated by \n
    print(ReceivedString)

SerialObj.close()  # Close the port
