import serial
import datetime

# Get timestamp
now = datetime.datetime.now()
timestamp = now.strftime('%Y_%m_%d-%H_%M_%S')

print(timestamp)

# Open the serial port
ser = serial.Serial("/dev/ttyUSB0", 115200)
ser.timeout = 3  # set the Read Timeout

# Open the output file
with open(f"{timestamp}.txt", "w") as f:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode("utf-8").strip()

        # Write the line to the file
        f.write(line + "\n")

        # Print the line to the console
        print(line)
