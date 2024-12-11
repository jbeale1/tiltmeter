# log data from serial port to file
# Dec.12 2024

import serial, os
from datetime import datetime

logDir = r'/home/john/Documents/tiltmeter'
now = datetime.now()
tsLog = now.strftime("%Y%m%d_%H%M%S_tilt3.csv")
logfile = os.path.join(logDir, tsLog)


# Configure the serial port
ser = serial.Serial(
    port='/dev/ttyACM1', 
    baudrate=115200,
    timeout=1  # Timeout for reading
)

with open(logfile, 'w') as f:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(line)  # Optional: Print to console
            f.write(line + '\n')
