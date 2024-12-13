# log data from serial port to file
# Dec.12 2024

import serial, os
from datetime import datetime

#logDir = r'/home/john/Documents/tiltmeter'
logDir = r'C:\Users\beale\Documents\Tiltmeter'

flushCount = 20   # how many lines to buffer before forcing data to disk file

now = datetime.now()
tsLog = now.strftime("%Y%m%d_%H%M%S_tilt3.csv")
logfile = os.path.join(logDir, tsLog)

sPort = 'COM7'  # serial port
# sPort='/dev/ttyACM1'

# Configure the serial port
ser = serial.Serial(
    port=sPort, 
    baudrate=115200,
    timeout=1  # Timeout for reading
)

fc = 0
with open(logfile, 'w') as f:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:            
            print(line)  # Optional: Print to console
            f.write(line + '\n')
            fc += 1
            if (fc > flushCount):
                f.flush()
                fc=0
