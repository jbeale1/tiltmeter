# log data from Pico sensor on serial port to file
# Dec.22 2024 JPB

import serial, os, time
from datetime import datetime

#logDir = r'/home/john/Documents/tiltmeter'
logDir = r'C:\Users\beale\Documents\Tiltmeter'

dRatio = 10       # input data rate decimation ratio
flushCount = 50   # how many lines to buffer before forcing data to disk file
fR = 0.05         # low-pass filter constant
# ----------------------------------------------------

def readCT(line):
    inList = line.strip().split(',')  # values separated by commas
    if (len(inList) == 2):
        cap = float(inList[0])
        temp = float(inList[1])
    return (cap, temp)

now = datetime.now()
tsLog = now.strftime("%Y%m%d_%H%M%S_tilt3.csv")
logfile = os.path.join(logDir, tsLog)

sPort = 'COM9'  # serial port
# sPort='/dev/ttyACM1'

# Configure the serial port
ser = serial.Serial(
    port=sPort, 
    baudrate=115200,
    timeout=1  # Timeout for reading
)

fc = 0
lCount = 0   # count of input lines before readout

line = ""
for i in range(3):  # throw away first two lines
    line = ""
    while not line: 
        line = ser.readline().decode('utf-8').strip()

(capF, tempF) = readCT(line)

with open(logfile, 'w') as f:
    print("Writing logfile %s" % logfile)
    hdr = "epoch, pF, degC"
    print(hdr)
    f.write(hdr + '\n')
    note = "# tilt log v0.3 JPB start %s" % now.strftime("%Y-%m-%d %H:%M:%S")
    print(note)
    f.write(note + '\n')

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:            
            # print(line)
            (capR, tempR) = readCT(line)
            capF = (1.0-fR)*capF + fR * capR # low-pass filter
            tempF = (1.0-fR)*tempF + fR * tempR
            lCount += 1
            if (lCount >= dRatio):
                lCount = 0

                epochTms = time.time() * 1000 # time in milliseconds
                epochT = (epochTms / 1000.0) # time in seconds                
                outS = ("%.1f, %.6f, %.3f" % (epochT,capF,tempF))
                print(outS)
                f.write(outS + '\n')
                fc += 1
                if (fc > flushCount):
                    f.flush()
                    fc=0            
