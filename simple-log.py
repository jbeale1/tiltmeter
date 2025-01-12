# log data from serial device to file
# Jan.2 2024 JPB

import serial, os, time
from datetime import datetime

#logDir = r'/home/john/Documents/tiltmeter'
logDir = r'C:\Users\beale\Documents\Tiltmeter'

dRatio = 10       # input data rate decimation ratio
flushCount = 250   # how many lines to buffer before forcing data to disk file
fR = 0.05         # low-pass filter constant
# ----------------------------------------------------

def readCT(line):
    inList = line.strip().split(',')  # values separated by commas
    if (len(inList) == 2):
        cap = float(inList[0])
        temp = float(inList[1])
    return (cap, temp)

now = datetime.now()
tsLog = now.strftime("%Y%m%d_%H%M%S_log3.csv")
logfile = os.path.join(logDir, tsLog)

sPort = 'COM9'  # serial port
# sPort='/dev/ttyACM1'

# Configure the serial port
ser = serial.Serial(
    port=sPort, 
    baudrate=115200,
    timeout=2  # Timeout for reading
)

fc = 0
lCount = 0   # count of input lines before readout

line = ""
#for i in range(3):  # throw away first two lines
#    line = ""
#    while not line: 
#        line = ser.readline().decode('utf-8').strip()

#(capF, tempF) = readCT(line)
#inList = line.strip().split(',')  # values separated by commas

first = True  # the very-first-line flag
note = "# tilt log v0.1 JPB start %s" % now.strftime("%Y-%m-%d %H:%M:%S")

with open(logfile, 'w') as f:
    print("Writing logfile %s" % logfile)        

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:            
            if first:
                outS = ("epoch, %s" % (line))
                print(outS)
                f.write(outS + '\n')
                print(note)
                f.write(note + '\n')
                first = False
                continue

            # print(line)
            epochTms = time.time() * 1000 # time in milliseconds
            epochT = (epochTms / 1000.0) # time in seconds                
            outS = ("%.1f, %s" % (epochT,line))
            print(outS)
            f.write(outS + '\n')
            fc += 1
            if (fc > flushCount):
                f.flush()
                fc=0            
