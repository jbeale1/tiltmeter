# Micropython / Pi Pico Read LM35D temp sensor on MCP3424 18-bit ADC via I2C
# uses firmware RPI_PICO_W-20240602-v1.23.0.uf2
# J.Beale 2-Jan-2025

from machine import Pin, I2C
from time import sleep_ms
import math          # for natural log
import array


ADC_SDA = 14 # Pico GP14: I2C SDA
ADC_SCL = 15 # Pico GP15: I2C SCL


# ----------------------------------------------
# Write 'data' byte(s) to I2C device at 'addr'
def wdat(addr,data):  
  i2c.writeto(addr, bytearray(list(data)) )
  sleep_ms(4)  # chip locks up with < 2 ms delay here
  
# Read n bytes from I2C device at 'addr'
def rdat(addr,n):
  sleep_ms(1)
  return( list(i2c.readfrom(addr, n)) )

# read ADC result (requires prior ADC start, or continuous mode)
def readT():
  buf = rdat(Adr,3)  # get 3-byte word (read more bytes for config data)
  val = (buf[0]&0x03)<<16 | buf[1]<<8 | buf[2]  # 18 bit result  
  if (val & (1 << (bits - 1))) != 0:   # if sign bit is set        
        val = val - (1 << bits)        # ...find negative value

  Vadc = val * sf  
  T = Vadc * 100.0    # calculate temperature in deg. C
  return T

# ----------------------------------------------

i2c = I2C(1, scl=Pin(ADC_SCL), sda=Pin(ADC_SDA), freq=100000) 
res = i2c.scan()
# print("I2C scan result: ",hex(res[0]))

# I2C address: 104d = 0x68
Adr = res[0]   # first device found on I2C bus. 0x68 for stock MCP3424

lnum = 0
bits = 18            # resolution of ADC mode in use
Nmax = 1 << (bits-1) # maximum value in this mode  (2^17 = 131072)
Vref = 2.048         # MCP3424 ADC internal Vref
PGA = 4              # ADC gain setting (1,2,4,8)
sf = Vref/(Nmax * PGA)  # ADC scale factor in Volts/Count
wTime = 280          # milliseconds to wait between ADC readings
bufSize = 10         # how many value for delta-T boxcar average

# Configure MCP3424 for continuous conversion in 18-bit mode
#wdat(Adr,[0x1c])  # 0x1c: Ch1, gain=1, 3.75 sps (18 bits)
#wdat(Adr,[0x1d])  # 0x1d: gain=2
wdat(Adr,[0x1e])  # 0x1c: Ch1, gain=4, 3.75 sps (18 bits)

avg = 4  # how many readings to average before printing
oldT = readT()  # our previous temperature reading
tBuf = [oldT] * bufSize
tPtr = 0  # pointer into temperature buffer tBuf[]
sleep_ms(wTime)

while True:
  Tsum = 0
  for i in range(avg):
    tVal = readT()     # get latest temperature reading
    # print("   ",tVal)
    Tsum += tVal
    sleep_ms(wTime)
  T = Tsum/avg
  dT = 1000.0*(T-oldT)  # instantaneous change in temperature
  tPtr = (tPtr + 1) % bufSize
  tBuf[tPtr] = dT
  tBC = 0
  for i in range(bufSize):
      tBC += tBuf[i]
  tBC = tBC / bufSize      
  print("%.4f, %.2f, %.3f" % (T,dT, tBC))     # temperature in degrees C
  oldT = T
