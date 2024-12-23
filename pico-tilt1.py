# Micropython / Pi Pico Read AD7747 Cap-sensor chip via I2C
# uses firmware RPI_PICO_W-20240602-v1.23.0.uf2
# J.Beale 22-Dec-2024

import machine
import time
import struct

VERSION = "Pico AD7747 v0.3 22-Dec-2024 JPB"
# -------------------------------------

SCL1 = 21 # I2C Clock: Pico GPIO pin GP21
SDA1 = 20 # I2C Data:  Pico GPIO pin GP20
RDY1 = 16 # chip /RDY on GPIO pin GP16

address1 = 0x48  # I2C bus address of AD7747 cap sensor

# convert 24-bit AD7747 result to units of pF
def convertCapData(buf):
    b4 = [0, buf[0], buf[1], buf[2]]
    capRaw = struct.unpack(">i", bytes( b4 ) )[0] - 0x800000
    cFloat = capRaw / 1024000.0
    return cFloat

# convert 24-bit AD7747 2nd channel to temperature in deg.C
def convertTempData(buf):
    b4 = [0, buf[0], buf[1], buf[2]]
    tempRaw = struct.unpack(">i", bytes( b4 ) )[0]    
    degC = (tempRaw / 2048.0) - 4096.0
    return degC

# -------------------------------------
# AD774X Register address Definition
ADR_STATUS     =  0  # Read Only
ADR_CAP_DATAH  =  1  # Read Only
ADR_CAP_DATAM  =  2  # Read Only
ADR_CAP_DATAL  =  3  # Read Only
ADR_VT_DATAH   =  4  # Read Only
ADR_VT_DATAM   =  5  # Read Only
ADR_VT_DATAL   =  6  # Read Only
ADR_CAP_SETUP  =  7  # CAP SETUP REGISTER
ADR_VT_SETUP   =  8  # VT SETUP REGISTER
ADR_EXC_SETUP  =  9  # EXC SETUP REGISTER
ADR_CFG        = 10  # CONFIGURATION REGISTER
ADR_CAPDACA    = 11  # CAP DAC A REGISTER
ADR_CAPDACB    = 12  # CAP DAC B REGISTER
ADR_CAP_OFFH   = 13  # CAP OFFSET CALIBRATION REGISTER HIGH
ADR_CAP_OFFL   = 14  # CAP OFFSET CALIBRATION REGISTER LOW
ADR_CAP_GAINH  = 15  # factory calibration
ADR_CAP_GAINL  = 16  # factory calibration
ADR_VOLT_GAINH = 17  # factory calibration
ADR_VOLT_GAINL = 18  # factory calibration

# ---------------------------------------------------
# AD7747 Register number and default value, in hex:
# Adr: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12
# Val: 07 00 00 00 00 00 00 00 00 03 a0 00 00 80 00 63 2c 5b 81
# ---------------------------------------------------

led = machine.Pin(25, machine.Pin.OUT)  # Onboard LED is connected to GPIO 25
dTime = 0.5
for i in range(3):
    led.value(1)    # turn onboard LED on 
    time.sleep(dTime)
    led.value(0)  
    time.sleep(dTime*3) # turn onboard LED off


AD7747_ready = machine.Pin(RDY1, machine.Pin.IN)  # /RDY signal from AD7747

i2c = machine.I2C(0, scl=machine.Pin(SCL1), sda=machine.Pin(SDA1), freq=100000)
# register values to write starting at register 7 (CAP_SETUP)
setupData = bytearray([0xa0, 0x81, 0x0e, 0xa1, 0x00,0x00,0x80,0x00 ])  # value to put in register
i2c.writeto_mem(address1, ADR_CAP_SETUP, setupData)
time.sleep(1)

print("pF, degC")
print("# " + VERSION)

while True:    
    while not AD7747_ready.value(): # wait for /RDY signal to go high
        pass
    while AD7747_ready.value():     # wait for /RDY signal to go low: data ready
        pass

    rData = i2c.readfrom_mem(address1, ADR_CAP_DATAH, 6)  # read 3 cap and 3 temp bytes
    cDat = convertCapData(rData[:3]) # get capacitance reading in pF
    degC = convertTempData(rData[3:6]) # get chip temp in deg. C

    # hex_string = ' '.join(['{:02x}'.format(b) for b in rData])
    print("%.6f, %.3f" % (cDat, degC))
