# Read data from two SHT31 Temp/RH sensors, print average
# 01-Feb-2025 jpb

from machine import Pin, I2C
import time

# --------------------------
sht31_addr = 0x44   # i2c address of SHT31 sensor chip

# Use default I2C pins (GPIO4 for SDA, GPIO5 for SCL)
i2c0 = I2C(0, sda=Pin(4), scl=Pin(5), freq=100000)
i2c1 = I2C(1, sda=Pin(18), scl=Pin(19), freq=100000)

"""
print("Scanning I2C bus 0...")
devices = i2c0.scan()
if len(devices) == 0:
    print("No I2C devices found.")
else:
    print(f"Found {len(devices)} I2C devices:")
    for device in devices:
        print(f"Decimal address: {device}, Hexadecimal address: {hex(device)}")

print("Scanning I2C bus 1...")
devices = i2c1.scan()
if len(devices) == 0:
    print("No I2C devices found.")
else:
    print(f"Found {len(devices)} I2C devices:")
    for device in devices:
        print(f"Decimal address: {device}, Hexadecimal address: {hex(device)}")
"""

def getTRH(dev):
    dev.writeto_mem(sht31_addr, 0x24, bytes([0x00]))
    time.sleep(0.5)

    # Read data back from 0x00(00), 6 bytes
    # Temp MSB, Temp LSB, Temp CRC, Humididty MSB, Humidity LSB, Humidity CRC
    data = dev.readfrom_mem(sht31_addr, 0x00, 6)

    temp16 = data[0] * 256 + data[1]
    cTemp = -45 + (175 * temp16 / 65535.0)
    humidity = 100 * (data[3] * 256 + data[4]) / 65535.0

    return(cTemp, humidity)

# --------------------------------

led = Pin(25, Pin.OUT) # The onboard LED is connected to GPIO pin 25

led.value(1)
time.sleep(4)
led.value(0)


print("degC0, RH0, degC1, RH1")
print("# Read 2 SHT31 Temp/RH sensors 01-Feb-2025 jpb")

N = 20 # number of readings to average
#(cAvg, rhAvg) = getTRH()

while True:
    cSum0 = 0
    rhSum0 = 0
    cSum1 = 0
    rhSum1 = 0
    for i in range(N):
        (c, rh) = getTRH(i2c0)
        cSum0 += c
        rhSum0 += rh
        (c, rh) = getTRH(i2c1)
        cSum1 += c
        rhSum1 += rh
        
    c0 = cSum0 / N
    rh0 = rhSum0 / N
    c1 = cSum1 / N
    rh1 = rhSum1 / N
    print("%.3f,%.3f,%.3f,%.3f" % (c0,rh0,c1,rh1))
    #time.sleep(0.5)
