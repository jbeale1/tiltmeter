# Read data from SHT31 Temp/RH sensor and print
# runs on Pi Pico using RPI_PICO-20241129-v1.24.1.uf2
# 31-Jan-2025 jpb

from machine import Pin, I2C
import time

# --------------------------
sht31_addr = 0x44   # i2c address of SHT31 sensor chip

# Use default I2C pins (GPIO4 for SDA, GPIO5 for SCL)
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)

"""
print("Scanning I2C bus...")
devices = i2c.scan()
if len(devices) == 0:
    print("No I2C devices found.")
else:
    print(f"Found {len(devices)} I2C devices:")
    for device in devices:
        print(f"Decimal address: {device}, Hexadecimal address: {hex(device)}")
"""        

def getTRH():
    # SHT31 address, 0x44(68)
    # Send measurement command, 0x2C(44)
    #		0x06(06)	High repeatability measurement

    #i2c.writeto_mem(device_address, register_address, bytes([data]))

    i2c.writeto_mem(sht31_addr, 0x2C, bytes([0x06]))
    time.sleep(0.5)
    # SHT31 address, 0x44(68)
    # Read data back from 0x00(00), 6 bytes
    # Temp MSB, Temp LSB, Temp CRC, Humididty MSB, Humidity LSB, Humidity CRC
    data = i2c.readfrom_mem(sht31_addr, 0x00, 6)

    # Convert the data
    temp = data[0] * 256 + data[1]
    cTemp = -45 + (175 * temp / 65535.0)
    #fTemp = -49 + (315 * temp / 65535.0)
    humidity = 100 * (data[3] * 256 + data[4]) / 65535.0

    # Output data to screen
    #print("Temperature in Celsius is : %.3f C" % cTemp)
    #print("Relative Humidity is : %.2f %%RH" % humidity)        
    #print("%.2f, %.2f"% (cTemp, humidity))
    return(cTemp, humidity)

# --------------------------------
print("degC, RH")
print("# Read SHT31 Temp/RH sensor 31-Jan-2025 jpb")

N = 20 # number of readings to average
#(cAvg, rhAvg) = getTRH()
while True:
    cSum = 0
    rhSum = 0
    for i in range(N):
        (c, rh) = getTRH()
        cSum += c
        rhSum += rh
        
    c = cSum / N
    rh = rhSum / N
    print("%.3f,%.3f" % (c,rh))
    #time.sleep(0.5)
