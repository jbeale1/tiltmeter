# Take a set of images with camera on XYZ translation stage
# J.Beale 9-Feb-2025

import cv2
import numpy as np
import os
import serial
import time

VERSION = "CamStack v0.3 2-9-2025 JPB"
serial_port = 'COM5'  # serial port of grbl interpreter
baud_rate = 115200
camNumber = 2  # 1 for USB microscope cam on Lenovo ideapad 330


nAverage = 25   # average together this many images
zStep = 0.10   # z step size in mm
zTotal = 10       # how many frames in Z-stack
xStep = 0.5     # x axis step size in mm
xTotal = 10      # how many x axis steps to make total
stepSpeed = 300 # step speed in mm/minute

saveDir = r"C:\Users\beale\Documents\Tiltmeter\jpeg"
#saveDir = r"C:\Users\tamar\Documents\Scope1"

def send_gcode(port, baudrate, gcode_command):
    try:
        with serial.Serial(port, baudrate) as ser:
            ser.write(f'{gcode_command}\n'.encode('utf-8'))
            time.sleep(0.1)  # Wait for the command to be processed
            while ser.in_waiting:
                print(ser.readline().decode('utf-8').strip())
    except serial.SerialException as e:
        print(f"Error: {e}")

# -------------------------------------------------------------

if __name__ == "__main__":
    
    print(VERSION)
    

    #cam = cv2.VideoCapture(camNumber)
    #cam = cv2.VideoCapture(camNumber, cv2.CAP_MSMF)
    cam = cv2.VideoCapture(camNumber, cv2.CAP_DSHOW)

    if not cam.isOpened():
        raise IOError("Cannot open webcam")

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1600) # set X and Y resolution
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

    frame_rate = 8
    time_interval = 1 / frame_rate

    prev_time = 0
    zCounter = 0 # how many Z steps so far
    xCounter = 0 # how many X axis steps so far
    
    
    send_gcode(serial_port, baud_rate, 'G91') # set relative positioning
    time.sleep(1)
    
    # -------------------------------------------------

    while True:
        time.sleep(0.5)
        
        images = []
        for i in range(nAverage): # capture this many images
            ret, frame = cam.read()
            if not ret:
                raise IOError("Failed to capture image")
            images.append(frame)
            cv2.imshow('Camera Image', frame)
            k = cv2.waitKey(1)
            if k & 0x7F == ord('q'):
                break

        if k & 0x7F == ord('q'):
            break
        
        frame = ((256/nAverage)*np.sum(images, axis=0)).astype(np.uint16) # combine images        
        
        fName = ("img_%02d_%04d.png" % (xCounter,zCounter))
        savePath = os.path.join(saveDir, fName)
        cv2.imwrite(savePath, frame)
        print("Saved %s" % fName)
        zString = ("G1 Z%.3f F%d" % (zStep,stepSpeed)) # for example "G1 Z0.050 F300"
        send_gcode(serial_port, baud_rate, zString) # Move Z up zStep at speed F
        zCounter += 1
        if (zCounter >= zTotal): # finished this Z-stack sequence?
            zCounter = 0
            zReturn = -(zTotal * zStep)
            zString = ("G1 Z%.3f F%d" % (zReturn,stepSpeed)) # return to original Z height
            send_gcode(serial_port, baud_rate, zString) # Move Z down at speed F
            
            xString = ("G1 X%.3f F%d" % (xStep,stepSpeed)) # return to original Z height
            send_gcode(serial_port, baud_rate, xString) # Move X up 1mm at speed F
            xCounter += 1
            if (xCounter >= xTotal):
                break
            time.sleep(1)
            

    # ----------------------------------------
    print("Imaging run complete.")
    send_gcode(serial_port, baud_rate, 'G90') # set abs positioning
    # Release the webcam and destroy all windows
    cam.release()
    cv2.destroyAllWindows()
    
