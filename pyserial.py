import time
import serial
import re

# configure the serial connections (the parameters differs on the device you are connecting to
ser = serial.Serial(
    port='/dev/ttyACM0', # change this value to /dev/ttyACMn for the correct port number. 
                         # to check which port the redboard is connected to, do an 'ls' command inside /dev/
                         # while only the redboard is connected to the laptop
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

bytearr = bytearray()
while True:
    byteread = ser.read()
    if (byteread == b'\n'):
        message = bytearr.decode('utf-8')
        print(message)
        bytearr.clear()
    else:   
        bytearr += byteread
