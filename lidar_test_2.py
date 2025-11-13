import serial

ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=112500,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
bytearr = bytearray()


while True:
    byteread = ser.read()
    #print(byteread)
    if (byteread == b'\n'):
        print(bytearr)
        #python has a data.decode() function you could look into
        if bytearr.startswith(b'S'):
            print("iran")
            raw_data = bytearr[1:]
            lidar_data = " ".join(f"{byte:02X}" for byte in raw_data)
            print(lidar_data)