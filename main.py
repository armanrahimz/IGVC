import time
import serial
import re
import struct
import math
import numpy as np

# configure the serial connections (the parameters differs on the device you are connecting to
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=112500,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=None
)

def decoding_string(data_from_red_board):
    if data_from_red_board[0] == 'L' or data_from_red_board[0] == 'R':
        return("Camera")
    if data_from_red_board[0] == 'G':
        return("GPS")
    else: return("Nothing")

def string_to_float_cam(camdata, past_data_cam):
    """"This function takes in the cam data and filters it into a float and identifyer
        I used the coded number as a check as well
        The coded data is

        -1 for error data
        3 for Left Angle
        5 for Right Angle
        7 for Left Distance
        9 for Right Distance
        """
    coded_num = -1
    filtered_data = past_data_cam
    char_filter = (re.sub(r'[^ADLR]','',camdata))
    for char in char_filter:
        if char == 'L':
            coded_num += 0
        elif char == 'R':
            coded_num += 2
        elif char == 'A':
            coded_num += 4
        elif char == 'D':
            coded_num += 8
    if coded_num == 3 or coded_num == 5 or coded_num == 7 or coded_num == 9:
        filtered_data = re.sub(r'[^\d.-]+',"",camdata)
        try:
            filtered_data = float(filtered_data)
            if filtered_data > 180:
                filtered_data = past_data_cam
            past_data_cam = filtered_data
        except Exception as e:
            print("Error value")
            print(e)
    return(coded_num,filtered_data)

def pi_control(last_time, process_var, control_out_max, control_bias, k_p, k_i, integral, integral_clamp):

    dt_start = time.monotonic()
    dt = dt_start - last_time
    last_time = dt_start

    integral += process_var* dt

    # this is a way for us to tweak how fast the integral scales, so that it wont absolutely throw our robot for a loop, some freaky 
    # shit chatgpt suggested ill look at it later i guess
    integral = max(min(integral, integral_clamp), -integral_clamp)

    integral_f = (k_i) * integral

    control_out = control_bias + k_p*process_var + integral_f
    #some more freaky shit that allows us to limit the maximum speed our controller is adjusted by. can use if need
    control_out = max(min(control_out, control_out_max), -control_out_max)

    return last_time, integral, control_out

def binary_string_to_bytes(binary_str):
    if len(binary_str) % 8 != 0:
        raise ValueError("Binary string length must be a multiple of 8")
    return bytes(int(binary_str[i:i+8], 2) for i in range(0, len(binary_str), 8))

def lidar_corrections(lidar_string,currLeftDist,currRightDist):
    #bytes_data = bytes.fromhex(lidar_string)
    bytes_data = binary_string_to_bytes(lidar_string)

    if len(bytes_data) % 2 != 0:
        raise ValueError("Hex must be an even amount of bytes")

    floats = []

    # Control flags
    apply_mask = False

    # Step through in 2-byte pairs
    i = 0
    while i < len(bytes_data):
        pair = bytes_data[i:i+2]

        # If it's an Ax 5x marker, skip it and toggle mask ON for the next values
        if pair[0] & 0xF0 == 0xA0 and pair[1] & 0xF0 == 0x50:
            apply_mask = True
            i += 2
            continue

        # Unpack the value
        value = struct.unpack('<H', pair)[0]

        # Apply mask if flag is set
        if apply_mask:
            value &= 0x7FFF
            value = value / 64
            apply_mask = False

        floats.append(float(value))
        i += 2

    counter = 0
    f_angle = floats[0]
    e_angle = floats[42]
    e_angle = 0
    values = 0
    tuple_lists = []
    while values < len(floats):
        counter += 1

        if(counter % 41 == 0 and counter > 42):
            try:
                e_angle = floats[counter]
            except  Exception as e:
                print("no end angle dumping")
                print(e)
            
            avg_gap = (e_angle - f_angle)
            if(avg_gap < 0):
                avg_gap += 360

            gap_dist = avg_gap/39
            dist_list = floats[1:-1]
            angle_list = [f_angle + i*gap_dist for i in range(40)]

            number = 0
            for i in angle_list:
                if i > 360.0:
                    angle_list[number] = i - 360.0
                number += 1

            tuple_list = list(zip(angle_list, dist_list))
            tuple_lists.append(tuple_list)

            f_angle = e_angle

        elif (counter == 42):
            e_angle = floats[counter -1]
            
            avg_gap = (e_angle - f_angle)
            if(avg_gap < 0):
                avg_gap += 360

            gap_dist = avg_gap/39
            dist_list = floats[1:-1]
            angle_list = [f_angle + i*gap_dist for i in range(40)]

            number = 0
            for i in angle_list:
                if i > 360.0:
                    angle_list[number] = i - 360.0
                number += 1

            tuple_list = list(zip(angle_list, dist_list))
            tuple_lists.append(tuple_list)

            f_angle = e_angle

        values += 1

    tuple_lists.pop()
    happy_values = []
    right_values = []
    left_values = []
    for tuple_list in tuple_lists:
        for value in tuple_list:
            #print(value[0])
            if value[0] < 15.0 or value[0] > 345.0:
                happy_values.append(value[1])
            happy_max = np.mean(happy_values)
            if happy_max < 5000:
                Lidar_mode = True
            if happy_max > 5000 and Lidar_mode:
                return (0, -300)

    for tuple_list in tuple_lists:
        for value in tuple_list:
            if value[0] > 15 and value[0] < 180:
                right_values.append(value[1])
    
            if value[0] < 345 and value[0] > 270:
                left_values.append(value[1])

    right_mean = np.mean(right_values)
    left_mean = np.mean(left_values)
    
    if currLeftDist < 20:
        return( -100 , -200)
    
    if currRightDist < 20:
        return( 100, -200)
    
    if happy_max > 5000 and Lidar_mode:
        return (0, -300)

    if right_mean > 8000 and left_mean > 8000:
        Lidar_mode = False

    if right_mean > left_mean:
        speed = ((happy_max / 350) - 1 ) * 15
        return(100 , speed) 
    else: 
        speed = ((happy_max / 350) - 1 ) * 15
        return(-100 , speed)

def gps_parseing(GPS_string):
    GPS_id = 0
    char_filter = (re.sub(r'[^GAO]','',GPS_string))
    for char in char_filter:
        if char == 'G':
            GPS_id += 1
        if char == 'A':
            GPS_id += 2
        if char == 'O':
            GPS_id += 3
    filtered_data = re.sub(r'[^\d.-]+',"",GPS_string)
    try:
        GPS_Float = float(filtered_data)
    except Exception as e:
        ("GPS Data reading error")
        print(e)
    if GPS_id == 3:
        return(GPS_Float,0)
    if GPS_id == 4:
        return(0,GPS_Float)

def calculate_bearing(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    x = math.sin(delta_lambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

    bearing = math.atan2(x, y)
    return (math.degrees(bearing) + 360) % 360


SOF = b'\xAA'                                    # 0xAA = 170
def send_packet(cmd: bytes, value: int):
    payload = struct.pack('>bh', cmd[0], value)  # 1-byte cmd, 2-byte int
    crc = (sum(payload) & 0xFF).to_bytes(1, 'big')
    ser.write(SOF + payload + crc)

######################tweaking knobs################
#Unchanged floats
process_var = 0.0
integral = 0.0
#Max motor power allowed
control_out_max = 200.0
#Fix veering
control_bias = 0.0
#How fast 
k_p = 2.0
#Weight of integral for osliation
k_i = 1.5
#Reducing integral max weight
integral_clamp = 100.0
#Last time is our DT
last_time = time.monotonic()
####################################################

####################################################
####################################################
################    GLOBALS   ######################
####################################################
####################################################  
bytearr = bytearray()

#GPS Globals
#Set values
endLatitude = 0
endLongitude = 0
startLatitude = 0
startLongitude = 0
#Non Changable
currLatitude = 0
currLongitude = 0
pastLatitude = 0
pastLongitude = 0
GPS_mode = False

#Lidar Globals
Lidar_mode = False

#Camera Globals
coded_num = 0
filtered_data = 0
rightAngletemp = 0
leftAngletemp = 0
currLeftDist = 0
currRightDist = 0  
past_data_cam = 0

#Others
motor_turn = 0

data_type = 0
while True:
    byteread = ser.read()
    if (byteread == b'\n'):
        red_board_data = bytearr.decode('utf-8')
        print(red_board_data)
        #data_type = decoding_string(red_board_data)
        #print(data_type)
        if data_type == "Camera":
            (coded_num,filtered_data) = string_to_float_cam(red_board_data,past_data_cam)
        if data_type == "Lidar":
            (Lidar_motor_control_L,Lidar_motor_control_F) = lidar_corrections(red_board_data,currLeftDist,currRightDist)
        if data_type == "GPS":
            (tempLat,tempLong) = gps_parseing(red_board_data)
            if tempLat != 0:
                currLatitude = tempLat
                pastLatitude = currLatitude
            if tempLong != 0:
                currLongitude = tempLong
                pastLongitude = currLongitude
            if currLatitude == startLatitude and currLongitude == startLongitude:
                GPS_mode = True
        if data_type == "Nothing":
            continue
            
        if coded_num == 3:
            leftAngletemp = filtered_data
            if leftAngletemp > 53.0:
                leftAngletemp = 0
        if coded_num == 5:
            rightAngletemp = filtered_data
            if rightAngletemp > 53.0:
                rightAngletemp = 0
        if coded_num == 7:
            currLeftDist = filtered_data
        if coded_num == 9:
            currRightDist = filtered_data

        #Main mode transfer
        if GPS_mode:
            desired_bearing = calculate_bearing(currLatitude,currLongitude,endLatitude,endLongitude)
            current_heading = calculate_bearing(pastLatitude,pastLongitude,currLatitude,currLongitude)

            angle_dif = (desired_bearing - current_heading + 360) % 360
            if angle_dif > 180:
                angle_dif -= 360
            
            if abs(angle_dif) > 5:
                if angle_dif > 0:
                    send_packet(b'L', -25)
                else:
                    send_packet(b'L', 25)
            else: send_packet(b'L', 0)
            send_packet(b'F', -350)
                    
            bytearr.clear()
            continue
        if Lidar_mode:
            send_packet(b'F', int(Lidar_motor_control_F))
            send_packet(b'L', int(Lidar_motor_control_L))
            bytearr.clear()
            continue
        else:
            process_var = (rightAngletemp - leftAngletemp)
            last_time, integral, motor_modifier = pi_control(last_time, process_var, control_out_max, control_bias, k_p, k_i, integral, integral_clamp)
            print("motor turn = ", motor_modifier)
            send_packet(b'F', -350)
            send_packet(b'L', int(motor_modifier))

            #Testing only
            bytearr.clear()
    else:   
        bytearr += byteread


    