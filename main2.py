import time
import serial
import re
import struct
import math

# configure the serial connections (the parameters differs on the device you are connecting to
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=112500,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

def string_to_float_cam(camdata, past_data):
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
    filtered_data = past_data
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
        filtered_data = re.sub(r'[^\d.]+',"",camdata)
        try:
            filtered_data = float(filtered_data)
            if filtered_data > 180:
                filtered_data = past_data
            past_data = filtered_data
        except Exception as e:
            print("Error value")
            print(e)
    return(coded_num,filtered_data)

# def white_line_corrections(coded_num,filtered_data):
#     """i want this shit to call our PI control, using it to measure time between calls and stuff like that. Most of the setup, including
#     where the time and integrams are stored will probably have to be moved around as our program evolves. I am treating this method
#     like its main, and will loop forever with no interruptions/new calls"""
 

#     while(True):
#         last_time, integral, motor_modifier = pi_control(last_time, process_var, control_out_max, control_bias, kc, reset_knob, integral)
#         print(motor_modifier)



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




def lidar_corrections(lidar_string,coded_num,filtered_data):
        lidar_data = [[],[]]
        float_string = 0
        index = -1
        lidar_string = lidar_string.split()
        for string in lidar_string:
            if string.startswith('['):
                print("iran")
                string = string.strip('[')
                index += 1
            try:
                float_string = float(string)
                lidar_data[index].append(float_string)
            except Exception as e:
                 print("Lidar number error")
        #print(lidar_data)

        """Strip the string given by embedded into the three numbers
           and save them into an array of arrays of all the data
           while throwing out all the bad data from there set a good zone and do 
           all the calculations for the power to the motors"""
        
#def power_conversions(motor_modifer):
    #increment = 50
    # CL = (left_power - right_power)
    # CF = min(left_power,right_power)
    # pwm = int(CF)
    # pwm = struct.pack('>h', pwm)
    # message = "CF".encode('utf-8') + pwm
    # ser.write(message)    
    # pwm = int(CL)
    # pwm = struct.pack('>h', pwm)
    # message = "CL".encode('utf-8') + pwm
    # ser.write(message)
    #rebounded_mm = 1/(1 + math.exp(-motor_modifier))
    #return increment * rebounded_mm

SOF = b'\xAA'                          # 0xAA = 170
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
k_p = 4.0
#Weight of integral for osliation
k_i = 1.3
#Reducing integral max weight
integral_clamp = 30.0
#Last time is our DT
last_time = time.monotonic()
####################################################

bytearr = bytearray()
coded_num = 0
filtered_data = 0
n = 0
Lcorrections = 0
Rcorrections = 0

testtime = 0
rightAngletemp = 0
leftAngletemp = 0
past_data = 0

motor_turn = 0

while True:
    byteread = ser.read()
    if (byteread == b'\n'):
        #python has a data.decode() function you could look into
        camdata = bytearr.decode('utf-8')
        #print(camdata)
        (coded_num,filtered_data) = string_to_float_cam(camdata,past_data)

        #print(filtered_data)
        #Testing only
        if coded_num == 3:
            leftAngletemp = filtered_data
            if leftAngletemp > 53.0:
                leftAngletemp = 0
        if coded_num == 5:
            rightAngletemp = filtered_data
            if rightAngletemp > 53.0:
                rightAngletemp = 0
        
        process_var = (rightAngletemp - leftAngletemp)
        last_time, integral, motor_modifier = pi_control(last_time, process_var, control_out_max, control_bias, k_p, k_i, integral, integral_clamp)
            # print("Right angle = ", rightAngletemp)
            # print("Left angle = ", leftAngletemp)
            # print("Right - Left = ", process_var)
            # print("motor modifier = ", motor_modifier)
            #print("last time was ", last_time)
            # motor_turn += motor_modifier
            # if motor_turn > 100:
            #     motor_turn = 100
            # if motor_turn < -100:
            #     motor_turn = -100
            # #print(integral)
        if testtime % 10 == 0:
            print("motor turn = ", motor_modifier)
            send_packet(b'F', -400)
            send_packet(b'L', int(-1*motor_modifier))

        testtime = testtime + 1
        #Testing only

        base_power = 400
        

        bytearr.clear()
    else:   
        bytearr += byteread
    n += 1

    