import time
import serial
import re
import struct
import math
import numpy as np

angle_constant = 0
dis_constant = 0

angle_th1 = 0
angle_th2 = 15

dis_th1 = 20
dis_th2 = 100


# configure the serial connections (the parameters differs on the device you are connecting to
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=112500,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, 
    bytesize=serial.EIGHTBITS
)

def string_to_float_cam(camdata):
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
    filtered_data = -100
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
        except Exception as e:
            print("Error value")
            print(e)
        if filtered_data > 180:
            filtered_data = 0
    return(coded_num,filtered_data)

def white_line_corrections(coded_num,filtered_data):
    """This code first normalizes the values given
    then multiplies the value by a constant for now"""
    angle_max = 5.300
    distance_max = 15.600
    fixed_data = 0
    Rcorrections = 0
    Lcorrections = 0
    if coded_num == 3: 
        fixed_data = filtered_data / angle_max
        Lcorrections = angle_constant * fixed_data
    if coded_num == 5:
        fixed_data = filtered_data / distance_max
        Rcorrections =  dis_constant * fixed_data
    if coded_num == 7:
        fixed_data = filtered_data / angle_max
        Lcorrections = angle_constant * fixed_data
    if coded_num == 9:
        fixed_data = filtered_data / distance_max
        Rcorrections = dis_constant * fixed_data
    return (Lcorrections,Rcorrections)


bytearr = bytearray()
coded_num = 0
filtered_data = 0
n = 0
test_avg_arr = [[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]
test_arr = [[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]
while n > 1000000:
    print("iran")
    byteread = ser.read()
    if (byteread == b'\n'):
        #python has a data.decode() function you could look into
        camdata = bytearr.decode('utf-8')
        #print(camdata)
        (coded_num,filtered_data) = string_to_float_cam(camdata)
        print("filtered_data")
        print(filtered_data)
        print("coded_num")
        print(coded_num)

        if coded_num == 3:
            if filtered_data <= angle_th1:
                test_arr[0] += 1
                test_avg_arr[0].append(filtered_data)
            if filtered_data > angle_th1 and filtered_data < angle_th2:
                test_arr[1] += 1
                test_avg_arr[1].append(filtered_data)
            if filtered_data >= angle_th2:
                test_arr[2] += 1
                test_avg_arr[2].append(filtered_data)
        if coded_num == 5:
            if filtered_data < angle_th1:
                test_arr[3] += 1
                test_avg_arr[3].append(filtered_data)
            if filtered_data >= angle_th1  and filtered_data < angle_th2:
                test_arr[4] += 1
                test_avg_arr[4].append(filtered_data)
            if filtered_data >= angle_th2:
                test_arr[5] += 1
                test_avg_arr[5].append(filtered_data)
        if coded_num == 7:
            if filtered_data <= dis_th1:
                test_arr[6] += 1
                test_avg_arr[6].append(filtered_data)
            if filtered_data > dis_th1 and filtered_data < dis_th2:
                test_arr[7] += 1
                test_avg_arr[7].append(filtered_data)
            if filtered_data <= dis_th2:
                test_arr[8] += 1
                test_avg_arr[8].append(filtered_data)
        if coded_num == 9:
            if filtered_data <= dis_th1:
                test_arr[9] += 1
                test_avg_arr[9].append(filtered_data)
            if filtered_data > dis_th1 and filtered_data < dis_th2:
                test_arr[10] += 1
                test_avg_arr[10].append(filtered_data)
            if filtered_data <= dis_th2:
                test_arr[11] += 1
                test_avg_arr[11].append(filtered_data)

        j = 0
        for arr in test_avg_arr:
            mean = np.mean(arr)
            test_avg_arr[j] = mean
            j += 1

        
        print("Amount of every distance and angle in thresholdes") #hey sean you spelled threshold wrong haha - Arman Rahimzadeh-Nasskhi
        print("For Left angle \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", angle_th1, test_arr[0], angle_th1, angle_th2, test_arr[1], angle_th2, test_arr[2])
        print("For Right angle \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", angle_th1, test_arr[3], angle_th1, angle_th2, test_arr[4], angle_th2, test_arr[5])
        print("For Left distance \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", dis_th1, test_arr[6], dis_th1, dis_th2, test_arr[7], dis_th2, test_arr[8])
        print("For Right distance \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", dis_th1, test_arr[9], dis_th1, dis_th2, test_arr[10], dis_th2, test_arr[11])

        print("Average Value for each catagory")
        print("For Left angle \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", angle_th1, test_avg_arr[0], angle_th1, angle_th2, test_avg_arr[1], angle_th2, test_avg_arr[2])
        print("For Right angle \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", angle_th1, test_avg_arr[3], angle_th1, angle_th2, test_avg_arr[4], angle_th2, test_avg_arr[5])
        print("For Left distance \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", dis_th1, test_avg_arr[6], dis_th1, dis_th2, test_avg_arr[7], dis_th2, test_avg_arr[8])
        print("For Right distance \n > %d [%d] \n %d <= %d [%d] \n %d > [%d]", dis_th1, test_avg_arr[9], dis_th1, dis_th2, test_avg_arr[10], dis_th2, test_avg_arr[11])

