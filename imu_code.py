import serial as ser
import pandas as pd
import numpy as np
import time
stopSendRequest = 10
counter = 0
PORT = "/dev/cu.usbmodem14303"

from collections import deque


all_data = []

my_ser_port = ser.Serial(PORT, 115200)
serial_cmd = "r"
encoded_cmd = serial_cmd.encode()

cur_IMU_reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#queue for storing image/IMU-reading sequence
data_queue = deque([None, None, None, None, None, None, None, None, None, None])


sendCount = 0
def request_data():
    global counter, stopSendRequest, sendCount
    prev_time = time.time_ns()

    # if counter < stopSendRequest:
    my_ser_port.write(encoded_cmd)
        # sendCount += 1
        # print(sendCount)
    # else:
    #     print("stop send request")
    serial_obj = my_ser_port.readline()

    cur_time = time.time_ns()

    # Poll the serial port
    line = str(serial_obj, 'utf-8')
    # counter += 1

    

    return line,(cur_time - prev_time)/1000000

def request_IMU_data():

    global cur_IMU_reading
    
    prev_time = time.time()

    my_ser_port.write(encoded_cmd)
    # my_ser_port.write(encoded_cmd)
    # serial_obj_0 = my_ser_port.readline()
    serial_obj_1 = my_ser_port.readline()
    # Poll the serial port
    line = str(serial_obj_1, 'utf-8').replace("\r\n", "").split("' ")
    to_return = cur_IMU_reading

    #check the length is correct
    if (len(line) != 15):
        return to_return

    #check the parsing is correct
    #if wrong just use the previous parsing 

    to_return = [float(element) for element in line]
    

    time_diff = time.time() - prev_time

    print(to_return)
    # print(time_diff)
    return to_return, (time_diff * 1000)


def IMU_analyze(data_que):
    #check if not enough data received
    if data_que[0] is None:
        return "still gathering"

    data_mat = np.vstack((data_que[0], 
                          data_que[1], 
                          data_que[2], 
                          data_que[3], 
                          data_que[4], 
                          data_que[5], 
                          data_que[6], 
                          data_que[7], 
                          data_que[8], 
                          data_que[9]))
    
    #accelerometer readings

    # xl_x value for validating bar tilt
    # should keep at around 0 (-0.1~+0.1) unit is G
    # left_tilt(tilt down to left): 
    #   positive value
    # right_tilt(tilt down to down):
    #   negative value    
    xl_x = data_mat[:, 0].reshape(1, 10)
    avg_xl_x = np.average(xl_x)

    # xl_y value for validating wrist bending
    # should keep at around -1 (-0.9~-1.1) unit is G
    # forward bend: 
    #   less negative value
    # backward bend:
    #   less negative value    
    xl_y = data_mat[:, 1].reshape(1, 10)
    avg_xl_y = np.average(xl_y)

    #gyrometer reading for unstability check
    gr_x = data_mat[:, 3].reshape(1, 10)
    gr_y = data_mat[:, 4].reshape(1, 10)

    # orient_y value for checking bar tilt 
    # should keep at around 0 (-2~+2) unit is degree
    # left_tilt: 
    #   negative value
    # right_tilt:
    #   positive value
    orient_x = data_mat[:, 9].reshape(1, 10)
    avg_orient_x = np.average(orient_x)

    # orient_x value for checking wrist bend
    # should keep at around -90 (-85~-95) unit is degree
    # forward_bend:
    #   more negative
    # backward_bend:
    #   less negative
    orient_y = data_mat[:, 10].reshape(1, 10)
    avg_orient_y = np.average(orient_y)


    #check unstability
    if (np.var(gr_x) > 20 or np.var(gr_y) > 20):
        return "unstable! settle down!"

    elif (avg_orient_x < -95 and avg_xl_y > -1.01):
        return "wrist bending forward!"
    
    elif (avg_orient_x > -85 and avg_xl_y > -1.01):
        return "wrist bending backward!"
    
    elif (avg_orient_y < -2 and avg_xl_x > 0.2):
        return "left tilt!"

    elif (avg_orient_y > 2 and avg_xl_x < -0.2):
        return "right tilt!"

    return "everything okay! keep up!"
    


def main():
    while (1):
        line, time_diff = request_IMU_data()
        time.sleep(0.02)

        data_queue.popleft()
        data_queue.append(np.array(line))

        print(IMU_analyze(data_queue))


if __name__ == "__main__":
    main()