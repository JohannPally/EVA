import socket
import PIL.Image as Image
import PIL
import io
import struct
import random
import time
import cv2
import os
import matplotlib.pyplot as plt
import numpy as np
from PIL import ImageFile
from PIL import ImageTk as itk
from tkinter import *

import signal
import serial as ser
import pandas as pd
import numpy as np

from collections import deque
from utils import Analyzer


PORT = "/dev/cu.usbmodem14303"
my_ser_port = ser.Serial(PORT, 115200)
serial_cmd = "r"
encoded_cmd = serial_cmd.encode()
# 
# #buffer for storing current IMU reading, size of 15: {cur-xl, cur-gr, cur-mg, cur-orient, cur-position}
cur_IMU_reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#queue for storing image/IMU-reading sequence
data_queue = deque([None, None, None, None, None, None, None, None, None, None])
image_folder = "images"
enabled_playback = False

def main():

    #global cur_IMU_reading

    HOST = "10.192.181.22" # IP address of your Raspberry PI
    PORT = 6650            # Port to listen on (non-privileged ports are > 1023)
    img_count = 0

    ImageFile.LOAD_TRUNCATED_IMAGES = True
    prev_prompt_time = time.time()
    ws = Tk()
    ws.title('EVA')
    canvas = Canvas(
        ws,
        width = 480, 
        height = 640
        )      
    canvas.pack()
    agent = Analyzer()
    img_ref = canvas.create_image(242, 320, anchor=CENTER)
    fps_ref = canvas.create_text(70, 50, text="FPS: 0.0", fill="black", font=('Helvetica 15 bold')) 
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        s.bind((HOST, PORT))
        s.listen()
        
        print("listening on: " + str(PORT))
        print("Before Accept")
        client, clientInfo = s.accept()
        client.settimeout(1)
        print("After Accept")
        prev = time.time()
        prev_show = time.time()
        timeout = time.time() + 45
        
        while (time.time() < timeout):
            if cv2.waitKey(1) & 0xFF == 27:
                break
            try:
                start_time = time.time() # start time of the loop
                diff = start_time - prev
                prev = start_time
                if diff == 0:
                    diff = 1
                fps = str(round(1.0 / diff, 2))

                #print("server recv from: ", clientInfo)
                
                (nums, img_file_size, img, img_itk, client_res, clientInfo_res) = getImage(client, clientInfo, s)

                

                client = client_res
                clientInfo = clientInfo_res
                if img is None:
                    continue
                time_before_agent = time.time()

                #get the current IMU reading
                cur_IMU_reading = request_IMU_data()
                #update the agent
                returned = agent.update(nums, cur_IMU_reading[0])
                if returned is not None:
                    print('\033[93m'+returned+'\033[93m')

#                 print("time diff image: " + str(time.time() - time_before_agent))
#                 print("Image Size: ", img_file_size)
                if time.time() - prev_show > 0.05:
#                     if data_queue[9] != None:
                    canvas.itemconfigure(img_ref, image=img_itk)
                canvas.itemconfig(fps_ref, text="FPS: " + fps)
                if enabled_playback:
                    img.save("./" + image_folder + "/" + str(time.time()) + ".png")
                ws.update()
                img_count += 1
                img.close()
#                 print("Image count: ", img_count)

                #get current IMU readings
                
                # time_before_IMU = time.time()

                

#                 print("time diff IMU: " + str(time.time() - time_before_IMU))

                #update the queue
                # data_queue.popleft()
                # data_queue.append((img, cur_IMU_reading))
            except:
                pass

            # print("Keyboard Interrupted")
            #     if enabled_playback:
            #         print("Post-processing Playback...")
            #         images_to_video()
            #     client.close()
            #     s.close()
            #     ws.destroy()
            #     agent.cleanup()
            #     agent.plot_segmentation()

#             except Exception as e: 
#                 print("Closing socket", e)
#                 client.close()
#                 s.close()
#                 canvas.destroy()
#                 break

        print("Timeout")
        if enabled_playback:
            print("Post-processing Playback...")
            images_to_video()
        client.close()
        s.close()
        ws.destroy()
        agent.cleanup()
        agent.plot_segmentation()

def images_to_video():
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter('plaback.avi', 0, 20, (width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()
    dir_name = image_folder + "/"
    all_images = os.listdir(dir_name)

    for item in all_images:
        if item.endswith(".png"):
            os.remove(os.path.join(dir_name, item))

def getImage(client, clientInfo, s):
    buf = b''
    try:
        count = 0
        while len(buf) < 4:
            buf += client.recv(4-len(buf))
            count += 1
            if(count > 5): raise ConnectionResetError
    except Exception as e:
        print("Before recv2")
        res_client, res_clientInfo = s.accept()
        return ([], 0, None, None, res_client, res_clientInfo)
    size = struct.unpack('!i', buf)[0]
    
    temp = time.time()
    num = RecvN(client, size)
    image = PIL.Image.open(io.BytesIO(num)).convert('RGBA').rotate(270, fillcolor = 0)
    ndarr = np.rot90(cv2.imdecode(np.frombuffer(num, np.uint8), -1))
    img_itk = itk.PhotoImage(image)
    return (ndarr, size, image, img_itk, client, clientInfo)

def RecvN(socket, n):
    totalContent = b''
    totalRecved = 0
    while totalRecved < n:
        onceContent = socket.recv(n - totalRecved)
        totalContent += onceContent
        totalRecved = len(totalContent)
 
    return totalContent


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

    # print(to_return)
    # print(time_diff)
    return to_return, (time_diff * 1000)



if __name__ == "__main__":
    main()



