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

#buffer for storing current IMU reading, size of 15: {cur-xl, cur-gr, cur-mg, cur-orient, cur-position}
cur_IMU_reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

image_folder = "images"
enabled_playback = False
agent = Analyzer()

def main(): # main function that runs continously

    HOST = "192.168.1.76" # IP address of the cloud server
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
    canvas.pack() # Initialize GUI canvas
    
    img_ref = canvas.create_image(242, 320, anchor=CENTER)
    fps_ref = canvas.create_text(70, 50, text="FPS: 0.0", fill="black", font=('Helvetica 15 bold')) 
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # set the socket addr to be able to be reused
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)         
        s.bind((HOST, PORT))
        s.listen() # Waiting for connection
        
        print("listening on: " + str(PORT))
        client, clientInfo = s.accept()
        client.settimeout(1)
        print("Accepted Connection")
        prev = time.time()
        prev_show = time.time()
        timeout = time.time() + 100
        
        while (time.time() < timeout):
            if cv2.waitKey(1) & 0xFF == 27: # quit on keyboard interrupt
                break
            try:
                start_time = time.time() # start time of the loop
                diff = start_time - prev
                prev = start_time
                if diff == 0:
                    diff = 1
                fps = str(round(1.0 / diff, 2)) # calculate FPS
                
                (nums, img_file_size, img, img_itk, client_res, clientInfo_res) = getImage(client, clientInfo, s)

                client = client_res
                clientInfo = clientInfo_res
                if img is None:
                    continue
                time_before_agent = time.time()

                #get the current IMU reading
                cur_IMU_reading = request_IMU_data()
                #update the agent
                returned = agent.update(nums, cur_IMU_reading[0]) # send data to analyer

                codes = [agent.FLEXION_BOTTOM_ERROR_CODE, agent.FLEXION_TOP_ERROR_CODE, agent.TILT_DOWN_ERROR_CODE,
                        agent.TILT_UP_ERROR_CODE, agent.INSTABILITY_ERROR_CODE, agent.ROTATOIN_ERROR_CODE]
                
                for code in codes:
                    if code == returned:
                        val = str(code).encode("utf-8")
                        client.send(val)
                        print("Sent Status Code: ", code)
                        break
                # match error code and send status to phone
                
                if time.time() - prev_show > 0.05:
                    canvas.itemconfigure(img_ref, image=img_itk) # update canvas image in a constrained speed, since it is very resource consuming
                canvas.itemconfig(fps_ref, text="FPS: " + fps)
                if enabled_playback:
                    img.save("./" + image_folder + "/" + str(time.time()) + ".png") # call playback function that converts frame images to video
                ws.update()
                img_count += 1
                img.close()
                
            except KeyboardInterrupt as e:
                print("Keyboard Interrupted")
                if enabled_playback:
                    print("Post-processing Playback...")
                    images_to_video() # post processing image frames to video
                client.close()
                s.close()
                ws.destroy()
                agent.cleanup()
                agent.plot_segmentation() # upon manual interruption, plot currently gathered data
            except Exception as e: 
                print("Closing socket", e)
                client.close()
                s.close()
                canvas.destroy()
                break

        print("Timeout")
        if enabled_playback:
            print("Post-processing Playback...")
            images_to_video()
        client.close()
        s.close()
        ws.destroy()
        agent.cleanup()
        agent.plot_segmentation()

def images_to_video(): # post processing function that converts image frames to video
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter('plaback.avi', 0, 20, (width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image))) # using opencv to convert images to video

    cv2.destroyAllWindows()
    video.release()
    dir_name = image_folder + "/"
    all_images = os.listdir(dir_name)

    for item in all_images:
        if item.endswith(".png"):
            os.remove(os.path.join(dir_name, item)) # remove all images afterwards

def getImage(client, clientInfo, s): # function to request image from phone once
    buf = b''
    try:
        count = 0
        while len(buf) < 4:
            buf += client.recv(4-len(buf))
            count += 1  # receive image size first in order to determine image buffer size
            if(count > 5): raise ConnectionResetError
    except Exception as e:
        agent.cleanup()
        agent.plot_segmentation()
        res_client, res_clientInfo = s.accept()
        return ([], 0, None, None, res_client, res_clientInfo)
    size = struct.unpack('!i', buf)[0]
    
    temp = time.time()
    num = RecvN(client, size)
    image = PIL.Image.open(io.BytesIO(num)).convert('RGBA').rotate(270, fillcolor = 0)
    ndarr = np.rot90(cv2.imdecode(np.frombuffer(num, np.uint8), -1)) # flip the image as it was send in row major bytes
    img_itk = itk.PhotoImage(image)
    return (ndarr, size, image, img_itk, client, clientInfo)

def RecvN(socket, n): # helper function that continuously perform socket receive
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
    return to_return, (time_diff * 1000)



if __name__ == "__main__":
    main()
