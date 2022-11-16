import socket
import PIL.Image as Image
import PIL
import io
import struct
import random
import time
from PIL import ImageFile
from PIL import ImageTk as itk
from tkinter import *

def main():
    HOST = "192.168.1.76" # IP address of your Raspberry PI
    PORT = 6650          # Port to listen on (non-privileged ports are > 1023)
    img_count = 0
    ImageFile.LOAD_TRUNCATED_IMAGES = True
    prev_prompt_time = time.time()
    ws = Tk()
    ws.title('Eva')
    canvas = Canvas(
        ws,
        width = 480, 
        height = 640
        )      
    canvas.pack()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print("listening on: " + str(PORT))
        try:
            print("Before Accept")
            client, clientInfo = s.accept()
            print("After Accept")
            while 1:
                print("New loop")
                start_time = time.time() # start time of the loop
                canvas.delete("all")
                #print("server recv from: ", clientInfo)
                buf = b''
                print("Before recv")
                while len(buf)<4:
                    buf += client.recv(4-len(buf))
                print("After recv")
                size = struct.unpack('!i', buf)[0]
                if size == 30:
                    client, clientInfo = s.accept()
                    continue
                print("Image Size: ", size)
                num = RecvN(client, size)
                image = PIL.Image.open(io.BytesIO(num)).convert('RGBA').rotate(270, fillcolor = 0)
                print(image.size)
                img = itk.PhotoImage(image)      
                canvas.create_image(
                    242,
                    320, 
                    anchor=CENTER, 
                    image=img
                )
                if start_time - prev_prompt_time > 7:
                    val = str(analyze()).encode('utf-8')
                    print(val)
                    client.send(len(val).to_bytes(2, byteorder='big'))
                    client.send(val)
                    prev_prompt_time = start_time
                diff = time.time() - start_time
                if diff == 0:
                    diff = 1
                fps = str(round(1.0 / diff, 2))
                canvas.create_text(70, 50, text="FPS: " + fps, fill="black", font=('Helvetica 15 bold'))
                ws.update()
                img_count += 1
                print("Image count: ", img_count)
        except KeyboardInterrupt: 
            print("Closing socket")
            client.close()
            s.close()
            ws.destroy()
        except Exception as e: 
            print("Closing socket", e)
            client.close()
            s.close()
            canvas.destroy()
        

def RecvN(socket, n):
    totalContent = b''
    totalRecved = 0
    while totalRecved < n:
        onceContent = socket.recv(n - totalRecved)
        totalContent += onceContent
        totalRecved = len(totalContent)
 
    return totalContent

def analyze():
    return random.randrange(5)

if __name__ == "__main__":
    main()
