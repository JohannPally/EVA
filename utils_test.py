
import cv2
import time
from utils import Analyzer
import matplotlib.pyplot as plt
import numpy as np
# import mediapipe as mp
# import mediapipe.python.solutions.drawing_utils
# from matplotlib.axis import Axis
# from matplotlib.widgets import Slider
# from mpl_toolkits.mplot3d import Axes3D

agent = Analyzer()
cap = cv2.VideoCapture('videos/side.MOV')
timeout = time.time() + 40
while cap.isOpened() and (time.time() < timeout):
    success, image = cap.read()
    if success:
        imu = [] #TODO include imu data
        agent.update(image, imu)

agent.cleanup()
agent.plot_segmentation()
