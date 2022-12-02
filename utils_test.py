
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
cap = cv2.VideoCapture('side.MOV')
timeout = time.time() + 40
while cap.isOpened() and (time.time() < timeout):
    success, image = cap.read()
    if success:
        imu = []
        agent.update(image, imu)

agent.cleanup()

plt.plot(np.arange(len(agent.all_ys)), agent.all_ys)
plt.axhline(y = agent.top_threshold, color = 'r', linestyle = '-')
plt.axhline(y = agent.bottom_threshold, color = 'b', linestyle = '-')
plt.ylim(-.5,1)

for down in agent.down_windows:
    plt.axvspan(down[0], down[1], color = 'lime')

for up in agent.up_windows:
    plt.axvspan(up[0], up[1], color = 'cyan')

for hold in agent.hold_windows:
    plt.axvspan(hold[0], hold[1], color = 'gray')

plt.show()
