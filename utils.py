import cv2
import mediapipe as mp
import time
import matplotlib.pyplot as plt
from matplotlib.axis import Axis
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class Analyzer:
    def __init__(self):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
                smooth_landmarks=True,
                smooth_segmentation=True)

        self.L_WRIST = self.mp_pose.PoseLandmark.LEFT_WRIST
        self.L_ELBOW = self.mp_pose.PoseLandmark.LEFT_ELBOW
        self.L_SHOULDER = self.mp_pose.PoseLandmark.LEFT_SHOULDER
        self.R_WRIST = self.mp_pose.PoseLandmark.RIGHT_WRIST
        self.R_ELBOW = self.mp_pose.PoseLandmark.RIGHT_ELBOW
        self.R_SHOULDER = self.mp_pose.PoseLandmark.RIGHT_SHOULDER

        self.skeletons = []
        self.images = []
        self.imu_readings = []
        self.times = []
        self.frame_async = -1

        self.moving_windows = []
        self.hold_windows = []
        self.down_windows = []
        self.up_windows = []

        self.top_threshold = .5
        self.bottom_threshold = .2

        self.reset_fsm()

        self.all_ys = []
        self.all_xs = []

    def reset_fsm(self):
        self.moving_flag = True
        self.moving_sf = 0
        self.down_flag = False
        self.hold_flag = False
        self.down_sf = -1
        self.up_sf = -1
        self.hold_sf = -1

    def update(self, image, imu_readings):
        if self.skeletonize(image, imu_readings):
            start, end = self.update_activity_segmentation()
            # if start is not None and end is not None:
            #     print(start, end)
            #TODO DO INJURY MOTION CHECKING WITH OUTPUT ^^
            #TODO ANGLE
        return
    
    def cleanup(self):
        if len(self.down_windows) > len(self.up_windows):
            self.down_windows.pop()

    def update_activity_segmentation(self):
        # print(self.moving_flag, self.down_flag, self.hold_flag)
        self.set_standardized_values()

        if self.moving_flag:
            if not self.detect_moving():
                print('no longer moving')
                self.moving_flag = False
                self.moving_windows.append([self.moving_sf])            
            return None, None
        
        #sitting at bench
        if self.detect_moving():
            print('moving, reset flags')
            self.reset_fsm()
            return None, None

        #going down
        if self.down_flag:
            #finish holding at the top
            if self.hold_flag and self.check_threshold(self.stnd_ys, self.top_threshold, '<'):
                print('start going down', self.stnd_ys)
                self.hold_flag = False
                self.down_sf = self.frame_async
                temp_wind = [self.hold_sf, self.frame_async]
                self.hold_windows.append(temp_wind)
                return 'hold_top', temp_wind
                
            #reach bottom of rep
            elif not self.hold_flag and self.check_threshold(self.stnd_ys, self.bottom_threshold, '<'):
                print('stop going down', self.stnd_ys)
                self.down_flag = False
                self.hold_flag = True
                self.hold_sf = self.frame_async
                temp_wind = [self.down_sf, self.frame_async]
                self.down_windows.append(temp_wind)
                print('down', temp_wind)
                return 'down', temp_wind
        
        #going up
        else:
            #finish holding at bottom
            if self.hold_flag and self.check_threshold(self.stnd_ys, self.bottom_threshold, '>'):
                print('start going up', self.stnd_ys)
                self.hold_flag = False
                self.up_sf = self.frame_async
                temp_wind = [self.hold_sf, self.frame_async]
                self.hold_windows.append(temp_wind)
                return 'hold_bottom', temp_wind
                
            #reach top of rep
            elif not self.hold_flag and self.check_threshold(self.stnd_ys, self.top_threshold, '>'):
                print('stop going up', self.stnd_ys)
                self.down_flag = True
                self.hold_flag = True
                self.hold_sf = self.frame_async

                if self.up_sf != -1:
                    temp_wind = [self.up_sf, self.frame_async]
                    self.up_windows.append(temp_wind)
                    print('up', temp_wind)
                    return 'up', temp_wind

        return None, None

    #========SUPPORT FUNCTIONS============

    def check_threshold(self, list, threshold, symbol):
        if symbol == '>':
            return all(it > threshold for it in list)
        elif symbol == '<':
            return all(it < threshold for it in list)
        else:
            return "Invalid symbol check_threshold"

    def set_standardized_values(self):
        pose = self.skeletons[self.frame_async]
        
        l_shld = pose[self.L_SHOULDER]
        l_wst = pose[self.L_WRIST]
        r_shld = pose[self.R_SHOULDER]
        r_wst = pose[self.R_WRIST]
        # print(l_shld, l_wst, r_shld, r_wst)

        stnd = np.linalg.norm([[l_shld.x-r_shld.x, l_shld.y-r_shld.y]])

        l_y = (l_wst.y - l_shld.y)/stnd
        l_x = (l_wst.x - l_shld.x)/stnd
        r_y = (r_wst.y - r_shld.y)/stnd
        r_x = (r_wst.x - r_shld.x)/stnd

        self.all_ys.append([l_y, r_y])
        self.all_xs.append([l_y, r_y])

        self.stnd_ys, self.stnd_xs =  [l_y, r_y],  [l_x, r_x]

    def detect_moving(self):
        if self.frame_async < 30:
            return True

        ske_temp = self.skeletons[self.frame_async-30:self.frame_async]
        lx,ly,rx,ry = [],[],[],[]

        for pose in ske_temp:
            lx.append(pose[self.L_SHOULDER].x)
            ly.append(pose[self.L_SHOULDER].y)
            rx.append(pose[self.R_SHOULDER].x)
            ry.append(pose[self.R_SHOULDER].y)
        
        motion_cond = self.check_threshold([np.var(lx), np.var(ly), np.var(rx), np.var(ry)], .001, '>')
        y_cond = self.check_threshold(self.stnd_ys, -0.2, '<') or self.check_threshold(self.stnd_ys, .8, '>')
        return motion_cond or y_cond

    def skeletonize(self, image, imu_read):
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)
        # mediapipe.python.solutions.drawing_utils.plot_landmarks(results.pose_landmarks)

        if results.pose_landmarks is not None:
            self.skeletons.append(results.pose_landmarks.landmark)
            self.images.append(image)
            self.imu_readings.append(imu_read)
            self.times.append(time.time)
            self.frame_async = self.frame_async + 1

            return True
        else:
            return False