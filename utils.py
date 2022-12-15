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
        self.bottom_threshold = .3

        self.reset_fsm()

        self.all_ys = []
        self.all_xs = []

        self.FLEXION_BOTTOM_ERROR_CODE = 1 #'caution, at the bottom of your rep, bend your arms to 90 degrees'
        self.FLEXION_TOP_ERROR_CODE = 2 #'caution, at the top of your rep, do not lock your elbows, allow for a slight bend'
        self.TILT_DOWN_ERROR_CODE = 3 #'caution, bar tilted while going down'
        self.TILT_UP_ERROR_CODE = 4 #'caution, bar tilted while going up'
        self.INSTABILITY_ERROR_CODE = 5 #'caution, the motion is shaky'
        self.ROTATOIN_ERROR_CODE = 6 #'caution, your wrist might be rotating'

    def reset_fsm(self):
        self.moving_flag = True
        self.moving_sf = 0
        self.down_flag = False
        self.hold_flag = False
        self.down_sf = -1
        self.up_sf = -1
        self.hold_sf = -1

    # ***************MAIN FUNCTIONS***************
    # ============================================
    def update(self, image, imu_readings):
        if self.skeletonize(image, imu_readings):
            # print('skeletonize successful')
            label, window = self.update_activity_segmentation()
            return self.error_check(label, window)
        else:
            # print('skeletonize unsuccessful')
            return None

    def error_check(self, label, window):
        if label is None:
            return True

        start_frame, end_frame = window
        alert = False

        # match label:
            # case 'down':

        if (label == "down"):
            if self.alert_tilt():
                return self.TILT_DOWN_ERROR_CODE
            if self.alert_instability():
                return self.INSTABILITY_ERROR_CODE
            if self.alert_rotation():
                return self.ROTATOIN_ERROR_CODE
        # case 'up':
        elif (label == "up"):
            if self.alert_tilt():
                return self.TILT_UP_ERROR_CODE
            if self.alert_instability():
                return self.INSTABILITY_ERROR_CODE
            if self.alert_rotation():
                return self.ROTATOIN_ERROR_CODE
        # case 'hold_top':

        elif (label == "hold_top"):
            if self.alert_flexion_top(start_frame, end_frame):
                return self.FLEXION_TOP_ERROR_CODE
            if self.alert_tilt():
                return self.TILT_UP_ERROR_CODE
            if self.alert_rotation():
                return self.ROTATOIN_ERROR_CODE
        # case 'hold_bottom':

        elif (label == "hold_bottom"):
            if self.alert_flexion_bottom(start_frame, end_frame):
                return self.FLEXION_BOTTOM_ERROR_CODE
            if self.alert_tilt():
                return self.TILT_UP_ERROR_CODE
            if self.alert_rotation():
                return self.ROTATOIN_ERROR_CODE

        return None

    def plot_segmentation(self):
        plt.plot(np.arange(len(self.all_ys)), self.all_ys)
        plt.axhline(y=self.top_threshold, color='r', linestyle='-')
        plt.axhline(y=self.bottom_threshold, color='b', linestyle='-')
        plt.ylim(-.5, 1)

        for down in self.down_windows:
            plt.axvspan(down[0], down[1], color='red')

        for up in self.up_windows:
            plt.axvspan(up[0], up[1], color='lime')

        for hold in self.hold_windows:
            plt.axvspan(hold[0], hold[1], color='purple')

        plt.show()
    # ============================================

    # ================IMU ERROR HELPER FUNCTIONS=============
    def alert_instability(self):
        # return False
        data_mat = self.get_data_mat()

        gr_x = data_mat[:, 3].reshape(1, 10)
        gr_y = data_mat[:, 4].reshape(1, 10)

        if (np.var(gr_x) > 20 or np.var(gr_y) > 20):
            return True
        else:
            return False

    def alert_rotation(self):
        # return False
        data_mat = self.get_data_mat()

        xl_y = data_mat[:, 1].reshape(1, 10)
        avg_xl_y = np.average(xl_y)
        orient_x = data_mat[:, 9].reshape(1, 10)
        avg_orient_x = np.average(orient_x)

        if (avg_orient_x < -95 and avg_xl_y > -1.01):
            return True

        if (avg_orient_x > -85 and avg_xl_y > -1.01):
            return True
        return False

    def alert_tilt(self):
        # return False
        data_mat = self.get_data_mat()
        xl_x = data_mat[:, 0].reshape(1, 10)
        avg_xl_x = np.average(xl_x)
        orient_y = data_mat[:, 10].reshape(1, 10)
        avg_orient_y = np.average(orient_y)
        if (avg_orient_y < -2 and avg_xl_x > 0.2):
            return True

        if (avg_orient_y > 2 and avg_xl_x < -0.2):
            return True

        return False
    # ================================================
    # ==============IMAGE ERROR HELPER FUNCS===========
    def get_data_mat(self):
        return np.vstack((
                              self.imu_readings[-10],
                              self.imu_readings[-9],
                              self.imu_readings[-8],
                              self.imu_readings[-7],
                              self.imu_readings[-6],
                              self.imu_readings[-5],
                              self.imu_readings[-4],
                              self.imu_readings[-3],
                              self.imu_readings[-2],
                              self.imu_readings[-1]))

    def alert_flexion_top(self, start, end):
        angles = []
        for frame in range(start, end):
            angles.append(self.get_flexion_angles(frame))

        np.sort(angles)
        for i in range(min(10, len(angles))):
            if any(a > 170 for a in angles[-1*i]):
                return True
        return False

    def alert_flexion_bottom(self, start, end):
        angles = []
        for frame in range(start, end):
            angles.append(self.get_flexion_angles(frame))

        np.sort(angles)
        for i in range(min(10, len(angles))):
            if any(a > 100 for a in angles[i]):
                return True
        return False

    def get_flexion_angles(self, frame):
        pose = self.skeletons[frame]
        r_wrst = pose[self.R_WRIST]
        r_elb = pose[self.R_ELBOW]
        r_shld = pose[self.R_SHOULDER]

        l_wrst = pose[self.L_WRIST]
        l_elb = pose[self.L_ELBOW]
        l_shld = pose[self.L_SHOULDER]

        l_fore = np.array([l_wrst.x-l_elb.x, l_wrst.y-l_elb.y])
        l_up = np.array([l_shld.x-l_elb.x, l_shld.y-l_elb.y])

        r_fore = np.array([r_wrst.x-r_elb.x, r_wrst.y-r_elb.y])
        r_up = np.array([r_shld.x-r_elb.x, r_shld.y-r_shld.y])

        l_num = np.dot(l_fore, l_up)
        r_num = np.dot(r_fore, r_up)

        l_den = np.linalg.norm(l_fore)*np.linalg.norm(l_up)
        r_den = np.linalg.norm(r_fore)*np.linalg.norm(r_up)

        l_angle = np.arccos(l_num/l_den)
        r_angle = np.arccos(r_num/r_den)

        return [l_angle, r_angle]
    # ================================================

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

        # sitting at bench
        if self.detect_moving():
            print('moving, reset flags')
            self.reset_fsm()
            return None, None

        # going down
        if self.down_flag:
            # finish holding at the top
            if self.hold_flag and self.check_threshold(self.stnd_ys, self.top_threshold, '<'):
                print('start going down')
                self.hold_flag = False
                self.down_sf = self.frame_async
                temp_wind = [self.hold_sf, self.frame_async]
                self.hold_windows.append(temp_wind)
                return 'hold_top', temp_wind

            # reach bottom of rep
            elif not self.hold_flag and self.check_threshold(self.stnd_ys, self.bottom_threshold, '<'):
                print('stop going down')
                self.down_flag = False
                self.hold_flag = True
                self.hold_sf = self.frame_async
                temp_wind = [self.down_sf, self.frame_async]
                self.down_windows.append(temp_wind)
                print('down', temp_wind)
                return 'down', temp_wind

        # going up
        else:
            # finish holding at bottom
            if self.hold_flag and self.check_threshold(self.stnd_ys, self.bottom_threshold, '>'):
                print('start going up')
                self.hold_flag = False
                self.up_sf = self.frame_async
                temp_wind = [self.hold_sf, self.frame_async]
                self.hold_windows.append(temp_wind)
                return 'hold_bottom', temp_wind

            # reach top of rep
            elif not self.hold_flag and self.check_threshold(self.stnd_ys, self.top_threshold, '>'):
                print('stop going up')
                self.down_flag = True
                self.hold_flag = True
                self.hold_sf = self.frame_async

                if self.up_sf != -1:
                    temp_wind = [self.up_sf, self.frame_async]
                    self.up_windows.append(temp_wind)
                    print('up', temp_wind)
                    return 'up', temp_wind

        return None, None

    # ========SUPPORT FUNCTIONS============

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

        self.stnd_ys, self.stnd_xs = [l_y, r_y],  [l_x, r_x]

    def detect_moving(self):
        if self.frame_async < 30:
            return True

        ske_temp = self.skeletons[self.frame_async-30:self.frame_async]
        lx, ly, rx, ry = [], [], [], []

        for pose in ske_temp:
            lx.append(pose[self.L_SHOULDER].x)
            ly.append(pose[self.L_SHOULDER].y)
            rx.append(pose[self.R_SHOULDER].x)
            ry.append(pose[self.R_SHOULDER].y)

        motion_cond = self.check_threshold(
            [np.var(lx), np.var(ly), np.var(rx), np.var(ry)], .001, '>')
        y_cond = self.check_threshold(self.stnd_ys, self.bottom_threshold-1,
                                      '<') or self.check_threshold(self.stnd_ys, self.top_threshold+1, '>')
        return motion_cond or y_cond

    def skeletonize(self, image, imu_read):
        if image is None:
            return False
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
