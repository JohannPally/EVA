import cv2
import mediapipe as mp
import time
import matplotlib.pyplot as plt
import mediapipe.python.solutions.drawing_utils
from matplotlib.axis import Axis
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

accum_results = []
accum_images = []
times = []

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('correct.MOV')

with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
    timeout = time.time() + 40
    while cap.isOpened() and (time.time() < timeout):
        success, image = cap.read()
        if not success:
            # If loading a video, use 'break'
            # If streaming from cam instead use 'continue'.
            break

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        # mediapipe.python.solutions.drawing_utils.plot_landmarks(results.pose_landmarks)

        if results.pose_landmarks is not None:
            accum_results.append((time.time(), results.pose_landmarks.landmark))
            accum_images.append(image)

        # Draw the pose annotation on the image.
        # image.flags.writeable = True
        # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # mp_drawing.draw_landmarks(
        #     image,
        #     results.pose_landmarks,
        #     mp_pose.POSE_CONNECTIONS,
        #     landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        # # Flip the image horizontally for a selfie-view display.
        #
        # cv2.imshow('MediaPipe Pose', cv2.flip(image,0))
        # if cv2.waitKey(5) & 0xFF == 27:
        #   break

cap.release()

# cv2.destroyAllWindows()

"""
=============================PLOTTING 3D==============================
def get_image_at_i(i):
    return accum_images[i]


def get_landmarks_at_i(i):
    # 0 tuple, 1 set of points, 0 is a point,
    # val = accum_results[0][1][0].x
    pse = get_key_points(i)
    x_out = []
    y_out = []
    z_out = []
    for pnt in pse:
        x_out.append(pnt.x)
        y_out.append(pnt.y)
        z_out.append(pnt.z)

    return z_out, x_out, y_out


def get_key_points(i):
    out = []
    pse = accum_results[i][1]
    out.append(pse[mp_pose.PoseLandmark.LEFT_WRIST])
    out.append(pse[mp_pose.PoseLandmark.LEFT_ELBOW])
    out.append(pse[mp_pose.PoseLandmark.LEFT_SHOULDER])
    out.append(pse[mp_pose.PoseLandmark.RIGHT_SHOULDER])
    out.append(pse[mp_pose.PoseLandmark.RIGHT_ELBOW])
    out.append(pse[mp_pose.PoseLandmark.RIGHT_WRIST])
    return out


def reset_axis_bounds(ax):
    ax.set_ylim3d(0, 1)
    ax.set_xlim3d(-.5, 1)
    ax.set_zlim3d(0, 1)
    ax.set_ylabel("y")
    ax.set_xlabel("x")
    ax.set_zlabel("z")


fig = plt.figure()
# TODO need to separate plts, consider GridSpec
ax = fig.add_subplot(111, projection='3d')
# im = fig.add_subplot(111)
reset_axis_bounds(ax)

n = len(accum_results)
axslide = plt.axes([0.25, 0.15, 0.65, 0.03])
slide = Slider(axslide, 'Time', 0, n - 1, n / 2)


def update(val):
    ax.clear()
    # im.clear()
    reset_axis_bounds(ax)
    sl_val = int(slide.val)
    x_new, y_new, z_new = get_landmarks_at_i(sl_val)
    ax.plot(x_new, y_new, z_new)
    ax.scatter(x_new, y_new, z_new, edgecolors = ["k", "c", "m", "m", "c", "k"])
    # im.imshow(get_image_at_i(sl_val))


slide.on_changed(update)
plt.show()
"""

"""
==================================PLOTTING 2D==================================
def get_times():
    out = []
    for pair in accum_results:
        out.append(pair[0])
    return out


def get_xs():
    out = [[], [], [], [], [], []]
    for i in range(len(accum_results)):
        pse = accum_results[i][1]
        out[0].append(pse[mp_pose.PoseLandmark.LEFT_WRIST].x)
        out[1].append(pse[mp_pose.PoseLandmark.LEFT_ELBOW].x)
        out[2].append(pse[mp_pose.PoseLandmark.LEFT_SHOULDER].x)
        out[3].append(pse[mp_pose.PoseLandmark.RIGHT_SHOULDER].x)
        out[4].append(pse[mp_pose.PoseLandmark.RIGHT_ELBOW].x)
        out[5].append(pse[mp_pose.PoseLandmark.RIGHT_WRIST].x)
    return out


def get_ys():
    out = [[], [], [], [], [], []]
    for i in range(len(accum_results)):
        pse = accum_results[i][1]
        out[0].append(pse[mp_pose.PoseLandmark.LEFT_WRIST].y)
        out[1].append(pse[mp_pose.PoseLandmark.LEFT_ELBOW].y)
        out[2].append(pse[mp_pose.PoseLandmark.LEFT_SHOULDER].y)
        out[3].append(pse[mp_pose.PoseLandmark.RIGHT_SHOULDER].y)
        out[4].append(pse[mp_pose.PoseLandmark.RIGHT_ELBOW].y)
        out[5].append(pse[mp_pose.PoseLandmark.RIGHT_WRIST].y)
    return out


def get_zs():
    out = [[], [], [], [], [], []]
    for i in range(len(accum_results)):
        pse = accum_results[i][1]
        out[0].append(pse[mp_pose.PoseLandmark.LEFT_WRIST].z)
        out[1].append(pse[mp_pose.PoseLandmark.LEFT_ELBOW].z)
        out[2].append(pse[mp_pose.PoseLandmark.LEFT_SHOULDER].z)
        out[3].append(pse[mp_pose.PoseLandmark.RIGHT_SHOULDER].z)
        out[4].append(pse[mp_pose.PoseLandmark.RIGHT_ELBOW].z)
        out[5].append(pse[mp_pose.PoseLandmark.RIGHT_WRIST].z)
    return out


def plt_mult(tms, st):
    for s in st:
        plt.plot(tms, s)


fig = plt.figure()
plt_mult(get_times(), get_ys())
plt.show()

fig = plt.figure()
plt_mult(get_times(), get_xs())
plt.show()

fig = plt.figure()
plt_mult(get_times(), get_zs())
plt.show()
"""


def get_times():
    out = []
    for pair in accum_results:
        out.append(pair[0])
    return out


def get_left_arm_angle(i):
    pse = accum_results[i][1]
    wri = pse[mp_pose.PoseLandmark.LEFT_WRIST]
    wri_pt = np.array([wri.x, wri.y, wri.z])
    elb = pse[mp_pose.PoseLandmark.LEFT_ELBOW]
    elb_pt = np.array([elb.x, elb.y, elb.z])
    sho = pse[mp_pose.PoseLandmark.LEFT_SHOULDER]
    sho_pt = np.array([sho.x, sho.y, elb.z])

    forearm_vec = wri_pt - elb_pt
    upperarm_vec = sho_pt - elb_pt

    num = np.dot(forearm_vec, upperarm_vec)
    den = (np.linalg.norm(forearm_vec) * np.linalg.norm(upperarm_vec))
    cos_theta = num / den
    return np.degrees(np.arccos(cos_theta))


def get_left_angles():
    out = []
    for i in range(len(accum_results)):
        out.append(get_left_arm_angle(i))
    return out


angs = get_left_angles()
tms = get_times()

fig = plt.figure()
plt.plot(tms, angs)
plt.show()

"""
===============================DUMPING TO JSON===============================
dump_json = json.dumps(accum_results, indent = 4)
with open("test_data.csv", "w") as outfile:
    writer = csv.writer(outfile)
    for i in range(len(times)):
        out = []
        out.append(str(times[i]))
        for lm in accum_results[i]:
            out.append(str(lm))
        writer.writerow(out)
"""
