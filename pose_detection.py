import cv2
import mediapipe as mp
import time
import matplotlib.pyplot as plt
import mediapipe.python.solutions.drawing_utils
from matplotlib.axis import Axis
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

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
            # print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
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
cv2.destroyAllWindows()


# 0 tuple, 1 set of points, 0 is a point,
# val = accum_results[0][1][0].x

def get_image_at_i(i):
    return accum_images[i]

def get_landmarks_at_i(i):
    pse = accum_results[i][1]
    x_out = []
    y_out = []
    z_out = []
    for pnt in pse:
        x_out.append(pnt.x)
        y_out.append(pnt.y)
        z_out.append(pnt.z)

    return x_out, y_out, z_out


def reset_axis_bounds(ax):
    ax.set_ylim3d(-1,1)
    ax.set_xlim3d(0,1)
    ax.set_zlim3d(-1,1)
    ax.set_ylabel("y")
    ax.set_xlabel("x")
    ax.set_zlabel("z")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
im = fig.add_subplot(111)
reset_axis_bounds(ax)

n = len(accum_results)
axslide = plt.axes([0.25, 0.15, 0.65, 0.03])
slide = Slider(axslide, 'Time', 0, n-1, n/2)



def update(val):
    ax.clear()
    im.clear()
    reset_axis_bounds(ax)
    sl_val = int(slide.val)
    x_new, y_new, z_new = get_landmarks_at_i(sl_val)
    ax.scatter(x_new, y_new, z_new)
    im.imshow(get_image_at_i(sl_val))

slide.on_changed(update)
plt.show()

# dump_json = json.dumps(accum_results, indent = 4)
# with open("test_data.csv", "w") as outfile:
#     writer = csv.writer(outfile)
#     for i in range(len(times)):
#         out = []
#         out.append(str(times[i]))
#         for lm in accum_results[i]:
#             out.append(str(lm))
#         writer.writerow(out)
