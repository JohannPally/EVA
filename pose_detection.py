import cv2
import mediapipe as mp
import time
import matplotlib.pyplot as plt
import mediapipe.python.solutions.drawing_utils
from mpl_toolkits.mplot3d import Axes3D

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

accum_results = []
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
          print("Ignoring empty camera frame.")
          # If loading a video, use 'break' instead of 'continue'.
          continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        # mediapipe.python.solutions.drawing_utils.plot_landmarks(results.pose_landmarks)

        if results.pose_landmarks is not None:
            times.append(time.time())
            accum_results.append(results.pose_landmarks.landmark)

        # Draw the pose annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        # Flip the image horizontally for a selfie-view display.

        cv2.imshow('MediaPipe Pose', cv2.flip(image,0))
        if cv2.waitKey(5) & 0xFF == 27:
          break

cap.release()
cv2.destroyAllWindows()


# dump_json = json.dumps(accum_results, indent = 4)
# with open("test_data.csv", "w") as outfile:
#     writer = csv.writer(outfile)
#     for i in range(len(times)):
#         out = []
#         out.append(str(times[i]))
#         for lm in accum_results[i]:
#             out.append(str(lm))
#         writer.writerow(out)