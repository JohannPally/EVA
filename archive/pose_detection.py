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
cap = cv2.VideoCapture('far.MOV')

with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        smooth_landmarks=True,
        smooth_segmentation=True
        ) as pose:
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
        
        
        #Draw the pose annotation on the image.
        # image.flags.writeable = True
        # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # mp_drawing.draw_landmarks(
        #     image,
        #     results.pose_landmarks,
        #     mp_pose.POSE_CONNECTIONS,
        #     landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        # # Flip the image horizontally for a selfie-view display.
        
        # cv2.imshow('MediaPipe Pose', cv2.flip(image,0))
        # if cv2.waitKey(1) & 0xFF == 27:
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
#==================================PLOTTING 2D==================================


labels = ["L Wrist", "L Elbow", "L Shoudler", "R Shoulder", "R Elbow", "R Wrist"]



def check_threshold(l, th):
        return all(it < th for it in l)


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


def plt_mult(tms, st, labs):
    for i in range(len(st)):
        plt.plot(tms, st[i], label = labs[i])
    plt.legend()

def plt_highlights(hs):
    for h in hs:
        plt.axvspan(h[0], h[1], color='red', alpha=0.5)


def plt_vars():
    #Getting the left shoulder coordinates
    ls_xs = get_xs()[2]
    ls_ys = get_ys()[2]
    rs_xs = get_xs()[3]
    rs_ys = get_ys()[3]
    # ls_zs = get_zs()[2]

    v_lx = []
    v_ly = []
    v_rx = []
    v_ry = []
    # v_z = []

    n = len(ls_xs)
    high_ranges = []
    prev = 0
    flag = True

    w_len = 30
    for i in range(len(ls_xs) - w_len + 1):
        nlx = np.var(ls_xs[i:i+w_len])
        nly = np.var(ls_ys[i:i + w_len])
        nrx = np.var(rs_xs[i:i+w_len])
        nry = np.var(rs_ys[i:i + w_len])
        
        if flag and check_threshold([nlx, nly, nrx, nry], .001):
            flag = False
            high_ranges.append((prev, i))

        elif not flag and not check_threshold([nlx, nly, nrx, nry], .001):
            flag = True
            prev = i

        v_lx.append(nlx)
        v_ly.append(nly)
        v_rx.append(nrx)
        v_ry.append(nry)
    
    high_ranges.append((prev,n-1))
    print(high_ranges)

    ts = np.arange(0, len(v_lx))

    plt_mult(ts, [v_lx, v_ly, v_rx, v_ry], ["L Shoulder x", "L Shoulder y", "R Shoulder x", "R Shoulder y"])
    plt_highlights(high_ranges)

# fig = plt.figure()
# plt_vars()
# plt.xlabel("Window (30 Frames)")
# plt.ylabel("Variance")
# plt.title("Variance Activity Segmentation Plot")
# plt.show()

# fig = plt.figure()
# plt_mult(get_times(), get_ys(), labels)
# plt.xlabel("Frame")
# plt.ylabel("Window Ratio")
# plt.title("y Motion Plot")

# plt.show()

# fig = plt.figure()
# plt_mult(get_times(), get_xs(), labels)
# plt.xlabel("Frame")
# plt.ylabel("Window Ratio")
# plt.title("x Motion Plot")
# plt.show()

# fig = plt.figure()
# plt_mult(get_times(), get_zs(), labels)
# plt.xlabel("Frame")
# plt.ylabel("Window Ratio")
# plt.title("z Motion Plot")
# plt.show()


"""

#==============================PLOTTING ANGLE========================
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

def get_right_arm_angle(i):
    pse = accum_results[i][1]
    wri = pse[mp_pose.PoseLandmark.RIGHT_WRIST]
    wri_pt = np.array([wri.x, wri.y, wri.z])
    elb = pse[mp_pose.PoseLandmark.RIGHT_ELBOW]
    elb_pt = np.array([elb.x, elb.y, elb.z])
    sho = pse[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    sho_pt = np.array([sho.x, sho.y, elb.z])

    forearm_vec = wri_pt - elb_pt
    upperarm_vec = sho_pt - elb_pt

    num = np.dot(forearm_vec, upperarm_vec)
    den = (np.linalg.norm(forearm_vec) * np.linalg.norm(upperarm_vec))
    cos_theta = num / den
    return np.degrees(np.arccos(cos_theta))


def get_angles():
    le = []
    ri = []
    for i in range(len(accum_results)):
        le.append(get_left_arm_angle(i))
        ri.append(get_right_arm_angle(i))
    return le, ri

def plt_first_deriv(xs,ys,n):
    #Getting the shoulder coordinates
    stnd = get_stnd(xs,ys,n)

    ls_xs = [temp/stnd for temp in xs[2]]
    ls_ys = [temp/stnd for temp in ys[2]]
    rs_xs = [temp/stnd for temp in xs[3]]
    rs_ys = [temp/stnd for temp in ys[3]]

    s_lsx = []
    s_lsy = []
    s_rsx = []
    s_rsy = []

    w_len = 10
    for i in range(len(ls_xs) - w_len + 1):
        s_lsx.append(np.mean(ls_xs[i:i+w_len]))
        s_lsy.append(np.mean(ls_ys[i:i + w_len]))
        s_rsx.append(np.mean(rs_xs[i:i+w_len]))
        s_rsy.append(np.mean(rs_ys[i:i + w_len]))
    
    dx = 10

    d_lsx = np.diff(s_lsx)/dx
    d_lsy = np.diff(s_lsy)/dx
    d_rsx = np.diff(s_rsx)/dx
    d_rsy = np.diff(s_rsy)/dx

    ts = np.arange(0, len(d_lsx))
    # d_labels = ["L Shoudler x", "L Shoulder y", "R Shoulder x", "R Shoulder y"]
    # plt_mult(ts, (d_lsx, d_lsy, d_rsx, d_rsy), d_labels)
    d_labels = ["L Shoulder y", "R Shoulder y"]
    plt_mult(ts, (d_lsy, d_rsy), d_labels)


fig = plt.figure()
plt_mult(get_times(), get_angles(),["L Arm Theta", "R Arm Theta"])
plt.xlabel("Frame")
plt.ylabel("Theta")
plt.title("Forearm, Upperarm Theta Plot")
plt.axhline(y = 90, color = 'r', linestyle = '-')
plt.show()


def get_times():
    out = []
    for pair in accum_results:
        out.append(pair[0])
    return out

"""

# TODO consider standardizing using shoulder length.

"""
#==================================PLOTTING 2D==================================
"""

def get_times(arr = accum_results):
    return np.arange(len(arr))

def plt_mult(tms, st, labs, tit = "", axes = ["",""], xlim = None, ylim = None, show = True):
    plt.figure()
    for i in range(len(st)):
        plt.plot(tms, st[i], label = labs[i])
    plt.legend()
    plt.title(tit)
    plt.xlabel(axes[0])
    plt.ylabel(axes[1])
    if xlim is not None:
        plt.xlim(xlim[0],xlim[1])
    if ylim is not None:
        plt.ylim(ylim[0],ylim[1])
    if show:
        plt.show()

def plt_x_highlights(hs, color='red'):
    for h in hs:
        plt.axvspan(h[0], h[1], color=color, alpha=0.5)

def plt_y_highlights(hs, color='purple'):
    for h in hs:
        plt.axhspan(h[0], h[1], color=color, alpha=0.5)


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


def get_shldr_dist():
    ls_xs = get_xs()[2]
    ls_ys = get_ys()[2]
    rs_xs = get_xs()[3]
    rs_ys = get_ys()[3]

    stnd = []
    n = len(ls_xs)
    for i in range(n):
        l_shld = np.array([ls_xs[i], ls_ys[i]])
        r_shld = np.array([rs_xs[i], rs_ys[i]])

        stnd.append(np.linalg.norm(l_shld-r_shld))
    return stnd

def get_stnd(xs,ys,n):
    lx = xs[2]
    ly = ys[2]
    rx = xs[3]
    ry = ys[3]

    sum = 0
    for i in range(n//2-2, n//2+3):
        l_shld = np.array([lx[i], ly[i]])
        r_shld = np.array([rx[i], ry[i]])

        sum += np.linalg.norm(l_shld-r_shld)

    return sum/5

def check_threshold(lst, th, lthan = True):
    if lthan:
        return all(it < th for it in lst)
    else:
        return all(it > th for it in lst)

def get_moving(xs, ys, n):
    #Getting the left shoulder coordinates
    ls_xs = xs[2]
    ls_ys = ys[2]
    rs_xs = xs[3]
    rs_ys = ys[3]
    # ls_zs = get_zs()[2]

    v_lx = []
    v_ly = []
    v_rx = []
    v_ry = []
    # v_z = []

    high_ranges = []
    prev = 0
    flag = True

    w_len = 30
    for i in range(len(ls_xs) - w_len + 1):
        nlx = np.var(ls_xs[i:i+w_len])
        nly = np.var(ls_ys[i:i + w_len])
        nrx = np.var(rs_xs[i:i+w_len])
        nry = np.var(rs_ys[i:i + w_len])
        
        if flag and check_threshold([nlx, nly, nrx, nry], .001):
            flag = False
            high_ranges.append((prev, i))

        elif not flag and not check_threshold([nlx, nly, nrx, nry], .001):
            flag = True
            prev = i

        v_lx.append(nlx)
        v_ly.append(nly)
        v_rx.append(nrx)
        v_ry.append(nry)
    
    high_ranges.append((prev,n-1))
    return high_ranges

def get_stnd_x_y_diff(xs, ys, n):
    stnd = get_stnd(xs,ys,n)
    print(stnd)

    x_lshldr = xs[2]
    x_lwrst = xs[0]
    y_lshldr = ys[2]
    y_lwrst = ys[0]

    x_rshldr = xs[3]
    x_rwrst = xs[5]
    y_rshldr = ys[3]
    y_rwrst = ys[5]

    olx = []
    oly = []
    orx = []
    ory = []

    
    for i in range(n):
        print(y_lwrst[i],y_lshldr[i])
        olx.append((x_lwrst[i]-x_lshldr[i])/stnd)
        oly.append((y_lwrst[i]-y_lshldr[i])/stnd)
        orx.append((x_rwrst[i]-x_rshldr[i])/stnd)
        ory.append((y_rwrst[i]-y_rshldr[i])/stnd)
    
    return (olx, oly, orx, ory)

    # plt_mult(tms=np.arange(n), st=[oly,ory], labs=["L Wrist - L Shoulder", "R Wrist - R Shoulder"],
    #  tit="Standardized y Difference (Close Video)", axes=["Frame", "Difference"], ylim=[-.5,1])
    # plt_mult(tms=np.arange(n), st=[olx,orx], labs=["L Wrist - L Shoulder", "R Wrist - R Shoulder"],
    #  tit="Standardized x Difference (Close Video)", axes=["Frame", "Difference"], ylim=[-1,1])


def activity_segment(xs, ys, n):
    moving = get_moving(xs, ys, n)
    __, sly, __, sry = get_stnd_x_y_diff(xs, ys, n)

    # NOTE for direction, down = True, up = False
    direction = False
    hold = False
    down = []
    up = []
    d_s, d_e, u_s, u_e = -1, -1, -1, -1
    d_threshold = .5
    u_threshold = .2

    for j in range(len(moving)-1):
        sitting = (moving[j][1], moving[j+1][0])
        for i in range(*sitting):
            sys = [sly[i], sry[i]]

            if direction:
                if hold and check_threshold(sys, d_threshold, True):
                    # print(2)
                    d_s = i
                    hold = False
                elif not hold and check_threshold(sys, u_threshold, True):
                    # print(3)
                    d_e = i
                    hold = True
                    direction = False
                    down.append((d_s, d_e))
            else:
                if hold and check_threshold(sys, u_threshold, False):
                    # print(4)
                    u_s = i
                    hold = False
                elif not hold and check_threshold(sys, d_threshold, False):
                    # print(1)
                    u_e = i
                    hold = True
                    direction = True
                    if u_s > -1:
                        up.append((u_s, u_e))

    #NOTE popping ending down motion
    down = down[:len(up)]

    return sly, sry, moving, down, up

def plt_activity_segment(xs, ys, n):
    sly, sry, moving, down, up = activity_segment(xs,ys,n)
    print(down)
    print(up)

    plt_mult(tms=np.arange(n), st=[sly, sry], labs=["L Wrist - L Shoulder", "R Wrist - R Shoulder"],
        tit="Activity Segmentation Plotting", axes=["Frame", "Difference"], ylim=[-.5,1], show = False)
    plt_x_highlights(moving, 'red')
    plt_x_highlights(up, 'cyan')
    plt_x_highlights(down, 'lime')
    plt.show()

xs = get_xs()
ys = get_ys()
zs = get_zs()
times = get_times()
n = len(xs[0])

plt_activity_segment(xs, ys, n)


# print(xs)
# print(n)

# plt_stnd_wrst_shldr_x_y_diff(xs, ys, n)

# fig = plt.figure()
# plt_vars()
# plt.show()

# fig = plt.figure()
# plt_mult(get_times(), get_ys())
# plt.show()

# fig = plt.figure()
# plt_mult(get_times(), get_xs())
# plt.show()

# fig = plt.figure()
# plt_mult(get_times(), get_zs())
# plt.show()

# fig = plt.figure()
# plt.plot(get_times(), get_shldr_dist())
# plt.show()

# fig = plt.figure()
# plt_first_deriv(xs,ys,n)
# plt.ylim(-.001,.001)
# plt.xlabel("Frame")
# plt.ylabel("d/dt")
# plt.show()

"""

"""
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
