import cv2
import mediapipe as mp
import serial
import math
import time

# Connect to Arduino over serial (Linux port: /dev/ttyXXXX, baudrate: 9600)
# If windows is used could be something like comX (X is a number)
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # give the Arduino time to reboot

# Open webcam (index 0 in my case usually the default camera; change to 1 if needed)
camera = cv2.VideoCapture(0)

# Setup Mediapipe bodykeypoints solution
mp_keypoints = mp.solutions.pose

def euclid(p1, p2):
    """Return Euclidean distance between two 2D points p1, p2."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def pt(lmk, idx, W, H):
    """
    Convert a normalized landmark (0..1, 0..1) into image pixels.
    - lmk: the results.pose_landmarks object
    - idx: landmark index (e.g., LEFT_SHOULDER)
    - W,H: frame width, height
    """
    p = lmk.landmark[idx]
    return (int(p.x * W), int(p.y * H))

def elbow_angle_deg(shoulder, elbow, wrist):
    """Angle at the elbow in degrees: 180° = straight; smaller = more bent."""
    ux, uy = shoulder[0] - elbow[0], shoulder[1] - elbow[1] # Vector starting from shoulder towrds elbow (x,y)
    vx, vy = wrist[0] - elbow[0], wrist[1] - elbow[1] # Vector starting from wrist towrds elbow
    nu = math.hypot(ux, uy); nv = math.hypot(vx, vy) # Length of vector is the sqrt(x²,y²)
    if nu == 0 or nv == 0:
        return None
   # cosine of angle via dot product formula
    cosang = (ux * vx + uy * vy) / (nu * nv) # cosine is: u.v/∥u∥∥v∥
    cosang = max(-1.0, min(1.0, cosang))  # clamp for safety
    return math.degrees(math.acos(cosang))

def linmap(x, a0, a1, b0, b1):
    """
    Linearly map x from source range [a0,a1] to target range [b0,b1],
    and clamp to the target range.
    """
    if a1 == a0:
        return b0                     # avoid divide-by-zero if misconfigured
    t = (x - a0) / (a1 - a0)          # normalize x to [0..1]
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)  # clamp
    return b0 + t * (b1 - b0)         # scale to target range

# Tunable parameters (behavior controls)
angle_smooth = None       # holds filtered elbow angle (deg)
last_send_t = 0.0         # last time we sent a serial update
SEND_HZ = 20.0            # send rate to Arduino (times per second)

SERVO_MIN, SERVO_MAX = 20, 170 # keep away from hard stops; swap to invert
THETA_MIN, THETA_MAX = 29.5, 166.5  # Tested on my body and position
ALPHA = 0.3                    # low-pass filter coefficient tune it to change servo react speed.

# main loop
with mp_keypoints.Pose(static_image_mode=False,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5) as keypoints:
    while True:
        ok, frame = camera.read()   # grab a frame from webcam
        if not ok:
            break                   # end if camera read failed

        H, W, C = frame.shape      # image height/width/channels
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # MediaPipe expects RGB
        results = keypoints.process(rgb_frame)              # run pose estimation

        if results.pose_landmarks:   # only proceed if a person/pose was found
            lmk = results.pose_landmarks

            # Get LEFT shoulder/elbow/wrist in pixel coords
            p11 = pt(lmk, mp_keypoints.PoseLandmark.LEFT_SHOULDER.value, W, H)
            p13 = pt(lmk, mp_keypoints.PoseLandmark.LEFT_ELBOW.value,    W, H)
            p15 = pt(lmk, mp_keypoints.PoseLandmark.LEFT_WRIST.value,    W, H)

            # metric for debug
            shoulder_wrist_d = euclid(p11, p15)

            # Compute instantaneous elbow angle (deg) 
            theta = elbow_angle_deg(p11, p13, p15)

            if theta is not None:
                # Smooth it (single-pole low-pass filter)
                if angle_smooth is None:
                    angle_smooth = theta
                else:
                    angle_smooth = (1 - ALPHA) * angle_smooth + ALPHA * theta

                #  Map human angle → servo angle (deg) 
                servo_angle = int(round(
                    linmap(angle_smooth, THETA_MIN, THETA_MAX,
                           SERVO_MIN, SERVO_MAX)
                ))

                # Rate-limit serial writes to Arduino
                now = time.time()
                if (now - last_send_t) >= (1.0 / SEND_HZ):
                    try:
                        arduino.write(f"{servo_angle}\n".encode("ascii"))
                    except Exception as e:
                        print("Serial error:", e)
                    last_send_t = now

                # Draw visualizations on the frame
                for p in (p11, p13, p15):
                    cv2.circle(frame, p, 5, (0, 255, 0), -1)  # landmarks
                cv2.line(frame, p11, p13, (255, 0, 0), 2)     # upper arm
                cv2.line(frame, p13, p15, (255, 0, 0), 2)     # forearm
                cv2.line(frame, p11, p15, (255, 0, 0), 2)     # shoulder→wrist chord

                # On window's text
                cv2.putText(frame, f"Dist SW: {shoulder_wrist_d:.1f}px",
                            (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(frame, f"Elbow: {theta:.1f} deg  Servo: {servo_angle}",
                            (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 120, 255), 2)
            
        cv2.imshow("Arm Tracking", frame)           # show live debug window
        if cv2.waitKey(10) == 27:          # ESC key to quit
            break
       
camera.release()
cv2.destroyAllWindows()
