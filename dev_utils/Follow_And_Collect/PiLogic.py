import numpy as np
import cv2
import serial

# ================= CONFIG =================
MIN_RADIUS   = 10

# Serial — 3-byte packet: [ 0xFF | x_byte | y_byte ]
# x/y bytes are clamped to 0-254 so they can never equal the 0xFF header.
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)

# HSV range for tennis / green ball
lower_ball = np.array([25,  50,  50])
upper_ball = np.array([60, 255, 255])

# ================= CAMERA =================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# NOTE: The old `ser.write(bytes([255]))` activation byte has been removed.
# It was inserting an orphan 0xFF into the Arduino's read buffer, permanently
# misaligning every subsequent 2-byte packet.  The Arduino now re-syncs on
# its own by scanning for the 0xFF header in every 3-byte packet.

# Grab one frame to determine actual frame dimensions
grabbed, frame = cap.read()
if not grabbed:
    raise RuntimeError("Camera not available")

FRAME_H, FRAME_W = frame.shape[:2]
CENTERLINE_Y = FRAME_H // 2
CENTERLINE_X = FRAME_W // 2

print("=== Ball Tracker Started ===")
print(f"Frame: {FRAME_W}x{FRAME_H} | Centerline X: {CENTERLINE_X} | Centerline Y: {CENTERLINE_Y}")
print("Protocol: 3-byte packets  [ 0xFF | x_byte | y_byte ]")

while True:
    grabbed, frame = cap.read()
    if not grabbed:
        break

    display = frame.copy()

    # Draw crosshairs
    cv2.line(display, (CENTERLINE_X, 0),      (CENTERLINE_X, FRAME_H), (255, 255, 0), 1)
    cv2.line(display, (0, CENTERLINE_Y),       (FRAME_W, CENTERLINE_Y), (255, 255, 0), 1)

    # HSV processing
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_ball, upper_ball)
    mask = cv2.erode(mask,  None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    ball_detected = False
    bx, by = 0, 0

    if len(contours) > 0:
        largest = max(contours, key=cv2.contourArea)
        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        cx, cy, radius = int(cx), int(cy), int(radius)

        # Valid ball: radius large enough AND centre at or below horizontal centreline
        if radius > MIN_RADIUS and cy >= CENTERLINE_Y:
            ball_detected = True
            bx, by = cx, cy
            cv2.circle(display, (cx, cy), radius, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 3,      (0,   0, 255), -1)
            cv2.putText(display, f"({cx},{cy})", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ── Send 3-byte packet when ball detected ──────────────────────
    # Byte 0 : 0xFF header — lets Arduino re-sync on misalignment
    # Byte 1 : x mapped to 0-254 (never 0xFF so it can't look like a header)
    # Byte 2 : y mapped to 0-254
    # No packet sent = no ball; Arduino detects loss via timeout.
    if ball_detected:
        x_byte = int(np.interp(bx, [0, FRAME_W - 1], [0, 254]))
        y_byte = int(np.interp(by, [0, FRAME_H - 1], [0, 254]))
        ser.write(bytes([0xFF, x_byte, y_byte]))

    # Status overlay
    label = f"BALL  x={bx} y={by}" if ball_detected else "No ball (or above centreline)"
    color = (0, 255, 0) if ball_detected else (0, 0, 255)
    cv2.putText(display, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow("Mask",         mask)
    cv2.imshow("Ball Tracker", display)

    if cv2.waitKey(1) & 0xFF == 27:   # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
ser.close()