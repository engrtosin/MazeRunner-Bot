import numpy as np
import cv2
import serial

# ================= CONFIG =================
FRAME_W = 480
FRAME_H = 360
MIN_RADIUS = 10

CENTERLINE_Y = FRAME_H // 2
CENTERLINE_X = FRAME_W // 2

# Serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)

# HSV range for tennis/green ball
lower_ball = np.array([25, 50, 50])
upper_ball = np.array([60, 255, 255])

# ================= CAMERA =================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(3, FRAME_W)
cap.set(4, FRAME_H)

# Send activation byte to Arduino
ser.write(bytes([255]))

print("=== Ball Tracker Started ===")
print(f"Frame: {FRAME_W}x{FRAME_H} | Centerline X: {CENTERLINE_X} | Centerline Y: {CENTERLINE_Y}")

while True:
    grabbed, frame = cap.read()
    if not grabbed:
        break

    display = frame.copy()

    # Draw crosshairs on display frame
    cv2.line(display, (CENTERLINE_X, 0), (CENTERLINE_X, FRAME_H), (255, 255, 0), 1)  # vertical - cyan
    cv2.line(display, (0, CENTERLINE_Y), (FRAME_W, CENTERLINE_Y), (255, 255, 0), 1)  # horizontal - cyan

    # HSV processing
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_ball, upper_ball)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    ball_detected = False
    bx, by = 0, 0

    if len(contours) > 0:
        largest = max(contours, key=cv2.contourArea)
        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        cx, cy, radius = int(cx), int(cy), int(radius)

        # Ball is valid only if:
        #   1. Radius meets minimum size
        #   2. Circle center (y-coordinate) is on or below the horizontal centerline
        if radius > MIN_RADIUS and cy >= CENTERLINE_Y:
            ball_detected = True
            bx, by = cx, cy

            cv2.circle(display, (cx, cy), int(radius), (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 3, (0, 0, 255), -1)
            cv2.putText(display, f"({cx},{cy})", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ── Pack and send 3 bytes to Arduino ──
    # Byte 0: status — 0=no ball, 1=ball detected
    # Byte 1: x_byte — 0-255 mapped from pixel x in range [0, FRAME_W-1]
    # Byte 2: y_byte — 0-255 mapped from pixel y in range [0, FRAME_H-1]
    if ball_detected:
        status = 1
        x_byte = int(np.interp(bx, [0, FRAME_W - 1], [0, 255]))
        y_byte = int(np.interp(by, [0, FRAME_H - 1], [0, 255]))
    else:
        status, x_byte, y_byte = 0, 0, 0

    ser.write(bytes([status, x_byte, y_byte]))

    # Status overlay
    label = f"BALL DETECTED  x={bx} y={by}" if ball_detected else "No ball (or above centerline)"
    color = (0, 255, 0) if ball_detected else (0, 0, 255)
    cv2.putText(display, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow("Ball Tracker", display)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
ser.close()