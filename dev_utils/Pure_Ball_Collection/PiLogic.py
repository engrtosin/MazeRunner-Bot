import numpy as np
import time
import cv2
import serial

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(3,480)
cap.set(4,360)

ser = serial.Serial('/dev/ttyACM0',9600)
time.sleep(2)

lower_tennis = np.array([25,50,50])
upper_tennis = np.array([60,255,255])

FRAME_WIDTH = 480
FRAME_HEIGHT = 360

while True:
    grabbed, frame = cap.read()
    if not grabbed:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_tennis, upper_tennis)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 10:
            cx = int(x)
            cy = int(y)

            x_byte = int((cx / FRAME_WIDTH) * 249)
            y_byte = int((cy / FRAME_HEIGHT) * 249)

            ser.write(bytes([x_byte, y_byte]))

            cv2.circle(frame, (cx, cy), int(radius), (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 3, (0,0,255), -1)
        else:
            ser.write(bytes([255,255]))
    else:
        ser.write(bytes([255,255]))

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()