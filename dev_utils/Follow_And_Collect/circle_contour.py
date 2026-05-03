import numpy as np
import cv2
import serial

# ================= CONFIG =================
MIN_RADIUS       = 10
LEARN_FRAMES     = 10        # how many green detections before "learning" is locked in
SIZE_TOLERANCE   = 0.5       # ±50% radius variation allowed for shape match
MIN_CIRCULARITY  = 0.65      # how round a non-green blob must be to count

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)

# HSV range for the GREEN reference ball (your known-good detector)
lower_green = np.array([25,  50,  50])
upper_green = np.array([60, 255, 255])

# ================= CAMERA =================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

grabbed, frame = cap.read()
if not grabbed:
    raise RuntimeError("Camera not available")

FRAME_H, FRAME_W = frame.shape[:2]
CENTERLINE_Y = FRAME_H // 2
CENTERLINE_X = FRAME_W // 2

# ================= MEMORY =================
learned_radius = None        # average ball radius once learned
radius_samples = []          # collected during learning phase

print("=== Ball Tracker Started (Learn from Green, Detect Any Color) ===")
print(f"Frame: {FRAME_W}x{FRAME_H} | Centerline X: {CENTERLINE_X} | Centerline Y: {CENTERLINE_Y}")
print(f"Show a GREEN ball to teach the size. Need {LEARN_FRAMES} good frames.")

while True:
    grabbed, frame = cap.read()
    if not grabbed:
        break

    display = frame.copy()

    # Draw crosshairs
    cv2.line(display, (CENTERLINE_X, 0), (CENTERLINE_X, FRAME_H), (255, 255, 0), 1)
    cv2.line(display, (0, CENTERLINE_Y), (FRAME_W, CENTERLINE_Y), (255, 255, 0), 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ── PHASE 1: Try green detection (your trusted method) ──────────
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    green_mask = cv2.erode(green_mask,  None, iterations=2)
    green_mask = cv2.dilate(green_mask, None, iterations=2)

    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    ball_detected = False
    bx, by = 0, 0
    detection_source = ""

    if green_contours:
        largest = max(green_contours, key=cv2.contourArea)
        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        cx, cy, radius = int(cx), int(cy), int(radius)

        if radius > MIN_RADIUS and cy >= CENTERLINE_Y:
            ball_detected = True
            bx, by = cx, cy
            detection_source = "GREEN"

            # ── LEARN: remember the radius ──
            radius_samples.append(radius)
            if len(radius_samples) > LEARN_FRAMES:
                radius_samples.pop(0)
            learned_radius = int(np.median(radius_samples))

            cv2.circle(display, (cx, cy), radius, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 3,      (0,   0, 255), -1)
            cv2.putText(display, f"GREEN ({cx},{cy}) r={radius}", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ── PHASE 2: No green found → look for any round ball at learned size ──
    if not ball_detected and learned_radius is not None and len(radius_samples) >= LEARN_FRAMES:
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 1.5)
        edges   = cv2.Canny(blurred, 40, 120)

        # Close gaps in the ball outline
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        edges  = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

        shape_contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_r = learned_radius * (1 - SIZE_TOLERANCE)
        max_r = learned_radius * (1 + SIZE_TOLERANCE)

        best = None
        best_score = 0

        for cnt in shape_contours:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity < MIN_CIRCULARITY:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cx, cy, radius = int(cx), int(cy), int(radius)

            # Match the learned size (within tolerance)
            if radius < min_r or radius > max_r:
                continue
            if cy < CENTERLINE_Y:
                continue

            # Pick the roundest match
            score = circularity
            if score > best_score:
                best_score = score
                best = (cx, cy, radius)

        if best is not None:
            cx, cy, radius = best
            ball_detected = True
            bx, by = cx, cy
            detection_source = "SHAPE"
            cv2.circle(display, (cx, cy), radius, (0, 165, 255), 2)   # orange for shape match
            cv2.circle(display, (cx, cy), 3,      (0,   0, 255), -1)
            cv2.putText(display, f"SHAPE ({cx},{cy}) r={radius}", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

    # ── Send packet ────────────────────────────────────────────────
    if ball_detected:
        x_byte = int(np.interp(bx, [0, FRAME_W - 1], [0, 254]))
        y_byte = int(np.interp(by, [0, FRAME_H - 1], [0, 254]))
        ser.write(bytes([0xFF, x_byte, y_byte]))

    # ── Status overlay ─────────────────────────────────────────────
    if ball_detected:
        label = f"{detection_source}  x={bx} y={by}"
        color = (0, 255, 0) if detection_source == "GREEN" else (0, 165, 255)
    else:
        label = "No ball"
        color = (0, 0, 255)
    cv2.putText(display, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # Show learning progress
    if learned_radius is None:
        progress = f"Learning: {len(radius_samples)}/{LEARN_FRAMES} (show green ball)"
        cv2.putText(display, progress, (10, FRAME_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    else:
        info = f"Learned radius: {learned_radius} px (samples: {len(radius_samples)})"
        cv2.putText(display, info, (10, FRAME_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("Green Mask",   green_mask)
    cv2.imshow("Ball Tracker", display)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
