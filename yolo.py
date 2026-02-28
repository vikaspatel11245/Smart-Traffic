import cv2
import serial
import time
from ultralytics import YOLO

# ===================== SERIAL SETUP =====================
# Change COM port according to your system
arduino = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)

# ===================== LOAD YOLO MODEL ==================
model = YOLO("yolov8n.pt")  # nano model (fast)

# ===================== VIDEO SOURCE =====================
cap = cv2.VideoCapture(0)  # webcam

# ===================== VEHICLE CLASSES ==================
vehicle_classes = ['car', 'truck', 'bus', 'motorcycle']

# ===================== LANE REGIONS =====================
# Format: (x1, y1, x2, y2)
lane_1 = (0, 0, 213, 480)
lane_2 = (213, 0, 426, 480)
lane_3 = (426, 0, 640, 480)

lanes = [lane_1, lane_2, lane_3]

# ===================== HELPER FUNCTION ==================
def count_vehicles(detections, lane):
    count = 0
    x1, y1, x2, y2 = lane

    for box in detections.boxes:
        cls = int(box.cls[0])
        label = model.names[cls]

        if label in vehicle_classes:
            bx1, by1, bx2, by2 = map(int, box.xyxy[0])
            cx = (bx1 + bx2) // 2
            cy = (by1 + by2) // 2

            if x1 < cx < x2 and y1 < cy < y2:
                count += 1

    return count

# ===================== MAIN LOOP ========================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.4)[0]

    counts = []
    for lane in lanes:
        counts.append(count_vehicles(results, lane))

    # Decide signal with maximum vehicles
    max_signal = counts.index(max(counts)) + 1
    green_time = min(10, max(5, counts[max_signal - 1] * 2))

    # Send to Arduino
    command = f"{max_signal} {green_time}\n"
    arduino.write(command.encode())

    # ===================== DISPLAY ======================
    for i, lane in enumerate(lanes):
        x1, y1, x2, y2 = lane
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"Lane {i+1}: {counts[i]}",
                    (x1 + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2)

    cv2.putText(frame,
                f"GREEN → Signal {max_signal} ({green_time}s)",
                (20, 450),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 0, 255), 2)

    cv2.imshow("Smart Traffic Signal - YOLO", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# ===================== CLEANUP ==========================
cap.release()
cv2.destroyAllWindows()
arduino.close()
