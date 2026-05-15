import cv2
import numpy as np
import threading
import time
import serial
import serial.tools.list_ports

# ─── CONFIGURATION ──────────────────────────────────────────
CAMERA_INDEXES = [0, 1, 2]   # ← Edit after running check_cameras.py

MIN_CAR_AREA  = 800
MAX_TRACKS    = 20
MAX_DISAPPEAR = 10

FRAME_W, FRAME_H = 640, 480
FRAME_AREA = FRAME_W * FRAME_H

# ── Density thresholds ───────────────────────────────────────
COUNT_LOW    = 2
COUNT_MEDIUM = 5
COUNT_HIGH   = 8

AREA_LOW    = 5
AREA_MEDIUM = 15
AREA_HIGH   = 30

# ── Adaptive Signal Settings ─────────────────────────────────
MIN_GREEN   = 5     # Minimum green time (seconds) — no lane gets less
MAX_GREEN   = 25    # Maximum green time (seconds) — no lane holds longer
YELLOW_TIME = 2     # Yellow transition time (seconds)

# Pressure added per second while lane is RED, based on density
# Higher density = pressure builds faster = gets green sooner
PRESSURE_RATE = {
    "LOW":    1,    # +1 pressure/sec
    "MEDIUM": 3,    # +3 pressure/sec
    "HIGH":   6,    # +6 pressure/sec
    "JAM":    10,   # +10 pressure/sec — urgent
    "N/A":    1,
}

# ── Arduino serial settings ──────────────────────────────────
ARDUINO_BAUD = 9600
# ────────────────────────────────────────────────────────────


# ─── Arduino connection ──────────────────────────────────────

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        if any(k in desc for k in ["arduino", "ch340", "cp210", "usb serial"]):
            print(f"  ✅ Arduino found on {port.device} ({port.description})")
            return port.device
    print("  ⚠️  Arduino not auto-detected.")
    for p in ports:
        print(f"     Available: {p.device} — {p.description}")
    return None


def connect_arduino():
    port = find_arduino_port()
    if port is None:
        print("  ❌ No Arduino found. Running in display-only mode.")
        return None
    try:
        ser = serial.Serial(port, ARDUINO_BAUD, timeout=1)
        time.sleep(2)
        print(f"  ✅ Arduino connected on {port}")
        return ser
    except Exception as e:
        print(f"  ❌ Could not open {port}: {e}")
        return None


def send_to_arduino(ser, states):
    if ser is None:
        return
    try:
        msg = ",".join(states) + "\n"
        ser.write(msg.encode())
    except Exception as e:
        print(f"  ⚠️  Arduino write error: {e}")


# ─── Adaptive Traffic Controller ─────────────────────────────

class AdaptiveTrafficController(threading.Thread):
    """
    Pressure-Based Adaptive Signal Control

    How it works:
    ─────────────
    Every lane that is RED builds up pressure over time.
    The rate of pressure buildup depends on density:
        LOW  → slow buildup
        JAM  → fast buildup
    When current green lane hits MIN_GREEN, we check if any
    other lane has higher pressure → switch to that lane.
    If current lane hits MAX_GREEN → force switch regardless.
    This guarantees:
        ✅ Busy lanes get green most often
        ✅ No lane ever starved (pressure always builds)
        ✅ Reacts instantly to traffic changes
        ✅ Smooth transitions via yellow phase
    """

    def __init__(self, arduino, density_store):
        super().__init__(daemon=True)
        self.arduino        = arduino
        self.density_store  = density_store   # list of 3 strings, updated by detection

        self.pressure       = [0.0, 0.0, 0.0] # pressure per lane
        self.light_states   = ['R', 'R', 'R']  # current light state per lane
        self.current_lane   = 0                # which lane is currently green
        self.green_elapsed  = 0                # seconds current lane has been green
        self.lock           = threading.Lock()
        self.running        = True

        # Log for display
        self.log            = []               # list of strings shown on screen

    # ── Public read methods ──────────────────────────────────

    def get_states(self):
        with self.lock:
            return list(self.light_states)

    def get_pressure(self):
        with self.lock:
            return list(self.pressure)

    def get_current_lane(self):
        with self.lock:
            return self.current_lane

    def get_green_elapsed(self):
        with self.lock:
            return self.green_elapsed

    def get_log(self):
        with self.lock:
            return list(self.log)

    # ── Internal helpers ─────────────────────────────────────

    def _set_states(self, states):
        with self.lock:
            self.light_states = list(states)
        send_to_arduino(self.arduino, states)

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        entry = f"[{ts}] {msg}"
        print(f"  {entry}")
        with self.lock:
            self.log.append(entry)
            if len(self.log) > 20:   # keep last 20 lines
                self.log.pop(0)

    def _pick_next_lane(self, exclude):
        """Return lane index with highest pressure, excluding current green lane."""
        best_lane     = -1
        best_pressure = -1
        for i in range(3):
            if i == exclude:
                continue
            if self.pressure[i] > best_pressure:
                best_pressure = self.pressure[i]
                best_lane     = i
        return best_lane

    # ── Main control loop ────────────────────────────────────

    def run(self):
        # Start: give Lane 0 green first
        self._activate_lane(0)

        while self.running:
            time.sleep(1)   # tick every second

            with self.lock:
                lane        = self.current_lane
                elapsed     = self.green_elapsed
                density_now = self.density_store[lane]

            # ── Build pressure on RED lanes ──────────────────
            for i in range(3):
                if i == lane:
                    continue   # green lane doesn't build pressure
                d    = self.density_store[i]
                rate = PRESSURE_RATE.get(d, 1)
                with self.lock:
                    self.pressure[i] += rate

            # ── Update green elapsed ─────────────────────────
            with self.lock:
                self.green_elapsed += 1
                elapsed = self.green_elapsed

            # ── Decision logic ───────────────────────────────

            # Not reached minimum green yet — hold current lane
            if elapsed < MIN_GREEN:
                continue

            # Find which other lane has highest pressure
            next_lane     = self._pick_next_lane(exclude=lane)
            next_pressure = self.pressure[next_lane] if next_lane >= 0 else 0

            force_switch  = elapsed >= MAX_GREEN
            better_lane   = next_pressure > self.pressure[lane] * 1.5   # 50% more pressure

            if force_switch:
                self._log(f"Lane {lane+1} MAX GREEN reached ({elapsed}s) → switching")
                self._switch_to(next_lane)

            elif better_lane:
                self._log(
                    f"Lane {next_lane+1} pressure ({next_pressure:.0f}) "
                    f"> Lane {lane+1} ({self.pressure[lane]:.0f}) → switching"
                )
                self._switch_to(next_lane)

            # else: current lane still has highest pressure — stay green

    def _activate_lane(self, lane):
        """Set a lane green, all others red."""
        states = ['R', 'R', 'R']
        states[lane] = 'G'
        self._set_states(states)
        with self.lock:
            self.current_lane  = lane
            self.green_elapsed = 0
            self.pressure[lane] = 0   # reset pressure when lane gets green
        self._log(
            f"Lane {lane+1} GREEN | density={self.density_store[lane]} "
            f"| pressure was {self.pressure[lane]:.0f}"
        )

    def _switch_to(self, next_lane):
        """Yellow on current lane, then green on next lane."""
        with self.lock:
            lane = self.current_lane

        # Yellow phase on current lane
        states = ['R', 'R', 'R']
        states[lane] = 'Y'
        self._set_states(states)
        self._log(f"Lane {lane+1} YELLOW → Lane {next_lane+1} next")
        time.sleep(YELLOW_TIME)

        if not self.running:
            return

        self._activate_lane(next_lane)

    def stop(self):
        self.running = False
        send_to_arduino(self.arduino, ['R', 'R', 'R'])


# ─── Density helpers ─────────────────────────────────────────

def count_based_density(num_cars):
    if num_cars <= COUNT_LOW:
        return "LOW",    (0, 255, 0)
    elif num_cars <= COUNT_MEDIUM:
        return "MEDIUM", (0, 255, 255)
    elif num_cars <= COUNT_HIGH:
        return "HIGH",   (0, 165, 255)
    else:
        return "JAM",    (0, 0, 255)


def area_based_density(trackers, frame_area=FRAME_AREA):
    covered = sum(w * h for _, _, w, h, _, _ in trackers)
    pct = (covered / frame_area) * 100
    if pct <= AREA_LOW:
        label, color = "LOW",    (0, 255, 0)
    elif pct <= AREA_MEDIUM:
        label, color = "MEDIUM", (0, 255, 255)
    elif pct <= AREA_HIGH:
        label, color = "HIGH",   (0, 165, 255)
    else:
        label, color = "JAM",    (0, 0, 255)
    return label, color, pct


def draw_density_bar(frame, pct, x=10, y=148, bar_w=180, bar_h=12):
    cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (50, 50, 50), -1)
    fill      = int(min(pct, 100) / 100 * bar_w)
    bar_color = (0,255,0) if pct < AREA_MEDIUM else (0,255,255) if pct < AREA_HIGH else (0,0,255)
    cv2.rectangle(frame, (x, y), (x + fill, y + bar_h), bar_color, -1)
    cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (180,180,180), 1)
    cv2.putText(frame, f"{pct:.1f}%",
                (x + bar_w + 5, y + 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)


LIGHT_BG    = {'G': (0,255,0),   'Y': (0,255,255), 'R': (0,0,255)}
LIGHT_LABEL = {'G': 'GREEN',     'Y': 'YELLOW',    'R': 'RED'}

def draw_traffic_light(frame, state, pressure=0.0, green_elapsed=0, is_active=False):
    """Traffic light icon in top-right with pressure bar below it."""
    h, w   = frame.shape[:2]
    cx, cy = w - 42, 55
    r      = 13

    for s, off in [('R', -28), ('Y', 0), ('G', 28)]:
        color = LIGHT_BG[s] if state == s else (35, 35, 35)
        cv2.circle(frame, (cx, cy + off), r, color, -1)
        cv2.circle(frame, (cx, cy + off), r, (160,160,160), 1)

    lbl_col = LIGHT_BG.get(state, (255,255,255))
    cv2.putText(frame, LIGHT_LABEL.get(state, '?'),
                (w - 92, cy + 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, lbl_col, 2)

    # Pressure bar (shown only when lane is RED)
    if state == 'R':
        bar_max = 100
        bar_w   = 80
        bx, by  = w - 95, cy + 58
        fill    = int(min(pressure, bar_max) / bar_max * bar_w)
        cv2.rectangle(frame, (bx, by), (bx + bar_w, by + 10), (50,50,50), -1)
        pcol = (0,255,0) if pressure < 30 else (0,255,255) if pressure < 60 else (0,0,255)
        cv2.rectangle(frame, (bx, by), (bx + fill, by + 10), pcol, -1)
        cv2.rectangle(frame, (bx, by), (bx + bar_w, by + 10), (150,150,150), 1)
        cv2.putText(frame, f"P:{pressure:.0f}",
                    (bx, by + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,200,200), 1)

    # Green elapsed timer
    if state == 'G':
        cv2.putText(frame, f"{green_elapsed}s / {MAX_GREEN}s",
                    (w - 90, cy + 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,255,200), 1)


def draw_camera_overlay(frame, cam_idx, num_cars, trackers,
                        light_state, pressure, green_elapsed):
    count_label, count_color     = count_based_density(num_cars)
    area_label,  area_color, pct = area_based_density(trackers)

    # Semi-transparent background
    overlay = frame.copy()
    cv2.rectangle(overlay, (5, 5), (310, 170), (0,0,0), -1)
    cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

    cv2.putText(frame, f"Cam {cam_idx+1}  |  Vehicles: {num_cars}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255,255,255), 2)
    cv2.putText(frame, f"Density : {count_label}",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.60, count_color, 2)
    cv2.putText(frame, f"Area    : {area_label}  ({pct:.1f}%)",
                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, area_color, 1)

    # Pressure rate info
    rate = PRESSURE_RATE.get(count_label, 1)
    cv2.putText(frame, f"Pressure rate: +{rate}/sec",
                (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180,180,180), 1)
    cv2.putText(frame, f"Pressure     : {pressure:.0f}",
                (10, 128), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180,180,180), 1)

    draw_density_bar(frame, pct, x=10, y=145)
    draw_traffic_light(frame, light_state, pressure, green_elapsed,
                       is_active=(light_state == 'G'))

    return count_label, area_label, pct


# ─── Camera class ────────────────────────────────────────────

class USBCamera:
    def __init__(self, device_index):
        self.device_index = device_index
        self.cap     = None
        self.frame   = None
        self.lock    = threading.Lock()
        self.running = True
        self._open()
        self.thread  = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _open(self):
        for _ in range(5):
            self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_DSHOW)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                self.cap.set(cv2.CAP_PROP_FPS, 20)
                print(f"  ✅ Camera {self.device_index} opened")
                return True
            time.sleep(0.5)
        print(f"  ❌ Camera {self.device_index} failed")
        return False

    def _reader(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    frame = cv2.resize(frame, (FRAME_W, FRAME_H))
                    with self.lock:
                        self.frame = frame
                else:
                    self.cap.release()
                    self._open()
            else:
                self._open()
            time.sleep(0.05)
        if self.cap:
            self.cap.release()

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.thread.join()


# ─── Detection helpers ───────────────────────────────────────

def iou(box1, box2):
    x1,y1,w1,h1,_ = box1
    x2,y2,w2,h2,_ = box2
    xi1,yi1 = max(x1,x2), max(y1,y2)
    xi2,yi2 = min(x1+w1,x2+w2), min(y1+h1,y2+h2)
    inter   = max(0,xi2-xi1)*max(0,yi2-yi1)
    union   = w1*h1 + w2*h2 - inter
    return inter/union if union > 0 else 0


def detect_cars(frame):
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7,7), 0)
    thresh  = cv2.adaptiveThreshold(blurred, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    cleaned = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN,  kernel)
    edges   = cv2.Canny(cleaned, 60, 160)
    edges   = cv2.dilate(edges, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    raw_rects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > MIN_CAR_AREA:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / float(h)
            if 0.6 < aspect < 1.8:
                hull_area = cv2.contourArea(cv2.convexHull(cnt))
                solidity  = float(area)/hull_area if hull_area > 0 else 0
                extent    = float(area)/(w*h)
                if solidity > 0.6 and extent > 0.4:
                    raw_rects.append((x, y, w, h, area))

    sorted_rects = sorted(raw_rects, key=lambda r: r[4], reverse=True)
    used = [False]*len(sorted_rects)
    current_rects = []
    for i, r1 in enumerate(sorted_rects):
        if used[i]: continue
        current_rects.append(r1[:4])
        for j in range(i+1, len(sorted_rects)):
            if iou(r1, sorted_rects[j]) > 0.3:
                used[j] = True
    current_rects.sort(key=lambda r: r[2]*r[3], reverse=True)
    return raw_rects, current_rects


def update_trackers(trackers, current_rects):
    new_trackers, matched_indices = [], set()
    for x, y, w, h, tid, disappear in trackers:
        disappear += 1
        if disappear > MAX_DISAPPEAR: continue
        cx1,cy1 = x+w//2, y+h//2
        matched = False
        for cidx, curr in enumerate(current_rects):
            if cidx in matched_indices: continue
            cx2,cy2 = curr[0]+curr[2]//2, curr[1]+curr[3]//2
            if ((cx1-cx2)**2+(cy1-cy2)**2)**0.5 < 80:
                new_trackers.append((curr[0],curr[1],curr[2],curr[3],tid,0))
                matched_indices.add(cidx)
                matched = True
                break
        if not matched:
            new_trackers.append((x,y,w,h,tid,disappear))

    next_id = max((t[4] for t in new_trackers), default=-1)+1
    for cidx, curr in enumerate(current_rects):
        if cidx not in matched_indices:
            new_trackers.append((*curr, next_id, 0))
            next_id += 1
            if len(new_trackers) >= MAX_TRACKS: break
    return new_trackers[:MAX_TRACKS]


# ─── Log panel (right side of combined view) ─────────────────

def draw_log_panel(log_lines, height, width=320):
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.rectangle(panel, (0,0), (width-1, height-1), (40,40,40), -1)
    cv2.putText(panel, "ADAPTIVE CONTROLLER LOG",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0,200,255), 1)
    cv2.line(panel, (5, 28), (width-5, 28), (80,80,80), 1)

    y = 48
    for line in log_lines[-14:]:    # show last 14 lines
        # colour-code by lane mentioned
        col = (180,180,180)
        if "Lane 1" in line: col = (100,255,100)
        if "Lane 2" in line: col = (100,200,255)
        if "Lane 3" in line: col = (255,180,100)
        if "YELLOW" in line: col = (0,255,255)
        if "MAX"    in line: col = (0,100,255)
        cv2.putText(panel, line[:42], (8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)
        y += 18

    return panel


# ─── Main ────────────────────────────────────────────────────

print("\n── Smart Adaptive Traffic Monitor ──────────")
print("── Connecting Arduino ───────────────────────")
arduino = connect_arduino()

print("\n── Opening USB Cameras ──────────────────────")
cameras = [USBCamera(idx) for idx in CAMERA_INDEXES]
camera_trackers = [[] for _ in cameras]

# Shared density store — written by detection, read by controller
density_store = ["LOW"] * 3

print("\n── Starting Adaptive Controller ─────────────")
controller = AdaptiveTrafficController(arduino, density_store)
controller.start()

print("── Starting Detection Loop ──────────────────")
print("   Press 'q' to quit\n")

while True:
    frames        = []
    total_cars    = 0
    cam_summaries = []

    light_states  = controller.get_states()
    pressures     = controller.get_pressure()
    green_elapsed = controller.get_green_elapsed()
    active_lane   = controller.get_current_lane()

    for cam_idx, cam in enumerate(cameras):
        frame = cam.read()

        if frame is None or frame.size == 0:
            ph = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
            cv2.putText(ph, f"Cam {cam_idx+1} — No Signal",
                        (80,240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            frames.append(ph)
            cam_summaries.append((cam_idx, 0, "N/A", "N/A", 0.0))
            continue

        raw_rects, current_rects = detect_cars(frame)
        camera_trackers[cam_idx] = update_trackers(camera_trackers[cam_idx], current_rects)
        trackers    = camera_trackers[cam_idx]
        num_cars    = len(trackers)
        total_cars += num_cars

        # Update shared density (controller reads this every second)
        density_store[cam_idx], _ = count_based_density(num_cars)

        # Draw detections
        for rect in raw_rects[:10]:
            x,y,w,h,_ = rect
            cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0), 1)
        for x,y,w,h,tid,_ in trackers:
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            cv2.putText(frame, f"#{tid}", (x,y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0,255,0), 2)

        ls      = light_states[cam_idx] if cam_idx < len(light_states) else 'R'
        pr      = pressures[cam_idx]    if cam_idx < len(pressures)    else 0.0
        ge      = green_elapsed         if cam_idx == active_lane       else 0

        count_lbl, area_lbl, pct = draw_camera_overlay(
            frame, cam_idx, num_cars, trackers, ls, pr, ge)

        cam_summaries.append((cam_idx, num_cars, count_lbl, area_lbl, pct))
        frames.append(frame)

    # ── Build combined display ───────────────────────────────
    H       = 360
    resized = [cv2.resize(f, (int(f.shape[1]*H/f.shape[0]), H)) for f in frames]
    cam_row = np.hstack(resized)

    # Log panel on the right
    log_panel = draw_log_panel(controller.get_log(), H, width=360)
    combined  = np.hstack([cam_row, log_panel])

    # Top bar
    avg = total_cars / max(len(cam_summaries), 1)
    if avg <= COUNT_LOW:       overall,ov_col = "LOW",    (0,255,0)
    elif avg <= COUNT_MEDIUM:  overall,ov_col = "MEDIUM", (0,255,255)
    elif avg <= COUNT_HIGH:    overall,ov_col = "HIGH",   (0,165,255)
    else:                      overall,ov_col = "JAM",    (0,0,255)

    header = np.zeros((42, combined.shape[1], 3), dtype=np.uint8)
    cv2.rectangle(header, (0,0), (header.shape[1], 42), (20,20,20), -1)
    cv2.putText(header,
                f"TOTAL: {total_cars} vehicles  |  TRAFFIC: {overall}  |  "
                f"Active Lane: {active_lane+1}  |  "
                f"Green: {green_elapsed}s / {MAX_GREEN}s  |  [q] quit",
                (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.62, ov_col, 2)

    display = np.vstack([header, combined])

    # Bottom strip
    strip_h = 24
    strip   = np.zeros((strip_h, display.shape[1], 3), dtype=np.uint8)
    for cam_idx, num_cars, count_lbl, area_lbl, pct in cam_summaries:
        pr  = pressures[cam_idx] if cam_idx < len(pressures) else 0
        txt = f"Cam{cam_idx+1}: {num_cars}v | {count_lbl} | {pct:.1f}% | P:{pr:.0f}"
        cv2.putText(strip, txt,
                    (10 + cam_idx * 310, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (170,170,170), 1)

    display = np.vstack([display, strip])

    cv2.imshow("Smart Adaptive Traffic Monitor", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ── Cleanup ──────────────────────────────────────────────────
controller.stop()
for cam in cameras:
    cam.stop()
if arduino:
    arduino.close()
cv2.destroyAllWindows()
print("Smart adaptive traffic monitor stopped.")
