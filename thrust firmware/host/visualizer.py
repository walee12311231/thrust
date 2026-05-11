#!/usr/bin/env python3
"""
6U CubeSat (2U × 3U × 1U) orientation visualizer driven by MPU6050 over USB serial.

Usage:
    python3 visualizer.py [PORT]

Serial format expected from Pico (115200 baud):
    ax,ay,az,gx,gy,gz   (accel in g, gyro in deg/s)

Dependencies:
    pip install pyserial pygame numpy
"""

import sys, math, time, threading, glob
import numpy as np
import pygame
import serial

# ── serial helpers ────────────────────────────────────────────────────────────

def _auto_detect_port() -> str:
    candidates = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") + \
                 glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    if candidates:
        return candidates[0]
    raise RuntimeError(
        "No serial port found. Pass the port as an argument: python3 visualizer.py /dev/cu.usbmodemXXX"
    )

# ── shared sensor state ───────────────────────────────────────────────────────

class SensorState:
    def __init__(self):
        self._lock = threading.Lock()
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.connected = False

    def update(self, ax, ay, az, gx, gy, gz):
        with self._lock:
            self.ax, self.ay, self.az = ax, ay, az
            self.gx, self.gy, self.gz = gx, gy, gz
            self.connected = True

    def snapshot(self):
        with self._lock:
            return (self.ax, self.ay, self.az,
                    self.gx, self.gy, self.gz,
                    self.connected)

def serial_reader(port: str, baud: int, state: SensorState):
    """Background thread: reads lines, parses CSV, updates state."""
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                while True:
                    line = ser.readline().decode("ascii", errors="ignore").strip()
                    parts = line.split(",")
                    if len(parts) == 6:
                        try:
                            vals = [float(p) for p in parts]
                            state.update(*vals)
                        except ValueError:
                            pass
        except serial.SerialException:
            state.connected = False
            time.sleep(1.0)

# ── 3-D geometry ──────────────────────────────────────────────────────────────
# 6U CubeSat: 2 U wide × 3 U tall × 1 U deep  (1 U = 10 cm)
W, H, D = 2.0, 3.0, 1.0

VERTS = np.array([
    [-W/2, -H/2, -D/2],  # 0  back-bottom-left
    [ W/2, -H/2, -D/2],  # 1  back-bottom-right
    [ W/2,  H/2, -D/2],  # 2  back-top-right
    [-W/2,  H/2, -D/2],  # 3  back-top-left
    [-W/2, -H/2,  D/2],  # 4  front-bottom-left
    [ W/2, -H/2,  D/2],  # 5  front-bottom-right
    [ W/2,  H/2,  D/2],  # 6  front-top-right
    [-W/2,  H/2,  D/2],  # 7  front-top-left
], dtype=np.float64)

# Faces: vertex indices (CCW from outside) + base colour
FACES = [
    ((4, 5, 6, 7), (70,  140, 210)),  # +Z front  – blue
    ((1, 0, 3, 2), (45,  90,  140)),  # -Z back   – dark blue
    ((0, 4, 7, 3), (80,  170, 100)),  # -X left   – green
    ((5, 1, 2, 6), (50,  130,  70)),  # +X right  – dark green
    ((3, 7, 6, 2), (220, 190,  50)),  # +Y top    – gold (solar panels)
    ((0, 1, 5, 4), (150, 130,  30)),  # -Y bottom – dark gold
]

EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

# ── math helpers ──────────────────────────────────────────────────────────────

def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=np.float64)

def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=np.float64)

def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=np.float64)

def rotation_matrix(roll, pitch, yaw):
    """ZYX convention: yaw → pitch → roll."""
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)

LIGHT = np.array([0.4, 0.8, 1.0])
LIGHT /= np.linalg.norm(LIGHT)

# ── projection ────────────────────────────────────────────────────────────────
SCALE  = 90    # pixels per unit
FOV    = 700   # focal length
CAM_Z  = 7.5   # camera z-distance in units

def project(v, cx, cy):
    x, y, z = v
    z = z + CAM_Z
    if z < 0.1:
        z = 0.1
    px = int(x * FOV / z * SCALE + cx)
    py = int(-y * FOV / z * SCALE + cy)
    return (px, py)

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else _auto_detect_port()
    print(f"Connecting to {port} at 115200 baud …")

    state = SensorState()
    t = threading.Thread(target=serial_reader, args=(port, 115200, state), daemon=True)
    t.start()

    pygame.init()
    W_SCR, H_SCR = 900, 700
    screen = pygame.display.set_mode((W_SCR, H_SCR))
    pygame.display.set_caption("6U CubeSat Orientation")
    clock = pygame.time.Clock()
    font  = pygame.font.SysFont("monospace", 16)
    font_big = pygame.font.SysFont("monospace", 20, bold=True)

    BG      = (18, 20, 30)
    OUTLINE = (200, 210, 230)
    RED     = (230, 80,  80)
    GREEN   = (80,  220, 120)
    CYAN    = (80,  200, 230)

    roll = pitch = yaw = 0.0
    prev_t = time.perf_counter()
    ALPHA  = 0.97   # complementary filter weight toward gyro

    cx, cy = W_SCR // 2, H_SCR // 2 + 20

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:   # reset orientation
                    roll = pitch = yaw = 0.0
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return

        now = time.perf_counter()
        dt  = now - prev_t
        prev_t = now

        ax, ay, az, gx, gy, gz, connected = state.snapshot()

        # Complementary filter for roll & pitch; yaw from gyro only
        accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if accel_norm > 0.1:
            accel_roll  = math.atan2(ay, az)
            accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            roll  = ALPHA * (roll  + math.radians(gx) * dt) + (1 - ALPHA) * accel_roll
            pitch = ALPHA * (pitch + math.radians(gy) * dt) + (1 - ALPHA) * accel_pitch
        else:
            roll  += math.radians(gx) * dt
            pitch += math.radians(gy) * dt
        yaw += math.radians(gz) * dt

        R = rotation_matrix(roll, pitch, yaw)
        rotated = (R @ VERTS.T).T  # shape (8, 3)

        # Sort visible faces back-to-front (painter's algorithm)
        face_data = []
        for indices, base_color in FACES:
            vs = rotated[list(indices)]
            e1 = vs[1] - vs[0]
            e2 = vs[2] - vs[0]
            normal = np.cross(e1, e2)
            if normal[2] <= 0:
                continue  # back-face cull
            n_unit = normal / (np.linalg.norm(normal) + 1e-9)
            intensity = max(0.25, float(np.dot(n_unit, LIGHT)))
            color = tuple(min(255, int(c * intensity)) for c in base_color)
            avg_z = float(vs[:, 2].mean())
            face_data.append((avg_z, vs, indices, color))

        face_data.sort(key=lambda x: x[0])  # ascending z = draw far first

        # ── draw ──────────────────────────────────────────────────────────────
        screen.fill(BG)

        # Axis indicator (bottom-left)
        origin_s = np.array([90.0, H_SCR - 80.0])
        axis_scale = 40
        for axis_vec, axis_color, label in [
            (R @ [1, 0, 0], RED,   "X"),
            (R @ [0, 1, 0], GREEN, "Y"),
            (R @ [0, 0, 1], CYAN,  "Z"),
        ]:
            tip = origin_s + np.array([axis_vec[0], -axis_vec[1]]) * axis_scale
            pygame.draw.line(screen, axis_color,
                             origin_s.astype(int), tip.astype(int), 2)
            lbl = font.render(label, True, axis_color)
            screen.blit(lbl, (int(tip[0]) + 3, int(tip[1]) - 8))

        # CubeSat faces
        for _, vs, indices, color in face_data:
            pts = [project(v, cx, cy) for v in vs]
            pygame.draw.polygon(screen, color, pts)
            pygame.draw.polygon(screen, OUTLINE, pts, 1)

        # HUD
        status_color = GREEN if connected else RED
        status_text  = "CONNECTED" if connected else "WAITING FOR SERIAL…"
        screen.blit(font_big.render(status_text, True, status_color), (20, 12))

        def hud(label, value, unit, y, color=CYAN):
            txt = font.render(f"{label:6s} {value:+8.2f}  {unit}", True, color)
            screen.blit(txt, (20, y))

        hud("roll",  math.degrees(roll),  "deg", 45)
        hud("pitch", math.degrees(pitch), "deg", 65)
        hud("yaw",   math.degrees(yaw),   "deg", 85)

        hud("ax", ax, "g",     125, (200, 120, 120))
        hud("ay", ay, "g",     145, (120, 200, 120))
        hud("az", az, "g",     165, (120, 150, 220))
        hud("gx", gx, "°/s",   185, (200, 120, 120))
        hud("gy", gy, "°/s",   205, (120, 200, 120))
        hud("gz", gz, "°/s",   225, (120, 150, 220))

        hint = font.render("R = reset orientation   ESC = quit", True, (80, 90, 110))
        screen.blit(hint, (20, H_SCR - 24))

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()

