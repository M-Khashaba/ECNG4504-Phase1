"""
maze_route_viewer.py - LIVE view of the Team5 maze robot (ESP32) over Bluetooth

SETUP (once): pair the car in Windows Bluetooth settings ("MazeRobot").
RUN:          double-click run_viewer.bat   (or: python maze_route_viewer.py [COM7])
TEST:         python maze_route_viewer.py --replay robot_log_....txt

What you see
  - the route the car drives, drawn from its wheel encoders and its turns
      pass 1 (exploring) grey, pass 2 (solved path) blue
  - junctions (small dots with the turn letter), dead ends (x),
    HOME (green) and GOAL (red)
  - the car (purple arrow) moving live
  - the path being learned (L S R B, dead ends removed) and the SOLVED path
  - the car's messages; everything is also saved to robot_log_<date>.txt

Keys (click the window first)
  L  start (left-hand rule)             R  start (right-hand rule)
  E  end / stop                         B  approve the return trip (at the goal)
  S  show the sensors
"""

import argparse
import os
import re
import sys
import threading
import time

import matplotlib.pyplot as plt

for _k in list(plt.rcParams):
    if _k.startswith("keymap."):
        plt.rcParams[_k] = []   # free all keys for the car commands
from matplotlib.animation import FuncAnimation

HERE = os.path.dirname(os.path.abspath(__file__))
LAST_PORT_FILE = os.path.join(HERE, "last_port.txt")
KEYS = set("lrebs")
KNOWN = ("LOG", "EVT", "PROG", "PATH", "SOLVED", "STATE", "PASS", "HB", "SNAP")
SILENCE_RECONNECT_S = 8
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
TURN = {"S": 0, "R": 1, "B": 2, "L": 3}
ARROW = ["^", ">", "v", "<"]


# ------------------------------------------------------------------ state
class Route:
    def __init__(self):
        self.lock = threading.Lock()
        self.reset_all()
        self.status = "connecting..."
        self.last_rx = 0.0

    def reset_all(self):
        self.traces = []          # list of (pass, [(x, y), ...])
        self.marks = []           # (kind, letter, x, y)
        self.log = []
        self.path = ""
        self.solved = ""
        self.state = "idle"
        self.new_pass(1)

    def new_pass(self, n):
        self.pass_no = n
        self.x, self.y, self.h = 0.0, 0.0, 0   # home, facing "up"
        self.prog = 0
        self.traces.append((n, [(0.0, 0.0)]))
        self.fresh = True          # the new pass has not moved yet
        if n == 1:
            self.marks = [m for m in self.marks if m[0] != "T"]
            self.marks.append(("H", "", 0.0, 0.0))

    def car(self):
        return (self.x + DX[self.h] * self.prog, self.y + DY[self.h] * self.prog, self.h)

    def feed(self, line):
        p = line.split()
        if not p:
            return
        with self.lock:
            self.last_rx = time.time()
            t = p[0]
            try:
                if t == "PASS":
                    n = int(p[1])
                    if n == 1:
                        self.reset_all()
                        self.state = "pass 1: exploring"
                    else:
                        self.new_pass(2)
                        self.state = "pass 2: solved path"
                elif t == "EVT" and len(p) >= 4:
                    self.fresh = False
                    kind, turn, mm = p[1], p[2], max(0, int(p[3]))
                    if len(p) >= 6:
                        # the car's own map position of this junction (loops snap to
                        # the known junction, so the drawing does not drift)
                        self.x, self.y = float(p[4]), float(p[5])
                    else:
                        self.x += DX[self.h] * mm
                        self.y += DY[self.h] * mm
                    self.prog = 0
                    self.traces[-1][1].append((self.x, self.y))
                    if kind in ("J", "D", "G", "H"):
                        self.marks.append((kind, turn if kind == "J" else "", self.x, self.y))
                    if kind == "G":
                        self.state = "GOAL found" if self.pass_no == 1 else "SOLVED - at the goal"
                    if kind == "H":
                        self.state = "home again - maze explored"
                    self.h = (self.h + TURN.get(turn, 0)) & 3
                elif t == "SNAP" and len(p) >= 3:
                    # the car is at this node (the car's own map position - known
                    # junctions are snapped to, so the drawing does not drift)
                    self.x, self.y, self.prog = float(p[1]), float(p[2]), 0
                    if len(p) >= 4:
                        self.h = int(p[3]) & 3          # heading 0 up, 1 right, 2 down, 3 left
                    pts = self.traces[-1][1]
                    if getattr(self, "fresh", False):
                        pts[0] = (self.x, self.y)       # a new pass starts here, not at home
                        self.fresh = False
                    else:
                        pts.append((self.x, self.y))
                elif t == "PROG" and len(p) >= 2:
                    self.prog = max(0, int(p[1]))
                elif t == "PATH":
                    self.path = p[1] if len(p) > 1 else ""
                elif t == "SOLVED":
                    self.solved = p[1] if len(p) > 1 else "(straight)"
                elif t == "STATE":
                    self.state = " ".join(p[1:])
                elif t == "LOG":
                    self.log.append(line[4:])
                    self.log = self.log[-8:]
            except ValueError:
                pass


# ------------------------------------------------------------------ connection
def open_port(port, baud):
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout, ser.write_timeout = port, baud, 0.2, 2
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class LineReader:
    def __init__(self, ser):
        self.ser, self.buf = ser, b""

    def lines(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self.buf += data
        out = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("ascii", errors="ignore").strip()
            if line:
                out.append(line)
        if len(self.buf) > 4096:
            self.buf = b""
        return out, bool(data)


def candidate_ports():
    """Bluetooth ports that can be the car, the one that worked last time first.
    USB ports are never opened automatically (that can reset the ESP32)."""
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports()
             if "bluetooth" in (p.description or "").lower() and "000000000000" not in (p.hwid or "").upper()]
    try:
        last = open(LAST_PORT_FILE).read().strip().upper()
    except Exception:
        last = ""
    ports.sort(key=lambda p: 0 if last and (last in (p.hwid or "").upper() or p.device.upper() == last) else 1)
    return ports


def remember_port(dev):
    from serial.tools import list_ports
    addr = dev
    for p in list_ports.comports():
        if p.device == dev:
            m = re.search(r"&([0-9A-F]{12})_", (p.hwid or "").upper())
            if m and m.group(1) != "000000000000":
                addr = m.group(1)
    try:
        open(LAST_PORT_FILE, "w").write(addr)
    except Exception:
        pass


def find_robot(baud, route):
    for p in candidate_ports():
        route.status = f"trying {p.device} ..."
        try:
            ser = open_port(p.device, baud)
        except Exception:
            continue
        try:
            ser.write(b"M")
            rd = LineReader(ser)
            end = time.time() + 5
            while time.time() < end:
                lines, _ = rd.lines()
                for line in lines:
                    if line.split(" ", 1)[0] in KNOWN:
                        route.feed(line)
                        remember_port(p.device)
                        return ser, p.device
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
    return None, None


class Link:
    def __init__(self, route, port, baud, logfile):
        self.route, self.port, self.baud, self.logfile = route, port, baud, logfile
        self.ser = None
        threading.Thread(target=self.run, daemon=True).start()

    def send(self, ch):
        try:
            if self.ser:
                self.ser.write(ch.encode())
                return True
        except Exception:
            pass
        return False

    def run(self):
        while True:
            if self.port:
                self.route.status = f"connecting to {self.port} ..."
                try:
                    self.ser = open_port(self.port, self.baud)
                    dev = self.port
                except Exception as e:
                    self.route.status = f"{self.port}: {e} - car on and paired? retrying..."
                    time.sleep(3)
                    continue
            else:
                self.ser, dev = find_robot(self.baud, self.route)
                if not self.ser:
                    self.route.status = "car not found - switched on and paired? retrying..."
                    time.sleep(3)
                    continue
                self.port = dev
            self.route.status = f"connected on {dev}"
            try:
                rd = LineReader(self.ser)
                last_rx = time.time()
                while True:
                    lines, got = rd.lines()
                    if got:
                        last_rx = time.time()
                    for line in lines:
                        if line != "HB":
                            if self.logfile:
                                self.logfile.write(line + "\n")
                                self.logfile.flush()
                            if not line.startswith("PROG"):
                                print(line)
                        self.route.feed(line)
                    if time.time() - last_rx > SILENCE_RECONNECT_S:
                        raise IOError("no data from the car for %d s" % SILENCE_RECONNECT_S)
            except Exception as e:
                self.route.status = f"connection lost ({e}) - reconnecting..."
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(2)


def replay(path, route, delay):
    route.status = f"replaying {path}"
    for line in open(path):
        line = line.strip()
        if line:
            route.feed(line)
            time.sleep(delay / 4 if line.startswith("PROG") else delay)
    route.status = "replay finished"


# ------------------------------------------------------------------ drawing
COL = {1: "#000000", 2: "#1565c0"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay")
    ap.add_argument("--delay", type=float, default=0.05)
    a = ap.parse_args()

    route = Route()
    link = None
    if a.replay:
        threading.Thread(target=replay, args=(a.replay, route, a.delay), daemon=True).start()
    else:
        logname = os.path.join(HERE, time.strftime("robot_log_%Y%m%d_%H%M%S.txt"))
        link = Link(route, a.port, a.baud, open(logname, "w"))

    fig = plt.figure(figsize=(11, 8.5))
    fig.canvas.manager.set_window_title("Team5 - live maze map")
    ax = fig.add_axes([0.03, 0.27, 0.94, 0.68])
    lax = fig.add_axes([0.03, 0.01, 0.94, 0.24]); lax.axis("off")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([]); ax.set_yticks([])

    def on_key(ev):
        if ev.key and len(ev.key) == 1 and ev.key.lower() in KEYS:
            ok = link.send(ev.key.upper()) if link else False
            route.feed(f"LOG >> sent {ev.key.upper()}" + ("" if ok else " (not connected)"))

    fig.canvas.mpl_connect("key_press_event", on_key)

    def tick(_):
        with route.lock:
            traces = [(n, list(pts)) for n, pts in route.traces]
            marks = list(route.marks)
            car = route.car()
            state, path, solved = route.state, route.path, route.solved
            log, status = list(route.log), route.status
        ax.clear(); ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_visible(False)
        xs, ys = [0], [0]
        for i, (n, pts) in enumerate(traces):
            if i == len(traces) - 1:
                pts = pts + [car[:2]]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=COL.get(n, "#888"),
                    lw=7 if n == 2 else 4.5, alpha=1.0, solid_capstyle="round", solid_joinstyle="round",
                    antialiased=True, zorder=1 + n)
            xs += [p[0] for p in pts]; ys += [p[1] for p in pts]
        for kind, letter, x, y in marks:
            if kind == "J":
                ax.plot(x, y, "o", color="white", mec="#333", ms=9, zorder=5)
                ax.text(x, y, letter, ha="center", va="center", fontsize=6, weight="bold", zorder=6)
            elif kind == "D":
                ax.plot(x, y, "x", color="#d64545", ms=9, mew=2, zorder=5)
            elif kind == "G":
                ax.plot(x, y, "o", color="#d64545", ms=16, mec="#222", zorder=5)
                ax.text(x, y, "G", ha="center", va="center", color="white", weight="bold", fontsize=8, zorder=6)
            elif kind == "H":
                ax.plot(x, y, "o", color="#2e9e4f", ms=16, mec="#222", zorder=5)
                ax.text(x, y, "H", ha="center", va="center", color="white", weight="bold", fontsize=8, zorder=6)
        ax.plot(car[0], car[1], marker=ARROW[car[2]], ms=16, color="#8a3ffc", mec="white", zorder=7)
        span = max(max(xs) - min(xs), max(ys) - min(ys), 600)
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        ax.set_xlim(cx - span * 0.6, cx + span * 0.6)
        ax.set_ylim(cy - span * 0.6, cy + span * 0.6)
        ax.set_title(f"{state}   |   black = exploring, blue = shortest way home   (up = the way the car left home)", fontsize=10)
        lax.clear(); lax.axis("off")
        lax.text(0, 0.98, status, fontsize=9, weight="bold", va="top",
                 color="#2e9e4f" if status.startswith("connected") else "#c0392b")
        lax.text(0, 0.80, f"path being learned:  {path or '-'}", fontsize=10, va="top", family="monospace")
        lax.text(0, 0.65, f"SOLVED path S->G:    {solved or '-'}", fontsize=10, va="top", family="monospace",
                 weight="bold", color="#2a7de1")
        lax.text(0, 0.48, "\n".join(log[-5:]), fontsize=8, va="top", family="monospace")
        lax.text(0, 0.0, "Keys:  L start left-hand   R start right-hand   E stop   B return trip (at goal)   S sensors",
                 fontsize=8, va="bottom")

    anim = FuncAnimation(fig, tick, interval=150, cache_frame_data=False)
    plt.show()
    return anim


if __name__ == "__main__":
    main()
