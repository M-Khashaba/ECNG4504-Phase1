# Maze robot live map viewer (laptop side, Python 3 + pyserial + matplotlib).
# Connects to the ESP32 over Bluetooth, draws the map as the robot explores
# it and lets the user send commands (E explore, G goal, H home, X stop ...).

import argparse
import json
import os
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
HEAD_MARK = ["^", ">", "v", "<"]
MODES = {0: "idle", 1: "EXPLORING", 2: "DRIVING SHORTEST PATH"}
KEYS = set("eghrmcxs")
KNOWN = ("LOG", "MAP", "NODE", "POS", "PROG", "PATH", "MAPEND", "#", "HB")
SILENCE_RECONNECT_S = 8
HERE = os.path.dirname(os.path.abspath(__file__))


# ----------------------------------------------------------------------
# Live state received from the robot: map nodes, shortest path, car position, log
# ----------------------------------------------------------------------
class Maze:
    def __init__(self):
        self.lock = threading.Lock()
        self.nodes = {}
        self.goal = -1
        self.path = []
        self.car = None
        self.mode = 0
        self.log = []
        self.status = "connecting..."
        self.want_png = False
        self.trail = []
        self.msgs = 0
        self.version = 0
        self.last_rx = 0.0

    def feed(self, line):
        p = line.split()
        if not p:
            return
        with self.lock:
            if not line.startswith("LOG >>"):
                self.msgs += 1
                self.last_rx = time.time()
            t = p[0]
            try:
                if t == "MAP" and len(p) == 3:
                    self.nodes.clear()
                    self.path = []
                    self.goal = int(p[2])
                    self.version += 1
                elif t == "NODE" and len(p) == 13:
                    i = int(p[1])
                    n = dict(x=int(p[2]), y=int(p[3]), goal=int(p[4]),
                             nb=[int(v) for v in p[5:9]],
                             len=[int(v) for v in p[9:13]])
                    if self.nodes.get(i) != n:
                        self.nodes[i] = n
                        self.version += 1
                    if n["goal"]:
                        self.goal = i
                elif t == "PATH":
                    self.path = [int(v) for v in p[1:]]
                    self.version += 1
                elif t == "POS" and len(p) >= 3:
                    node = int(p[1])
                    self.car = (node, int(p[2]), 0) if node >= 0 else None
                    if len(p) >= 4:
                        self.mode = int(p[3])
                    if self.mode == 1 and node == 0 and len(self.nodes) <= 1:
                        self.trail = []
                elif t == "PROG" and len(p) >= 4:
                    node = int(p[1])
                    self.car = (node, int(p[2]), int(p[3])) if node >= 0 else None
                elif t == "MAPEND":
                    self.want_png = True
                elif t == "LOG":
                    self.log.append(line[4:])
                    self.log = self.log[-7:]
            except ValueError:
                pass

    def car_xy(self):
        if not self.car or self.car[0] not in self.nodes:
            return None
        n = self.nodes[self.car[0]]
        h, d = self.car[1], self.car[2]
        return n["x"] + DX[h] * d, n["y"] + DY[h] * d, h

    def save(self):
        with open(os.path.join(HERE, "maze_map.json"), "w") as f:
            json.dump({"goal": self.goal, "nodes": self.nodes, "path": self.path}, f, indent=1)


# ----------------------------------------------------------------------
# Connection: finding the robot's Bluetooth / USB serial port and reading its lines
# ----------------------------------------------------------------------
def is_robot_line(line):
    return line.split(" ", 1)[0] in KNOWN

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

def find_robot(baud, maze):
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    ports.sort(key=lambda p: 0 if "bluetooth" in (p.description or "").lower() else 1)
    for p in ports:
        maze.status = f"trying {p.device} ({p.description}) ..."
        try:
            ser = open_port(p.device, baud)
        except Exception:
            continue
        try:
            ser.write(b"M")
            rd = LineReader(ser)
            end = time.time() + 4
            while time.time() < end:
                lines, _ = rd.lines()
                for line in lines:
                    if is_robot_line(line):
                        maze.feed(line)
                        return ser, p.device
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
    return None, None


# ----------------------------------------------------------------------
# Background thread: keeps the link to the robot open and sends key commands
# ----------------------------------------------------------------------
class Link:
    def __init__(self, maze, port, baud, logfile):
        self.maze, self.port, self.baud, self.logfile = maze, port, baud, logfile
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
                self.maze.status = f"connecting to {self.port} ..."
                try:
                    self.ser = open_port(self.port, self.baud)
                    self.ser.write(b"M")
                    dev = self.port
                except Exception as e:
                    self.maze.status = f"{self.port}: {e} - is the car on and paired? retrying..."
                    time.sleep(3)
                    continue
            else:
                self.ser, dev = find_robot(self.baud, self.maze)
                if not self.ser:
                    self.maze.status = "car not found - is it switched on and paired? retrying..."
                    time.sleep(3)
                    continue
                self.port = dev
            self.maze.status = f"connected on {dev}"
            try:
                rd = LineReader(self.ser)
                last_ask = time.time()
                last_rx = time.time()
                while True:
                    lines, got = rd.lines()
                    if got:
                        last_rx = time.time()
                    for line in lines:
                        if line != "HB":
                            if self.logfile:
                                self.logfile.write(line + "\n")
                            if not line.startswith("PROG"):
                                print(line)
                        self.maze.feed(line)
                    if time.time() - last_rx > SILENCE_RECONNECT_S:
                        raise IOError("no data from the car for %d s" % SILENCE_RECONNECT_S)
                    if not self.maze.nodes and time.time() - last_ask > 5:
                        self.ser.write(b"M")
                        last_ask = time.time()
            except Exception as e:
                self.maze.status = f"connection lost ({e}) - reconnecting..."
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(2)


# ----------------------------------------------------------------------
# Replay of a saved log file (testing without the robot)
# ----------------------------------------------------------------------
def replay(path, maze, delay):
    maze.status = f"replaying {path}"
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                maze.feed(line)
                time.sleep(delay if not line.startswith("PROG") else delay / 3)
    maze.status = "replay finished"


# ----------------------------------------------------------------------
# Drawing: the maze map, shortest path and the live car marker (matplotlib)
# ----------------------------------------------------------------------
class View:

    def __init__(self, fig, ax, lax):
        self.fig, self.ax, self.lax = fig, ax, lax
        self.version = -1
        ax.set_aspect("equal", adjustable="box")
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_visible(False)
        self.roads = LineCollection([], colors="#2b2b2b", linewidths=7, capstyle="round", zorder=1)
        self.stubs = LineCollection([], colors="#e8a33a", linewidths=4, linestyles=":", zorder=1)
        ax.add_collection(self.roads)
        ax.add_collection(self.stubs)
        self.trail, = ax.plot([], [], color="#8a3ffc", lw=1.5, alpha=0.35, zorder=2)
        self.path, = ax.plot([], [], color="#2a7de1", lw=3.5, zorder=3, label="shortest path")
        self.dots = None
        self.labels = []
        self.car, = ax.plot([], [], marker="^", ms=17, color="#8a3ffc", mec="white",
                            ls="none", zorder=6, label="car (live)")
        ax.legend(loc="upper left", fontsize=8, markerscale=0.6)
        self.title = ax.set_title("", fontsize=11)
        self.hint = ax.text(0.5, 0.5, "", transform=ax.transAxes, ha="center", va="center",
                            fontsize=12, color="#555")
        lax.axis("off")
        self.status = lax.text(0, 1, "", fontsize=9, weight="bold", va="top")
        self.logtxt = lax.text(0, 0.78, "", fontsize=8, family="monospace", va="top")
        lax.text(0, 0, "Keys:  X stop   E explore   G go to goal   H go home   M resend map   S sensors"
                 "      (up = the way the car faced at the start)", fontsize=8, va="bottom")
        self.bounds = None

    def rebuild_map(self, nodes, path, goal):
        lens = [l for n in nodes.values() for l in n["len"] if l > 0]
        unit = sorted(lens)[len(lens) // 2] if lens else 1000
        stub = unit * 0.3
        roads, stubs = [], []
        for i, n in nodes.items():
            for d in range(4):
                j = n["nb"][d]
                if j >= 0 and j in nodes and i < j:
                    m = nodes[j]
                    roads.append([(n["x"], n["y"]), (m["x"], m["y"])])
                elif j == -1:
                    stubs.append([(n["x"], n["y"]), (n["x"] + DX[d] * stub, n["y"] + DY[d] * stub)])
        self.roads.set_segments(roads)
        self.stubs.set_segments(stubs)
        pts = [(nodes[i]["x"], nodes[i]["y"]) for i in path if i in nodes]
        self.path.set_data([p[0] for p in pts], [p[1] for p in pts])
        if self.dots is not None:
            self.dots.remove()
            self.dots = None
        for t in self.labels:
            t.remove()
        self.labels = []
        ids = sorted(nodes)
        if not ids:
            self.bounds = None
            return
        self.dots = self.ax.scatter([nodes[i]["x"] for i in ids], [nodes[i]["y"] for i in ids], s=230,
                                    c=["#2e9e4f" if i == 0 else "#d64545" if i == goal else "white" for i in ids],
                                    edgecolors="#222", zorder=4)
        for i in ids:
            n = nodes[i]
            lbl = "S" if i == 0 else "G" if i == goal else str(i)
            self.labels.append(self.ax.text(n["x"], n["y"], lbl, ha="center", va="center", fontsize=7,
                                            color="white" if i in (0, goal) else "black",
                                            zorder=5, weight="bold"))
        xs = [nodes[i]["x"] for i in ids]
        ys = [nodes[i]["y"] for i in ids]
        self.bounds = [min(xs), max(xs), min(ys), max(ys)]
        self.fit()

    def fit(self, car=None):
        if not self.bounds:
            return
        x0, x1, y0, y1 = self.bounds
        if car:
            x0, x1, y0, y1 = min(x0, car[0]), max(x1, car[0]), min(y0, car[1]), max(y1, car[1])
            self.bounds = [x0, x1, y0, y1]
        span = max(x1 - x0, y1 - y0, 1000)
        m = span * 0.15
        self.ax.set_xlim(x0 - m, x1 + m)
        self.ax.set_ylim(y0 - m, y1 + m)

    def update(self, maze):
        with maze.lock:
            changed = maze.version != self.version
            if changed:
                nodes = {k: dict(v) for k, v in maze.nodes.items()}
                path = list(maze.path)
                self.version = maze.version
            goal = maze.goal
            log = list(maze.log)
            mode = maze.mode
            status = maze.status
            msgs = maze.msgs
            ago = time.time() - maze.last_rx if maze.last_rx else None
            car = maze.car_xy()
            if car and (not maze.trail or maze.trail[-1] != car[:2]):
                maze.trail.append(car[:2])
                maze.trail = maze.trail[-400:]
            trail = list(maze.trail)
            nnodes = len(maze.nodes)
            npath = len(maze.path)
        if changed:
            self.rebuild_map(nodes, path, goal)
        self.trail.set_data([p[0] for p in trail], [p[1] for p in trail])
        if car:
            self.car.set_data([car[0]], [car[1]])
            self.car.set_marker(HEAD_MARK[car[2]])
            b = self.bounds
            if b and not (b[0] <= car[0] <= b[1] and b[2] <= car[1] <= b[3]):
                self.fit(car)
        else:
            self.car.set_data([], [])
        title = f"{nnodes} nodes   |   car: {MODES.get(mode, '?')}"
        if goal >= 0:
            title += "   |   goal found"
        if npath > 1:
            title += f"   |   shortest path: {npath - 1} roads"
        self.title.set_text(title)
        if status.startswith("connected"):
            status += f"   -   {msgs} messages" + (f", last {ago:.0f} s ago" if ago is not None and ago > 2 else "")
        self.hint.set_text("" if nnodes else
                           ("Connected - waiting for map data.\nPut the car on the start and press EN:\n"
                            "the map draws itself as it explores." if status.startswith("connected")
                            else "Not connected to the car yet - see the red line below."))
        self.status.set_text(status)
        self.status.set_color("#2e9e4f" if status.startswith("connected") else "#c0392b")
        self.logtxt.set_text("\n".join(log[-5:]))


# ----------------------------------------------------------------------
# Main program: open the window, connect to the robot, refresh the drawing
# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="COM port (optional - found automatically)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--replay", help="replay a saved robot log instead of connecting")
    ap.add_argument("--replay-delay", type=float, default=0.05)
    a = ap.parse_args()

    if a.list:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(p.device, "-", p.description)
        return

    maze = Maze()
    link = None
    if a.replay:
        threading.Thread(target=replay, args=(a.replay, maze, a.replay_delay), daemon=True).start()
    else:
        name = time.strftime("robot_log_%Y%m%d_%H%M%S.txt")
        logfile = open(os.path.join(HERE, name), "w", buffering=1)
        print(f"saving everything the car sends to {name}")
        link = Link(maze, a.port, a.baud, logfile)

    for k in ("keymap.home", "keymap.back", "keymap.grid", "keymap.forward", "keymap.save"):
        plt.rcParams[k] = []
    fig, (ax, lax) = plt.subplots(2, 1, figsize=(9, 9.5), gridspec_kw={"height_ratios": [5, 1.1]})
    fig.canvas.manager.set_window_title("ECNG 4504 maze robot - live map")
    view = View(fig, ax, lax)

    def on_key(ev):
        if ev.key and ev.key.lower() in KEYS:
            ok = link.send(ev.key.upper()) if link else False
            maze.feed(f"LOG >> sent {ev.key.upper()}" + ("" if ok else " (not connected)"))

    def tick(_):
        view.update(maze)
        if maze.want_png:
            maze.want_png = False
            maze.save()
            fig.savefig(os.path.join(HERE, "maze_map.png"), dpi=110)
            maze.feed("LOG map saved as maze_map.png + maze_map.json")

    fig.canvas.mpl_connect("key_press_event", on_key)
    _anim = FuncAnimation(fig, tick, interval=100, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    sys.exit(main())
