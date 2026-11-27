// ==========================================================================
// ECNG 4504 - Phase 1: maze exploration, mapping and shortest path (ESP32)
// A 4-wheel car follows a black line with 5 IR sensors, explores the whole
// maze (right-hand priority), builds a map of junctions and roads, and then
// drives the shortest path from the start dot to the goal dot. The live map
// is sent over Bluetooth to the Python viewer on the laptop.
// ==========================================================================

#include <LittleFS.h>
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include <esp_system.h>


// ==========================================================================
// PINS - L298N motor driver, 5-channel IR line sensor and status LEDs
// ==========================================================================
#define ENA    25
#define ENB    26
#define L_FWD  27
#define L_BACK 14
#define R_BACK 23
#define R_FWD  33

#define LED_PIN      18
#define LED_ONBOARD   2

const uint8_t sensor[5] = {32, 34, 35, 36, 39};
const bool BLACK_IS_HIGH = false;

const bool SENSORS_REVERSED = false;
const bool MOTORS_SWAPPED   = false;

const long USB_BAUD = 115200;
const char* BT_NAME = "MazeRobot";
const bool AUTO_RUN = true;
const unsigned long BT_WAIT_MS = 0;


// ==========================================================================
// SPEED - motor speeds and the PD line-following gains
// ==========================================================================

const int SPEED_PERCENT = 56;

const int TUNED_PERCENT  = 73;
const int MOTOR_DEADBAND = 15;

#define SPD(v) ((int)((v) * SPEED_PERCENT / 100))
#define TMS(ms) ((unsigned long)((ms) * 100.0f / TUNED_PERCENT * \
                 (float)(TUNED_PERCENT - MOTOR_DEADBAND) / (float)(SPEED_PERCENT - MOTOR_DEADBAND)))

const int SPEED        = SPD(100);
const int MAX_FOLLOW   = SPD(125);
const int SLOW_SPEED   = SPEED;
const int SCAN_SPEED   = SPD(100);

const int TURN_SPEED      = 140;
const int TURN_SLOW_SPEED = 115;

const unsigned long SPIN_STALL_MS   = 700;
const unsigned long FOLLOW_STALL_MS = 1200;

const int DOT_DRIVE = 85;

const unsigned long SPIN_BRAKE_MS = 25;

const int SPIN_KICK_PWM = 170;
const unsigned long SPIN_KICK_MS = 60;
const int SHARP_SPEED     = 100;
const float KP         = 28.0f * SPEED_PERCENT / 100;

const float KD         = 10.0f * SPEED_PERCENT / 100;
const unsigned long D_WINDOW_MS = 40;

const unsigned long BRAKE_MS = 60;


// ==========================================================================
// TIMING - junction creeping, turning and line-recovery times
// ==========================================================================

const unsigned long NODE_TIME          = TMS(130);
const unsigned long CLEAR_MAX_MS       = TMS(120);
const unsigned long DOT_CONFIRM_MS     = TMS(120);
const unsigned long PIVOT_CREEP_MS     = TMS(140);

const unsigned long COOLDOWN           = TMS(300);
const unsigned long DEADEND_CONFIRM_MS = TMS(120);
const unsigned long GOAL_BACKOFF_MS    = TMS(230);

const unsigned long BACKUP_MAX_MS      = TMS(900);
const unsigned long BACKUP_EXTRA_MS    = TMS(150);
const unsigned long SEARCH_BACK_MS     = TMS(500);

const unsigned long MIN_TURN           = 250;
const unsigned long MAX_TURN_PER_LINE  = 2700;

const unsigned long T90_GUESS_MS       = 750;
const float SEARCH_SWEEP_DEG = 50;
const float SHARP_MAX_DEG    = 45;
const float SPIN_GIVEUP_QTRS = 2.5f;

const unsigned long LINE_DEBOUNCE      = 20;
const unsigned long LOST_GRACE_MS      = 150;
const unsigned long SWEEP_MS           = 180;
const unsigned long BRANCH_MIN_MS      = 15;


// ==========================================================================
// MAPPING - map size limits and matching tolerances
// ==========================================================================

const float   MATCH_TOL = TMS(450);

const float   LOOP_ERR  = 0.15f;

const float   SAME_JUNCTION = TMS(450) * 0.6f;
const float   STUCK_DIST = TMS(11700);
const float   SHARP_PROGRESS = TMS(44);
const uint8_t MAX_NODES = 120;

const int8_t NO_EXIT    = -2;
const int8_t UNEXPLORED = -1;

const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};

struct Node {
  int16_t  x, y;
  int8_t   nb[4];
  uint16_t len[4];
  uint8_t  goal;
};

Node    nodes[MAX_NODES];
uint8_t nodeCount = 0;
int8_t  goalNode  = -1;
uint8_t visits[MAX_NODES];
bool    doneReported[MAX_NODES];


// ==========================================================================
// ROBOT STATE - map storage, current node, heading and run mode
// ==========================================================================
enum Mode : uint8_t { IDLE, EXPLORE, NAVIGATE };
Mode mode = IDLE;

int8_t  curNode   = 0;
uint8_t heading   = 0;
int8_t  nextNode  = UNEXPLORED;
bool    posKnown  = false;
bool    onGoalSquare = false;
bool    autoSolveAfterHome = false;
int8_t  solveFrom = 0, solveTo = -1;
bool    solveRunActive = false;
bool    slowApproach = false;
float   slowEndDist = 0;
bool    mapRequested = false;
unsigned long sharpSince = 0;

int   sharpTries = 0;
float sharpAtDist = -1;
unsigned long lastSpinMs = 0;
bool    skipTurnCheck = false;
float   t90avg = 0;
int     t90n = 0;
int     followL = 0, followR = 0;

float   segDist  = 0;
unsigned long lastTick = 0;

bool nodeArmed = false;

bool settleNeeded = false;
unsigned long settleStart = 0, centredSince = 0;
const unsigned long SETTLE_MS = 80;
const unsigned long SETTLE_MAX_MS = 600;
unsigned long lastNode = 0;
int   lastDirection = 1;
float lastError = 0;

int8_t  path[MAX_NODES];
uint8_t pathLen = 0, pathIdx = 0;
int8_t  navTarget = -1;
unsigned long navStartMs = 0;
unsigned long lastCentredMs = 0;

float   odoScale = 1.0f;
bool    weakTrigger = false;
int8_t  forcedSide = 0;
int8_t  cameFromDir = -1;
bool    turnSuspicious = false;
unsigned long exploreStartMs = 0;

const uint32_t INF = 0xFFFFFFFF;
uint32_t distArr[MAX_NODES];
int8_t   prevArr[MAX_NODES];

const char* MAP_FILE = "/maze.bin";
const uint16_t MAP_MAGIC = 0x4D31;

struct Scan { bool L, R, S, goal, deadEnd; float creep; bool fake; };

void arriveAtNode(Scan sc);
void startExplore();
bool startNavigate(int8_t target);
void halt(const char* why);
bool recoverLine(bool reverseFirst);
bool sweepLook(int dir, float deg);
unsigned long t90ms();
bool dotWiggleTest();
void printSensors(const char* what, byte s);
void stuckHalt();
bool stopRequested();


// ==========================================================================
// STATUS LED
// ==========================================================================
void ledSet(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
  digitalWrite(LED_ONBOARD, on ? HIGH : LOW);
}


// ==========================================================================
// MOTORS - direction pins + PWM speed through the L298N, active brake
// ==========================================================================

const uint32_t MOTOR_PWM_HZ = 1000;
const float LEFT_GAIN  = 1.00;
const float RIGHT_GAIN = 1.00;

// one motor side: direction + speed
void setMotor(uint8_t enPin, uint8_t fwdPin, uint8_t backPin, int sp) {
  digitalWrite(fwdPin,  sp >= 0 ? HIGH : LOW);
  digitalWrite(backPin, sp <  0 ? HIGH : LOW);
  float g = (enPin == ENA) ? LEFT_GAIN : RIGHT_GAIN;
  analogWrite(enPin, constrain((int)(abs(sp) * g + 0.5f), 0, 255));
}

const int KICK_PWM = 150;
const unsigned long KICK_MS = 40;
int lastCmdL = 0, lastCmdR = 0;

// both sides at once (with a short kick from standstill)
void drive(int left, int right) {
  if (MOTORS_SWAPPED) { int t = left; left = right; right = t; }
  bool fromRest  = (lastCmdL == 0 && lastCmdR == 0);
  bool sameWay   = (left > 0 && right > 0) || (left < 0 && right < 0);
  bool needsKick = fromRest && sameWay && abs(left) < KICK_PWM && abs(right) < KICK_PWM;
  lastCmdL = left; lastCmdR = right;
  if (needsKick) {
    int k = (left > 0) ? KICK_PWM : -KICK_PWM;
    setMotor(ENA, L_FWD, L_BACK, k);
    setMotor(ENB, R_FWD, R_BACK, k);
    delay(KICK_MS);
  }
  setMotor(ENA, L_FWD, L_BACK, left);
  setMotor(ENB, R_FWD, R_BACK, right);
}

void stopCar() {
  lastCmdL = lastCmdR = 0;
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(L_FWD, LOW); digitalWrite(L_BACK, LOW);
  digitalWrite(R_FWD, LOW); digitalWrite(R_BACK, LOW);
}

void brakeCar(unsigned long ms = BRAKE_MS) {
  lastCmdL = lastCmdR = 0;
  digitalWrite(L_FWD, LOW); digitalWrite(L_BACK, LOW);
  digitalWrite(R_FWD, LOW); digitalWrite(R_BACK, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  delay(ms);
  stopCar();
}


// ==========================================================================
// SENSORS - reading the 5 IR sensors and classifying the pattern
// ==========================================================================
bool sensorBlack(int i) {
  bool high = digitalRead(sensor[i]) == HIGH;
  return BLACK_IS_HIGH ? high : !high;
}

// 5 sensors -> bit pattern (bit 0 = left)
byte readLine() {
  byte s = 0;
  for (int i = 0; i < 5; i++)
    if (sensorBlack(i)) s |= (1 << (SENSORS_REVERSED ? 4 - i : i));
  return s;
}

int blackCount(byte s) {
  int n = 0;
  for (int i = 0; i < 5; i++) if (s & (1 << i)) n++;
  return n;
}

bool isDot(byte s) { return blackCount(s) >= 4; }

bool leftBranch(byte s)  { return (s & 0b00011) == 0b00011; }
bool rightBranch(byte s) { return (s & 0b11000) == 0b11000; }

const bool STRICT_BRANCH = true;

// is the pattern a side road off the line?
bool junctionAhead(byte s) {
  bool side = leftBranch(s) || rightBranch(s);
  if (!STRICT_BRANCH) return side;
  return side && (s & 0b00100);
}


// ==========================================================================
// USB + BLUETOOTH - messages to the laptop viewer (non-blocking) and command input
// ==========================================================================
BluetoothSerial SerialBT;
bool btReady = false;

const size_t BT_BUF_SIZE = 16384;
StreamBufferHandle_t btBuf = NULL;
uint32_t btDropped = 0;
bool     btMayWait = false;
volatile uint8_t btEvent = 0;

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {

  if (event == ESP_SPP_SRV_OPEN_EVT) btEvent = 1;
  else if (event == ESP_SPP_CLOSE_EVT) btEvent = 2;
}

void btSenderTask(void*) {
  uint8_t chunk[256];
  for (;;) {
    size_t k = xStreamBufferReceive(btBuf, chunk, sizeof(chunk), pdMS_TO_TICKS(50));
    if (k && SerialBT.hasClient()) SerialBT.write(chunk, k);

  }
}

void startBtSender() {
  btBuf = xStreamBufferCreate(BT_BUF_SIZE, 1);
  if (btBuf) xTaskCreatePinnedToCore(btSenderTask, "btSender", 4096, NULL, 1, NULL, 0);
}

void btQueue(const uint8_t* b, size_t n) {
  if (!btBuf || !btReady || !SerialBT.hasClient() || n == 0) return;
  if (n > BT_BUF_SIZE) { btDropped++; return; }
  if (btMayWait) {

    unsigned long t0 = millis();
    while (xStreamBufferSpacesAvailable(btBuf) < n && millis() - t0 < 1000) delay(2);
  }
  if (xStreamBufferSpacesAvailable(btBuf) >= n) xStreamBufferSend(btBuf, b, n, 0);
  else btDropped++;
}

struct DualOut : public Print {
  char   line[256];
  size_t len = 0;
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) {
      line[len++] = (char)b[i];
      if (b[i] == '\n' || len >= sizeof(line)) sendLine();
    }
    return n;
  }
  void sendLine() {
    if (!len) return;
    Serial.write((const uint8_t*)line, len);
    btQueue((const uint8_t*)line, len);
    len = 0;
  }
} Out;

int inAvailable() { return Serial.available() + (btReady ? SerialBT.available() : 0); }
int inRead()      { return (btReady && SerialBT.available()) ? SerialBT.read() : Serial.read(); }

char stopReason[160] = "it never started - see the first lines after EN";
void setStopReason(const char* m) { strncpy(stopReason, m, sizeof(stopReason) - 1); stopReason[sizeof(stopReason) - 1] = 0; }
void logMsg(const char* m) { Out.print("LOG "); Out.println(m); }

void btHousekeeping() {
  uint8_t ev = btEvent;
  if (ev) {
    btEvent = 0;
    logMsg(ev == 1 ? "Bluetooth: laptop CONNECTED" : "Bluetooth: laptop disconnected");
  }
  static uint32_t reportedDrops = 0;
  static unsigned long lastDropNote = 0;
  if (btDropped != reportedDrops && millis() - lastDropNote > 5000) {
    lastDropNote = millis();
    reportedDrops = btDropped;
    Out.printf("LOG Bluetooth: %lu lines dropped so far (laptop not reading fast enough)\n", (unsigned long)btDropped);
  }
}

void logNode(int i);

void sendNodeRaw(int i) {
  Node &n = nodes[i];
  Out.printf("NODE %d %d %d %d %d %d %d %d %u %u %u %u\n",
             i, n.x, n.y, n.goal,
             n.nb[0], n.nb[1], n.nb[2], n.nb[3],
             n.len[0], n.len[1], n.len[2], n.len[3]);
}

void sendNode(int i) { logNode(i); sendNodeRaw(i); }

int unexploredRoads() {
  int u = 0;
  for (int k = 0; k < nodeCount; k++) for (int d = 0; d < 4; d++) if (nodes[k].nb[d] == UNEXPLORED) u++;
  return u;
}

bool nodeHasUnexplored(int i) {
  for (int d = 0; d < 4; d++) if (nodes[i].nb[d] == UNEXPLORED) return true;
  return false;
}

void logNode(int i) {
  Node &n = nodes[i];
  const char* dn = "NESW";
  Out.printf("LOG node %d at (%d, %d)%s visit %d:", i, n.x, n.y,
             n.goal ? " GOAL" : (i == 0 ? " START" : ""), visits[i]);
  for (int d = 0; d < 4; d++) {
    if (n.nb[d] >= 0)               Out.printf(" %c=%d", dn[d], n.nb[d]);
    else if (n.nb[d] == UNEXPLORED) Out.printf(" %c=?", dn[d]);
    else                            Out.printf(" %c=-", dn[d]);
  }
  Out.printf("   | map: %d nodes, %d roads still to explore | %lu s\n", nodeCount, unexploredRoads(),
             exploreStartMs ? (millis() - exploreStartMs) / 1000 : 0UL);
}

void sendPos() { Out.printf("POS %d %d %d\n", posKnown ? curNode : -1, heading, (int)mode); }

bool setupDone = false;
void heartbeat() {
  btHousekeeping();
  static unsigned long last = 0, lastIdleNote = 0;
  if (millis() - last < 1000) return;
  last = millis();
  btQueue((const uint8_t*)"HB\n", 3);
  if (setupDone && mode == IDLE && millis() - lastIdleNote > 3000) {
    lastIdleNote = millis();
    Out.printf("LOG car is STOPPED. Reason: %s\n", stopReason);
  }
}

void sendProgress() {
  static unsigned long last = 0;
  if (millis() - last < 200) return;
  last = millis();
  Out.printf("PROG %d %d %d\n", posKnown ? curNode : -1, heading, (int)segDist);
}

// send the whole map to the viewer
void sendMap() {
  btMayWait = true;
  Out.printf("MAP %d %d\n", nodeCount, goalNode);
  for (int i = 0; i < nodeCount; i++) { sendNodeRaw(i); yield(); }
  Out.println("MAPEND");
  btMayWait = false;
}

char nodeLabel(int i) {
  if (i == 0) return 'S';
  if (i == goalNode) return 'G';
  if (i < 10) return '0' + i;
  if (i < 36) return 'a' + (i - 10);
  if (i < 62) return 'A' + (i - 36);
  return '#';
}

void printTextMap() {
  if (nodeCount == 0) { Out.println("# (map is empty)"); return; }
  const int W = 61, H = 31;
  static char g[H][W + 1];
  int minX = nodes[0].x, maxX = minX, minY = nodes[0].y, maxY = minY;
  for (int i = 1; i < nodeCount; i++) {
    minX = min(minX, (int)nodes[i].x); maxX = max(maxX, (int)nodes[i].x);
    minY = min(minY, (int)nodes[i].y); maxY = max(maxY, (int)nodes[i].y);
  }

  float sx = (W - 3) / 2.0f / max(1, maxX - minX);
  float sy = (H - 3) / (float)max(1, maxY - minY);
  float sc = min(sx, sy);
  auto col = [&](int x) { return 1 + (int)lroundf((x - minX) * sc * 2); };
  auto row = [&](int y) { return 1 + (int)lroundf((maxY - y) * sc); };

  for (int r = 0; r < H; r++) { memset(g[r], ' ', W); g[r][W] = 0; }
  for (int i = 0; i < nodeCount; i++) {
    int c1 = col(nodes[i].x), r1 = row(nodes[i].y);
    for (int d = 0; d < 4; d++) {
      int j = nodes[i].nb[d];
      if (j >= 0 && j > i) {
        int c2 = col(nodes[j].x), r2 = row(nodes[j].y);
        int steps = max(abs(c2 - c1), abs(r2 - r1));
        for (int k = 1; k < steps; k++) {
          int c = c1 + (c2 - c1) * k / steps, r = r1 + (r2 - r1) * k / steps;
          if (r >= 0 && r < H && c >= 0 && c < W) g[r][c] = (r1 == r2) ? '-' : (c1 == c2) ? '|' : '.';
        }
      } else if (j == UNEXPLORED) {
        int c = c1 + (d == 1 ? 2 : d == 3 ? -2 : 0), r = r1 + (d == 0 ? -1 : d == 2 ? 1 : 0);
        if (r >= 0 && r < H && c >= 0 && c < W) g[r][c] = '?';
      }
    }
  }
  for (int i = 0; i < nodeCount; i++) {
    int c = col(nodes[i].x), r = row(nodes[i].y);
    if (r >= 0 && r < H && c >= 0 && c < W) g[r][c] = nodeLabel(i);
  }

  btMayWait = true;
  Out.println("# ===== SAVED MAZE MAP  (S start, G goal, ? = road never explored) =====");
  for (int r = 0; r < H; r++) {
    int end = W - 1;
    while (end >= 0 && g[r][end] == ' ') end--;
    if (end < 0) continue;
    g[r][end + 1] = 0;
    Out.print("# ");
    Out.println(g[r]);
  }
  Out.printf("# %d junctions/corners/dead ends, goal %s, %d roads never explored\n",
             nodeCount, goalNode >= 0 ? "found" : "NOT found", unexploredRoads());

  const char* dn[4] = {"up", "right", "down", "left"};
  for (int i = 0; i < nodeCount; i++) {
    Out.printf("#  %c:", nodeLabel(i));
    for (int d = 0; d < 4; d++) {
      int j = nodes[i].nb[d];
      if (j >= 0)               Out.printf("  %s->%c", dn[d], nodeLabel(j));
      else if (j == UNEXPLORED) Out.printf("  %s->?", dn[d]);
    }
    Out.println();
  }
  btMayWait = false;
}

void sendPath(const int8_t* p, uint8_t n) {
  Out.print("PATH");
  for (int i = 0; i < n; i++) { Out.print(' '); Out.print(p[i]); }
  Out.println();
}

void printHelp() {
  logMsg("Commands: E explore | G go to goal | H go home | R robot at start | M map | C clear | S sensors | X stop");
}


// ==========================================================================
// FLASH STORAGE - saving / loading the explored map (LittleFS)
// ==========================================================================
void saveMap() {
  File f = LittleFS.open(MAP_FILE, "w");
  if (!f) { logMsg("flash save FAILED"); return; }
  f.write((const uint8_t*)&MAP_MAGIC, 2);
  f.write(nodeCount);
  f.write((uint8_t)goalNode);
  f.write((const uint8_t*)nodes, sizeof(Node) * nodeCount);
  f.close();
  logMsg("map saved to flash");
}

bool loadMap() {
  File f = LittleFS.open(MAP_FILE, "r");
  if (!f) return false;
  uint16_t magic = 0;
  uint8_t n = 0, g = 0;
  bool ok = f.read((uint8_t*)&magic, 2) == 2 && magic == MAP_MAGIC &&
            f.read(&n, 1) == 1 && n <= MAX_NODES &&
            f.read(&g, 1) == 1 &&
            f.read((uint8_t*)nodes, sizeof(Node) * n) == (int)(sizeof(Node) * n);
  f.close();
  if (!ok) { nodeCount = 0; goalNode = -1; return false; }
  nodeCount = n;
  goalNode  = (int8_t)g;
  if (goalNode >= nodeCount) goalNode = -1;
  return true;
}


// ==========================================================================
// MAP HELPERS - adding nodes, linking roads, node lookup
// ==========================================================================
int8_t addNode(float x, float y, const bool ex[4]) {
  if (nodeCount >= MAX_NODES) return -1;
  Node &n = nodes[nodeCount];
  n.x = (int16_t)lroundf(x);
  n.y = (int16_t)lroundf(y);
  n.goal = 0;
  for (int d = 0; d < 4; d++) { n.nb[d] = ex[d] ? UNEXPLORED : NO_EXIT; n.len[d] = 0; }
  visits[nodeCount] = 0;
  doneReported[nodeCount] = false;
  return nodeCount++;
}

void dijkstra(int8_t src);
bool isDotNode(int i) { return i == 0 || nodes[i].goal; }
float nodeDist(int a, int b) { return max(fabsf(nodes[a].x - nodes[b].x), fabsf(nodes[a].y - nodes[b].y)); }


// ==========================================================================
// MERGING DUPLICATE NODES - two map nodes that are the same junction
// ==========================================================================

int8_t aliasOf[MAX_NODES];
int8_t remap[MAX_NODES];
bool   mapChanged = false;

int8_t resolveNode(int8_t i) {
  int guard = 0;
  while (i >= 0 && aliasOf[i] >= 0 && guard++ < MAX_NODES) i = aliasOf[i];
  return i;
}
int8_t nbOf(int i, int d) { int8_t v = nodes[i].nb[d]; return v >= 0 ? resolveNode(v) : v; }

const int MQ = 24;
int8_t mqA[MQ], mqB[MQ];
int    mqN = 0;
void queueMerge(int8_t a, int8_t b) { if (mqN < MQ) { mqA[mqN] = a; mqB[mqN] = b; mqN++; } }

void mergePair(int8_t a, int8_t b) {
  a = resolveNode(a); b = resolveNode(b);
  if (a < 0 || b < 0 || a == b) return;
  int8_t keep = min(a, b), drop = max(a, b);
  if (isDotNode(keep) != isDotNode(drop)) {
    Out.printf("LOG MERGE refused: node %d and %d (one is a big dot, the other not)\n", keep, drop);
    return;
  }
  if (nodeDist(keep, drop) > MATCH_TOL * 1.5f) {
    Out.printf("LOG MERGE refused: node %d and %d are %d apart\n", keep, drop, (int)nodeDist(keep, drop));
    return;
  }
  Out.printf("LOG MERGE node %d is the same junction as node %d - joined into one\n", drop, keep);
  Node &K = nodes[keep];
  Node &D = nodes[drop];
  aliasOf[drop] = keep;
  for (int d = 0; d < 4; d++) {
    uint8_t o = (d + 2) & 3;
    int8_t kd = nbOf(keep, d);
    int8_t dd = (D.nb[d] >= 0) ? resolveNode(D.nb[d]) : D.nb[d];
    if (kd == keep) kd = NO_EXIT;
    if (dd == keep) dd = NO_EXIT;
    if (dd >= 0) {
      if (kd < 0) {
        K.nb[d] = dd; K.len[d] = D.len[d];
        nodes[dd].nb[o] = keep;
      } else if (kd == dd) {
        K.nb[d] = kd;
        nodes[kd].nb[o] = keep;
      } else {

        float lk = K.len[d], ld = D.len[d];
        float tolL = MATCH_TOL + LOOP_ERR * max(lk, ld);
        if (fabsf(lk - ld) <= tolL) {
          K.nb[d] = kd;
          queueMerge(kd, dd);
        } else {
          int8_t nearN = (lk < ld) ? kd : dd, farN = (lk < ld) ? dd : kd;
          float  nearL = min(lk, ld), farL = max(lk, ld);
          K.nb[d] = nearN; K.len[d] = (uint16_t)nearL;
          nodes[nearN].nb[o] = keep;
          int8_t nn = nbOf(nearN, d);
          if (nn < 0) {
            nodes[nearN].nb[d] = farN; nodes[nearN].len[d] = (uint16_t)max(1.0f, farL - nearL);
            nodes[farN].nb[o] = nearN; nodes[farN].len[o] = nodes[nearN].len[d];
            Out.printf("LOG node %d lies on the road %d-%d - road split\n", nearN, keep, farN);
          } else if (nn != farN) queueMerge(nn, farN);
        }
      }
    } else if (kd < 0) {
      K.nb[d] = (kd == UNEXPLORED || dd == UNEXPLORED) ? UNEXPLORED : NO_EXIT;
    } else K.nb[d] = kd;
  }
  if (D.goal) K.goal = 1;
  visits[keep] = (uint8_t)min(255, visits[keep] + visits[drop]);
  mapChanged = true;
}

void compactMap() {
  int8_t n = 0;
  for (int i = 0; i < nodeCount; i++) remap[i] = (aliasOf[i] < 0) ? n++ : -1;
  for (int i = 0; i < nodeCount; i++) if (aliasOf[i] >= 0) remap[i] = remap[resolveNode(i)];
  for (int i = 0; i < nodeCount; i++) {
    if (aliasOf[i] >= 0) continue;
    int8_t ni = remap[i];
    Node tmp = nodes[i];
    for (int d = 0; d < 4; d++) {
      if (tmp.nb[d] >= 0) {
        tmp.nb[d] = remap[tmp.nb[d]];
        if (tmp.nb[d] == ni) { tmp.nb[d] = NO_EXIT; tmp.len[d] = 0; }
      }
    }
    nodes[ni] = tmp;
    visits[ni] = visits[i];
    doneReported[ni] = doneReported[i];
  }
  auto fix = [&](int8_t v) -> int8_t { return v >= 0 ? remap[v] : v; };
  curNode  = fix(curNode);
  goalNode = fix(goalNode);
  nextNode = fix(nextNode);
  nodeCount = n;
  for (int i = 0; i < MAX_NODES; i++) aliasOf[i] = -1;

  for (int i = 0; i < nodeCount; i++)
    for (int d = 0; d < 4; d++) {
      int8_t j = nodes[i].nb[d];
      if (j >= 0 && nodes[j].nb[(d + 2) & 3] < 0) { nodes[j].nb[(d + 2) & 3] = i; nodes[j].len[(d + 2) & 3] = nodes[i].len[d]; }
    }
}

void mergeNodes(int8_t a, int8_t b) {
  for (int i = 0; i < MAX_NODES; i++) aliasOf[i] = -1;
  mqN = 0;
  queueMerge(a, b);
  for (int k = 0; k < mqN && k < MQ; k++) mergePair(mqA[k], mqB[k]);
  mqN = 0;
  compactMap();
}


// ==========================================================================
// RECOGNISING A KNOWN NODE - closing a loop in the maze
// ==========================================================================

int8_t findNode(float x, float y, uint8_t arriveFrom, const bool ex[4], bool onDot) {
  dijkstra(curNode);
  int8_t best = -1, nearest = -1;
  float bestScore = 1.0f, nearD = 1e9f;
  const char* nearWhy = "";
  for (int i = 0; i < nodeCount; i++) {
    if (i == curNode) continue;
    Node &k = nodes[i];
    float d = max(fabsf(k.x - x), fabsf(k.y - y));

    float loop = segDist + (distArr[i] == INF ? 0.0f : (float)distArr[i]);
    float tol = min(MATCH_TOL + LOOP_ERR * loop, MATCH_TOL * 1.25f);

    float ahead = (k.x - nodes[curNode].x) * DX[heading] + (k.y - nodes[curNode].y) * DY[heading];

    int diff = 0;
    for (int dd = 0; dd < 4; dd++)
      if (dd != arriveFrom && ex[dd] != (k.nb[dd] != NO_EXIT)) diff++;
    int8_t e = k.nb[arriveFrom];
    const char* why = NULL;
    if (isDotNode(i) != onDot)       why = "one is a big dot, the other not";
    else if (d > tol)                why = "too far away";
    else if (ahead < 0.4f * segDist) why = "not ahead of the car";

    else if (diff > (d < tol * 0.5f ? 2 : 1)) why = "its roads look different";

    else if (e == NO_EXIT && d > tol * 0.6f) why = "it has no road towards us";

    if (!why) {
      float sc = d / tol;
      if (sc < bestScore) { bestScore = sc; best = i; }
    } else if (d < nearD) { nearD = d; nearest = i; nearWhy = why; }
  }
  if (best >= 0)
    Out.printf("LOG MATCH: this is node %d again (off by %d)\n", best,
               (int)max(fabsf(nodes[best].x - x), fabsf(nodes[best].y - y)));
  else if (nearest >= 0)
    Out.printf("LOG MATCH: new node (nearest known: node %d, %d away - %s)\n", nearest, (int)nearD, nearWhy);
  return best;
}

void relaxPositions() {
  static float px[MAX_NODES], py[MAX_NODES];
  for (int i = 0; i < nodeCount; i++) { px[i] = nodes[i].x; py[i] = nodes[i].y; }
  for (int it = 0; it < 30; it++)
    for (int i = 1; i < nodeCount; i++) {
      float sx = 0, sy = 0; int n = 0;
      for (int d = 0; d < 4; d++) {
        int8_t j = nodes[i].nb[d];
        if (j < 0) continue;
        sx += px[j] - DX[d] * (float)nodes[i].len[d];
        sy += py[j] - DY[d] * (float)nodes[i].len[d];
        n++;
      }
      if (n) { px[i] = sx / n; py[i] = sy / n; }
    }
  for (int i = 1; i < nodeCount; i++) {
    nodes[i].x = (int16_t)constrain(lroundf(px[i]), -32000L, 32000L);
    nodes[i].y = (int16_t)constrain(lroundf(py[i]), -32000L, 32000L);
  }
}

bool scanMatches(int8_t i, const bool ex[4]) {
  if (i < 0) return false;
  for (int d = 0; d < 4; d++) if (ex[d] != (nodes[i].nb[d] != NO_EXIT)) return false;
  return true;
}

void linkRoad(int8_t a, int8_t b, float len) {
  uint16_t L = (uint16_t)constrain(len, 1.0f, 65000.0f);
  uint8_t back = (heading + 2) & 3;
  nodes[a].nb[heading] = b; nodes[a].len[heading] = L;
  nodes[b].nb[back]    = a; nodes[b].len[back]    = L;
}

int8_t attachToChain(int8_t k) {
  uint8_t back = (heading + 2) & 3;
  int8_t at = k;
  float cum = 0;
  for (int guard = 0; guard < MAX_NODES; guard++) {
    int8_t nx = nodes[at].nb[back];
    if (nx == curNode) return k;
    if (nx < 0) {
      linkRoad(curNode, at, segDist - cum);
      if (at != k) Out.printf("LOG node %d connected to node %d (drove through it unseen)\n", curNode, at);
      return k;
    }
    float L = nodes[at].len[back];
    if (fabsf(cum + L - segDist) <= MATCH_TOL && isDotNode(nx) == isDotNode(curNode)) {

      mergeNodes(nx, curNode);
      return remap[k];
    }
    if (cum + L > segDist) {

      float rest = cum + L - segDist;
      linkRoad(curNode, at, segDist - cum);
      int8_t cb = nodes[curNode].nb[back];
      if (cb < 0) {
        uint16_t R = (uint16_t)max(1.0f, rest);
        nodes[curNode].nb[back] = nx; nodes[curNode].len[back] = R;
        nodes[nx].nb[heading] = curNode; nodes[nx].len[heading] = R;
        Out.printf("LOG node %d lies on the road %d-%d - road split\n", curNode, at, nx);
        return k;
      }
      if (cb != nx) { mergeNodes(cb, nx); return remap[k]; }
      return k;
    }
    Out.printf("LOG drove straight through node %d without seeing it\n", nx);
    cum += L;
    at = nx;
  }
  linkRoad(curNode, k, segDist);
  return k;
}

int8_t dirTo(int8_t a, int8_t b) {
  int8_t best = -1;
  for (int d = 0; d < 4; d++)
    if (nodes[a].nb[d] == b && (best < 0 || nodes[a].len[d] < nodes[a].len[best])) best = d;
  return best;
}

void clearMap() {
  nodeCount = 0; goalNode = -1; posKnown = false; onGoalSquare = false;
  memset(visits, 0, sizeof(visits));
  memset(doneReported, 0, sizeof(doneReported));
  for (int i = 0; i < MAX_NODES; i++) aliasOf[i] = -1;
  LittleFS.remove(MAP_FILE);
}


// ==========================================================================
// DIJKSTRA - shortest path between two nodes over the driven roads
// ==========================================================================

// shortest distances from src to every node
void dijkstra(int8_t src) {
  bool done[MAX_NODES];
  for (int i = 0; i < nodeCount; i++) { distArr[i] = INF; prevArr[i] = -1; done[i] = false; }
  distArr[src] = 0;

  while (true) {
    int8_t u = -1;
    uint32_t best = INF;
    for (int i = 0; i < nodeCount; i++)
      if (!done[i] && distArr[i] < best) { best = distArr[i]; u = i; }
    if (u < 0) break;
    done[u] = true;

    for (int d = 0; d < 4; d++) {
      int8_t v = nodes[u].nb[d];
      if (v < 0) continue;
      uint32_t alt = distArr[u] + max<uint16_t>(1, nodes[u].len[d]);
      if (alt < distArr[v]) { distArr[v] = alt; prevArr[v] = u; }
    }
  }
}

uint8_t buildPath(int8_t src, int8_t dst, int8_t* out) {
  if (dst < 0 || dst >= nodeCount || distArr[dst] == INF) return 0;
  int8_t tmp[MAX_NODES];
  uint8_t n = 0;
  for (int8_t v = dst; v >= 0 && n < MAX_NODES; v = prevArr[v]) {
    tmp[n++] = v;
    if (v == src) break;
  }
  if (n == 0 || tmp[n - 1] != src) return 0;
  for (uint8_t i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
  return n;
}

void sendShortest() {
  if (goalNode < 0 || nodeCount == 0) { logMsg("no goal in map yet"); return; }
  dijkstra(0);
  int8_t p[MAX_NODES];
  uint8_t n = buildPath(0, goalNode, p);
  if (!n) { logMsg("goal unreachable from start"); return; }
  sendPath(p, n);
  Out.printf("LOG shortest start->goal: %u nodes, length %lu\n", n, (unsigned long)distArr[goalNode]);
}


// ==========================================================================
// COMMANDS - E explore, G goal, H home, R reset position, M map, C clear, S sensors, X stop
// ==========================================================================
void halt(const char* why) {
  brakeCar();
  mode = IDLE;
  posKnown = false;
  autoSolveAfterHome = false;
  solveRunActive = false;
  ledSet(false);
  logMsg(why);
  setStopReason(why);
  sendPos();
}

void showSensors() { printSensors("sensors (OUT1 left -> OUT5 right, # = black)", readLine()); }

// handle one-letter commands from the laptop
void pollCommands() {
  while (inAvailable()) {
    char c = toupper(inRead());
    if (c == '\r' || c == '\n' || c == ' ') continue;

    if (c == 'X') {
      if (mode != IDLE) halt("STOPPED by user - send R after placing robot at start");
      else { stopCar(); autoSolveAfterHome = false; solveRunActive = false; ledSet(false); }
      continue;
    }
    if (mode != IDLE) {

      if (c == 'M') { mapRequested = true; logMsg("map will be sent at the next junction"); }
      else if (c == 'S') showSensors();
      else logMsg("busy - send X first");
      continue;
    }

    switch (c) {
      case 'E': startExplore(); break;
      case 'G': startNavigate(goalNode); break;
      case 'H': startNavigate(0); break;
      case 'R':
        if (nodeCount == 0) { logMsg("no map - use E"); break; }
        curNode = 0; heading = 0; posKnown = true; onGoalSquare = true;
        logMsg("position set: start node, facing N");
        sendPos();
        break;
      case 'M': sendMap(); sendShortest(); printTextMap(); sendPos(); break;
      case 'C': clearMap(); logMsg("map cleared"); sendMap(); break;
      case 'S': showSensors(); break;
      default:  printHelp();
    }
  }
}

bool stopRequested() {
  heartbeat();
  pollCommands();
  return mode == IDLE;
}

bool waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    heartbeat();
    while (inAvailable()) {
      char c = toupper(inRead());
      if (c == 'X') {
        stopCar();
        mode = IDLE;
        autoSolveAfterHome = false;
        solveRunActive = false;
        ledSet(false);
        logMsg("AUTO cancelled by X - idle (position kept: G / H still work)");
        setStopReason("cancelled by an X command");
        sendPos();
        return false;
      }
      if (c == 'M') { sendMap(); sendShortest(); printTextMap(); sendPos(); }
      else if (c == 'S') showSensors();
    }
    delay(10);
  }
  return true;
}


// ==========================================================================
// MOVEMENT - turning on the spot, centring on the line, scanning and leaving junctions, the big dots
// ==========================================================================

float lineError(byte s) {
  int sum = 0, count = 0;
  for (int i = 0; i < 5; i++) if (s & (1 << i)) { sum += i - 2; count++; }
  return count ? (float)sum / count : 0;
}

void centreOnLine(int dir) {
  unsigned long t0 = millis();
  while (millis() - t0 < 400) {
    byte s = readLine();
    if (s && (s & 0b00100)) {
      float e = lineError(s);

      if ((dir > 0 && e <= 0) || (dir < 0 && e >= 0)) return;
    }
    yield();
  }
}

void fixOvershoot(int dir) {
  byte s = readLine();
  float e = lineError(s);
  bool over = (s == 0) || (dir > 0 && e < -0.3) || (dir < 0 && e > 0.3);
  if (!over) return;
  drive(-dir * TURN_SLOW_SPEED, dir * TURN_SLOW_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < 600) {
    s = readLine();
    if (s & 0b00100) {
      e = lineError(s);
      if ((dir > 0 && e >= 0) || (dir < 0 && e <= 0)) break;
    }
    yield();
  }
  brakeCar();
}

unsigned long t90ms() { return t90n >= 2 ? (unsigned long)t90avg : T90_GUESS_MS; }

// turn on the spot until the wanted line is under the centre sensor
bool spin(int dir, int lines, int quarters) {
  stopCar();
  delay(30);
  unsigned long start = millis();
  drive(dir * SPIN_KICK_PWM, -dir * SPIN_KICK_PWM);
  bool kicking = true;
  int spd = TURN_SPEED;

  unsigned long limit = (t90n >= 2) ? (unsigned long)(t90avg * (quarters + SPIN_GIVEUP_QTRS))
                                    : MAX_TURN_PER_LINE * lines;
  bool armed = false;
  unsigned long whiteSince = 0, blackSince = 0;
  int found = 0;

  unsigned long minTurn = (quarters >= 2) ? max(MIN_TURN, (unsigned long)(1.3f * t90ms())) : MIN_TURN;
  byte lastPat = 0xFF;
  unsigned long patSince = millis();
  unsigned long stallMs = 0;

  bool slowed = false;
  while (millis() - start < limit) {
    if (stopRequested()) return false;
    byte sNow = readLine();

    bool c = (sNow & 0b00100) && blackCount(sNow) <= 3;
    unsigned long now = millis();

    if (sNow != lastPat) { lastPat = sNow; patSince = now; }
    else if (!kicking && now - patSince > SPIN_STALL_MS) {
      logMsg("spin stalled - kicking again");
      stallMs += SPIN_STALL_MS;
      drive(dir * SPIN_KICK_PWM, -dir * SPIN_KICK_PWM);
      delay(SPIN_KICK_MS);
      drive(dir * spd, -dir * spd);
      patSince = now = millis();
    }

    if (kicking && now - start >= SPIN_KICK_MS) { kicking = false; if (!slowed) drive(dir * spd, -dir * spd); }

    byte outer = (dir > 0) ? 0b10000 : 0b00001;
    if (!slowed && found == lines - 1 && armed && now - start > minTurn && (sNow & outer)) {
      spd = TURN_SLOW_SPEED;
      kicking = false;
      drive(dir * spd, -dir * spd);
      slowed = true;
    }

    if (!c) {
      blackSince = 0;
      if (!whiteSince) whiteSince = now;
      if (now - whiteSince >= LINE_DEBOUNCE) armed = true;
    } else {
      whiteSince = 0;
      if (armed && now - start > minTurn) {
        if (!blackSince) blackSince = now;
        if (now - blackSince >= LINE_DEBOUNCE) {
          found++;
          armed = false;
          blackSince = 0;
          if (found >= lines) {
            lastSpinMs = now - start;

            if (stallMs) skipTurnCheck = true;
            if (!slowed) drive(dir * TURN_SLOW_SPEED, -dir * TURN_SLOW_SPEED);
            centreOnLine(dir);
            drive(-dir * TURN_SPEED, dir * TURN_SPEED);
            delay(SPIN_BRAKE_MS);
            brakeCar();
            fixOvershoot(dir);
            return true;
          }
        }
      }
    }
    yield();
  }
  brakeCar();
  return false;
}

bool turnRelative(uint8_t rel) {
  Node &n = nodes[curNode];
  bool hasRight = n.nb[(heading + 1) & 3] != NO_EXIT;
  bool hasLeft  = n.nb[(heading + 3) & 3] != NO_EXIT;

  if (rel == 1) return spin(+1, 1, 1);
  if (rel == 3) return spin(-1, 1, 1);

  int rightLines = (hasRight ? 1 : 0) + 1;
  int leftLines  = (hasLeft  ? 1 : 0) + 1;
  if (leftLines < rightLines) return spin(-1, leftLines, 2);
  return spin(+1, rightLines, 2);
}

void backOffGoal() {
  drive(-DOT_DRIVE, -DOT_DRIVE);
  unsigned long t0 = millis();
  while (isDot(readLine()) && millis() - t0 < 1500) {
    if (stopRequested()) return;
    yield();
  }
  delay(GOAL_BACKOFF_MS);
  brakeCar();
}

const int  DOT_PUSH_PWM = 200;
const unsigned long DOT_PUSH_MS = 150;
void driveOffDot() {
  unsigned long t0 = millis(), lineSince = 0, lastPush = millis();
  setMotor(ENA, L_FWD, L_BACK, DOT_PUSH_PWM);
  setMotor(ENB, R_FWD, R_BACK, DOT_PUSH_PWM);
  lastCmdL = lastCmdR = DOT_PUSH_PWM;
  delay(DOT_PUSH_MS);
  drive(DOT_DRIVE, DOT_DRIVE);
  while (millis() - t0 < 7000) {
    if (stopRequested()) return;
    int b = blackCount(readLine());
    if (b >= 1 && b <= 3) {
      if (!lineSince) lineSince = millis();
      if (millis() - lineSince >= 40) return;
    } else lineSince = 0;
    if (b >= 4 && millis() - lastPush > 1200) {
      logMsg("still on the dot - pushing harder");
      setMotor(ENA, L_FWD, L_BACK, 255);
      setMotor(ENB, R_FWD, R_BACK, 255);
      delay(DOT_PUSH_MS);
      drive(DOT_DRIVE, DOT_DRIVE);
      lastPush = millis();
    }
    yield();
  }
  if (blackCount(readLine()) >= 4) stuckHalt();
}

void stuckHalt() {
  stopCar();
  logMsg("!!! STUCK: the motors were driven but the car did not move off the dot !!!");
  logMsg("!!! Check the IN3 wire is on GPIO 23, ESP32 GND joined to L298N GND, supply current >= 3 A !!!");
  halt("STUCK - wheels not turning");
}

// turn to the chosen road and start driving along it
void leaveNode(int8_t dir) {
  if (dir < 0) { halt("internal error: no road"); return; }
  uint8_t rel = (dir - heading + 4) & 3;

  if (onGoalSquare) {
    if (rel == 0) driveOffDot();
    else backOffGoal();
    onGoalSquare = false;
    if (mode == IDLE) return;
  }

  if (rel != 0 && !turnRelative(rel)) {
    if (mode == IDLE) return;

    logMsg("!!! WARN: the turn found no line at all - looking nearby (the map may be off after this)");
    lastDirection = (rel == 3) ? -1 : 1;
    if (!recoverLine(false)) { if (mode != IDLE) halt("TURN FAILED - no line found near the junction - STOPPED"); return; }
    skipTurnCheck = true;
  }
  else if (rel == 1 || rel == 3) {

    Out.printf("LOG turn took %lu ms (normal 90 degrees: %d ms)\n", lastSpinMs, (int)t90avg);
    if (mode == EXPLORE && t90n >= 3 && !skipTurnCheck && nodes[curNode].nb[dir] == UNEXPLORED &&
        lastSpinMs > t90avg * 1.75f) {
      int q = constrain((int)lroundf(lastSpinMs / t90avg), 2, 3);
      uint8_t realRel = (rel == 1) ? q : (4 - q);
      Out.printf("LOG no road on the %s after all (turn took %lu ms, normal %d) - fixed the map\n",
                 rel == 1 ? "right" : "left", lastSpinMs, (int)t90avg);
      if (nodes[curNode].nb[dir] == UNEXPLORED) nodes[curNode].nb[dir] = NO_EXIT;
      dir = (heading + realRel) & 3;
      if (nodes[curNode].nb[dir] == NO_EXIT) nodes[curNode].nb[dir] = UNEXPLORED;
      rel = realRel;
      sendNode(curNode);
    } else if (!skipTurnCheck && lastSpinMs > MIN_TURN && (t90n < 2 || lastSpinMs < t90avg * 1.7f)) {

      t90avg = t90n ? 0.7f * t90avg + 0.3f * lastSpinMs : lastSpinMs;
      t90n++;
    }
  }

  turnSuspicious = (rel == 1 || rel == 3) && (skipTurnCheck || (t90n >= 2 && lastSpinMs > 1.45f * t90avg));
  slowApproach = false;
  skipTurnCheck = false;

  heading   = dir;
  nextNode  = nodes[curNode].nb[dir];
  if (rel == 1) lastDirection = 1;
  if (rel == 3) lastDirection = -1;

  nodeArmed = false;
  sharpTries = 0; sharpAtDist = -1;
  settleNeeded = (rel != 0);
  settleStart = millis(); centredSince = 0;
  lastNode  = millis();
  sharpSince = 0;
  followL = followR = 0;
  segDist   = 0;
  lastTick  = millis();
  sendPos();
}

void resumeRoad() {
  nodeArmed = false;
  settleNeeded = false;
  sharpSince = 0;
  followL = followR = 0;
  lastNode = millis();
  lastTick = millis();
}

// creep over a junction and record which roads exist
Scan scanNode(byte s) {
  Scan r = {false, false, false, false, false, 0, false};

  unsigned long lSeen = 0, rSeen = 0, tPrev = millis();
  auto watch = [&](byte x) {
    unsigned long now = millis(), dt = max(1UL, now - tPrev);
    tPrev = now;
    if (leftBranch(x))  { lSeen += dt; if (lSeen >= BRANCH_MIN_MS) r.L = true; }
    if (rightBranch(x)) { rSeen += dt; if (rSeen >= BRANCH_MIN_MS) r.R = true; }
  };
  watch(s);

  int8_t fside = forcedSide;
  bool weak = weakTrigger;
  forcedSide = 0;
  weakTrigger = false;

  unsigned long movedMs = 0, moveStart = millis();

  drive(SCAN_SPEED, SCAN_SPEED);
  unsigned long t0 = millis();
  byte x = s;
  while (millis() - t0 < NODE_TIME) {
    x = readLine();
    watch(x);
    yield();
  }

  if (isDot(x)) {
    t0 = millis();
    while (isDot(x) && millis() - t0 < DOT_CONFIRM_MS) { x = readLine(); yield(); }
    if (isDot(x)) {
      movedMs += millis() - moveStart;
      brakeCar();
      if (dotWiggleTest()) {
        r.goal = true;
        r.L = r.R = false;
        r.creep = movedMs * (float)SCAN_SPEED / MAX_FOLLOW;
        return r;
      }
      drive(SCAN_SPEED, SCAN_SPEED);
      moveStart = millis();
      x = readLine();
    }
  }

  t0 = millis();
  while ((x & 0b10001) && millis() - t0 < CLEAR_MAX_MS) {
    x = readLine();
    watch(x);
    yield();
  }

  int n2 = 0, hit2 = 0;
  t0 = millis();
  while (millis() - t0 < PIVOT_CREEP_MS) {
    if (millis() - t0 > PIVOT_CREEP_MS / 2) {
      n2++;
      if (readLine() & 0b01110) hit2++;
    }
    yield();
  }

  movedMs += millis() - moveStart;
  brakeCar();
  delay(20);
  x = readLine();
  r.creep = movedMs * (float)SCAN_SPEED / MAX_FOLLOW;

  if (isDot(x)) {
    r.goal = true;
    r.L = r.R = false;
  } else {
    bool nowOn = (x & 0b01110) != 0;
    r.S = n2 > 0 && (hit2 * 2 >= n2 || (nowOn && hit2 * 3 >= n2));
    if (weak) {
      bool seenSide = r.L || r.R;
      if (!seenSide && r.S) {

        r.fake = true;
      } else if (!seenSide) {

        if (fside > 0) r.R = true;
        if (fside < 0) r.L = true;
      }
    }
  }
  return r;
}

const unsigned long DOT_WIGGLE_MS = 120;
bool dotWiggleTest() {
  const int dirs[3] = {1, -1, 1};
  const unsigned long dur[3] = {DOT_WIGGLE_MS, 2 * DOT_WIGGLE_MS, DOT_WIGGLE_MS};
  int n = 0, black = 0;
  for (int k = 0; k < 3; k++) {
    drive(dirs[k] * TURN_SPEED, -dirs[k] * TURN_SPEED);
    unsigned long t0 = millis();
    while (millis() - t0 < dur[k]) {
      n++;
      if (isDot(readLine())) black++;
      yield();
    }
  }
  brakeCar();
  return n > 0 && black * 100 >= n * 85;
}


// ==========================================================================
// EXPLORATION - depth-first search with right-hand priority (right > straight > left)
// ==========================================================================

void reportExploredNodes() {
  for (int i = 0; i < nodeCount; i++) {
    if (!doneReported[i] && !nodeHasUnexplored(i)) {
      doneReported[i] = true;
      Out.printf("LOG node %d fully explored (all its roads driven, visited %d times)\n", i, visits[i]);
    }
  }
}

// next road to explore from this node
int8_t pickExploreDir(int8_t id) {

  const uint8_t order[4] = {1, 0, 3, 2};
  for (int k = 0; k < 4; k++) {
    uint8_t d = (heading + order[k]) & 3;
    if (nodes[id].nb[d] == UNEXPLORED) return d;
  }

  dijkstra(id);
  int8_t target = -1;
  for (int i = 0; i < nodeCount; i++) {
    if (nodeHasUnexplored(i) && distArr[i] != INF && (target < 0 || distArr[i] < distArr[target])) target = i;
  }
  if (target < 0) {
    if (unexploredRoads() > 0) logMsg("WARN: some roads can't be reached over driven roads - they are skipped");
    return -1;
  }

  Out.printf("LOG node %d done - going back to node %d (it still has an undriven road)\n", id, target);
  int8_t step = target;
  while (prevArr[step] != id && prevArr[step] >= 0) step = prevArr[step];
  return dirTo(id, step);
}

void startExplore() {
  clearMap();
  t90avg = 0; t90n = 0;
  bool ex[4] = {true, false, false, false};
  addNode(0, 0, ex);
  visits[0] = 1;
  curNode = 0; heading = 0; posKnown = true;
  onGoalSquare = true;
  autoSolveAfterHome = false;
  mode = EXPLORE;
  cameFromDir = -1;
  exploreStartMs = millis();
  odoScale = 1.0f;
  logMsg("EXPLORING (right-hand priority: right > straight > left)");
  sendMap();
  delay(500);
  leaveNode(0);
}

unsigned long routeEstimateMs(int8_t from, int8_t to, int* nodesOut) {
  dijkstra(from);
  int8_t p[MAX_NODES];
  uint8_t n = buildPath(from, to, p);
  if (nodesOut) *nodesOut = n;
  if (!n) return 0;
  return (unsigned long)(distArr[to] / 0.8f) + (unsigned long)(n - 1) * (t90ms() + 800UL);
}
void printTimeEstimate(int8_t first, int8_t other) {
  int nh = 0, ng = 0;
  unsigned long toFirst = (curNode == first) ? 0 : routeEstimateMs(curNode, first, &nh);
  unsigned long run     = routeEstimateMs(first, other, &ng);
  Out.printf("LOG ESTIMATE: to the %s dot ~%lu s (%d nodes), wait 3 s, shortest path to the %s dot ~%lu s (%d nodes) -> solved in ~%lu s\n",
             first == 0 ? "start" : "goal", toFirst / 1000, nh, other == 0 ? "start" : "goal",
             run / 1000, ng, (toFirst + 3000 + run) / 1000 + 2);
}

void blinkLed(int times) {
  for (int i = 0; i < times; i++) {
    ledSet(true);  if (!waitMs(200)) { ledSet(false); return; }
    ledSet(false); if (!waitMs(200)) return;
  }
}

// all roads driven: blink, drive to S, then shortest path S -> G
void finishExplore() {
  brakeCar();
  mode = IDLE;
  Out.printf("LOG ===== ALL NODES EXPLORED: %d nodes, every road driven, in %lu s =====\n",
             nodeCount, (millis() - exploreStartMs) / 1000);
  saveMap();
  sendMap();
  sendShortest();
  printTextMap();
  sendPos();

  if (goalNode < 0) {
    logMsg("no goal was found - stopping");
    setStopReason("exploration finished but the goal (big black dot) was never found");
    return;
  }

  dijkstra(curNode);
  int8_t first = (distArr[0] <= distArr[goalNode]) ? 0 : goalNode;
  int8_t other = (first == 0) ? goalNode : 0;
  printTimeEstimate(first, other);
  blinkLed(5);
  if (!AUTO_RUN) { setStopReason("exploration finished - send H (home) then G (goal)"); return; }
  if (mode != IDLE) return;

  solveFrom = first; solveTo = other;
  if (curNode == first) {
    Out.printf("LOG AUTO: already on the %s dot - shortest path to the %s dot in 3 s\n",
               first == 0 ? "start" : "goal", other == 0 ? "start" : "goal");
    ledSet(true);
    if (!waitMs(3000)) return;
    solveRunActive = true;
    if (!startNavigate(other)) { solveRunActive = false; ledSet(false); }
  } else {
    Out.printf("LOG AUTO: driving to the nearer dot (the %s), then the shortest path to the other\n",
               first == 0 ? "start" : "goal");
    autoSolveAfterHome = true;
    if (!startNavigate(first)) autoSolveAfterHome = false;
  }
}

// drive the shortest path to a target node
bool startNavigate(int8_t target) {
  const char* err = NULL;
  if (nodeCount == 0)                       err = "no map - use E";
  else if (target < 0 || target >= nodeCount) err = "no goal in map";
  else if (!posKnown)                       err = "position unknown - place robot at start facing first road, send R";
  else if (target == curNode)               err = "already there";
  if (err) { logMsg(err); setStopReason(err); return false; }

  dijkstra(curNode);
  pathLen = buildPath(curNode, target, path);
  if (pathLen < 2) { logMsg("target unreachable"); setStopReason("target unreachable on the map"); return false; }

  sendPath(path, pathLen);
  Out.printf("LOG driving shortest path to node %d: %u nodes, length %lu\n",
             target, pathLen, (unsigned long)distArr[target]);
  navTarget = target;
  pathIdx = 0;
  mode = NAVIGATE;
  navStartMs = millis();
  delay(300);
  leaveNode(dirTo(curNode, path[1]));
  return true;
}

int8_t whereAfterOvershoot(int8_t first, float driven, float len, float* rest, bool* intoNew) {
  int8_t at = first;
  float cum = len;
  *intoNew = false; *rest = 0;
  for (int guard = 0; guard < MAX_NODES; guard++) {
    int8_t nx = nodes[at].nb[heading];
    if (nx == UNEXPLORED) {
      if (driven > cum + MATCH_TOL + 0.15f * cum) { *intoNew = true; *rest = driven - cum; }
      break;
    }
    if (nx < 0) break;
    float l2 = nodes[at].len[heading];
    if (driven < cum + 0.5f * l2) break;
    cum += l2;
    at = nx;
  }
  return at;
}

bool navCornerHere(bool right) {
  if (mode != NAVIGATE || pathIdx + 1 >= pathLen) return false;
  Node &nx = nodes[path[pathIdx + 1]];
  uint8_t turn = (heading + (right ? 1 : 3)) & 3;
  float expLen = nodes[curNode].len[heading];
  return nx.nb[turn] != NO_EXIT && nx.nb[heading] == NO_EXIT && segDist >= 0.5f * expLen;
}


// ==========================================================================
// ARRIVING AT A NODE - updating the map and choosing the next road
// ==========================================================================

void learnFromKnownRoad(int8_t a, uint8_t dir, float meas, bool averageLength) {
  int8_t b = nodes[a].nb[dir];
  float L = nodes[a].len[dir];
  if (b < 0 || L < MATCH_TOL) return;
  float ratio = meas / L;
  if (ratio < 0.75f || ratio > 1.33f) return;
  float k = 1.0f + 0.3f * (ratio - 1.0f);
  odoScale = constrain(odoScale * k, 0.6f, 1.6f);
  float m = meas / k;
  if (averageLength) {
    uint16_t nl = (uint16_t)lroundf(0.8f * L + 0.2f * m);
    nodes[a].len[dir] = nl;
    nodes[b].len[(dir + 2) & 3] = nl;
  }
  Out.printf("LOG distance scale %.3f (road %d-%d: map %d, measured %d)\n", odoScale, a, b, (int)L, (int)meas);
}

// a junction / dead end / dot was reached
void arriveAtNode(Scan sc) {
  if (mapRequested) { mapRequested = false; sendMap(); sendShortest(); }
  segDist  /= odoScale;
  sc.creep /= odoScale;
  uint8_t back = (heading + 2) & 3;
  bool ex[4] = {false, false, false, false};
  ex[back] = true;
  if (sc.S) ex[heading] = true;
  if (sc.R) ex[(heading + 1) & 3] = true;
  if (sc.L) ex[(heading + 3) & 3] = true;

  if (mode == EXPLORE && turnSuspicious && cameFromDir >= 0 && heading != cameFromDir) {

    int8_t P = nodes[curNode].nb[cameFromDir];

    bool intendedFits = false;
    if (nextNode >= 0) {
      float Li = nodes[curNode].len[heading];
      intendedFits = scanMatches(nextNode, ex) && fabsf(segDist + sc.creep - Li) < max(MATCH_TOL, 0.25f * Li);
    }
    if (P >= 0 && !intendedFits) {
      uint8_t h2 = cameFromDir, b2 = (h2 + 2) & 3;
      bool ex2[4] = {false, false, false, false};
      ex2[b2] = true;
      if (sc.S) ex2[h2] = true;
      if (sc.R) ex2[(h2 + 1) & 3] = true;
      if (sc.L) ex2[(h2 + 3) & 3] = true;
      float L = nodes[curNode].len[cameFromDir];
      int diff = 0;
      for (int d = 0; d < 4; d++) if (ex2[d] != (nodes[P].nb[d] != NO_EXIT)) diff++;
      bool sameLen = fabsf(segDist + sc.creep - L) < max(MATCH_TOL, 0.25f * L);
      int roadsP = 0;
      for (int d = 0; d < 4; d++) if (nodes[P].nb[d] != NO_EXIT) roadsP++;
      bool samePlace = sc.goal    ? isDotNode(P) :
                       sc.deadEnd ? (!isDotNode(P) && roadsP == 1) :
                                    (!isDotNode(P) && diff <= 1);
      if (sameLen && samePlace) {
        Out.printf("LOG the turn went all the way round - this is node %d again (where we came from), not a new road\n", P);
        heading = h2;
        nextNode = P;
        back = b2;
        for (int d = 0; d < 4; d++) ex[d] = ex2[d];
        skipTurnCheck = true;
      }
    }
  }

  if (mode == EXPLORE) {

    if (nextNode < 0 && segDist < SAME_JUNCTION && !sc.goal) {
      Node &c = nodes[curNode];
      if (sc.deadEnd) {
        logMsg("no road straight on after all - fixed the map");
        c.nb[heading] = NO_EXIT;
      } else {
        logMsg("same junction seen again a few cm on - still ONE node, its roads updated");
        c.nb[heading] = sc.S ? UNEXPLORED : NO_EXIT;
        uint8_t rd = (heading + 1) & 3, ld = (heading + 3) & 3;
        if (sc.R && c.nb[rd] == NO_EXIT) c.nb[rd] = UNEXPLORED;
        if (sc.L && c.nb[ld] == NO_EXIT) c.nb[ld] = UNEXPLORED;
      }
      skipTurnCheck = true;
      sendNode(curNode);
      reportExploredNodes();
      int8_t again = pickExploreDir(curNode);
      if (again < 0) { finishExplore(); return; }
      leaveNode(again);
      return;
    }

    int8_t id = -1;
    if (nextNode >= 0 && !sc.goal && !sc.deadEnd) {
      float expLen = nodes[curNode].len[heading];
      float driven = segDist + sc.creep;
      bool early = driven < expLen - max(MATCH_TOL * 0.5f, 0.2f * expLen);

      if (early && scanMatches(nextNode, ex) && driven > 0.5f * expLen) early = false;
      if (early) {
        if (driven < SAME_JUNCTION || !(sc.L || sc.R)) {
          Out.printf("LOG false junction ignored on a known road (%d of %d driven) - carrying on\n", (int)driven, (int)expLen);
          segDist = driven * odoScale;
          resumeRoad();
          return;
        }
        int8_t far = nextNode;
        bool ex2[4] = {ex[0], ex[1], ex[2], ex[3]};
        ex2[heading] = true;
        id = addNode(nodes[curNode].x + DX[heading] * driven, nodes[curNode].y + DY[heading] * driven, ex2);
        if (id < 0) { halt("MAP FULL"); return; }
        uint16_t a = (uint16_t)max(1.0f, driven), b = (uint16_t)max(1.0f, expLen - driven);
        nodes[curNode].nb[heading] = id;  nodes[curNode].len[heading] = a;
        nodes[id].nb[back] = curNode;     nodes[id].len[back] = a;
        nodes[id].nb[heading] = far;      nodes[id].len[heading] = b;
        nodes[far].nb[back] = id;         nodes[far].len[back] = b;
        Out.printf("LOG junction on the known road %d-%d that was missed before - added as node %d\n", curNode, far, id);
        sendMap();
      }
    }
    segDist += sc.creep;
    if (id < 0 && nextNode >= 0) {

      float rest; bool intoNew;
      int8_t at = whereAfterOvershoot(nextNode, segDist, nodes[curNode].len[heading], &rest, &intoNew);
      if (at != nextNode || intoNew)
        Out.printf("LOG drove straight past node %d without seeing it (road %d long, drove %d) - now %s %d\n",
                   nextNode, (int)nodes[curNode].len[heading], (int)segDist, intoNew ? "on a new road after node" : "at node", at);
      if (intoNew) { curNode = at; segDist = rest; nextNode = UNEXPLORED; }
      else id = at;
      if (!intoNew && at == nextNode && scanMatches(at, ex)) learnFromKnownRoad(curNode, heading, segDist, true);
    }
    if (id < 0) {
      float nx = nodes[curNode].x + DX[heading] * segDist;
      float ny = nodes[curNode].y + DY[heading] * segDist;

      id = sc.deadEnd ? -1 : findNode(nx, ny, back, ex, sc.goal);
      bool closedLoop = (id >= 0);
      if (id < 0) {
        id = addNode(nx, ny, ex);
        if (id < 0) { halt("MAP FULL"); return; }
      }
      int8_t kb = nodes[id].nb[back];
      if (kb >= 0 && kb != curNode) {

        id = attachToChain(id);
        mapChanged = true;
      } else linkRoad(curNode, id, segDist);
      if (closedLoop) { relaxPositions(); mapChanged = true; }
      if (mapChanged) { mapChanged = false; sendMap(); }
      else sendNode(curNode);
    }

    if (sc.goal && !nodes[id].goal && id != 0) {
      nodes[id].goal = 1;
      goalNode = id;
      logMsg("GOAL FOUND - continuing exploration");
    }
    curNode = id;
    cameFromDir = back;
    onGoalSquare = sc.goal;
    if (visits[id] < 255) visits[id]++;
    sendNode(id);
    reportExploredNodes();

    int8_t dir = pickExploreDir(id);
    if (dir < 0) { finishExplore(); return; }
    leaveNode(dir);
    return;
  }

  if (pathIdx + 1 >= pathLen) { halt("path overrun"); return; }
  int8_t expected = path[pathIdx + 1];
  float  expLen   = nodes[curNode].len[heading];
  float  driven   = segDist + sc.creep;

  bool early = driven < expLen - max(MATCH_TOL * 0.5f, 0.2f * expLen);
  bool looksRight = scanMatches(expected, ex) && driven > 0.5f * expLen;
  if (!sc.goal && !sc.deadEnd && early && !looksRight) {
    Out.printf("LOG false junction ignored (%d of %d driven) - carrying on\n", (int)driven, (int)expLen);
    segDist = driven * odoScale;
    resumeRoad();
    return;
  }

  float rest; bool intoNew;
  int8_t at = whereAfterOvershoot(expected, driven, expLen, &rest, &intoNew);
  if (intoNew) { halt("LOST - drove past the end of the known map"); return; }
  if (at != expected) {
    Out.printf("LOG drove straight past node %d without seeing it (road %d long, drove %d) - now at node %d, re-planning\n",
               expected, (int)expLen, (int)driven, at);
    curNode = at;
    onGoalSquare = sc.goal;
    if (at != navTarget) {
      unsigned long t0 = navStartMs;
      if (startNavigate(navTarget)) navStartMs = t0;
      else halt("could not re-plan the route");
      return;
    }
  } else {
    if (scanMatches(expected, ex) || sc.goal) learnFromKnownRoad(curNode, heading, driven, false);
    pathIdx++;
    curNode = expected;
    onGoalSquare = sc.goal;
  }

  for (int d = 0; d < 4; d++)
    if (ex[d] != (nodes[curNode].nb[d] != NO_EXIT) && !sc.goal) {
      Out.printf("LOG WARN node %d looks different than the map\n", curNode);
      break;
    }

  if (curNode == navTarget) {
    brakeCar();
    mode = IDLE;
    unsigned long secs10 = (millis() - navStartMs) / 100;
    const char* where = curNode == goalNode ? "GOAL" : curNode == 0 ? "START" : "node";
    if (solveRunActive) {
      solveRunActive = false;
      ledSet(false);
      Out.printf("LOG ===== SOLVED: shortest path driven, ARRIVED AT %s in %lu.%lu s =====\n",
                 where, secs10 / 10, secs10 % 10);
      setStopReason("SOLVED - shortest path driven");
    } else {
      Out.printf("LOG ARRIVED AT %s in %lu.%lu s\n", where, secs10 / 10, secs10 % 10);
      setStopReason(curNode == goalNode ? "ARRIVED AT GOAL - finished" : "ARRIVED");
    }
    sendPos();
    if (autoSolveAfterHome && curNode == solveFrom) {
      autoSolveAfterHome = false;
      ledSet(true);
      Out.printf("LOG AUTO: on the %s dot - shortest path to the %s dot in 3 s\n",
                 solveFrom == 0 ? "start" : "goal", solveTo == 0 ? "start" : "goal");
      if (!waitMs(3000)) return;
      solveRunActive = true;
      if (!startNavigate(solveTo)) { solveRunActive = false; ledSet(false); }
    }
    return;
  }
  leaveNode(dirTo(curNode, path[pathIdx + 1]));
}


// ==========================================================================
// LINE FOLLOWING - PD controller and sharp-bend handling
// ==========================================================================

// PD steering on the line
void followLine(byte s) {
  int sum = 0, count = 0;
  for (int i = 0; i < 5; i++)
    if (s & (1 << i)) { sum += i - 2; count++; }
  if (!count) return;

  float error = (float)sum / count;
  lastError = error;
  if (error < -0.3) lastDirection = -1;
  else if (error > 0.3) lastDirection = 1;

  if (error >= 1.75 || error <= -1.75) {
    if (!sharpSince) {

      if (sharpAtDist >= 0 && fabsf(segDist - sharpAtDist) < SHARP_PROGRESS) sharpTries++;
      else sharpTries = 1;
      sharpAtDist = segDist;
      if (sharpTries >= 3 && (mode == EXPLORE || navCornerHere(error > 0))) {
        sharpTries = 0; sharpAtDist = -1;
        printSensors(error > 0 ? "stuck at a sharp bend - taking it as a corner to the RIGHT" : "stuck at a sharp bend - taking it as a corner to the LEFT", s);
        brakeCar();
        Scan sc = {error < 0, error > 0, false, false, false, 0, false};
        arriveAtNode(sc);
        return;
      }
      sharpSince = millis();

      printSensors(error > 0 ? "sharp bend RIGHT (not a junction?) sensors" : "sharp bend LEFT (not a junction?) sensors", s);
      int d = (error > 0) ? 1 : -1;

      drive(d * KICK_PWM, -d * KICK_PWM);
      delay(50);
    }

    unsigned long sharpMax = (unsigned long)(t90ms() * SHARP_MAX_DEG / 90.0f *
                             (float)(TURN_SPEED - MOTOR_DEADBAND) / (float)(SHARP_SPEED - MOTOR_DEADBAND));
    if (millis() - sharpSince > sharpMax) {

      sharpSince = 0;
      brakeCar();
      logMsg("sharp bend: no line within 45 degrees - turning back");
      int d = (error > 0) ? 1 : -1;
      if (sweepLook(-d, SHARP_MAX_DEG)) { lastTick = millis(); return; }
      if (!recoverLine(true) && mode != IDLE) halt("LINE NOT FOUND - STOPPED (it reversed and looked 50 degrees each side)");
      return;
    }
    int d = (error > 0) ? 1 : -1;
    drive(d * SHARP_SPEED, -d * SHARP_SPEED);
    followL = followR = 0;
    lastTick = millis();
    return;
  }
  sharpSince = 0;

  static float errPrev = 0;
  static unsigned long tPrev = 0;
  static float dErr = 0;
  unsigned long tn = millis();
  if (tn - tPrev > 300) { errPrev = error; tPrev = tn; dErr = 0; }
  else if (tn - tPrev >= D_WINDOW_MS) { dErr = error - errPrev; errPrev = error; tPrev = tn; }

  int base = slowApproach ? SLOW_SPEED : SPEED;
  int top  = slowApproach ? SLOW_SPEED + (MAX_FOLLOW - SPEED) : MAX_FOLLOW;
  int correction = (int)(KP * error + KD * dErr);
  int l = constrain(base + correction, 0, top);
  int r = constrain(base - correction, 0, top);
  drive(l, r);
  followL = l; followR = r;

  unsigned long now = millis();
  segDist += (now - lastTick) * (l + r) / (2.0f * MAX_FOLLOW);
  lastTick = now;
}


// ==========================================================================
// LOST-LINE RECOVERY - bounded search for the line, never spins in circles
// ==========================================================================

bool sweepLook(int dir, float deg) {
  unsigned long ms = (unsigned long)(t90ms() * deg / 90.0f);
  unsigned long t0 = millis();
  drive(dir * SPIN_KICK_PWM, -dir * SPIN_KICK_PWM);
  bool kicking = true;
  while (millis() - t0 < ms) {
    if (stopRequested()) return false;
    if (kicking && millis() - t0 >= SPIN_KICK_MS) { kicking = false; drive(dir * TURN_SPEED, -dir * TURN_SPEED); }
    if (readLine() & 0b01110) { brakeCar(); return true; }
    yield();
  }
  brakeCar();
  return false;
}

bool reverseLook(unsigned long maxMs, unsigned long* took) {
  drive(-SCAN_SPEED, -SCAN_SPEED);
  unsigned long t0 = millis();
  int seen = 0;
  bool found = false;
  while (millis() - t0 < maxMs) {
    if (stopRequested()) break;
    if (readLine()) { if (++seen >= 2) { found = true; break; } }
    else seen = 0;
    yield();
  }
  if (took) *took = millis() - t0;
  if (mode == IDLE) return false;
  if (!found) brakeCar();
  return found;
}

bool lineFound(const char* how) {
  brakeCar();
  if (mode == IDLE) return false;
  Out.printf("LOG line found again (%s)\n", how);
  lastTick = millis();
  followL = followR = 0;
  return true;
}

// find the line again after losing it
bool recoverLine(bool reverseFirst) {
  logMsg(reverseFirst ? "line lost - reversing onto it, then looking max 50 degrees each side"
                      : "line lost - looking max 50 degrees each side, then reversing");
  int d = lastDirection;
  for (int attempt = 0; attempt < 2; attempt++) {
    if (mode == IDLE) return false;
    if (reverseFirst && reverseLook(SEARCH_BACK_MS, NULL)) return lineFound("reversing");
    if (sweepLook(d, SEARCH_SWEEP_DEG))      return lineFound("looking to the side it was last seen");
    if (sweepLook(-d, 2 * SEARCH_SWEEP_DEG)) return lineFound("looking to the other side");
    if (sweepLook(d, SEARCH_SWEEP_DEG))      return lineFound("turning back to straight");
    if (!reverseFirst && reverseLook(SEARCH_BACK_MS, NULL)) return lineFound("reversing");
  }
  brakeCar();
  return false;
}

bool miniSweep() {
  int d = lastDirection;
  const int dirs[3] = {d, -d, d};
  const unsigned long dur[3] = {SWEEP_MS, 2 * SWEEP_MS, SWEEP_MS};
  for (int k = 0; k < 3; k++) {
    drive(dirs[k] * TURN_SLOW_SPEED, -dirs[k] * TURN_SLOW_SPEED);
    unsigned long t0 = millis();
    while (millis() - t0 < dur[k]) {
      if (readLine()) { brakeCar(); lastTick = millis(); return true; }
      yield();
    }
  }
  brakeCar();
  return false;
}

void onLineLost() {
  brakeCar();
  bool offSide = fabsf(lastError) >= 1.5f;

  if (!offSide && miniSweep()) return;

  if (!slowApproach) {
    logMsg(offSide ? "line slipped off to the side - backing up onto it"
                   : "line ended - backing up to look again");
    unsigned long backed = 0;
    if (!reverseLook(BACKUP_MAX_MS, &backed)) {
      if (mode == IDLE) return;
      if (!recoverLine(false) && mode != IDLE)
        halt("LINE NOT FOUND - STOPPED (it reversed and looked 50 degrees each side)");
      return;
    }
    delay(BACKUP_EXTRA_MS);
    backed += BACKUP_EXTRA_MS;
    brakeCar();
    float back = backed * (float)SCAN_SPEED / MAX_FOLLOW;
    segDist = max(0.0f, segDist - back);
    slowApproach = true;
    slowEndDist = segDist + 2 * back + SAME_JUNCTION;
    nodeArmed = true;
    lastTick = millis();
    return;
  }

  if (offSide) {
    if (!recoverLine(true) && mode != IDLE)
      halt("LINE NOT FOUND - STOPPED (it reversed and looked 50 degrees each side)");
    return;
  }

  drive(SCAN_SPEED, SCAN_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < DEADEND_CONFIRM_MS) {
    if (readLine()) return;
    yield();
  }
  brakeCar();
  Scan sc = {false, false, false, false, true, 0, false};
  arriveAtNode(sc);
}


// ==========================================================================
// AUTO START - starts by itself after power-on, no laptop needed
// ==========================================================================

void printSensors(const char* what, byte s) {
  char buf[16];
  for (int i = 0; i < 5; i++) { buf[2 * i] = (s & (1 << i)) ? '#' : '.'; buf[2 * i + 1] = ' '; }
  buf[10] = 0;
  Out.printf("LOG %s: %s\n", what, buf);
}

void autoStart() {

  if (BT_WAIT_MS > 0 && btReady && !SerialBT.hasClient()) {
    logMsg("AUTO: waiting for the laptop viewer to connect...");
    unsigned long t0 = millis();
    while (!SerialBT.hasClient() && millis() - t0 < BT_WAIT_MS) { heartbeat(); delay(50); }
    logMsg(SerialBT.hasClient() ? "AUTO: laptop connected" : "AUTO: no laptop - starting anyway");
  }
  logMsg("AUTO: starting in 2 s (X = cancel)");
  if (!waitMs(2000)) return;

  if (readLine() == 0) {

    logMsg("AUTO: on white - waiting. Put it on the start, facing the line. (M = map, X = cancel)");
    unsigned long seen = 0, lastPrint = 0;
    while (true) {
      while (inAvailable()) {
        char c = toupper(inRead());
        if (c == 'M') { sendMap(); sendShortest(); printTextMap(); }
        if (c == 'S') showSensors();
        if (c == 'X') { logMsg("AUTO cancelled - idle. Commands: E G H R M C S X"); setStopReason("cancelled by an X command from the laptop"); return; }
      }
      byte s = readLine();
      if (s != 0) {
        if (!seen) seen = millis();
        if (millis() - seen > 2000) break;
      } else seen = 0;
      if (millis() - lastPrint > 3000) { lastPrint = millis(); printSensors("waiting for the start", s); }
      heartbeat();
      delay(10);
    }
  }
  printSensors("starting, sensors", readLine());
  startExplore();
}


// ==========================================================================
// SETUP AND MAIN LOOP
// ==========================================================================

// hardware, Bluetooth and saved map
void setup() {

  pinMode(L_FWD, OUTPUT);  pinMode(L_BACK, OUTPUT);
  pinMode(R_FWD, OUTPUT);  pinMode(R_BACK, OUTPUT);
  pinMode(ENA, OUTPUT);    pinMode(ENB, OUTPUT);

  analogWriteFrequency(ENA, MOTOR_PWM_HZ);
  analogWriteFrequency(ENB, MOTOR_PWM_HZ);
  stopCar();
  for (int i = 0; i < 5; i++) pinMode(sensor[i], INPUT);
  for (int i = 0; i < MAX_NODES; i++) aliasOf[i] = -1;
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_ONBOARD, OUTPUT);
  ledSet(false);

  Serial.setTxBufferSize(2048);
  Serial.begin(USB_BAUD);

  SerialBT.register_callback(btCallback);
  btReady = SerialBT.begin(BT_NAME);
  startBtSender();

  Out.println();
  if (!btReady) logMsg("!!! Bluetooth did NOT start - USB serial only !!!");
  else Out.printf("LOG Bluetooth ready as \"%s\"\n", BT_NAME);
  if (!btBuf) logMsg("!!! no memory for the Bluetooth buffer - Bluetooth output is off !!!");

  if (!LittleFS.begin(true)) logMsg("flash filesystem FAILED");
  else if (loadMap()) {
    Out.printf("LOG map loaded from flash: %u nodes\n", nodeCount);
    sendMap();
    sendShortest();
    printTextMap();
  } else logMsg("no saved map");

  esp_reset_reason_t why = esp_reset_reason();
  if (why == ESP_RST_BROWNOUT)
    logMsg("!!! RESTARTED: BROWNOUT - battery voltage dropped too low. Charge/replace the batteries, or power the ESP32 separately !!!");
  else if (why == ESP_RST_POWERON) logMsg("power-on start");
  else if (why == ESP_RST_EXT)     logMsg("EN button reset");
  else Out.printf("LOG restarted, reason code %d\n", (int)why);

  Out.printf("LOG speed %d %% (follow %d, max %d, spin %d), junction creep %lu ms\n",
             SPEED_PERCENT, SPEED, MAX_FOLLOW, TURN_SPEED, PIVOT_CREEP_MS);
  logMsg("Robot ready");
  printHelp();
  sendPos();

  if (AUTO_RUN) autoStart();
  setupDone = true;
}

// follow the line, detect junctions, run the exploration
void loop() {
  pollCommands();
  if (mode == IDLE) { stopCar(); heartbeat(); delay(5); return; }
  btHousekeeping();

  byte s = readLine();

  static unsigned long whiteSince = 0;
  if (s == 0) {
    unsigned long now = millis();

    segDist += (now - lastTick) * (followL + followR) / (2.0f * MAX_FOLLOW);
    lastTick = now;
    if (!whiteSince) whiteSince = now;
    if (now - whiteSince < LOST_GRACE_MS) return;
    whiteSince = 0;
    onLineLost();
    return;
  }
  whiteSince = 0;

  int bc = blackCount(s);
  if (settleNeeded) {
    bool centred = (s & 0b00100) && fabsf(lineError(s)) <= 0.5f;
    if (centred) { if (!centredSince) centredSince = millis(); }
    else centredSince = 0;
    if ((centredSince && millis() - centredSince >= SETTLE_MS) || millis() - settleStart > SETTLE_MAX_MS)
      settleNeeded = false;
  }
  if (!settleNeeded && bc >= 1 && bc <= 2) nodeArmed = true;

  static unsigned long dotSince = 0;
  if (bc >= 4 && !nodeArmed) {
    if (!dotSince) dotSince = millis();
    if (millis() - dotSince > 4000) { dotSince = 0; stuckHalt(); return; }
  } else dotSince = 0;

  if (segDist > STUCK_DIST) {
    logMsg("!!! no junction for a very long time - the car is probably NOT moving !!!");
    halt("STUCK - no junction found");
    return;
  }

  if (slowApproach && segDist > slowEndDist) slowApproach = false;

  if ((s & 0b00100) && bc <= 2 && fabsf(lineError(s)) <= 0.5f) lastCentredMs = millis();
  bool side = leftBranch(s) || rightBranch(s);
  bool recent = millis() - lastCentredMs < 200;

  bool outerOnly = (s == 0b10000 || s == 0b00001);
  bool junction = junctionAhead(s) || (side && (slowApproach || recent)) || (outerOnly && recent);
  if (nodeArmed && millis() - lastNode > COOLDOWN && junction) {
    forcedSide = (s & 0b10000) && !(s & 0b00001) ? 1 : (s & 0b00001) && !(s & 0b10000) ? -1 : 0;
    weakTrigger = !junctionAhead(s);
    Scan sc = scanNode(s);
    if (sc.fake) {
      logMsg("no junction - the car only corrected its path - carrying on");
      segDist += sc.creep;
      resumeRoad();
      return;
    }
    arriveAtNode(sc);
    return;
  }

  static byte stallPat = 0xFF;
  static unsigned long stallSince = 0;
  if (s != stallPat || followL + followR == 0) { stallPat = s; stallSince = millis(); }
  else if (millis() - stallSince > FOLLOW_STALL_MS) {
    logMsg("wheels seem stalled - kick");
    drive(KICK_PWM, KICK_PWM);
    delay(80);
    stallSince = millis();
  }
  followLine(s);
  sendProgress();
}
