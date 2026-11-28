/*
  zvey_code.ino - selectable LEFT / RIGHT hand rule maze robot
  Ported from the Arduino Mega "phase 0" code (rightleftphase0) to our ESP32 car:

    Mega                          ESP32 (this car)
    HM-10 on Serial2              ESP32's own Bluetooth "MazeRobot" (+ USB Serial)
    ENA 3 / IN1 26 / IN2 28       left  side: ENA 25, forward 27, back 14
    ENB 2 / IN3 22 / IN4 24       right side: ENB 26, forward 33, back 23
    IR 30,32,34,36,38             IR 32,34,35,36,39 (far left ... far right), LOW = black
    EEPROM                        Preferences (flash), same two routes
    -                             wheel encoders GPIO 16 / 17 -> live map on the laptop

  The driving logic is the phase 0 code unchanged: PD line following, junction
  scan, left / right hand rule, U-turn at dead ends, route recording and the
  shortest-path simplification, return journey on 'B'.
  Changes for the ESP32 car (marked "ESP32:" below):
    - speeds for our 4-wheel car (65 / 130 stalled it - see SPEED CONTROLS)
    - turns give up after a few seconds instead of spinning forever
    - route messages for the map viewer (view_map\run_viewer.bat)

  Commands (Bluetooth app, Serial Monitor or the map window):
    L = start (left-hand rule)    R = start (right-hand rule)
    E = end / stop                B = approve the return trip (at the goal)
    S = show the sensors
*/

#include "BluetoothSerial.h"
#include <Preferences.h>
#include "driver/pulse_cnt.h"

BluetoothSerial btSerial;
Preferences prefs;

// --- MOTORS (L298N) ---
const int ENA   = 25;   // left  side speed
const int L_FWD = 27;   // IN1
const int L_BCK = 14;   // IN2
const int ENB   = 26;   // right side speed
const int R_BCK = 23;   // IN3
const int R_FWD = 33;   // IN4

// --- IR SENSORS (OUT1 = car's far left ... OUT5 = far right) ---
const int irFarLeft   = 32;
const int irNearLeft  = 34;
const int irCenter    = 35;
const int irNearRight = 36;
const int irFarRight  = 39;

const int BLACK = 0;
const int WHITE = 1;

// --- SPEED CONTROLS ---
// ESP32: the Mega values were 65 / 130 / boost 100. On this 4-wheel car 60-65
// stalls and 130-150 is too weak to spin it, so:
const int baseSpeed = 75;    // (Mega: 65)
const int turnSpeed = 160;   // (Mega: 130)

// --- PD-CONTROLLER PARAMS ---
const float Kp = 20.0;
const float Kd = 35.0; // Dampens the swing to completely eliminate wiggling

// --- KICKSTART CONTROLS ---
const int boostSpeed = 150;  // ESP32: (Mega: 100)
const int boostTime  = 40;
bool isStopped = true;

// ESP32: a turn that never finds the line gives up after this
const unsigned long TURN_TIMEOUT_MS = 4000;

int L2, L1, C, R1, R2;
int lastError = 0;
unsigned long timeLost = 0;
unsigned long lastTurnTime = 0; // Added to prevent false intersection triggers after turns

// --- ROUTE RECORDING VARIABLES ---
const int MAX_ROUTE_LENGTH = 100;

// Array 1: Every turn the robot actually takes
char route[MAX_ROUTE_LENGTH];
int routeLength = 0;

// Array 2: The mathematically simplified shortest path
char shortRoute[MAX_ROUTE_LENGTH];
int shortRouteLength = 0;

// --- REPLAY & EXPLORATION VARIABLES ---
bool isReplaying = false;
int replayIndex = 0;
bool useLeftHandRule = true; // Determines which logic the robot uses to explore

enum RobotState {
  FOLLOWING_LINE,
  INTERSECTION_DETECTED,
  DEAD_END,
  STOPPED,
  WAITING_AT_GOAL,
  FINISHED
};

RobotState currentState = STOPPED;
RobotState lastSentState = STOPPED; // To track and display state changes via Bluetooth

// ======================================================================
// ESP32: output to USB and Bluetooth at once
void say(const String& s)  { Serial.println(s); if (btSerial.hasClient()) btSerial.println(s); }
void logMsg(const String& s) { say("LOG " + s); }

// ======================================================================
// ESP32: WHEEL ENCODERS (hardware pulse counters) -> distance for the map
const int ENC_L_PIN = 16;
const int ENC_R_PIN = 17;
const float MM_PER_TICK = 2.0f;
pcnt_unit_handle_t pcntL = NULL, pcntR = NULL;
int encBaseL = 0, encBaseR = 0;

bool pcntSetup(int pin, pcnt_unit_handle_t* unit) {
  pcnt_unit_config_t uc = {};
  uc.low_limit = -30000;
  uc.high_limit = 30000;
  uc.flags.accum_count = 1;
  if (pcnt_new_unit(&uc, unit) != ESP_OK) return false;
  pcnt_glitch_filter_config_t fc = {};
  fc.max_glitch_ns = 10000;
  pcnt_unit_set_glitch_filter(*unit, &fc);
  pcnt_chan_config_t cc = {};
  cc.edge_gpio_num = pin;
  cc.level_gpio_num = -1;
  pcnt_channel_handle_t ch = NULL;
  if (pcnt_new_channel(*unit, &cc, &ch) != ESP_OK) return false;
  pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_unit_add_watch_point(*unit, uc.high_limit);
  if (pcnt_unit_enable(*unit) != ESP_OK) return false;
  pcnt_unit_clear_count(*unit);
  return pcnt_unit_start(*unit) == ESP_OK;
}
int encCount(pcnt_unit_handle_t u) { int c = 0; if (u) pcnt_unit_get_count(u, &c); return c; }
void encReset() { encBaseL = encCount(pcntL); encBaseR = encCount(pcntR); }
void encSetMM(int mm) {            // added 28.11.: the distance since the last node is `mm` again
  int t = (int)(mm / MM_PER_TICK);
  encBaseL = encCount(pcntL) - t; encBaseR = encCount(pcntR) - t;
}
int encMM() {                     // driven since the last map event (mm)
  int dl = encCount(pcntL) - encBaseL, dr = encCount(pcntR) - encBaseR;
  int d = (pcntL && pcntR) ? (dl + dr) / 2 : max(dl, dr);
  return (int)(d * MM_PER_TICK);
}

// ESP32: MAP MESSAGES for view_map\maze_route_viewer.py
//   EVT <kind> <turn> <mm>   kind: J junction, D dead end, G goal, H home, T other
//   PROG <mm>                live distance along the current road
int pendingMM = -1;             // distance of a junction / dead end, measured when it was found
void sendEvt(char kind, char turn, int mm) {
  say(String("EVT ") + kind + " " + turn + " " + mm);
}
void sendProg() {
  static unsigned long last = 0;
  if (millis() - last < 200) return;
  last = millis();
  say("PROG " + String(encMM()));
}
void sendPath() {
  String p = "";
  for (int i = 0; i < routeLength; i++) p += route[i];
  say("PATH " + p);
}

// ======================================================================
// JUNCTION MAP (added 28.11. - only RECORDS what the car does; the driving and
// exploring are unchanged).
// Every junction / corner / dead end / the goal becomes a NODE with its position
// (mm, from the encoders and the car's heading) and its 4 roads:
//   a road it DROVE (-> which node it leads to and how long it is),
//   a road it SAW but did NOT take (?), or no road (-).
// A junction reached again (loops) is recognised by its position.
// On B the way home is the SHORTEST way through this map (Dijkstra), not the
// L/R letters reversed.
const int MAXN = 80;
const int8_t NO_EXIT = -2, UNEXPLORED = -1;
const float MATCH_MM = 80;           // a junction within 8 cm of a known one IS that one (roads here can be ~10 cm)
struct MNode { int16_t x, y; int8_t nb[4]; uint16_t len[4]; uint8_t kind; };   // kind 0 junction, 1 dead end, 2 goal, 3 start
MNode mn[MAXN];
int mnCount = 0, curN = 0, hdg = 0, goalN = -1;   // hdg: 0 up (the way the car left the start), 1 right, 2 down, 3 left
const int MDX[4] = {0, 1, 0, -1}, MDY[4] = {1, 0, -1, 0};
const char* HDG_NAME = "NESW";

void mapReset() {
  mnCount = 1; curN = 0; hdg = 0; goalN = -1;
  mn[0].x = 0; mn[0].y = 0; mn[0].kind = 3;
  for (int d = 0; d < 4; d++) { mn[0].nb[d] = (d == 0) ? UNEXPLORED : NO_EXIT; mn[0].len[d] = 0; }
}

void mapTurn(char c) {               // the car turned: update the heading
  if (c == 'R') hdg = (hdg + 1) & 3;
  else if (c == 'L') hdg = (hdg + 3) & 3;
  else if (c == 'B' || c == 'U') hdg = (hdg + 2) & 3;
}

String nodeText(int i) {
  String s = "node " + String(i) + " (" + mn[i].x + "," + mn[i].y + ")";
  s += mn[i].kind == 2 ? " GOAL" : mn[i].kind == 1 ? " dead-end" : mn[i].kind == 3 ? " START" : "";
  for (int d = 0; d < 4; d++) {
    s += String("  ") + HDG_NAME[d] + ":";
    if (mn[i].nb[d] == NO_EXIT) s += "-";
    else if (mn[i].nb[d] == UNEXPLORED) s += "? (not taken)";
    else s += String(mn[i].nb[d]) + " (" + mn[i].len[d] + "mm)";
  }
  return s;
}

// the car is at a node now: `mm` driven from the last node, roads seen L / S / R
int mapArrive(int mm, bool L, bool S, bool R, uint8_t kind, char took) {
  if (mm < 0) mm = 0;
  float x = mn[curN].x + MDX[hdg] * mm, y = mn[curN].y + MDY[hdg] * mm;
  int back = (hdg + 2) & 3;
  bool ex[4] = {false, false, false, false};
  ex[back] = true;
  if (S) ex[hdg] = true;
  if (R) ex[(hdg + 1) & 3] = true;
  if (L) ex[(hdg + 3) & 3] = true;

  // a known node here? (dead ends are always new; the goal only matches the goal)
  int id = -1; float best = MATCH_MM;
  if (kind != 1) {
    for (int i = 0; i < mnCount; i++) {
      if (i == curN || mn[i].kind == 1) continue;
      if ((kind == 2) != (mn[i].kind == 2)) continue;
      float d = max(fabsf(mn[i].x - x), fabsf(mn[i].y - y));
      // that node must have a road towards us that is not driven yet (or is
      // the road we just came along) - otherwise it is a different junction
      int e = mn[i].nb[back];
      if ((e == UNEXPLORED || e == curN) && d < best) { best = d; id = i; }
    }
  }
  if (id >= 0) {
    for (int d = 0; d < 4; d++) if (ex[d] && mn[id].nb[d] == NO_EXIT) mn[id].nb[d] = UNEXPLORED;   // a road seen now
    logMsg("known junction again (loop) - " + String(id));
  } else {
    if (mnCount >= MAXN) { logMsg("!!! map full"); return curN; }
    id = mnCount++;
    mn[id].x = (int16_t)x; mn[id].y = (int16_t)y; mn[id].kind = kind;
    for (int d = 0; d < 4; d++) { mn[id].nb[d] = ex[d] ? UNEXPLORED : NO_EXIT; mn[id].len[d] = 0; }
  }
  // the road just driven, both ways
  uint16_t L16 = (uint16_t)max(1, mm);
  if (mn[curN].nb[hdg] >= 0 && mn[curN].nb[hdg] != id) logMsg("!!! map: that road led elsewhere before");
  if (mn[curN].len[hdg] == 0 || L16 < mn[curN].len[hdg]) mn[curN].len[hdg] = L16;
  mn[curN].nb[hdg] = id;
  if (mn[id].len[back] == 0 || L16 < mn[id].len[back]) mn[id].len[back] = L16;
  mn[id].nb[back] = curN;
  curN = id;
  if (kind == 2) goalN = id;
  logMsg(nodeText(id) + "  -> took " + String(took));
  return id;
}

// shortest way (by road length) from node a to node b. Fills path[], returns its length (0 = none)
int planPath(int a, int b, int* path) {
  static uint32_t dist[MAXN]; static int8_t prv[MAXN]; static bool done[MAXN];
  for (int i = 0; i < mnCount; i++) { dist[i] = 0xFFFFFFFF; prv[i] = -1; done[i] = false; }
  dist[a] = 0;
  for (int it = 0; it < mnCount; it++) {
    int u = -1;
    for (int i = 0; i < mnCount; i++) if (!done[i] && dist[i] != 0xFFFFFFFF && (u < 0 || dist[i] < dist[u])) u = i;
    if (u < 0) break;
    done[u] = true;
    for (int d = 0; d < 4; d++) {
      int v = mn[u].nb[d];
      if (v < 0) continue;
      uint32_t nd = dist[u] + max((int)mn[u].len[d], 1);
      if (nd < dist[v]) { dist[v] = nd; prv[v] = u; }
    }
  }
  if (dist[b] == 0xFFFFFFFF) return 0;
  int n = 0, tmp[MAXN];
  for (int v = b; v >= 0 && n < MAXN; v = prv[v]) { tmp[n++] = v; if (v == a) break; }
  for (int i = 0; i < n; i++) path[i] = tmp[n - 1 - i];
  return n;
}

int dirTo(int a, int b) {             // which road of node a leads to node b (shortest one)
  int best = -1;
  for (int d = 0; d < 4; d++)
    if (mn[a].nb[d] == b && (best < 0 || mn[a].len[d] < mn[a].len[best])) best = d;
  return best;
}
char relTurn(int fromHdg, int toHdg) { return "SRBL"[(toHdg - fromHdg + 4) & 3]; }

// the turns (S / R / L) at every node of a path, arriving at path[0] with heading h0
String pathTurns(const int* p, int n, int h0) {
  String s = ""; int h = h0;
  for (int k = 1; k < n; k++) {
    int d = dirTo(p[k - 1], p[k]);
    if (k > 1) s += relTurn(h, d);
    h = d;
  }
  return s;
}

void mapPrint() {
  logMsg("===== MAP: " + String(mnCount) + " nodes (? = road seen but not taken) =====");
  for (int i = 0; i < mnCount; i++) logMsg(nodeText(i));
  prefs.begin("zvey", false);
  prefs.putBytes("map", mn, sizeof(MNode) * mnCount);
  prefs.putInt("mapN", mnCount);
  prefs.putInt("goalN", goalN);
  prefs.end();
}

// the planned way home (filled on B)
int planN = 0, planIdx = 0;
int plan[MAXN];
char planMove[MAXN];                  // the turn to make AT plan[k]
uint16_t planLen[MAXN];               // road length from plan[k-1] to plan[k]

void sendNodeEvt(char kind, char turn, int mm, int node) {   // EVT with the map position (drawing does not drift)
  say(String("EVT ") + kind + " " + turn + " " + mm + " " + mn[node].x + " " + mn[node].y);
}

// ======================================================================
void setup() {
  pinMode(ENA, OUTPUT); pinMode(L_FWD, OUTPUT); pinMode(L_BCK, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(R_FWD, OUTPUT); pinMode(R_BCK, OUTPUT);
  analogWriteFrequency(ENA, 1000);
  analogWriteFrequency(ENB, 1000);

  pinMode(irFarLeft, INPUT); pinMode(irNearLeft, INPUT); pinMode(irCenter, INPUT);
  pinMode(irNearRight, INPUT); pinMode(irFarRight, INPUT);

  pinMode(ENC_L_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN, INPUT_PULLUP);
  if (!pcntSetup(ENC_L_PIN, &pcntL)) pcntL = NULL;
  if (!pcntSetup(ENC_R_PIN, &pcntR)) pcntR = NULL;

  Serial.begin(115200);
  btSerial.begin("MazeRobot");   // ESP32: replaces the HM-10
  stopMotors();

  say("Phase 5: Selectable Left/Right Hand Rule Exploration");
  say("--- Robot Ready ---");
  say("Commands:");
  say("L = Start (Left-Hand Rule)");
  say("R = Start (Right-Hand Rule)");
  say("E = End / Stop");
  say("B = Approve Return Trip");

  // --- READ PREVIOUS ROUTES ON STARTUP ---
  prefs.begin("zvey", true);
  String raw = prefs.getString("raw", "");
  String shrt = prefs.getString("short", "");
  prefs.end();
  if (raw.length()) say("Mem Raw Route: " + raw);   else say("No valid Raw Route found in memory.");
  if (shrt.length()) say("Mem Short Route: " + shrt); else say("No valid Short Route found in memory.");

  delay(2000);
  readSensors();

  // Start stopped until Bluetooth Start command is sent.
  currentState = STOPPED;
  say("STATE stopped - send L or R to start");
}

void loop() {
  checkBluetooth();     // Constantly listen for App commands
  sendStateIfChanged(); // Update App if State changes

  // Halt motor execution if in any of these passive states
  if (currentState == STOPPED || currentState == WAITING_AT_GOAL || currentState == FINISHED) {
    stopMotors();
    heartbeat();
    delay(5);
    return;
  }

  readSensors();
  determineState();
  executeState();
}

// ESP32: the map viewer checks that the car is alive
void heartbeat() {
  static unsigned long last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  say("HB");
}

// --- BLUETOOTH FUNCTIONS ---

void checkBluetooth() {
  int cmd = -1;
  if (btSerial.available()) cmd = btSerial.read();       // ESP32: Bluetooth ...
  else if (Serial.available()) cmd = Serial.read();      // ... or the USB Serial Monitor
  if (cmd < 0) return;

  // Ignore invisible newline and carriage return characters sent by apps!
  if (cmd == '\n' || cmd == '\r') {
    return;
  }

  Serial.print("Bluetooth Received: ");
  Serial.println((char)cmd);

  // Command 'L' = Start using Left-Hand Rule
  if (cmd == 'L' || cmd == 'l') {
    say(">> App: START (LEFT-HAND)");
    isReplaying = false;
    useLeftHandRule = true;
    routeLength = 0;
    lastTurnTime = millis(); // Give it a buffer against falsely scanning the start box!
    say("PASS 1"); encReset(); sendPath(); mapReset(); pendingMM = -1;
    currentState = FOLLOWING_LINE;
  }
  // Command 'R' = Start using Right-Hand Rule
  else if (cmd == 'R' || cmd == 'r') {
    say(">> App: START (RIGHT-HAND)");
    isReplaying = false;
    useLeftHandRule = false;
    routeLength = 0;
    lastTurnTime = millis(); // Give it a buffer against falsely scanning the start box!
    say("PASS 1"); encReset(); sendPath(); mapReset(); pendingMM = -1;
    currentState = FOLLOWING_LINE;
  }
  // Command 'E' = End / Stop
  else if (cmd == 'E' || cmd == 'e') {
    say(">> App: END / STOPPING");
    isReplaying = false;
    currentState = STOPPED;
    stopMotors();
  }
  // Command 'B' = Approve Return Journey
  else if (cmd == 'B' || cmd == 'b') {
    if (currentState == WAITING_AT_GOAL) {
      say(">> App: Goal Confirmed! Returning...");
      startReturnJourney();
    } else {
      say(">> App: Ignored (Not at Goal)");
    }
  }
  // ESP32: S = show the sensors, M = the map viewer asking "who is there"
  else if (cmd == 'S' || cmd == 's') {
    readSensors();
    say(String("LOG sensors (L..R, 0 = black): ") + L2 + " " + L1 + " " + C + " " + R1 + " " + R2);
  }
  else if (cmd == 'M' || cmd == 'm') {
    lastSentState = (RobotState)-1;     // resend the state
  }
}

void sendStateIfChanged() {
  if (currentState != lastSentState) {
    String s;
    switch(currentState) {
      case FOLLOWING_LINE: s = isReplaying ? "RETURNING HOME" : "FOLLOWING LINE"; break;
      case INTERSECTION_DETECTED: s = "INTERSECTION DETECTED"; break;
      case DEAD_END: s = "DEAD END"; break;
      case STOPPED: s = "STOPPED"; break;
      case WAITING_AT_GOAL: s = "WAITING FOR APPROVAL"; break;
      case FINISHED: s = "FINISHED MAZE"; break;
    }
    say("STATE " + s);
    lastSentState = currentState;
  }
}

// ---------------------------

void readSensors() {
  L2 = digitalRead(irFarLeft);
  L1 = digitalRead(irNearLeft);
  C  = digitalRead(irCenter);
  R1 = digitalRead(irNearRight);
  R2 = digitalRead(irFarRight);
}

void determineState() {
  int blackCount = 0;
  if (L2 == BLACK) blackCount++;
  if (L1 == BLACK) blackCount++;
  if (C  == BLACK) blackCount++;
  if (R1 == BLACK) blackCount++;
  if (R2 == BLACK) blackCount++;

  if (blackCount > 0) {
    // 28.11.: the line came back during the short dead-end sweep - the sweep was
    // turning on the spot, not driving: take its encoder counts off the road length
    if (timeLost != 0 && pendingMM >= 0) encSetMM(pendingMM);
    timeLost = 0;
  }

  if (blackCount >= 3) {
    // Check if 300ms has elapsed since the last turn finish, prevents picking up overshoots or double thick lines as "extra lefts".
    if (millis() - lastTurnTime > 300) {
      currentState = INTERSECTION_DETECTED;
    } else {
      currentState = FOLLOWING_LINE;
    }
  }
  else if (blackCount == 0) {
    currentState = DEAD_END;
  }
  else {
    currentState = FOLLOWING_LINE;
  }
}

void executeState() {
  switch (currentState) {
    case FOLLOWING_LINE: {
      int error = lastError;

      // Perfectly linear scale prevents sudden jerky jumps and tracks angles seamlessly
      if      (L2 == BLACK && L1 == WHITE) error = -4;
      else if (L2 == BLACK && L1 == BLACK) error = -3;
      else if (L1 == BLACK && C == WHITE)  error = -2;
      else if (L1 == BLACK && C == BLACK)  error = -1;
      else if (R2 == BLACK && R1 == WHITE) error = 4;
      else if (R2 == BLACK && R1 == BLACK) error = 3;
      else if (R1 == BLACK && C == WHITE)  error = 2;
      else if (R1 == BLACK && C == BLACK)  error = 1;
      else if (C  == BLACK)                error = 0;

      int errorDiff = error - lastError;
      lastError = error;

      // Apply PD control: 'P' tracks the line, 'D' dampens it actively to kill wiggles
      int correction = (Kp * error) + (Kd * errorDiff);

      int leftPWM  = baseSpeed + correction;
      int rightPWM = baseSpeed - correction;

      bool leftDir = HIGH;
      bool rightDir = HIGH;

      // REVERSE MINIMUM REMOVED: Motors now dynamically brake perfectly
      if (leftPWM < 0) {
        leftDir = LOW;
        leftPWM = -leftPWM;
      }
      if (rightPWM < 0) {
        rightDir = LOW;
        rightPWM = -rightPWM;
      }

      leftPWM  = constrain(leftPWM, 0, 255);
      rightPWM = constrain(rightPWM, 0, 255);

      setMotors(leftPWM, rightPWM, leftDir, rightDir);
      sendProg();                                   // ESP32: live position for the map
      break;
    }

    case INTERSECTION_DETECTED: {
      int mmAtJunction = encMM();                   // ESP32: road length for the map
      setMotors(baseSpeed, baseSpeed, HIGH, HIGH);

      unsigned long alignStart = millis();
      int leftHits = 0, rightHits = 0, loopCount = 0;
      bool endBubble = false;

      while (true) {
        unsigned long elapsed = millis() - alignStart;
        readSensors();

        if (elapsed < 80) {
          if (L2 == BLACK) leftHits++;
          if (R2 == BLACK) rightHits++;
          loopCount++;
        }

        if (elapsed >= 80) {
          int bCount = 0;
          if (L2 == BLACK) bCount++;
          if (L1 == BLACK) bCount++;
          if (C  == BLACK) bCount++;
          if (R1 == BLACK) bCount++;
          if (R2 == BLACK) bCount++;

          if (bCount <= 2) {
            break;
          }

          if (elapsed >= 300) {
            if (bCount >= 4) {
              endBubble = true;
            }
            break;
          }
        }
        delayMicroseconds(200);                     // ESP32: the loop is much faster than the Mega's
      }

      stopMotors();

      // --- END BUBBLE LOGIC ---
      if (endBubble) {
        say("END BUBBLE DETECTED! Destination Reached.");
        say(">> END BUBBLE DETECTED!");
        if (isReplaying) {                          // ESP32: arrived at the start box on the way back
          sendNodeEvt('H', 'S', encMM(), 0);
          say(">> Back at Start!");
          currentState = FINISHED;
          isReplaying = false;
          break;
        }
        pendingMM = encMM();                        // ESP32: drawn when the return trip starts
        {
          int node = mapArrive(pendingMM, false, false, false, 2, 'G');  // 28.11.: the goal into the map
          sendNodeEvt('G', 'S', pendingMM, node);
          mapPrint();
        }

        saveRouteToEEPROM(); // Calculates shortest path and prints it out.
        {
          // 28.11.: the shortest path start -> goal worked out from the MAP
          int tmp[MAXN];
          int n2 = planPath(0, goalN, tmp);
          if (n2 >= 2) {
            String t = pathTurns(tmp, n2, 0);
            say("SOLVED " + (t.length() ? t : String("(straight)")));
            logMsg("shortest path start -> goal through the map: " + (t.length() ? t : String("straight")) + "  (" + String(n2) + " nodes)");
          }
        }

        say("Waiting for 'B' command to start return journey...");
        say(">> Send 'B' to Return!");

        currentState = WAITING_AT_GOAL; // Puts the car into pause mode
        break;
      }

      delay(100);

      bool leftDetected  = (leftHits * 5 >= loopCount);
      bool rightDetected = (rightHits * 5 >= loopCount);

      readSensors();
      bool straightDetected = (C == BLACK || L1 == BLACK || R1 == BLACK);
      int mm = encMM();                             // ESP32: up to the turning point
      (void)mmAtJunction;

      // --- REPLAY MODE LOGIC (Navigating Back) ---
      // (28.11.: follows the planned way home through the MAP. A junction seen
      //  much too early is a false one and is driven straight over; a straight-on
      //  junction that was passed without being seen is skipped by its distance.)
      if (isReplaying) {
        // skip planned straight-on nodes that were driven past unseen
        while (planIdx < planN - 1 && planMove[planIdx] == 'S' && mm > planLen[planIdx] + 80) {
          logMsg("passed node " + String(plan[planIdx]) + " (straight on) without seeing it");
          mm -= planLen[planIdx];
          planIdx++;
        }
        if (planIdx < planN && mm < (int)planLen[planIdx] / 2 && mm < (int)planLen[planIdx] - 60) {
          logMsg("junction seen after " + String(mm) + " mm, expected " + String(planLen[planIdx]) + " mm - ignored, straight on");
          setMotors(baseSpeed, baseSpeed, HIGH, HIGH);
          delay(150);
          lastTurnTime = millis();
        }
        else if (planIdx < planN - 1) {
          char nextMove = planMove[planIdx];
          int node = plan[planIdx];
          planIdx++;

          say(String(">> Return Turn: ") + nextMove + " at node " + node);
          sendNodeEvt('J', nextMove, mm, node);

          if (nextMove == 'L') {
            tankTurnLeft();
          } else if (nextMove == 'R') {
            tankTurnRight();
          } else if (nextMove == 'S') {
            encReset();
            setMotors(baseSpeed, baseSpeed, HIGH, HIGH);
            delay(150);
            lastTurnTime = millis();
          }
        } else {
          // the last node of the plan = the start: we have safely navigated back to Start!
          sendNodeEvt('H', 'S', mm, 0);
          say("RETURN JOURNEY COMPLETE! Back at Start.");
          say(">> Back at Start!");
          currentState = FINISHED;
          isReplaying = false;
        }
      }
      // --- EXPLORATION LOGIC (Solving the maze) ---
      else {
        // which way the hand rule goes (exactly as before)
        char c = 0;
        if (useLeftHandRule) {
          // --- LEFT HAND RULE --- (Priority: Left -> Straight -> Right)
          if (leftDetected) c = 'L';
          else if (straightDetected) c = 'S';
          else if (rightDetected) c = 'R';
        }
        else {
          // --- RIGHT HAND RULE --- (Priority: Right -> Straight -> Left)
          if (rightDetected) c = 'R';
          else if (straightDetected) c = 'S';
          else if (leftDetected) c = 'L';
        }
        if (c) {
          // 28.11.: write the junction into the map: the road taken AND the ones not taken
          int node = mapArrive(mm, leftDetected, straightDetected, rightDetected, 0, c);
          sendNodeEvt('J', c, mm, node);
          recordTurn(c);
          if (c == 'L') {
            tankTurnLeft();
          } else if (c == 'S') {
            encReset();
            setMotors(baseSpeed, baseSpeed, HIGH, HIGH);
            delay(150);
            lastTurnTime = millis();
          } else {
            tankTurnRight();
          }
          mapTurn(c);
        }
      }
      break;
    }

    case DEAD_END:
      if (timeLost == 0) {
        timeLost = millis();
        pendingMM = encMM();                        // ESP32: where the line ended (before the sweep)
      }

      // REDUCED FROM 800ms to 150ms!
      // This minimizes false twists and prevents it from falsely registering missed left turns.
      if (millis() - timeLost > 450) {
        if (isReplaying) {
          // If we hit a dead end while replaying, we have arrived backwards at the Start Box
          sendNodeEvt('H', 'S', pendingMM, 0);
          say("RETURN JOURNEY COMPLETE! (Reached Start Box)");
          say(">> Arrived at Start!");
          currentState = FINISHED;
          isReplaying = false;
          stopMotors();
        } else {
          int node = mapArrive(pendingMM, false, false, false, 1, 'B');   // 28.11.: dead end into the map
          sendNodeEvt('D', 'B', pendingMM, node);
          recordTurn('U');
          uTurn();
          mapTurn('B');
        }
        timeLost = 0;
      }
      else {
        // Keeps the brief 150ms sweep to recover from tiny glitches (bumpy tape)
        if (lastError < 0) {
          setMotors(turnSpeed, turnSpeed, LOW, HIGH);
        } else {
          setMotors(turnSpeed, turnSpeed, HIGH, LOW);
        }
      }
      break;

    case STOPPED:
    case WAITING_AT_GOAL:
    case FINISHED:
      stopMotors();
      break;
  }
}

// --- RECORDING & OPTIMIZATION FUNCTIONS ---

void recordTurn(char move) {
  if (routeLength < MAX_ROUTE_LENGTH) {
    route[routeLength] = move;
    routeLength++;

    // Print locally and transmit via Bluetooth
    say(String(">> Executed Turn: ") + move);
    sendPath();
  }
}

void simplifyPath() {
  say("Calculating Shortest Path...");

  // 1. Copy the raw route to the short route array
  shortRouteLength = routeLength;
  for(int i = 0; i < routeLength; i++) {
    shortRoute[i] = route[i];
  }

  // 2. Continually look for 'U' turns and simplify the steps around them
  bool simplified = true;
  while(simplified) {
    simplified = false;

    for(int i = 1; i < shortRouteLength - 1; i++) {
      if(shortRoute[i] == 'U') {
        char newTurn = '?';
        char x = shortRoute[i-1];
        char y = shortRoute[i+1];

        // Maze substitution mathematical rules based on heading vectors
        // (These are geometry-based, so they work the same for Left & Right hand rules!)
        if(x == 'L' && y == 'L') newTurn = 'S';
        else if(x == 'L' && y == 'S') newTurn = 'R';
        else if(x == 'L' && y == 'R') newTurn = 'U';
        else if(x == 'S' && y == 'L') newTurn = 'R';
        else if(x == 'S' && y == 'S') newTurn = 'U';
        else if(x == 'S' && y == 'R') newTurn = 'L';
        else if(x == 'R' && y == 'L') newTurn = 'U';
        else if(x == 'R' && y == 'S') newTurn = 'L';
        else if(x == 'R' && y == 'R') newTurn = 'S';

        // If a substitution was found, update the array
        if(newTurn != '?') {
          shortRoute[i-1] = newTurn;

          // Shift remaining array left by 2 spaces to delete the 'U' and 'y'
          for(int j = i; j < shortRouteLength - 2; j++) {
            shortRoute[j] = shortRoute[j+2];
          }

          shortRouteLength -= 2;
          simplified = true;
          break;
        }
      }
    }
  }
}

void saveRouteToEEPROM() {
  // First calculate the short route
  simplifyPath();

  say("Saving to Memory...");

  // ESP32: Preferences (flash) instead of the Mega's EEPROM
  String raw = "", shrt = "";
  for (int i = 0; i < routeLength; i++) raw += route[i];
  for (int i = 0; i < shortRouteLength; i++) shrt += shortRoute[i];
  prefs.begin("zvey", false);
  prefs.putString("raw", raw);
  prefs.putString("short", shrt);
  prefs.end();

  // --- PRINT RESULTS TO BLUETOOTH & SERIAL ---
  String rawS = "", shortS = "";
  for (int i = 0; i < routeLength; i++) { rawS += route[i]; rawS += " "; }
  for (int i = 0; i < shortRouteLength; i++) { shortS += shortRoute[i]; shortS += " "; }
  say("Raw Route Saved: " + rawS);
  say(">> Final Short Path: " + shortS);
  say("SOLVED " + (shrt.length() ? shrt : String("(straight)")));
}

void reverseShortPath() {
  // 1. Reverse the array sequence completely
  for(int i = 0; i < shortRouteLength / 2; i++) {
    char temp = shortRoute[i];
    shortRoute[i] = shortRoute[shortRouteLength - 1 - i];
    shortRoute[shortRouteLength - 1 - i] = temp;
  }

  // 2. Swap L and R (approaching an intersection backward flips directions)
  for(int i = 0; i < shortRouteLength; i++) {
    if(shortRoute[i] == 'L') {
      shortRoute[i] = 'R';
    } else if(shortRoute[i] == 'R') {
      shortRoute[i] = 'L';
    }
    // 'S' (straight) correctly remains straight
  }

  String s = "";
  for(int i = 0; i < shortRouteLength; i++) { s += shortRoute[i]; s += " "; }
  say(">> Configured Return Path: " + s);
}

void startReturnJourney() {
  // (28.11.: the way home is the SHORTEST way through the junction map, worked
  //  out from the coordinates - not the L/R letters reversed.)
  int n = (goalN >= 0) ? planPath(goalN, 0, plan) : 0;
  if (n < 2) {
    say("!!! no way home in the map - stopped");
    currentState = FINISHED;
    return;
  }
  planN = n;
  for (int k = 1; k < n; k++) {
    int arrive = dirTo(plan[k - 1], plan[k]);
    planLen[k] = mn[plan[k - 1]].len[arrive];
    planMove[k] = (k < n - 1) ? relTurn(arrive, dirTo(plan[k], plan[k + 1])) : 'H';
  }
  String nodes = "", turns = "";
  for (int k = 0; k < n; k++) nodes += String(plan[k]) + (k < n - 1 ? "-" : "");
  for (int k = 1; k < n - 1; k++) turns += planMove[k];
  say(">> Configured Return Path: " + turns + "   (nodes " + nodes + ")");

  // 2. Set the variables indicating we are replaying a path backwards
  isReplaying = true;
  replayIndex = 0;

  int leaveDir = dirTo(goalN, plan[1]);
  int Lg = encMM();                              // how far the car is past the last junction
  say("PASS 2");
  say("SNAP " + String(mn[goalN].x) + " " + String(mn[goalN].y) + " " + String(leaveDir));

  if (n >= 3 && Lg <= 220) {
    // The last junction is close behind the goal: after a U-turn on the goal the
    // sensors would already be past it (it was missed - the old return went wrong
    // there). So BACK UP exactly to that junction (the same spot the car turned
    // at on the way in) and turn from there straight into the road home.
    say(">> Reversing " + String(Lg) + " mm to the last junction...");
    encReset();
    setMotors(baseSpeed, baseSpeed, LOW, LOW); // LOW, LOW reverses the motors
    unsigned long t0 = millis();
    while (encMM() < Lg && millis() - t0 < 3000) delay(2);
    stopMotors();
    delay(200);
    int node = plan[1];
    int out = dirTo(node, plan[2]);
    char t = relTurn(hdg, out);                  // the car still faces the goal
    say(String(">> Return Turn at node ") + node + ": " + (t == 'R' ? "right" : t == 'L' ? "left" : "round"));
    sendNodeEvt('J', 'S', 0, node);
    if (t == 'R') tankTurnRight();
    else if (t == 'L') tankTurnLeft();
    else {
      uTurn();                                   // turns clockwise: stops at a road on the right first
      if (mn[node].nb[(hdg + 1) & 3] != NO_EXIT) tankTurnRight();   // ...so turn on to the next one
    }
    say("SNAP " + String(mn[node].x) + " " + String(mn[node].y) + " " + String(out));
    planIdx = 2;
    encReset();
  } else {
    // goal far from the last junction: the original way - back off the bubble, U-turn
    say(">> Reversing...");
    encReset();
    setMotors(baseSpeed, baseSpeed, LOW, LOW); // LOW, LOW reverses the motors
    delay(450); // Wait long enough to travel off the bubble so sensors can detect empty space
    stopMotors();
    delay(200);
    int rev = encMM();
    say(">> Turning around...");
    uTurn();
    encSetMM(rev);                               // distance already covered towards the next junction
    planIdx = 1;
  }
  hdg = leaveDir;

  // 4. Send the car on its way
  currentState = FOLLOWING_LINE;
}

// --- MOTOR CONTROL FUNCTIONS ---

// ESP32: one side of the L298N. fwd HIGH = forward.
void setSide(int en, int fwdPin, int bckPin, int pwm, bool fwd) {
  digitalWrite(fwdPin, fwd ? HIGH : LOW);
  digitalWrite(bckPin, fwd ? LOW : HIGH);
  analogWrite(en, pwm);
}

void setMotors(int leftPWM, int rightPWM, bool leftFwd, bool rightFwd) {
  if (isStopped && (leftPWM > 0 || rightPWM > 0)) {
    setSide(ENA, L_FWD, L_BCK, boostSpeed, leftFwd);
    setSide(ENB, R_FWD, R_BCK, boostSpeed, rightFwd);
    delay(boostTime);

    isStopped = false;
  }

  setSide(ENA, L_FWD, L_BCK, leftPWM, leftFwd);
  setSide(ENB, R_FWD, R_BCK, rightPWM, rightFwd);

  if (leftPWM == 0 && rightPWM == 0) {
    isStopped = true;
  }
}

void stopMotors() {
  setMotors(0, 0, HIGH, HIGH);
}

// ESP32: spin until the centre sensor finds a line, but never forever
void spinUntilLine() {
  unsigned long t0 = millis();
  do {
    readSensors();
    if (millis() - t0 > TURN_TIMEOUT_MS) { logMsg("!!! turn: no line found"); break; }
    checkBluetooth();
    if (currentState == STOPPED) break;            // E pressed during the turn
  } while (C == WHITE);
}

void tankTurnLeft() {
  say("TANK TURN LEFT");
  setMotors(turnSpeed, turnSpeed, LOW, HIGH);
  delay(250);
  spinUntilLine();
  stopMotors();

  delay(100);
  encReset();                     // ESP32 map: the spin is not road distance
  lastTurnTime = millis(); // Refresh timestamp to avoid "extra lefts" false-reads
}

void tankTurnRight() {
  say("TANK TURN RIGHT");
  setMotors(turnSpeed, turnSpeed, HIGH, LOW);
  delay(250);
  spinUntilLine();
  stopMotors();
  delay(100);
  encReset();
  lastTurnTime = millis(); // Refresh timestamp to avoid "extra lefts" false-reads
}

void uTurn() {
  say("CLOCKWISE TANK U-TURN");
  setMotors(turnSpeed, turnSpeed, HIGH, LOW);
  delay(300);
  spinUntilLine();
  stopMotors();
  delay(100);
  encReset();
  lastTurnTime = millis(); // Refresh timestamp to avoid "extra lefts" false-reads
}
