/*
  AGV MECANUM 4 BANH - PS2 + DO LINE 8 MAT + NE VAT CAN + DI MAP RFID (P1-P9) + OLED 1.3"
  Board: Arduino Mega 2560 + PCA9685 (0x60) + RFID RC522 (SDA=48, RST=49) + OLED 1.3" I2C (SH1106)
  Map luoi 3x3:   P1 P2 P3
                  P4 P5 P6
                  P7 P8 P9      (P8 la diem vao map, xe vao P8 huong Bac)
  RC522 dat duoi bung xe, phia sau cam bien line.
  V5: Khong re som: mat ngoai cung sang -> khoa chay thang (khong PID, khong pivot), tien toi khi doc duoc the.
      Doc the xong -> nhich NODE_NUDGE_MS de tam xoay vao giao diem -> quay tai cho (calib) -> bam line.
      Calib: o man chon dich MAP bam SQUARE (phai 90), R2 (trai 90), L2 (180).
*/

#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <U8g2lib.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// ================= OLED =================
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ================= RFID RC522 =================
#define RFID_SDA_PIN 48
#define RFID_RST_PIN 49
MFRC522 rfid(RFID_SDA_PIN, RFID_RST_PIN);

const char* const NODE_UIDS[10] = {
  "",
  "1303DD30",         // P1
  "04C363EC360289",   // P2
  "04737A3B370289",   // P3
  "04C33046370289",   // P4
  "04A31CD3360289",   // P5
  "04E312EE360289",   // P6
  "43678931",         // P7
  "0453C8D4360289",   // P8
  "406BD20B"          // P9
};

int currentNode = 0;       // Diem RFID xe dang dung / vua doc (0: chua biet)
int targetNode  = 1;       // Dich chon tren OLED
unsigned long lastRfidPoll = 0;

// ================= CHE DO =================
#define MODE_PS2  0
#define MODE_LINE 1
#define MODE_MAP  2
int robotMode = MODE_PS2;

#define MAP_WAIT_SELECT  0
#define MAP_EXECUTING    1
int mapSubState = MAP_WAIT_SELECT;

// ================= DINH VI + HUONG + DIEU HUONG =================
// Huong: 0=Bac(len P2), 1=Dong(sang phai), 2=Nam(xuong), 3=Tay(sang trai)
const char HEAD_CH[4] = {'B', 'D', 'N', 'T'};
int curHeading = 0;

#define NAV_IDLE         0
#define NAV_PAUSE        1   // Dung tai point, tim duong ngan nhat tiep theo
#define NAV_TURN         2   // Dang xoay tai cho (theo thoi gian calib)
#define NAV_DRIVE        3   // Dang bam line toi point ke tiep
#define NAV_NUDGE        4   // Nhich them de dua tam xoay vao giao diem
#define NAV_DEST_PAUSE   5   // Dung o dich
#define NAV_BACK         6   // Lui tim the khi di qua tam ma chua doc duoc the
#define NAV_TURN_SETTLE  7   // Dung sau khi xoay cho het quan tinh
int navState = NAV_IDLE;

int navDest = 8;
bool navReturning = false;
int nextNode = 0, nextHeading = 0, fromNode = 0;
int turnDir = 1;
unsigned long turnMs = 0, navTimer = 0, navLostStart = 0;

// >>> THONG SO CALIB QUAY TAI CHO <<<
const int  TURN_PWM                  = 2550;  // PWM xoay CO DINH (khong phu thuoc speedLevel)
const unsigned long TURN_90_RIGHT_MS = 2100;   // Thoi gian xoay PHAI 90 do
const unsigned long TURN_90_LEFT_MS  = 2100;   // Thoi gian xoay TRAI 90 do
const unsigned long TURN_180_MS      = 4400;  // Thoi gian quay dau 180 do
const unsigned long TURN_SETTLE_MS   = 150;   // Dung cho het quan tinh truoc khi bam line

// >>> THONG SO TIEN TOI NODE (CHONG RE SOM) <<<
const int  APPROACH_PWM               = 1100;  // PWM tien thang khi qua vach ngang / het line cho toi khi doc the
const unsigned long JUNCTION_HOLD_MS  = 100;   // Khoa thang bao lau sau lan cuoi thay mat ngoai cung sang
const unsigned long APPROACH_MAX_MS   = 1800;  // Het line sau vach ngang: tien thang toi da bao lau de cho the
const unsigned long LOST_MAX_MS       = 300;   // Het line nhung CHUA thay vach ngang: bao loi sau bao lau
const unsigned long NODE_NUDGE_MS     = 80;    // Sau khi doc the, nhich tien de TAM XOAY vao giao diem
                                               //   Xe van xoay som -> TANG (100..200) | Xoay qua tam -> GIAM / 0
const int  NUDGE_PWM                  =1200;  // PWM luc nhich
const unsigned long BACK_MAX_MS       = 1000;  // Lui tim the toi da bao lau
const bool ACCEPT_LINE_END            = false; // false = khong doc duoc the thi BAO LOI (khong tu doan vi tri)

// >>> THONG SO DI MAP <<<
const int MAP_TURN_SPEED              = 850;   // Chi dung cho smartTurnAlign (che do LINE khi ne vat can)
const unsigned long NODE_PAUSE_MS     = 400;   // Dung tai moi point de tinh duong
const unsigned long DEST_PAUSE_MS     = 1200;  // Dung tai dich truoc khi ve P8
const unsigned long DRIVE_TIMEOUT_MS  = 15000; // Di 1 canh qua lau -> bao loi

bool crossArmed = false;                       // Da roi khoi vach ngang cua node xuat phat
bool junctionSeen = false;                     // Canh nay da thay vach ngang (sap toi node)
unsigned long junctionUntil = 0;

#define STEP_COST   10
#define TURN90_COST 4
#define TURN180_COST 7

// --- Test calib quay (o man chon dich) ---
bool calActive = false;
int calDir = 1;
unsigned long calMs = 0, calStart = 0;

// ================= CAU HINH DO LINE =================
#define NUM_SENSORS 8
const int linePins[NUM_SENSORS] = {A8, A9, A10, A11, A12, A13, A14, A15};
const int lineWeights[NUM_SENSORS] = {-4000, -3000, -2000, -1000, 1000, 2000, 3000, 4000};
#define LINE_THRESHOLD   400
#define LINE_BLACK_HIGH  0

#define LINE_BASE_VY     1875
#define LINE_MIN_VY      250
#define LINE_PIVOT_PWM   3800
#define LINE_KP          2.20
#define LINE_KD          2.50

#define CORNER_SEARCH_MS 400
#define STRAIGHT_LOSS_MS 80

float lastLineError = 0;
int lastTurnDir = 0;

// Ket qua doc cam bien (readLineSensors)
int lineAct = 0;
bool lineEdge = false;     // mat ngoai cung (0 hoac 7) dang tren vach

// ================= HC-SR04 =================
#define TRIG_FRONT 22
#define ECHO_FRONT 23
#define TRIG_RIGHT 24
#define ECHO_RIGHT 25
#define TRIG_LEFT  44
#define ECHO_LEFT  45
#define TRIG_REAR  46
#define ECHO_REAR  47

int avoidState = 0;
int avoidBypassDir = -1;
unsigned long avoidTimer = 0;
unsigned long lastPingTime = 0;

// ================= PCA9685 =================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);
#define PCA_ADDR 0x60
unsigned int i2cErrCount = 0;
uint16_t pcaOn[16], pcaOff[16];
bool pcaForce = false;
unsigned long lastPcaCheck = 0, lastPcaRefresh = 0;
#define PCA_CHECK_MS    1000
#define PCA_REFRESH_MS  300

void i2cConfig() {
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(25000, true);
#endif
}

bool i2cTimedOut() {
#ifdef WIRE_HAS_TIMEOUT
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    i2cErrCount++;
    return true;
  }
#endif
  return false;
}

void pcaCacheClear() {
  for (byte i = 0; i < 16; i++) { pcaOn[i] = 0xFFFF; pcaOff[i] = 0xFFFF; }
}

void pcaSet(uint8_t ch, uint16_t on, uint16_t off) {
  if (pcaOn[ch] == on && pcaOff[ch] == off) return;
  pcaOn[ch] = on; pcaOff[ch] = off;
  pwm.setPWM(ch, on, off);
}

void pcaRefreshAll() {
  for (byte ch = 0; ch < 16; ch++) {
    if (pcaOff[ch] != 0xFFFF) pwm.setPWM(ch, pcaOn[ch], pcaOff[ch]);
  }
  pcaForce = false;
}

bool pcaAlive() {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1) != 1) return false;
  byte m1 = Wire.read();
  return (m1 & 0x10) == 0;
}

void pcaRecover() {
  pwm.begin();
  pwm.setPWMFreq(1000);
  i2cConfig();
  Wire.setClock(400000);
  pcaForce = true;
}

void pcaService(unsigned long now) {
  if (now - lastPcaCheck >= PCA_CHECK_MS) {
    lastPcaCheck = now;
    if (!pcaAlive()) pcaRecover();
  }
  if (pcaForce || now - lastPcaRefresh >= PCA_REFRESH_MS) {
    lastPcaRefresh = now;
    pcaRefreshAll();
  }
  if (i2cTimedOut()) pcaForce = true;
}

// ================= PS2 =================
PS2X ps2x;
#define PS2_DAT 12
#define PS2_CMD 11
#define PS2_SEL 10
#define PS2_CLK 13
#define PS2_USE_READ_RESULT 1

#define M1_A 8
#define M1_B 9
#define M2_A 10
#define M2_B 11
#define M3_A 15
#define M3_B 14
#define M4_A 13
#define M4_B 12

int M1_INV = 1; int M2_INV = 1; int M3_INV = 1; int M4_INV = 1;
#define DEADZONE 50
#define STRAFE_INV 1
#define TURN_INV   1

bool running = true;
bool halted = false;
bool ps2Ready = false;
byte ps2Bad = 0;
unsigned long lastRetry = 0;
unsigned long lastLoopTick = 0;
bool lastStart = false, lastCross = false, lastTriangle = false, lastCircle = false;
bool lastL1 = false, lastR1 = false, lastSelect = false;
bool lastSquare = false, lastL2 = false, lastR2 = false;
bool lastPadUp = false, lastPadDown = false, lastPadLeft = false, lastPadRight = false;

int dz(int v) { return abs(v) < DEADZONE ? 0 : v; }
int scaleAxis(int v, int range) { return constrain((long)v * range / 127, -range, range); }
bool edge(unsigned int key, bool &old) { bool n = ps2x.Button(key); bool p = n && !old; old = n; return p; }

bool ps2Connect() {
  int err = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
  if (err == 0) Serial.println(F("PS2 Connected!"));
  return err == 0;
}

// ================= TOC DO & DONG CO =================
const int SPEED_LEVELS[4] = {1500, 2250, 3500, 3750};
const int NUM_LEVELS = 4;
int speedLevel = 2;
#define SPIN_BTN 1000
#define RAMP_ENABLE    1
#define RAMP_PER_MS    60

int curM[4] = {0, 0, 0, 0};
int tgtM[4] = {0, 0, 0, 0};
unsigned long lastRamp = 0;

void setPwm(uint8_t ch, int value) { pcaSet(ch, 0, constrain(value, 0, 4095)); }

void motorRun(uint8_t a, uint8_t b, int speedValue) {
  speedValue = constrain(speedValue, -4095, 4095);
  if (speedValue > 0) { setPwm(a, speedValue); setPwm(b, 0); }
  else if (speedValue < 0) { setPwm(a, 0); setPwm(b, -speedValue); }
  else { setPwm(a, 0); setPwm(b, 0); }
}

void writeMotors() {
  motorRun(M1_A, M1_B, curM[0] * M1_INV);
  motorRun(M2_A, M2_B, curM[1] * M2_INV);
  motorRun(M3_A, M3_B, curM[2] * M3_INV);
  motorRun(M4_A, M4_B, curM[3] * M4_INV);
}

void rampStep(unsigned long now) {
#if RAMP_ENABLE
  if (robotMode != MODE_PS2) {
    for (byte i = 0; i < 4; i++) curM[i] = tgtM[i];
  } else {
    unsigned long dt = now - lastRamp;
    lastRamp = now;
    if (dt > 20) dt = 20;
    int maxd = (int)(dt * RAMP_PER_MS);
    for (byte i = 0; i < 4; i++) {
      int d = tgtM[i] - curM[i];
      if (d > maxd) d = maxd; else if (d < -maxd) d = -maxd;
      curM[i] += d;
    }
  }
#else
  for (byte i = 0; i < 4; i++) curM[i] = tgtM[i];
#endif
  writeMotors();
}

void setMotors(int m1, int m2, int m3, int m4) {
  unsigned long now = millis();
  if (m1 == 0 && m2 == 0 && m3 == 0 && m4 == 0) {
    for (byte i = 0; i < 4; i++) { tgtM[i] = 0; curM[i] = 0; }
    lastRamp = now; writeMotors(); return;
  }
  tgtM[0] = constrain(m1, -4095, 4095); tgtM[1] = constrain(m2, -4095, 4095);
  tgtM[2] = constrain(m3, -4095, 4095); tgtM[3] = constrain(m4, -4095, 4095);
  rampStep(now);
}

void stopAll() {
  for (byte i = 0; i < 4; i++) { tgtM[i] = 0; curM[i] = 0; }
  pcaSet(M1_A, 0, 0); pcaSet(M1_B, 0, 0);
  pcaSet(M2_A, 0, 0); pcaSet(M2_B, 0, 0);
  pcaSet(M3_A, 0, 0); pcaSet(M3_B, 0, 0);
  pcaSet(M4_A, 0, 0); pcaSet(M4_B, 0, 0);
}

void driveMecanum(int vy, int vx, int wz) {
  vx *= STRAFE_INV; wz *= TURN_INV;
  int raw[4] = { vy + vx + wz, vy - vx + wz, vy + vx - wz, vy - vx - wz };
  int largest = 1000;
  for (uint8_t i = 0; i < 4; i++) largest = max(largest, abs(raw[i]));

  int maxPwm = SPEED_LEVELS[speedLevel];
  int out[4];
  for (uint8_t i = 0; i < 4; i++) out[i] = (int)((long)raw[i] * maxPwm / largest);

  setMotors(out[0], out[1], out[2], out[3]);
}

// Chay thang (p>0 tien, p<0 lui) bang PWM co dinh, khong phu thuoc speedLevel
void straightPwm(int p) {
  setMotors(p, p, p, p);
}

// Xoay tai cho voi PWM co dinh TURN_PWM (dir: +1 phai/CW, -1 trai/CCW)
void spinInPlace(int dir) {
  int p = TURN_PWM * dir * TURN_INV;
  setMotors(p, p, -p, -p);
}

void ps2Drive(int LX, int LY, int RX) {
  int vy = 0, vx = 0, wz = 0;
  int dy = 0, dx = 0;
  if (ps2x.Button(PSB_PAD_UP))    dy += 1;
  if (ps2x.Button(PSB_PAD_DOWN))  dy -= 1;
  if (ps2x.Button(PSB_PAD_RIGHT)) dx += 1;
  if (ps2x.Button(PSB_PAD_LEFT))  dx -= 1;

  if (dx != 0 || dy != 0) { vy = dy * 1000; vx = dx * 1000; }
  else { vy = scaleAxis(dz(128 - LY), 1000); vx = scaleAxis(dz(LX - 127), 1000); }

  if (ps2x.Button(PSB_SQUARE)) wz = -SPIN_BTN;
  else if (ps2x.Button(PSB_CIRCLE)) wz = SPIN_BTN;
  else wz = scaleAxis(dz(RX - 127), 1000);

  driveMecanum(vy, vx, wz);
}

// ================= OLED =================
bool oledNeedsUpdate = true;
String oledStatusMsg = "SAN SANG!";

void updateOledDisplay() {
  if (!oledNeedsUpdate) return;
  oledNeedsUpdate = false;

  oled.firstPage();
  do {
    oled.setFont(u8g2_font_6x12_tr);
    oled.drawStr(0, 10, "AGV MECANUM - K&NY");
    oled.drawHLine(0, 13, 128);

    oled.setCursor(0, 26);
    oled.print(F("MODE: "));
    if (halted) oled.print(F("E-STOP (X)"));
    else if (!running) oled.print(F("PAUSED"));
    else if (robotMode == MODE_PS2) { oled.print(F("PS2 (Lv")); oled.print(speedLevel + 1); oled.print(F(")")); }
    else if (robotMode == MODE_LINE) oled.print(F("LINE + NE CAN"));
    else if (robotMode == MODE_MAP)  oled.print(F("MAP AUTO"));

    // Vi tri + huong + dich
    oled.setCursor(0, 40);
    oled.print(F("VT:P")); oled.print(currentNode);
    oled.print(F(" H:")); oled.print(HEAD_CH[curHeading & 3]);
    oled.print(F(" DICH:P"));
    if (robotMode == MODE_MAP && mapSubState == MAP_EXECUTING) oled.print(navDest);
    else oled.print(targetNode);

    if (robotMode == MODE_MAP && mapSubState == MAP_WAIT_SELECT) {
      oled.setCursor(0, 52);
      oled.print(F("D-PAD:Chon | O:Chay"));
      oled.setCursor(0, 63);
      for (int i = 1; i <= 9; i++) {
        if (i == targetNode) { oled.print(F("[")); oled.print(i); oled.print(F("]")); }
        else { oled.print(F(" ")); oled.print(i); }
      }
    } else {
      oled.setCursor(0, 56);
      oled.print(oledStatusMsg);
    }
  } while (oled.nextPage());
}

void setOledStatus(const String &msg) {
  oledStatusMsg = msg;
  oledNeedsUpdate = true;
  updateOledDisplay();
}

// ================= DOC THE RFID (tra ve so P1..P9, 0 neu khong co) =================
int checkRfidNode() {
  unsigned long now = millis();
  if (now - lastRfidPoll < 10) return 0;
  lastRfidPoll = now;

  if (!rfid.PICC_IsNewCardPresent()) return 0;
  if (!rfid.PICC_ReadCardSerial()) return 0;

  String uidStr = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uidStr += "0";
    uidStr += String(rfid.uid.uidByte[i], HEX);
  }
  uidStr.toUpperCase();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  Serial.print(F("[RFID] UID: ")); Serial.println(uidStr);

  for (int i = 1; i <= 9; i++) {
    if (uidStr.equals(NODE_UIDS[i])) {
      Serial.print(F(">>> NHAN DIEN DIEM: P")); Serial.println(i);
      return i;
    }
  }
  return 0;
}

// ================= SIEU AM =================
long getDistance(int trig, int echo, unsigned long timeout) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, timeout);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ================= DO LINE (GIU NGUYEN - DUNG CHO CHE DO LINE) =================
void lineFollowCore() {
  unsigned long now = millis();
  long weightedSum = 0;
  int activeCount = 0;
  static unsigned long lastTimeOnLine = millis();

  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = analogRead(linePins[i]);
    bool onLine = (LINE_BLACK_HIGH) ? (val > LINE_THRESHOLD) : (val < LINE_THRESHOLD);
    if (onLine) { weightedSum += lineWeights[i]; activeCount++; }
  }

  if (activeCount == 0) {
    unsigned long lostDuration = now - lastTimeOnLine;
    if (lastTurnDir != 0 && lostDuration <= CORNER_SEARCH_MS) {
      if (lastTurnDir < 0) setMotors(-LINE_PIVOT_PWM, -LINE_PIVOT_PWM, LINE_PIVOT_PWM, LINE_PIVOT_PWM);
      else setMotors(LINE_PIVOT_PWM, LINE_PIVOT_PWM, -LINE_PIVOT_PWM, -LINE_PIVOT_PWM);
      return;
    }
    if (lostDuration > STRAIGHT_LOSS_MS) stopAll();
    return;
  }

  lastTimeOnLine = now;
  float currentError = (float)weightedSum / activeCount;

  if (currentError <= -1500)      lastTurnDir = -1;
  else if (currentError >= 1500)  lastTurnDir = 1;
  else if (abs(currentError) <= 1000) lastTurnDir = 0;

  float pTerm = currentError * LINE_KP;
  float dTerm = (currentError - lastLineError) * LINE_KD;
  lastLineError = currentError;

  int turn = (int)(pTerm + dTerm);
  float absErr = abs(currentError);
  int leftSpeed = 0, rightSpeed = 0;

  if (absErr >= 2500) {
    int spinForce = constrain(abs(turn), LINE_PIVOT_PWM, 4095);
    if (currentError < 0) { leftSpeed = -spinForce; rightSpeed = spinForce; }
    else { leftSpeed = spinForce; rightSpeed = -spinForce; }
  } else {
    int dynamicBase = map((long)absErr, 0, 2500, LINE_BASE_VY, LINE_MIN_VY);
    leftSpeed  = dynamicBase + turn; rightSpeed = dynamicBase - turn;
  }

  setMotors(constrain(leftSpeed, -4095, 4095), constrain(leftSpeed, -4095, 4095),
            constrain(rightSpeed, -4095, 4095), constrain(rightSpeed, -4095, 4095));
}

// Doc cam bien 1 lan: tong so mat den + co mat ngoai cung (0/7) sang
void readLineSensors() {
  lineAct = 0; lineEdge = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = analogRead(linePins[i]);
    bool onLine = (LINE_BLACK_HIGH) ? (val > LINE_THRESHOLD) : (val < LINE_THRESHOLD);
    if (!onLine) continue;
    lineAct++;
    if (i == 0 || i == NUM_SENSORS - 1) lineEdge = true;
  }
}

// Bam line cho che do MAP. Tra ve false neu mat line hoan toan.
// Duong thang binh thuong: mat ngoai cung (0/7) KHONG sang. Chi khi gap vach ngang cua node
// (ke ca vach chi o 1 phia nhu P1,P3,P7,P9 -> chi sang NUA mang) thi mat ngoai cung moi sang.
// => Thay mat ngoai cung sang: KHOA CHAY THANG (khong PID, khong pivot) de khong re som.
bool mapLineFollow(unsigned long now) {
  readLineSensors();
  if (lineAct == 0) return false;

  if (!lineEdge) crossArmed = true;     // da roi khoi vach ngang cua node xuat phat

  if (lineEdge && crossArmed) {
    if (now >= junctionUntil) Serial.println(F("[NAV] gap vach ngang -> khoa thang, cho the"));
    junctionUntil = now + JUNCTION_HOLD_MS;
    junctionSeen = true;
  }

  if (now < junctionUntil) {
    straightPwm(APPROACH_PWM);
    lastLineError = 0;
    return true;
  }

  lineFollowCore();
  return true;
}

// ================= XOAY THONG MINH BAT LINE (chi dung khi ne vat can o che do LINE) =================
bool smartTurnAlign(int dir, unsigned long blindMs, unsigned long elapsed) {
  if (elapsed < blindMs) {
    driveMecanum(0, 0, dir * MAP_TURN_SPEED);
    return false;
  }

  long weightedSum = 0;
  int activeCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = analogRead(linePins[i]);
    bool onLine = (LINE_BLACK_HIGH) ? (val > LINE_THRESHOLD) : (val < LINE_THRESHOLD);
    if (onLine) { weightedSum += lineWeights[i]; activeCount++; }
  }

  if (activeCount == 0) {
    driveMecanum(0, 0, (int)(dir * MAP_TURN_SPEED * 0.55));
  } else {
    float currentError = (float)weightedSum / activeCount;
    if (abs(currentError) <= 1000) {
      stopAll();
      lastLineError = 0;
      lastTurnDir = 0;
      return true;
    } else {
      int pTurn = constrain((int)(currentError * 1.8), -1800, 1800);
      setMotors(pTurn, pTurn, -pTurn, -pTurn);
    }
  }

  if (elapsed > 4500) {
    stopAll();
    return true;
  }
  return false;
}

// ================= BAN DO + TIM DUONG NGAN NHAT =================
// Node 1..9 -> hang = (n-1)/3, cot = (n-1)%3. Hang 0 o phia Bac.
int neighborOf(int n, int h) {
  int r = (n - 1) / 3, c = (n - 1) % 3;
  if (h == 0) r--;
  else if (h == 1) c++;
  else if (h == 2) r++;
  else c--;
  if (r < 0 || r > 2 || c < 0 || c > 2) return 0;
  return r * 3 + c + 1;
}

int turnCost(int from, int to) {
  int d = (to - from + 4) & 3;
  if (d == 0) return 0;
  if (d == 2) return TURN180_COST;
  return TURN90_COST;
}

// Tu (node, huong) tim buoc di ke tiep toi dest: tra ve node ke tiep + huong can quay mat
bool planNext(int node, int heading, int dest, int &outNode, int &outHeading) {
  if (node < 1 || node > 9 || dest < 1 || dest > 9 || node == dest) return false;

  const int INF = 20000;
  int cost[10][4];
  for (int n = 1; n <= 9; n++)
    for (int h = 0; h < 4; h++) cost[n][h] = (n == dest) ? 0 : INF;

  bool changed = true;
  int guard = 0;
  while (changed && guard++ < 40) {
    changed = false;
    for (int n = 1; n <= 9; n++) {
      if (n == dest) continue;
      for (int h = 0; h < 4; h++) {
        int best = cost[n][h];
        for (int h2 = 0; h2 < 4; h2++) {
          int nb = neighborOf(n, h2);
          if (nb == 0 || cost[nb][h2] >= INF) continue;
          int c = turnCost(h, h2) + STEP_COST + cost[nb][h2];
          if (c < best) best = c;
        }
        if (best < cost[n][h]) { cost[n][h] = best; changed = true; }
      }
    }
  }

  // Uu tien: di thang, re phai, re trai, quay dau
  const int order[4] = {0, 1, 3, 2};
  int bestCost = INF;
  bool found = false;
  for (int k = 0; k < 4; k++) {
    int h2 = (heading + order[k]) & 3;
    int nb = neighborOf(node, h2);
    if (nb == 0 || cost[nb][h2] >= INF) continue;
    int c = turnCost(heading, h2) + STEP_COST + cost[nb][h2];
    if (c < bestCost) { bestCost = c; outNode = nb; outHeading = h2; found = true; }
  }
  return found;
}

// Chuoi hien thi duong di, vd "8>9"
String routeText(int from, int heading, int dest) {
  String s = String(from);
  int n = from, h = heading, nn = 0, nh = 0;
  for (int i = 0; i < 9 && n != dest; i++) {
    if (!planNext(n, h, dest, nn, nh)) break;
    s += ">";
    s += nn;
    n = nn; h = nh;
  }
  return s;
}

// ================= MAY TRANG THAI DI MAP (KHONG DELAY) =================
void navFail(const char* msg) {
  stopAll();
  navState = NAV_IDLE;
  mapSubState = MAP_WAIT_SELECT;
  setOledStatus(msg);
  Serial.println(msg);
}

void navFinish() {
  stopAll();
  navState = NAV_IDLE;
  mapSubState = MAP_WAIT_SELECT;
  setOledStatus("DA VE P8! CHON TIEP");
  Serial.println(F(">>> HOAN THANH NHIEM VU MAP -> DA VE P8! <<<"));
}

void beginDrive(unsigned long now) {
  fromNode = currentNode;
  navState = NAV_DRIVE;
  navTimer = now;
  navLostStart = 0;
  crossArmed = false;
  junctionSeen = false;
  junctionUntil = 0;
  lastLineError = 0;
  lastTurnDir = 0;
  setOledStatus("DI P" + String(currentNode) + " > P" + String(nextNode));
}

// d: 1 = phai 90, 3 = trai 90, 2 = quay dau 180
void beginTurn(int d, unsigned long now) {
  if (d == 1)      { turnDir = 1;  turnMs = TURN_90_RIGHT_MS; setOledStatus("RE PHAI 90"); }
  else if (d == 3) { turnDir = -1; turnMs = TURN_90_LEFT_MS;  setOledStatus("RE TRAI 90"); }
  else             { turnDir = 1;  turnMs = TURN_180_MS;      setOledStatus("QUAY DAU 180"); }
  Serial.print(F("[NAV] Xoay tai cho ")); Serial.print(turnMs); Serial.println(F(" ms"));
  navTimer = now;
  navState = NAV_TURN;
}

void startCalTurn(int dir, unsigned long ms) {
  calDir = dir; calMs = ms; calStart = millis(); calActive = true;
  Serial.print(F("[CALIB] Xoay thu ")); Serial.print(dir > 0 ? F("PHAI ") : F("TRAI "));
  Serial.print(ms); Serial.println(F(" ms"));
}

void runMapMode() {
  unsigned long now = millis();

  if (mapSubState == MAP_WAIT_SELECT) {
    // Dang xoay thu de calib
    if (calActive) {
      if (now - calStart < calMs) {
        spinInPlace(calDir);
      } else {
        stopAll();
        calActive = false;
        Serial.println(F("[CALIB] Xong. Do goc thuc te roi chinh TURN_xx_MS"));
      }
      return;
    }
    stopAll();
    // Nhac xe dat len diem khac -> the tu cap nhat vi tri (huong chinh bang nut Tam giac)
    int det = checkRfidNode();
    if (det != 0 && det != currentNode) { currentNode = det; oledNeedsUpdate = true; }
    updateOledDisplay();
    return;
  }

  switch (navState) {
    case NAV_PAUSE: {
      stopAll();
      if (now - navTimer < NODE_PAUSE_MS) break;

      if (currentNode == navDest) {
        if (!navReturning) {
          navState = NAV_DEST_PAUSE;
          navTimer = now;
          setOledStatus("DA DEN P" + String(navDest) + "!");
          Serial.print(F(">>> DA DEN DICH P")); Serial.println(navDest);
        } else {
          navFinish();
        }
        break;
      }

      if (!planNext(currentNode, curHeading, navDest, nextNode, nextHeading)) {
        navFail("LOI LO TRINH!");
        break;
      }
      Serial.print(F("[NAV] P")); Serial.print(currentNode);
      Serial.print(F(" huong ")); Serial.print(HEAD_CH[curHeading]);
      Serial.print(F(" -> P")); Serial.print(nextNode);
      Serial.print(F(" huong ")); Serial.println(HEAD_CH[nextHeading]);

      int d = (nextHeading - curHeading + 4) & 3;
      if (d == 0) beginDrive(now);
      else beginTurn(d, now);
      break;
    }

    case NAV_TURN: {
      // Xoay tai cho dung thoi gian calib, xong thi dung
      if (now - navTimer < turnMs) {
        spinInPlace(turnDir);
      } else {
        stopAll();
        curHeading = nextHeading;          // Cap nhat huong sau khi xoay xong
        navTimer = now;
        navState = NAV_TURN_SETTLE;
      }
      break;
    }

    case NAV_TURN_SETTLE: {
      stopAll();                           // Cho het quan tinh roi moi bam line
      if (now - navTimer >= TURN_SETTLE_MS) beginDrive(now);
      break;
    }

    case NAV_DRIVE: {
      if (mapLineFollow(now)) {
        navLostStart = 0;
      } else {
        // Het line hoan toan
        if (navLostStart == 0) navLostStart = now;
        unsigned long limit = junctionSeen ? APPROACH_MAX_MS : LOST_MAX_MS;
        if (now - navLostStart > limit) {
          if (junctionSeen) {
            stopAll();
            navTimer = now;
            navState = NAV_BACK;
            setOledStatus("LUI TIM THE...");
            Serial.println(F("[NAV] Tien qua tam ma chua doc duoc the -> lui tim"));
          } else {
            navFail("MAT LINE! DAT XE LAI");
          }
          break;
        }
        // Da qua vach ngang (diem cuoi duong): tien THANG cham cho RC522 toi the
        straightPwm(APPROACH_PWM);
      }

      int det = checkRfidNode();
      if (det != 0 && det != fromNode) {
        currentNode = det;                 // Tin the: cap nhat vi tri thuc te
        if (det != nextNode) Serial.println(F("[NAV] Canh bao: the khac du kien -> tinh lai duong"));
        navTimer = now;
        if (NODE_NUDGE_MS > 0) {
          navState = NAV_NUDGE;            // Nhich tien de tam xoay vao giao diem
        } else {
          stopAll();
          navState = NAV_PAUSE;
        }
        setOledStatus("TOI P" + String(det));
        break;
      }

      if (now - navTimer > DRIVE_TIMEOUT_MS) navFail("QUA THOI GIAN!");
      break;
    }

    case NAV_BACK: {
      straightPwm(-APPROACH_PWM);
      int det = checkRfidNode();
      if (det != 0 && det != fromNode) {
        currentNode = det;
        stopAll();
        navTimer = now;
        navState = NAV_PAUSE;
        setOledStatus("TOI P" + String(det));
        break;
      }
      if (now - navTimer > BACK_MAX_MS) {
        if (ACCEPT_LINE_END && junctionSeen) {
          currentNode = nextNode;
          stopAll();
          navTimer = now;
          navState = NAV_PAUSE;
          setOledStatus("TOI P" + String(nextNode) + " (KO THE)");
          Serial.println(F("[NAV] Khong doc duoc the -> chap nhan theo cuoi line"));
        } else {
          navFail("MAT THE RFID!");
        }
      }
      break;
    }

    case NAV_NUDGE: {
      straightPwm(NUDGE_PWM);
      if (now - navTimer >= NODE_NUDGE_MS) {
        stopAll();
        navTimer = now;
        navState = NAV_PAUSE;
      }
      break;
    }

    case NAV_DEST_PAUSE: {
      stopAll();
      if (now - navTimer >= DEST_PAUSE_MS) {
        navReturning = true;
        navDest = 8;
        navTimer = now;
        navState = NAV_PAUSE;
        setOledStatus("QUAY VE P8: " + routeText(currentNode, curHeading, 8));
      }
      break;
    }

    default:
      navFail("TRANG THAI LOI");
      break;
  }
}

// Khi tam dung/E-STOP trong luc chay map thi dong ho cac buoc cung "dung" theo
void navHold(unsigned long dt) {
  calActive = false;                       // Huy test xoay neu dang E-STOP / tam dung
  if (robotMode == MODE_MAP && mapSubState == MAP_EXECUTING) navTimer += dt;
}

// ================= DO LINE + NE VAT CAN + BAT THE P8 =================
void lineFollowAndAvoid() {
  unsigned long now = millis();

  // Cham P8: dung ngay (RC522 duoi bung ~ tam xe), vao che do MAP, xe dang huong Bac
  int node = checkRfidNode();
  if (node == 8) {
    stopAll();
    avoidState = 0;
    currentNode = 8;
    curHeading = 0;
    robotMode = MODE_MAP;
    mapSubState = MAP_WAIT_SELECT;
    navState = NAV_IDLE;
    setOledStatus("DA TOI P8! CHON DICH");
    Serial.println(F("\n>>> DA CHAM P8 -> CHUYEN SANG CHE DO DI MAP! <<<"));
    return;
  }

  const int TURN_SPEED = 800;
  const int FWD_SPEED = 600;
  const unsigned long TURN_OUT_MS = 1200;
  const unsigned long FWD_OUT_MS = 1400;
  const unsigned long TURN_IN_MS = 1800;
  const unsigned long CENTER_AXLE_MS = 150;
  const unsigned long MIN_ALIGN_MS = 300;
  const unsigned long STOP_MS = 250;

  if (avoidState == 0) {
    if (now - lastPingTime > 40) {
      lastPingTime = now;
      long distF = getDistance(TRIG_FRONT, ECHO_FRONT, 5000);
      if (distF > 2 && distF <= 23) {
        stopAll(); avoidState = 1; avoidTimer = now;
        setOledStatus("PHAT HIEN VAT CAN!");
        Serial.println(F("\n>>> PHAT HIEN VAT CAN 23CM -> DUNG XE! <<<"));
        return;
      }
    }
    lineFollowCore();
  }
  else if (avoidState == 1) {
    stopAll();
    if (now - avoidTimer > 1000) {
      long distL = getDistance(TRIG_LEFT, ECHO_LEFT, 5000);
      long distR = getDistance(TRIG_RIGHT, ECHO_RIGHT, 5000);

      if (distL > 25 || distL == 999) { avoidBypassDir = -1; Serial.println(F(">>> NE TRAI")); }
      else if (distR > 25 || distR == 999) { avoidBypassDir = 1; Serial.println(F(">>> DAO SANG NE PHAI!")); }
      else { avoidBypassDir = -1; Serial.println(F(">>> KET 2 BEN -> EP NE TRAI")); }

      avoidState = 2; avoidTimer = now;
    }
  }
  else if (avoidState == 2) {
    driveMecanum(0, 0, avoidBypassDir * TURN_SPEED);
    if (now - avoidTimer > TURN_OUT_MS) { stopAll(); avoidState = 3; avoidTimer = now; }
  }
  else if (avoidState == 3) {
    if (now - avoidTimer > STOP_MS) { avoidState = 4; avoidTimer = now; }
  }
  else if (avoidState == 4) {
    driveMecanum(FWD_SPEED, 0, 0);
    if (now - avoidTimer > FWD_OUT_MS) { stopAll(); avoidState = 5; avoidTimer = now; }
  }
  else if (avoidState == 5) {
    if (now - avoidTimer > STOP_MS) { avoidState = 6; avoidTimer = now; }
  }
  else if (avoidState == 6) {
    driveMecanum(0, 0, -avoidBypassDir * TURN_SPEED);
    if (now - avoidTimer > TURN_IN_MS) { stopAll(); avoidState = 7; avoidTimer = now; }
  }
  else if (avoidState == 7) {
    if (now - avoidTimer > STOP_MS) { avoidState = 8; avoidTimer = now; }
  }
  else if (avoidState == 8) {
    driveMecanum(FWD_SPEED - 100, 0, 0);
    if (now - avoidTimer > 400) {
      int activeCount = 0;
      for (int i = 1; i < NUM_SENSORS - 1; i++) {
        int val = analogRead(linePins[i]);
        if ((LINE_BLACK_HIGH) ? (val > LINE_THRESHOLD) : (val < LINE_THRESHOLD)) activeCount++;
      }
      if (activeCount >= 2 || now - avoidTimer > 6000) {
        avoidState = 85; avoidTimer = now;
      }
    }
  }
  else if (avoidState == 85) {
    driveMecanum(FWD_SPEED - 150, 0, 0);
    if (now - avoidTimer > CENTER_AXLE_MS) { stopAll(); avoidState = 9; avoidTimer = now; }
  }
  else if (avoidState == 9) {
    if (now - avoidTimer > STOP_MS) { avoidState = 10; avoidTimer = now; }
  }
  else if (avoidState == 10) {
    if (smartTurnAlign(avoidBypassDir, MIN_ALIGN_MS, now - avoidTimer)) {
      avoidState = 11; avoidTimer = now;
    }
  }
  else if (avoidState == 11) {
    if (now - avoidTimer > STOP_MS) {
      avoidState = 0;
      lastLineError = 0;
      lastTurnDir = avoidBypassDir;
      setOledStatus("BAM LINE TOI P8...");
    }
  }
}

// ================= SETUP & LOOP =================
void setup() {
  pcaCacheClear();
  Serial.begin(9600);
  delay(500);
  Serial.println(F("AGV Mecanum: PS2 + DO LINE + NE CAN + RFID MAP (P1-P9) + OLED 1.3"));
  Serial.println(F("CALIB QUAY: che do MAP, man chon dich: SQUARE=phai90, R2=trai90, L2=180"));

  for (int i = 0; i < NUM_SENSORS; i++) pinMode(linePins[i], INPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_REAR,  OUTPUT); pinMode(ECHO_REAR,  INPUT);

  Wire.begin();
  i2cConfig();
  pwm.begin();
  pwm.setPWMFreq(1000);
  i2cConfig();
  Wire.setClock(400000);
  delay(200);
  stopAll();

  oled.begin();
  updateOledDisplay();

  SPI.begin();
  rfid.PCD_Init();
  Serial.println(F("RFID RC522 Ready (SDA=48, RST=49)!"));

  for (int i = 0; i < 15 && !ps2Ready; i++) {
    ps2Ready = ps2Connect();
    if (!ps2Ready) delay(500);
  }
  setOledStatus("SAN SANG! BAM SELECT");
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTick < 5) return;          // Nhip 5ms, khong dung delay
  unsigned long dt = now - lastLoopTick;
  lastLoopTick = now;

  pcaService(now);
  int LX = 127, LY = 128, RX = 127;

  if (!ps2Ready) {
    stopAll();
    navHold(dt);
    if (now - lastRetry > 1000) { lastRetry = now; ps2Ready = ps2Connect(); ps2Bad = 0; }
  } else {
    bool readOk;
#if PS2_USE_READ_RESULT
    readOk = ps2x.read_gamepad(false, 0);
#else
    ps2x.read_gamepad(false, 0); readOk = true;
#endif

    LX = ps2x.Analog(PSS_LX); LY = ps2x.Analog(PSS_LY); RX = ps2x.Analog(PSS_RX);
    int RY = ps2x.Analog(PSS_RY);
    bool bad = !readOk || (LX == 255 && LY == 255 && RX == 255 && RY == 255) || (LX == 0 && LY == 0 && RX == 0 && RY == 0);

    if (bad) {
      stopAll();
      navHold(dt);
      if (++ps2Bad >= 5) { ps2Ready = false; ps2Bad = 0; Serial.println(F("Mat ket noi tay cam")); }
    } else {
      ps2Bad = 0;

      bool crossBtn = edge(PSB_CROSS, lastCross);
      bool triBtn   = edge(PSB_TRIANGLE, lastTriangle);

      // X: E-STOP
      if (crossBtn) {
        halted = true; running = false; stopAll();
        oledNeedsUpdate = true; updateOledDisplay();
        Serial.println(F("EMERGENCY STOP"));
      }
      // Tam giac: mo khoa E-STOP
      if (halted && triBtn) {
        halted = false; running = true;
        oledNeedsUpdate = true; updateOledDisplay();
        Serial.println(F("E-STOP released"));
      }
      // START: tam dung / chay tiep
      if (edge(PSB_START, lastStart) && !halted) {
        running = !running;
        oledNeedsUpdate = true; updateOledDisplay();
      }

      // SELECT: PS2 -> LINE+AVOID -> MAP
      if (edge(PSB_SELECT, lastSelect) && !halted) {
        robotMode = (robotMode + 1) % 3;
        lastLineError = 0; lastTurnDir = 0; avoidState = 0;
        mapSubState = MAP_WAIT_SELECT;
        navState = NAV_IDLE;
        calActive = false;
        stopAll(); running = true;
        if (robotMode == MODE_PS2)  setOledStatus("DIEU KHIEN TAY CAM");
        if (robotMode == MODE_LINE) setOledStatus("DO LINE + NE CAN");
        if (robotMode == MODE_MAP)  {
          // Gia dinh dat xe tai P8, mat huong Bac (len P5)
          currentNode = 8; curHeading = 0;
          setOledStatus("CHON DICH P1-P9");
        }
      }

      bool r1 = edge(PSB_R1, lastR1);
      bool l1 = edge(PSB_L1, lastL1);
      bool padUp = edge(PSB_PAD_UP, lastPadUp);
      bool padDown = edge(PSB_PAD_DOWN, lastPadDown);
      bool padLeft = edge(PSB_PAD_LEFT, lastPadLeft);
      bool padRight = edge(PSB_PAD_RIGHT, lastPadRight);
      bool circleBtn = edge(PSB_CIRCLE, lastCircle);
      bool squareBtn = edge(PSB_SQUARE, lastSquare);
      bool l2Btn = edge(PSB_L2, lastL2);
      bool r2Btn = edge(PSB_R2, lastR2);

      if (robotMode == MODE_MAP && mapSubState == MAP_WAIT_SELECT && !halted) {
        if (padRight && targetNode < 9) { targetNode++; oledNeedsUpdate = true; }
        if (padLeft  && targetNode > 1) { targetNode--; oledNeedsUpdate = true; }
        if (padDown  && targetNode <= 6) { targetNode += 3; oledNeedsUpdate = true; }
        if (padUp    && targetNode >= 4) { targetNode -= 3; oledNeedsUpdate = true; }
        if (r1 && targetNode < 9) { targetNode++; oledNeedsUpdate = true; }
        if (l1 && targetNode > 1) { targetNode--; oledNeedsUpdate = true; }

        // Tam giac: chinh huong xe thu cong (B -> D -> N -> T) khi dat lai xe
        if (triBtn) { curHeading = (curHeading + 1) & 3; oledNeedsUpdate = true; }

        // CALIB: test xoay tai cho
        if (!calActive) {
          if (squareBtn)    startCalTurn(1,  TURN_90_RIGHT_MS);
          else if (r2Btn)   startCalTurn(-1, TURN_90_LEFT_MS);
          else if (l2Btn)   startCalTurn(1,  TURN_180_MS);
        }

        updateOledDisplay();

        // O: xac nhan chay toi dich
        if (circleBtn && !calActive && currentNode >= 1 && currentNode <= 9) {
          navDest = targetNode;
          navReturning = false;
          navState = NAV_PAUSE;
          navTimer = now;
          mapSubState = MAP_EXECUTING;
          running = true;
          Serial.print(F(">>> CHAY MAP TOI P")); Serial.println(targetNode);
          setOledStatus("L:" + routeText(currentNode, curHeading, navDest));
        }
      } else {
        if (r1 && speedLevel < NUM_LEVELS - 1) { speedLevel++; oledNeedsUpdate = true; updateOledDisplay(); }
        if (l1 && speedLevel > 0) { speedLevel--; oledNeedsUpdate = true; updateOledDisplay(); }
      }

      // Thuc thi
      if (halted || !running) {
        stopAll();
        navHold(dt);
      } else {
        if (robotMode == MODE_PS2)       ps2Drive(LX, LY, RX);
        else if (robotMode == MODE_LINE) lineFollowAndAvoid();
        else if (robotMode == MODE_MAP)  runMapMode();
      }
    }
  }
}
