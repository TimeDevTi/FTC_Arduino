#include <Arduino.h>
/*
  Arduino_Mecanum_USB  —  หุ่นล้อ mecanum คุมด้วยรีโมท FlySky (iBUS) และ/หรือหน้าเว็บผ่านสาย USB
  ------------------------------------------------------------------
  รีโมท FlySky: รีซีฟส่งค่าแบบ iBUS เข้า Arduino -> คำนวณความเร็ว 4 ล้อเอง (ไม่ต้องต่อคอม)
  หน้าเว็บ (ไม่บังคับ): เปิด Mecanum_Control.html ด้วย Chrome/Edge -> "เชื่อมต่อ" -> เลือกพอร์ต Arduino
    ใช้ตั้งค่าล้อ ทดสอบล้อ ดูค่าช่องรีโมท และขับแทนรีโมทได้ตอนปิดรีโมท

  ลำดับว่าใครคุม: ปุ่มทดสอบล้อในเว็บ (ตอนกดค้าง) > รีโมท > จอยในเว็บ

  บอร์ด: Arduino Mega 2560 (Tools > Board > Arduino AVR Boards > Arduino Mega or Mega 2560)
         ใช้กับ UNO R4 Minima ได้ด้วย (ต้องแก้ MOTORS[] ให้ตรงขา)

  การต่อสาย (Mega)
    รีซีฟ FlySky (เช่น FS-iA6B พอร์ต iBUS/SERVO):  สัญญาณ -> ขา 19 (RX1),  + -> 5V,  - -> GND
      ** อย่าต่อที่ขา 0 บน Mega — ขา 0 ใช้ร่วมกับ USB: อัปโหลดไม่ได้ หน้าเว็บสั่งไม่ได้
         และตอนเปิดเครื่องอาจค้างใน bootloader (โค้ดนี้ยังอ่านขา 0 ให้ แต่จะเตือนในหน้าเว็บ) **
    สาย USB -> คอมพิวเตอร์ (เฉพาะตอนใช้หน้าเว็บ)
    M1 = D2, D3    M2 = D4, D5      (driver แบบ IN1/IN2)
    M3 = TB6612 ช่อง A : D6 -> AIN1, D7 -> AIN2
    M4 = TB6612 ช่อง B : D8 -> BIN1, D9 -> BIN2
    TB6612: PWMA, PWMB, STBY, VCC -> 5V ของ Arduino,  VM -> แบตมอเตอร์ (ไม่เกิน 13.5V)
    GND ของ driver ทุกตัวต่อร่วมกับ GND ของ Arduino
    D13 (LED บนบอร์ด) ติดค้าง = กำลังรับคำสั่ง (รีโมทหรือเว็บ), กะพริบ = รอคำสั่ง (มอเตอร์หยุด)

  !! ตั้ง Failsafe ที่รีโมทด้วย: ช่อง 1, 2, 4 = ตรงกลาง (FS-i6: Functions setup > Failsafe)
     ไม่อย่างนั้นถ้ารีโมทดับ/หลุดระยะ รีซีฟจะส่งค่าล่าสุดค้างไว้ หุ่นจะวิ่งต่อ !!

  คำสั่ง (ข้อความทีละบรรทัด, 115200) — ลองพิมพ์ใน Serial Monitor ได้ (ตั้งท้ายบรรทัดเป็น Newline)
    D x y r                    ขับ: x = สไลด์ขวา, y = เดินหน้า, r = หมุนตามเข็ม (-100..100)
    T w v                      ทดสอบล้อ w (0=FL 1=FR 2=RL 3=RR, 4 = ทุกล้อ, -1 = ไม่มี) ความเร็ว v (-255..255)
    S                          หยุด
    C m0..m3 i0..i3 [g0..g3]   ตั้งช่องมอเตอร์ (0..3 = M1..M4), กลับทิศ (0/1)
                               และความแรง (30..100 %) ของล้อ FL FR RL RR
    W                          บันทึกค่าตั้งลง EEPROM
    G                          ขอค่าตั้งปัจจุบัน
  ไม่มีคำสั่งใหม่เกิน 0.5 วินาที -> หยุดเอง (คำสั่งที่พิมพ์ใน Serial Monitor จึงหมุนแค่ครึ่งวินาที)
  ตอบกลับเป็น JSON ทุก 100 ms: {"w":[FL,FR,RL,RR],"s":ผู้คุม}  (0 ไม่มี, 1 รีโมท, 2 เว็บ, 3 ทดสอบ)
                               {"rc":[สัญญาณ,เปิดให้ขับ,เฟรมดี,เฟรมเสีย,ไบต์],"ch":[CH1..CH8]}

  ไม่ต้องลง library เพิ่ม
*/
#include <EEPROM.h>

// ======================= ตั้งค่า =======================
const uint32_t BAUD = 115200;          // ต้องตรงกับในหน้าเว็บ

// ชนิด driver ของแต่ละช่อง (ตั้งแยกกันได้)
//   PWM_DIR : มอเตอร์ละ 1 ขา PWM + 1 ขาทิศทาง เช่น Cytron MD10C / MDD10A
//             ขา A = PWM (ต้องเป็นขา ~), ขา B = DIR
//   IN1_IN2 : มอเตอร์ละ 2 ขา IN1/IN2  เช่น L298N (เสียบจัมเปอร์ ENA/ENB ไว้), L9110S, MX1508, DRV8833
//             ขา A = IN1, ขา B = IN2  (ถ้าขาไหนไม่มี PWM โค้ดจัดการให้เองอัตโนมัติ)
enum { PWM_DIR = 0, IN1_IN2 = 1 };
struct MotorCh { uint8_t a, b, type; };

// ไม่ต้องสนใจว่าช่องไหนคือล้อไหน หรือหมุนกลับทาง — ตั้งได้จากหน้าเว็บ (แท็บ "ตั้งค่าล้อ")
// TB6612: PWMA, PWMB, STBY, VCC ต่อ 5V แล้วใช้ PWM ที่ขา IN แทน จึงตั้งเป็น IN1_IN2
const MotorCh MOTORS[4] = {
  {2, 3, IN1_IN2},     // M1: IN1 = D2, IN2 = D3
  {4, 5, IN1_IN2},     // M2: IN1 = D4, IN2 = D5
  {6, 7, IN1_IN2},     // M3: TB6612  AIN1 = D6, AIN2 = D7
  {8, 9, IN1_IN2},     // M4: TB6612  BIN1 = D8, BIN2 = D9
};

const uint8_t  MIN_PWM        = 0;     // ถ้ามอเตอร์ไม่ยอมหมุนตอนความเร็วต่ำ ลองตั้ง 40..80
const uint16_t CMD_TIMEOUT_MS = 500;   // ไม่มีคำสั่งนานเท่านี้ -> หยุด
const int      RAMP_UP        = 20;    // เร่งได้สูงสุดกี่หน่วย PWM ต่อ 20 ms (0->255 ≈ 0.26 วินาที)
const int      RAMP_DOWN      = 40;    // ตอนชะลอ/หยุด
const uint8_t  LED_PIN        = LED_BUILTIN;

// ---------- รีโมท FlySky (iBUS: Mega ขา 19 / UNO R4 ขา 0) ----------
// เลขช่องตามรีโมท (CH1..CH14) — ค่าเริ่มต้นเป็นรีโมท FS-i6 / FS-i6X แบบ Mode 2
// ดูค่าแต่ละช่องสดๆ ได้ในหน้าเว็บ แท็บ "ตั้งค่าล้อ"
#define RC_ENABLED 1
const uint8_t  RC_CH_X       = 1;      // สไลด์ซ้าย/ขวา   = คันขวา ซ้าย-ขวา
const uint8_t  RC_CH_Y       = 2;      // เดินหน้า/ถอย    = คันขวา ขึ้น-ลง
const uint8_t  RC_CH_R       = 4;      // หมุนตัว          = คันซ้าย ซ้าย-ขวา
const uint8_t  RC_CH_SPEED   = 3;      // ความเร็วสูงสุด  = คันซ้าย ขึ้น-ลง (คันเร่ง) ล่างสุด = ช้า, บนสุด = เต็ม  (0 = ไม่ใช้)
const uint8_t  RC_SPEED_MIN  = 30;     // % ความเร็วตอนคันเร่งอยู่ล่างสุด
const uint8_t  RC_CH_ARM     = 0;      // ช่องสวิตช์ "เปิดให้ขับ" เช่น 5 (ต้องเกิน 1700 ถึงขับได้)  0 = ไม่ใช้
const bool     RC_REV_X = false, RC_REV_Y = false, RC_REV_R = false;   // คันบังคับทิศกลับ -> true
const int      RC_DEADBAND   = 40;     // ช่วงกลางคันที่ถือว่าปล่อยคัน (µs)
const uint16_t RC_TIMEOUT_MS = 150;    // ไม่มีสัญญาณจากรีซีฟนานเท่านี้ -> หยุด
// =======================================================

// พอร์ตรับรีโมท: Serial1 = Mega ขา 19 (RX1) / UNO R4 ขา 0  -> ใช้รีโมทกับหน้าเว็บพร้อมกันได้
// นอกจากนี้ยังอ่าน iBUS ที่ขา 0 ของบอร์ด AVR (Mega/UNO R3) ให้ด้วย เผื่อต่อไว้ขา 0
// (ขา 0 ของบอร์ด AVR ใช้ร่วมกับ USB — ใช้ได้แต่ไม่แนะนำ)
#if RC_ENABLED && (defined(ARDUINO_ARCH_RENESAS) || defined(HAVE_HWSERIAL1))
  #define HAS_RC 1
  #define RC_SERIAL Serial1
  #if defined(ARDUINO_ARCH_RENESAS)
    #define RC_RX_PIN 0
  #else
    #define RC_RX_PIN 19
  #endif
#else
  #define HAS_RC 0
  #define RC_RX_PIN 0
#endif
#if RC_ENABLED && defined(ARDUINO_ARCH_AVR)
  #define RC_ON_PIN0 1                 // ขา 0 = Serial (ร่วมกับ USB)
#else
  #define RC_ON_PIN0 0
#endif

enum { FL = 0, FR = 1, RL = 2, RR = 3 };          // ซ้ายหน้า ขวาหน้า ซ้ายหลัง ขวาหลัง
enum { MODE_NONE = 0, MODE_DRIVE, MODE_TEST };

struct Config {
  uint8_t magic;
  uint8_t map[4];        // ล้อ FL/FR/RL/RR ใช้ช่อง M1..M4 ช่องไหน (0..3)
  uint8_t inv[4];        // 1 = กลับทิศล้อนั้น
  uint8_t gain[4];       // ความแรงของล้อนั้น 30..100 % (ใช้ลดล้อที่หมุนเร็วกว่าล้ออื่น)
};
const uint8_t CFG_MAGIC_V1 = 0xB7;     // รุ่นเก่า (ไม่มี gain) — ยังอ่านได้ ค่าช่อง/กลับทิศไม่หาย
const uint8_t CFG_MAGIC    = 0xB8;
const uint8_t GAIN_MIN     = 30;
Config cfg = {CFG_MAGIC, {0, 1, 2, 3}, {0, 0, 0, 0}, {100, 100, 100, 100}};

uint8_t  mode = MODE_NONE;
int      cmdX = 0, cmdY = 0, cmdR = 0;
int      testWheel = -1, testValue = 0;
uint32_t lastCmd = 0;
int16_t  out[4]     = {0, 0, 0, 0};   // ความเร็วแต่ละล้อหลัง ramp (ส่งกลับให้เว็บแสดง)
int16_t  current[4] = {0, 0, 0, 0};   // ค่าที่สั่งช่อง M1..M4 อยู่ตอนนี้
char     line[64];
uint8_t  lineLen  = 0;
bool     lineSkip = false;

enum { SRC_NONE = 0, SRC_RC = 1, SRC_WEB = 2, SRC_TEST = 3 };
uint8_t  source = SRC_NONE;          // ตอนนี้ใครคุมอยู่

uint16_t rcCh[14];                   // ค่าช่องรีโมท 1000..2000 (rcCh[0] = CH1)
uint32_t rcLastFrame = 0;
uint16_t rcFrames = 0, rcErrors = 0;
uint32_t rcBytes = 0;                // ไบต์ที่เข้าพอร์ตรีโมท (Serial1)
uint8_t  rcPort = 1;                 // เฟรมล่าสุดมาจาก 1 = Serial1, 0 = ขา 0
uint32_t rcPin0Frame = 0;            // เวลาที่เจอเฟรม iBUS ที่ขา 0 ล่าสุด

// ตัวแกะเฟรม iBUS (แยกกันต่อพอร์ต)
struct IbusParser { uint8_t buf[32]; uint8_t len; };
IbusParser ibus1 = {{0}, 0}, ibus0 = {{0}, 0};

// ---------------- มอเตอร์ ----------------
// ขาที่ทำ PWM ได้ (ขา ~)
//   Mega: 2..13, 44..46 / UNO R3, Nano: 3,5,6,9,10,11
//   UNO R4: ใช้แค่ 3,5,6,9,10,11 (digitalPinHasPWM ของ R4 นับขาอื่นที่อาจชนตัวจับเวลาด้วย)
static bool pwmCapable(uint8_t pin) {
#if defined(ARDUINO_ARCH_AVR) && defined(digitalPinHasPWM)
  return digitalPinHasPWM(pin);
#else
  return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11;
#endif
}

// ตั้งขาเป็น HIGH/LOW ธรรมดา
// UNO R4: หลัง analogWrite() ขานั้นถูกยกให้ตัวจับเวลา PWM — ถ้าสั่ง digitalWrite() เฉยๆ จะไม่มีผล
// (PWM ยังวิ่งอยู่ = มอเตอร์หมุนไม่หยุด) ต้อง pinMode() คืนเป็นขาธรรมดาก่อน
static void setLevel(uint8_t pin, uint8_t level) {
#if defined(ARDUINO_ARCH_RENESAS)
  if (pwmCapable(pin)) pinMode(pin, OUTPUT);
#endif
  digitalWrite(pin, level);
}

// dir: +1 / -1 / 0, mag: 0..255
static void driveChannel(const MotorCh &m, int8_t dir, uint8_t mag) {
  uint8_t a = m.a, b = m.b;
  if (m.type == PWM_DIR) {
    // Cytron MD10C (sign-magnitude): PWM = ความเร็ว, DIR = ทิศ, PWM = LOW -> มอเตอร์หยุด (เบรก)
    if (dir == 0 || mag == 0) { setLevel(a, LOW); return; }
    setLevel(b, dir < 0 ? HIGH : LOW);
    if (pwmCapable(a)) analogWrite(a, mag);
    else setLevel(a, mag >= 128 ? HIGH : LOW);   // ขา PWM ไม่ใช่ขา ~ -> ได้แค่เปิด/ปิด
    return;
  }
  if (dir == 0 || mag == 0) {          // ปล่อย (L298N = เบรก, DRV8833/TB6612 = ปล่อยไหล)
    setLevel(a, LOW);
    setLevel(b, LOW);
    return;
  }
  bool pa = pwmCapable(a), pb = pwmCapable(b);
  if (dir > 0) {                       // เดินหน้า: A ขับ, B = LOW
    if (pa)      { setLevel(b, LOW);  analogWrite(a, mag); }
    else if (pb) { setLevel(a, HIGH); analogWrite(b, 255 - mag); }   // ช่วงที่ B=HIGH คือเบรก
    else         { setLevel(b, LOW);  setLevel(a, mag >= 128 ? HIGH : LOW); }
  } else {                             // ถอยหลัง: B ขับ, A = LOW
    if (pb)      { setLevel(a, LOW);  analogWrite(b, mag); }
    else if (pa) { setLevel(b, HIGH); analogWrite(a, 255 - mag); }   // ช่วงที่ A=HIGH คือเบรก
    else         { setLevel(a, LOW);  setLevel(b, mag >= 128 ? HIGH : LOW); }
  }
}

static void setMotor(uint8_t i, int16_t s) {
  s = constrain(s, -255, 255);
  if (s == current[i]) return;
  current[i] = s;
  uint8_t mag = (uint8_t)abs(s);
  if (mag && MIN_PWM) mag = MIN_PWM + (uint16_t)mag * (255 - MIN_PWM) / 255;
  driveChannel(MOTORS[i], s > 0 ? 1 : (s < 0 ? -1 : 0), mag);
}

// ---------------- mecanum ----------------
// มองจากด้านบน ลูกกลิ้งของ 4 ล้อต้องเรียงเป็นรูปตัว X
static void mecanum(int x, int y, int r, int t[4]) {
  int v[4];
  v[FL] = y + x + r;
  v[FR] = y - x - r;
  v[RL] = y - x + r;
  v[RR] = y + x - r;
  int m = 100;                                   // ถ้าผลรวมเกิน 100 ให้ย่อทุกล้อด้วยสัดส่วนเดียวกัน
  for (uint8_t i = 0; i < 4; i++) if (abs(v[i]) > m) m = abs(v[i]);
  for (uint8_t i = 0; i < 4; i++) t[i] = (int)((long)v[i] * 255 / m);
}

static int stepToward(int cur, int target) {
  bool slowing = abs(target) < abs(cur) || (cur != 0 && (cur > 0) != (target > 0));
  int step = slowing ? RAMP_DOWN : RAMP_UP;
  if (target > cur) return min(cur + step, target);
  if (target < cur) return max(cur - step, target);
  return cur;
}

// ---------------- รีโมท FlySky iBUS ----------------
// เฟรม 32 ไบต์ ทุก ~7 ms: [0x20][0x40][CH1 lo][CH1 hi] ... [CH14 hi][chk lo][chk hi]
// chk = 0xFFFF - ผลรวม 30 ไบต์แรก
// ใส่ทีละไบต์ คืน true เมื่อได้เฟรมที่ถูกต้อง (ค่าช่องอัปเดตใน rcCh แล้ว)
static bool ibusFeed(IbusParser &p, uint8_t c, bool countErrors) {
  if (p.len == 0 && c != 0x20) return false;                     // รอหัวเฟรม
  if (p.len == 1 && c != 0x40) { p.len = (c == 0x20) ? 1 : 0; return false; }
  p.buf[p.len++] = c;
  if (p.len < 32) return false;
  p.len = 0;
  uint16_t sum = 0xFFFF;
  for (uint8_t i = 0; i < 30; i++) sum -= p.buf[i];
  if (sum != (uint16_t)(p.buf[30] | (p.buf[31] << 8))) { if (countErrors) rcErrors++; return false; }
  for (uint8_t i = 0; i < 14; i++)
    rcCh[i] = (uint16_t)(p.buf[2 + 2 * i] | (p.buf[3 + 2 * i] << 8)) & 0x0FFF;
  rcLastFrame = millis();
  rcFrames++;
  return true;
}

static void readRC() {
#if HAS_RC
  while (RC_SERIAL.available()) {
    uint8_t c = (uint8_t)RC_SERIAL.read();
    rcBytes++;
    if (ibusFeed(ibus1, c, true)) rcPort = 1;
  }
#endif
}

static uint16_t rcRaw(uint8_t ch) { return (ch >= 1 && ch <= 14) ? rcCh[ch - 1] : 1500; }

static bool rcValid(uint32_t now) {
  if (!(HAS_RC || RC_ON_PIN0) || rcFrames == 0 || now - rcLastFrame > RC_TIMEOUT_MS) return false;
  uint16_t y = rcRaw(RC_CH_Y);
  return y >= 800 && y <= 2200;                                  // ค่าเพี้ยน = ไม่เชื่อ
}

static bool rcArmed() { return RC_CH_ARM == 0 || rcRaw(RC_CH_ARM) > 1700; }

// คัน 1000..2000 -> -100..100 (มีช่วงกลางที่ถือว่า 0)
static int rcAxis(uint8_t ch, bool rev) {
  int v = (int)rcRaw(ch) - 1500;
  if (abs(v) <= RC_DEADBAND) return 0;
  v += (v > 0) ? -RC_DEADBAND : RC_DEADBAND;
  long o = constrain((long)v * 100 / (500 - RC_DEADBAND), -100L, 100L);
  return rev ? -(int)o : (int)o;
}

static int rcSpeedPct() {
  if (RC_CH_SPEED == 0) return 100;
  long t = constrain((long)rcRaw(RC_CH_SPEED), 1000L, 2000L);
  return RC_SPEED_MIN + (int)((t - 1000) * (100 - RC_SPEED_MIN) / 1000);
}

// ---------------- คำสั่งจากเว็บ ----------------
static uint8_t parseInts(const char *s, int *vals, uint8_t maxN) {
  uint8_t n = 0;
  while (n < maxN) {
    while (*s == ' ' || *s == ',' || *s == '\t') s++;
    if (!*s) break;
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s) break;
    vals[n++] = (int)constrain(v, -30000L, 30000L);
    s = end;
  }
  return n;
}

// ส่งข้อความ 1 บรรทัดให้เว็บ
// UNO R4 ใช้ USB แท้: ถ้าเปิดพอร์ตค้างไว้แต่คอมไม่อ่าน การส่งจะรอไม่จบ loop จะค้าง
// และระบบหยุดอัตโนมัติจะไม่ทำงาน — จึงข้ามบรรทัดนั้นไปถ้าบัฟเฟอร์ไม่พอ
static void sendLine(const char *s) {
#if defined(ARDUINO_ARCH_RENESAS)
  if (Serial.availableForWrite() < (int)strlen(s) + 2) return;
#endif
  Serial.println(s);
}

static void printConfig(uint8_t saved) {
  // rcm = ช่องรีโมทของ สไลด์,เดินหน้า,หมุน,ความเร็ว,เปิดขับ, มีรีโมทไหม, ขาที่แนะนำให้ต่อรีซีฟ
  // mp  = ขามอเตอร์ M1..M4 เป็นชุด {A, B, ชนิด(0 = Cytron PWM/DIR, 1 = IN1/IN2)} ให้เว็บแสดงชื่อช่อง
  char b[240];
  snprintf(b, sizeof b, "{\"cfg\":{\"m\":[%u,%u,%u,%u],\"v\":[%u,%u,%u,%u],\"g\":[%u,%u,%u,%u]},"
                        "\"rcm\":[%u,%u,%u,%u,%u,%u,%u],\"mp\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],\"saved\":%u}",
           cfg.map[0], cfg.map[1], cfg.map[2], cfg.map[3],
           cfg.inv[0], cfg.inv[1], cfg.inv[2], cfg.inv[3],
           cfg.gain[0], cfg.gain[1], cfg.gain[2], cfg.gain[3],
           RC_CH_X, RC_CH_Y, RC_CH_R, RC_CH_SPEED, RC_CH_ARM, (HAS_RC || RC_ON_PIN0) ? 1 : 0, RC_RX_PIN,
           MOTORS[0].a, MOTORS[0].b, MOTORS[0].type, MOTORS[1].a, MOTORS[1].b, MOTORS[1].type,
           MOTORS[2].a, MOTORS[2].b, MOTORS[2].type, MOTORS[3].a, MOTORS[3].b, MOTORS[3].type, saved);
  sendLine(b);
}

static void handleLine(char *s) {
  while (*s == ' ') s++;
  char c = (char)toupper(*s);
  int a[12];
  uint8_t n = parseInts(s + 1, a, 12);
  uint32_t now = millis();
  switch (c) {
    case 'D':
      if (n == 3) {
        mode = MODE_DRIVE;
        cmdX = constrain(a[0], -100, 100);
        cmdY = constrain(a[1], -100, 100);
        cmdR = constrain(a[2], -100, 100);
        lastCmd = now;
      }
      break;
    case 'T':
      if (n == 2) {
        mode = MODE_TEST;
        testWheel = constrain(a[0], -1, 4);           // 4 = ทุกล้อพร้อมกัน
        testValue = constrain(a[1], -255, 255);
        lastCmd = now;
      }
      break;
    case 'S':
      mode = MODE_DRIVE;
      cmdX = cmdY = cmdR = 0;
      lastCmd = now;
      break;
    case 'C':
      if (n == 8 || n == 12) {
        for (uint8_t i = 0; i < 4; i++) {
          cfg.map[i] = (uint8_t)constrain(a[i], 0, 3);
          cfg.inv[i] = a[4 + i] ? 1 : 0;
          if (n == 12) cfg.gain[i] = (uint8_t)constrain(a[8 + i], GAIN_MIN, 100);
        }
      }
      printConfig(0);
      break;
    case 'W':
      EEPROM.put(0, cfg);
      printConfig(1);
      break;
    case 'G':
      printConfig(0);
      break;
  }
}

static void readSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
#if RC_ON_PIN0
    // บอร์ด AVR: ถ้ารีซีฟต่อขา 0 ไบต์ iBUS จะมาปนในพอร์ตนี้ -> แกะเป็นรีโมท และหยุดอ่านคำสั่งข้อความ
    if (ibusFeed(ibus0, (uint8_t)ch, false)) { rcPort = 0; rcPin0Frame = millis(); }
    if (rcPin0Frame && millis() - rcPin0Frame < 1000) { lineLen = 0; lineSkip = true; continue; }
#endif
    if (ch == '\n' || ch == '\r') {
      if (lineLen && !lineSkip) { line[lineLen] = 0; handleLine(line); }
      lineLen = 0;
      lineSkip = false;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = ch;
    } else {
      lineSkip = true;                  // บรรทัดยาวเกิน -> ทิ้งทั้งบรรทัด
    }
  }
}

// ---------------- main ----------------
void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTORS[i].a, OUTPUT);
    pinMode(MOTORS[i].b, OUTPUT);
    digitalWrite(MOTORS[i].a, LOW);
    digitalWrite(MOTORS[i].b, LOW);
  }
  pinMode(LED_PIN, OUTPUT);

  Config saved;
  EEPROM.get(0, saved);
  if (saved.magic == CFG_MAGIC || saved.magic == CFG_MAGIC_V1) {
    for (uint8_t i = 0; i < 4; i++) {
      cfg.map[i] = saved.map[i] & 3;
      cfg.inv[i] = saved.inv[i] ? 1 : 0;
      if (saved.magic == CFG_MAGIC) cfg.gain[i] = constrain(saved.gain[i], GAIN_MIN, 100);
    }
  }

  for (uint8_t i = 0; i < 14; i++) rcCh[i] = 1500;
#if HAS_RC
  RC_SERIAL.begin(115200);              // iBUS 115200 8N1
#endif
  Serial.begin(BAUD);
  sendLine("{\"ready\":1}");
  printConfig(0);
}

void loop() {
  readSerial();
  readRC();
  uint32_t now = millis();

  static uint32_t lastTick = 0;
  if (now - lastTick >= 20) {                   // อัปเดตมอเตอร์ 50 ครั้ง/วินาที
    lastTick = now;
    bool webFresh = mode != MODE_NONE && now - lastCmd < CMD_TIMEOUT_MS;
    bool rcDrive  = rcValid(now) && rcArmed();
    int target[4] = {0, 0, 0, 0};

    // ใครคุม: ปุ่มทดสอบล้อ (กดค้างอยู่) > รีโมท > จอยในเว็บ
    if (webFresh && mode == MODE_TEST && testWheel >= 0) {
      source = SRC_TEST;
      for (uint8_t w = 0; w < 4; w++)
        if (testWheel == 4 || testWheel == w) target[w] = testValue;
    } else if (rcDrive) {
      source = SRC_RC;
      int s = rcSpeedPct();
      mecanum(rcAxis(RC_CH_X, RC_REV_X) * s / 100,
              rcAxis(RC_CH_Y, RC_REV_Y) * s / 100,
              rcAxis(RC_CH_R, RC_REV_R) * s / 100, target);
    } else if (webFresh && mode == MODE_DRIVE) {
      source = SRC_WEB;
      mecanum(cmdX, cmdY, cmdR, target);
    } else {
      source = SRC_NONE;                        // ไม่มีใครสั่ง -> หยุด
    }
    bool fresh = source != SRC_NONE;

    int16_t ch[4] = {0, 0, 0, 0};
    for (uint8_t w = 0; w < 4; w++) {
      out[w] = (int16_t)stepToward(out[w], target[w]);
      int16_t v = (int16_t)((long)out[w] * cfg.gain[w] / 100);   // ลดความแรงล้อที่เร็วกว่า
      ch[cfg.map[w] & 3] = cfg.inv[w] ? -v : v;
    }
    for (uint8_t i = 0; i < 4; i++) setMotor(i, ch[i]);

    // ไฟ D13: ติดค้าง = ได้รับคำสั่ง, กะพริบ 1 ครั้ง/วินาที = รอคำสั่ง
    digitalWrite(LED_PIN, fresh ? HIGH : (((now / 500) & 1) ? HIGH : LOW));
  }

  static uint32_t lastStatus = 0;
  if (now - lastStatus >= 100) {                // ส่งความเร็วล้อกลับไปให้เว็บแสดง
    lastStatus = now;
    int v[4];                                   // ค่าจริงที่ส่งให้มอเตอร์ (หลังปรับความแรง)
    for (uint8_t w = 0; w < 4; w++) v[w] = (int)((long)out[w] * cfg.gain[w] / 100);
    char b[160];
    snprintf(b, sizeof b, "{\"w\":[%d,%d,%d,%d],\"s\":%u}", v[0], v[1], v[2], v[3], source);
    sendLine(b);
#if HAS_RC || RC_ON_PIN0
    snprintf(b, sizeof b, "{\"rc\":[%d,%d,%u,%u,%lu,%u],\"ch\":[%u,%u,%u,%u,%u,%u,%u,%u]}",
             rcValid(now) ? 1 : 0, rcArmed() ? 1 : 0, rcFrames, rcErrors, (unsigned long)rcBytes, rcPort,
             rcCh[0], rcCh[1], rcCh[2], rcCh[3], rcCh[4], rcCh[5], rcCh[6], rcCh[7]);
    sendLine(b);
#endif
  }
}
