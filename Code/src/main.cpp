#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#define DISABLE_FS_H_WARNING
#include <SdFat.h>
#include <ESP32Servo.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_BMP5xx.h>
#include <atomic>

#define FLIGHT_MODE        0
#define CALIBRATE_ON_BOOT  1
#define SERIAL_TELEMETRY   1

constexpr int PIN_SRV1 = 4, PIN_SRV2 = 5, PIN_BLDC1 = 6, PIN_BLDC2 = 7;
constexpr int PIN_I2C_SDA = 8, PIN_I2C_SCL = 9;                  // BMPSDI / BMPSCK
constexpr int PIN_SD_CS = 10, PIN_SPI_MOSI = 11, PIN_SPI_SCK = 12, PIN_SPI_MISO = 13;
constexpr int PIN_BMP_INT = 15, PIN_IMU_INT1 = 16, PIN_IMU_INT2 = 17;   // unused for now
constexpr int PIN_IMU_CS = 18;

#define GRAVDIR BMI2_GRAVITY_POS_Z

constexpr uint32_t LOOP_HZ = 200;
constexpr uint32_t LOG_HZ = 50;
constexpr uint32_t IMU_SPI_HZ = 8000000;   // drop to 1-2 MHz if init fails (layout / wiring)
constexpr uint32_t SD_SPI_HZ = 20000000;
constexpr uint32_t I2C_HZ = 400000;
constexpr float TAKEOFF_ALT_M = 1.5f;
constexpr float MAX_ALT_M = 3.0f;
constexpr float CLIMB_RATE_MS = 0.3f;          // setpoint slew for takeoff / landing / API changes
constexpr float MAX_TILT_CMD_DEG = 15.0f;
constexpr float TILT_CUTOFF_DEG = 60.0f;       // beyond this in flight -> KILL
constexpr uint32_t ARM_DELAY_MS = 10000;       // countdown after boot before auto-takeoff
constexpr float HOVER_THROTTLE = 0.45f;        // feed-forward guess, tune this first
constexpr float THROTTLE_MIN = 0.10f;          // while flying (0..1 of ESC range)
constexpr float THROTTLE_MAX = 0.80f;
constexpr float THROTTLE_SLEW = 0.5f;          // collective change per second
constexpr int ESC_MIN_US = 1000, ESC_MAX_US = 2000, ESC_PWM_HZ = 50;
constexpr int SERVO_CENTER_US = 1500, SERVO_TRAVEL_US = 300, SERVO_PWM_HZ = 50;
constexpr int SERVO1_TRIM_US = 0, SERVO2_TRIM_US = 0;
constexpr float SERVO1_DIR = 1.0f, SERVO2_DIR = 1.0f;   // -1 if a servo is mounted mirrored
constexpr float ACC_TAU_S = 2.0f;      // accel->attitude correction time constant
constexpr float GYRO_LPF_HZ = 60.0f;   // for the D terms only
constexpr float ALT_W = 4.0f;          // baro/accel altitude filter bandwidth (rad/s)

constexpr char AP_SSID[] = "FlightConrolelr";
constexpr char AP_PASS[] = "123456";

enum FcState : uint8_t { ST_IDLE, ST_WAIT, ST_TAKEOFF, ST_HOLD, ST_LAND, ST_KILL };
static const char *STATE_NAME[] = {"idle", "wait", "takeoff", "hold", "land", "kill"};
enum Axis { AX_ALT, AX_ROLL, AX_PITCH, AX_YAW };

struct Targets { float alt, roll, pitch, yaw; };
struct Gains { float kp, ki, kd; };
struct Telemetry {
  uint32_t t_ms; uint8_t state;
  float roll, pitch, yaw, gx, gy, gz;
  float alt, vz, altSp, rollSp, pitchSp, yawSp;
  int m1, m2, s1, s2;
};

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static Targets g_target = {TAKEOFF_ALT_M, 0, 0, 0};
static Gains g_gains[4] = {
	{0.25f, 0.10f, 0.15f},     // out: throttle fraction, err in m, D on vz
	{0.010f, 0.005f, 0.0015f}, // out: differential throttle, err in deg, D on gyro
	{0.030f, 0.010f, 0.0040f}, // out: -1..1 servo deflection
	{0.020f, 0.000f, 0.0040f}, // out: -1..1 differential servo deflection
};
static Telemetry g_tel = {};
static std::atomic<bool> g_cmdTakeoff{false}, g_cmdLand{false}, g_cmdKill{false};
static QueueHandle_t logQ = nullptr;

BMI270 imu;
Adafruit_BMP5xx bmp;
Servo srv1, srv2, esc1, esc2;
SdFs SD;
WebServer server(80);

static float p0Hpa = 1013.25f;
static bool sensorsOk = false;
static FcState state = ST_IDLE;
static uint32_t waitUntilMs = 0;

struct Est { float roll = 0, pitch = 0, yaw = 0, gx = 0, gy = 0, gz = 0, alt = 0, vz = 0; };
static Est est;

static float wrap180(float a) { while (a > 180.f) a -= 360.f; while (a < -180.f) a += 360.f; return a; }

static float baroAltitude(float pHpa) { return 44330.0f * (1.0f - powf(pHpa / p0Hpa, 0.1903f)); }

static float verticalAccel(float ax, float ay, float az) {
  float r = est.roll * DEG_TO_RAD, p = est.pitch * DEG_TO_RAD;
  float f = -ax * sinf(p) + ay * sinf(r) * cosf(p) + az * cosf(r) * cosf(p);
  return (f - 1.0f) * 9.80665f;
}

static bool readBaro(float &pHpa) {
  if (!bmp.dataReady()) return false;
  if (!bmp.performReading()) return false;
  pHpa = (float)bmp.pressure;       // hPa
  return true;
}

struct PID {
  float iLimit, outLimit, integ;
  PID(float il, float ol) : iLimit(il), outLimit(ol), integ(0) {}
  void reset() { integ = 0; }
  float update(const Gains &g, float err, float rate, float dt, bool iEnable) {
    if (iEnable) integ = constrain(integ + g.ki * err * dt, -iLimit, iLimit);
    return constrain(g.kp * err + integ - g.kd * rate, -outLimit, outLimit);
  }
};
static PID pid[4] = {PID(0.15f, 0.30f), PID(0.05f, 0.15f), PID(0.20f, 1.0f), PID(0.20f, 1.0f)};
static void resetPids() { for (auto &p : pid) p.reset(); }

static bool initIMU() {
  if (imu.beginSPI(PIN_IMU_CS, IMU_SPI_HZ, SPI) != BMI2_OK) {
    Serial.println("Can't begin IMU");
    return false;
  }
#if CALIBRATE_ON_BOOT
  Serial.println("Calibrating IMU, keep it still...");
  int s0 = imu.performAccelOffsetCalibration(GRAVDIR);
  int s1 = imu.performGyroOffsetCalibration();
  int s2 = imu.performComponentRetrim();
  if (s0 < 0 || s1 < 0 || s2 < 0) {
    Serial.printf("Calibration error (%d %d %d)\n", s0, s1, s2);
    return false;
  }
  if (s0 || s1 || s2) Serial.printf("Calibration warning (%d %d %d)\n", s0, s1, s2);
#endif
  return true;
}

static bool initBMP() {
  if (!bmp.begin(0x46, &Wire)) {
    Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
    return false;
  }
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  bmp.enablePressure(true);
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH,
                         BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);
  return true;
}

static bool initSD() {
  if (!SD.begin(PIN_SD_CS)) { Serial.println("Can't begin SD"); return false; }
  return true;
}

static bool captureBaseline() {
  float sx = 0, sy = 0, sz = 0; const int N = 100;
  for (int i = 0; i < N; i++) {
    imu.getSensorData();
    float ax = imu.data.accelX, ay = imu.data.accelY, az = imu.data.accelZ;
    sx += ax; sy += ay; sz += az;
    delay(10);
  }
  sx /= N; sy /= N; sz /= N;
  est.roll = atan2f(sy, sz) * RAD_TO_DEG;
  est.pitch = atan2f(-sx, sqrtf(sy * sy + sz * sz)) * RAD_TO_DEG;
  est.yaw = 0; est.alt = 0; est.vz = 0;

  float psum = 0; int n = 0, seen = 0; uint32_t t0 = millis();
  while (n < 50 && millis() - t0 < 4000) {
    float p;
    if (readBaro(p) && ++seen > 10) { psum += p; n++; }   // skip first 10 (settling)
    delay(5);
  }
  if (n < 30) { Serial.println("Baro baseline failed"); return false; }
  p0Hpa = psum / n;
  Serial.printf("Baseline: roll %.2f pitch %.2f p0 %.2f hPa\n", est.roll, est.pitch, p0Hpa);
  return true;
}

static void writeOutputs(int m1, int m2, int s1, int s2) {
  esc1.writeMicroseconds(m1); esc2.writeMicroseconds(m2);
  srv1.writeMicroseconds(s1); srv2.writeMicroseconds(s2);
}
static void idleOutputs() {
  writeOutputs(ESC_MIN_US, ESC_MIN_US, SERVO_CENTER_US + SERVO1_TRIM_US, SERVO_CENTER_US + SERVO2_TRIM_US);
}

static void sendJson(int code, const char *body) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json", body);
}

static void sendState() {
  Telemetry t; Targets tg;
  portENTER_CRITICAL(&mux); t = g_tel; tg = g_target; portEXIT_CRITICAL(&mux);
  char b[560];
  snprintf(b, sizeof b,
    "{\"state\":\"%s\",\"t_ms\":%lu,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"alt\":%.3f,\"vz\":%.3f,\"alt_sp\":%.3f,"
    "\"target\":{\"alt\":%.2f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f},"
    "\"motor_us\":[%d,%d],\"servo_us\":[%d,%d],\"flight_mode\":%d,\"sensors_ok\":%d}",
    STATE_NAME[t.state], (unsigned long)t.t_ms, t.roll, t.pitch, t.yaw, t.alt, t.vz, t.altSp,
    tg.alt, tg.roll, tg.pitch, tg.yaw, t.m1, t.m2, t.s1, t.s2, (int)FLIGHT_MODE, (int)sensorsOk);
  sendJson(200, b);
}

static void handleTarget() {
  Targets t;
  portENTER_CRITICAL(&mux); t = g_target; portEXIT_CRITICAL(&mux);
  if (server.hasArg("alt"))   t.alt   = constrain(server.arg("alt").toFloat(), 0.3f, MAX_ALT_M);
  if (server.hasArg("roll"))  t.roll  = constrain(server.arg("roll").toFloat(), -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);
  if (server.hasArg("pitch")) t.pitch = constrain(server.arg("pitch").toFloat(), -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);
  if (server.hasArg("yaw"))   t.yaw   = wrap180(server.arg("yaw").toFloat());
  portENTER_CRITICAL(&mux); g_target = t; portEXIT_CRITICAL(&mux);
  sendState();
}

static void handlePid() {
  String a = server.arg("axis");
  int ix = a == "alt" ? AX_ALT : a == "roll" ? AX_ROLL : a == "pitch" ? AX_PITCH : a == "yaw" ? AX_YAW : -1;
  if (ix < 0) { sendJson(400, "{\"error\":\"axis must be alt|roll|pitch|yaw\"}"); return; }
  Gains g;
  portENTER_CRITICAL(&mux); g = g_gains[ix]; portEXIT_CRITICAL(&mux);
  if (server.hasArg("kp")) g.kp = fmaxf(0, server.arg("kp").toFloat());
  if (server.hasArg("ki")) g.ki = fmaxf(0, server.arg("ki").toFloat());
  if (server.hasArg("kd")) g.kd = fmaxf(0, server.arg("kd").toFloat());
  portENTER_CRITICAL(&mux); g_gains[ix] = g; portEXIT_CRITICAL(&mux);
  char b[112];
  snprintf(b, sizeof b, "{\"axis\":\"%s\",\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f}", a.c_str(), g.kp, g.ki, g.kd);
  sendJson(200, b);
}

static void webTask(void *) {
  for (;;) { server.handleClient(); vTaskDelay(pdMS_TO_TICKS(2)); }
}

static void startWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP up, IP: "); Serial.println(WiFi.softAPIP());
  server.on("/state", HTTP_GET, sendState);
  server.on("/target", HTTP_POST, handleTarget);
  server.on("/pid", HTTP_POST, handlePid);
  server.on("/land", HTTP_POST, []() { g_cmdLand = true; sendJson(200, "{\"ok\":true}"); });
  server.on("/kill", HTTP_POST, []() { g_cmdKill = true; sendJson(200, "{\"ok\":true}"); });
  server.on("/takeoff", HTTP_POST, []() {
    if (!FLIGHT_MODE) { sendJson(409, "{\"error\":\"bench mode\"}"); return; }
    g_cmdTakeoff = true; sendJson(200, "{\"ok\":true}");
  });
  server.begin();
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 0);
}


static char logBuf[4096];

static void logTask(void *) {
  char name[24];
  for (int i = 0; i < 1000; i++) {
    snprintf(name, sizeof name, "/flight_%03d.csv", i);
    if (!SD.exists(name)) break;
  }
  FsFile f = SD.open(name, FILE_WRITE);
  if (!f) { Serial.println("Log open failed"); vTaskDelete(nullptr); }
  Serial.printf("Logging to %s\n", name);
  f.println("t_ms,state,roll,pitch,yaw,gx,gy,gz,alt,vz,alt_sp,roll_sp,pitch_sp,yaw_sp,m1_us,m2_us,s1_us,s2_us");

  size_t n = 0; uint32_t lastFlush = millis(); Telemetry r;
  for (;;) {
    if (xQueueReceive(logQ, &r, pdMS_TO_TICKS(100)) == pdTRUE) {
      n += snprintf(logBuf + n, sizeof logBuf - n,
        "%lu,%s,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%d,%d,%d\n",
        (unsigned long)r.t_ms, STATE_NAME[r.state], r.roll, r.pitch, r.yaw, r.gx, r.gy, r.gz,
        r.alt, r.vz, r.altSp, r.rollSp, r.pitchSp, r.yawSp, r.m1, r.m2, r.s1, r.s2);
    }

    if (n > sizeof logBuf - 256 || (n && millis() - lastFlush > 1000)) {
      f.write((const uint8_t *)logBuf, n);
      f.flush();
      n = 0; lastFlush = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  vTaskPrioritySet(nullptr, 5);  

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  esc1.setPeriodHertz(ESC_PWM_HZ); esc2.setPeriodHertz(ESC_PWM_HZ);
  srv1.setPeriodHertz(SERVO_PWM_HZ); srv2.setPeriodHertz(SERVO_PWM_HZ);
  esc1.attach(PIN_BLDC1, ESC_MIN_US, ESC_MAX_US);
  esc2.attach(PIN_BLDC2, ESC_MIN_US, ESC_MAX_US);
  srv1.attach(PIN_SRV1, 500, 2500);
  srv2.attach(PIN_SRV2, 500, 2500);
  idleOutputs();

  pinMode(PIN_IMU_CS, OUTPUT); digitalWrite(PIN_IMU_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);  digitalWrite(PIN_SD_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);   
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);

  bool imuOk = initIMU();
  bool bmpOk = initBMP();
  bool sdOk = initSD();
  sensorsOk = imuOk && bmpOk && captureBaseline();
  if (!sensorsOk) Serial.println("!! Sensor init failed, flight disabled");

  startWeb();

  if (sdOk) {
    logQ = xQueueCreate(128, sizeof(Telemetry));
    xTaskCreatePinnedToCore(logTask, "log", 8192, nullptr, 1, nullptr, 0);
  }

  if (sensorsOk && FLIGHT_MODE) {
    state = ST_WAIT;
    waitUntilMs = millis() + ARM_DELAY_MS;
    Serial.printf("FLIGHT MODE: takeoff in %lu s. POST /kill to abort.\n", (unsigned long)(ARM_DELAY_MS / 1000));
  } else {
    state = ST_IDLE;
  }
}

void loop() {
  static TickType_t lastWake = xTaskGetTickCount();
  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / LOOP_HZ));

  static uint32_t prevUs = micros(), lastBaroUs = micros(), loopCount = 0, landZeroMs = 0, frozen = 0;
  static float altSp = 0, yawSp = 0, thr = 0, lastAx = 0, lastGx = 0;

  const uint32_t nowUs = micros(), nowMs = millis();
  const float dt = constrain((nowUs - prevUs) * 1e-6f, 0.001f, 0.02f);
  prevUs = nowUs; loopCount++;

  if (!sensorsOk) { idleOutputs(); return; }

  imu.getSensorData();
  float ax = imu.data.accelX, ay = imu.data.accelY, az = imu.data.accelZ;
  float gx = imu.data.gyroX,  gy = imu.data.gyroY,  gz = imu.data.gyroZ;

  if (ax == lastAx && gx == lastGx) frozen++; else frozen = 0;   // stuck IMU detector
  lastAx = ax; lastGx = gx;

  est.roll += gx * dt; est.pitch += gy * dt; est.yaw = wrap180(est.yaw + gz * dt);
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag > 0.8f && mag < 1.2f) {                       // only trust accel when ~1 g
    float k = dt / (ACC_TAU_S + dt);
    est.roll  += k * (atan2f(ay, az) * RAD_TO_DEG - est.roll);
    est.pitch += k * (atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG - est.pitch);
  }
  float lp = dt / (dt + 1.0f / (TWO_PI * GYRO_LPF_HZ));
  est.gx += lp * (gx - est.gx); est.gy += lp * (gy - est.gy); est.gz += lp * (gz - est.gz);

  float aZ = verticalAccel(ax, ay, az);
  est.alt += est.vz * dt + 0.5f * aZ * dt * dt;
  est.vz += aZ * dt;
  float pHpa;
  if ((loopCount & 1) == 0 && readBaro(pHpa)) {
    float dtb = constrain((nowUs - lastBaroUs) * 1e-6f, 0.005f, 0.2f);
    lastBaroUs = nowUs;
    float e = baroAltitude(pHpa) - est.alt;
    est.alt += 2.0f * ALT_W * dtb * e;
    est.vz += ALT_W * ALT_W * dtb * e;
  }

  if (g_cmdKill.exchange(false)) state = ST_KILL;
  if (g_cmdLand.exchange(false) && (state == ST_TAKEOFF || state == ST_HOLD)) { state = ST_LAND; landZeroMs = 0; }
  if (g_cmdTakeoff.exchange(false) && state == ST_IDLE && FLIGHT_MODE) { waitUntilMs = nowMs + 3000; state = ST_WAIT; }

  bool airborne = (state == ST_TAKEOFF || state == ST_HOLD || state == ST_LAND);
  if (airborne && (fabsf(est.roll) > TILT_CUTOFF_DEG || fabsf(est.pitch) > TILT_CUTOFF_DEG)) state = ST_KILL;
  if (frozen > LOOP_HZ / 4) state = ST_KILL;

  if (state == ST_WAIT && (int32_t)(nowMs - waitUntilMs) >= 0) {
    altSp = fmaxf(est.alt, 0.0f); yawSp = est.yaw; thr = 0; resetPids();
    state = ST_TAKEOFF;
  }

  Targets tg; Gains g[4];
  portENTER_CRITICAL(&mux); tg = g_target; memcpy(g, g_gains, sizeof g); portEXIT_CRITICAL(&mux);

  const bool flying = (state == ST_TAKEOFF || state == ST_HOLD || state == ST_LAND);
  const bool bench = !FLIGHT_MODE && state != ST_KILL;

  float rollSp = 0, pitchSp = 0;
  if (state == ST_TAKEOFF || state == ST_HOLD) {
    altSp += constrain(tg.alt - altSp, -CLIMB_RATE_MS * dt, CLIMB_RATE_MS * dt);
    if (state == ST_TAKEOFF && fabsf(tg.alt - altSp) < 0.01f) state = ST_HOLD;
  }
  if (state == ST_LAND) {
    altSp = fmaxf(altSp - CLIMB_RATE_MS * dt, 0.0f);
    if (altSp <= 0.0f) {
      if (landZeroMs == 0) landZeroMs = nowMs;
      if (est.alt < 0.12f || nowMs - landZeroMs > 2500) state = ST_IDLE;   // timeout in case baro drifted
    }
  }
  if (state == ST_HOLD || bench) { rollSp = tg.roll; pitchSp = tg.pitch; yawSp = tg.yaw; }

  float altOut = 0, rollOut = 0, pitchOut = 0, yawOut = 0;
  if (flying || bench) {
    const bool iOn = flying && est.alt > 0.15f;           // no integral windup while sitting on the ground
    if (flying) altOut = pid[AX_ALT].update(g[AX_ALT], altSp - est.alt, est.vz, dt, iOn);
    rollOut  = pid[AX_ROLL].update(g[AX_ROLL], rollSp - est.roll, est.gx, dt, iOn);
    pitchOut = pid[AX_PITCH].update(g[AX_PITCH], pitchSp - est.pitch, est.gy, dt, iOn);
    yawOut   = pid[AX_YAW].update(g[AX_YAW], wrap180(yawSp - est.yaw), est.gz, dt, iOn);
  } else {
    resetPids();
  }

  int m1us = ESC_MIN_US, m2us = ESC_MIN_US;
  if (flying && FLIGHT_MODE) {
    float target = HOVER_THROTTLE + altOut;
    thr += constrain(target - thr, -THROTTLE_SLEW * dt, THROTTLE_SLEW * dt);
    float m1 = constrain(thr + rollOut, THROTTLE_MIN, THROTTLE_MAX);
    float m2 = constrain(thr - rollOut, THROTTLE_MIN, THROTTLE_MAX);
    m1us = ESC_MIN_US + (int)(m1 * (ESC_MAX_US - ESC_MIN_US));
    m2us = ESC_MIN_US + (int)(m2 * (ESC_MAX_US - ESC_MIN_US));
  } else {
    thr = 0;
  }
  int s1us = SERVO_CENTER_US + SERVO1_TRIM_US + (int)(SERVO1_DIR * constrain(pitchOut + yawOut, -1.0f, 1.0f) * SERVO_TRAVEL_US);
  int s2us = SERVO_CENTER_US + SERVO2_TRIM_US + (int)(SERVO2_DIR * constrain(pitchOut - yawOut, -1.0f, 1.0f) * SERVO_TRAVEL_US);
  writeOutputs(m1us, m2us, s1us, s2us);

  Telemetry t = {nowMs, (uint8_t)state, est.roll, est.pitch, est.yaw, gx, gy, gz,
                 est.alt, est.vz, altSp, rollSp, pitchSp, yawSp, m1us, m2us, s1us, s2us};
  portENTER_CRITICAL(&mux); g_tel = t; portEXIT_CRITICAL(&mux);
  if (logQ && loopCount % (LOOP_HZ / LOG_HZ) == 0) xQueueSend(logQ, &t, 0);   // never block the loop

#if SERIAL_TELEMETRY
  if (loopCount % (LOOP_HZ / 5) == 0)
    Serial.printf("%-7s r%6.1f p%6.1f y%6.1f | alt %5.2f vz %5.2f sp %4.2f | m %d/%d s %d/%d\n",
                  STATE_NAME[state], est.roll, est.pitch, est.yaw, est.alt, est.vz, altSp, m1us, m2us, s1us, s2us);
#endif
}