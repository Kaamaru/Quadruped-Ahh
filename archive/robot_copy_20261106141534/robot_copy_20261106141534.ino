#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>
#include <math.h>

// ---- Configuration & Pins ----
#define SDA_PIN 21
#define SCL_PIN 22
#define SERVO_FREQ 50
#define SERVOMIN 125   // PCA9685 pulse ticks for 0°
#define SERVOMAX 540   // PCA9685 pulse ticks for 180°

#define MAG_ADDR 0x2C
#define I2C_CLOCK 400000

// Balance PID Variables (changed from #define so they can be updated live)
float balKp = 2.5f;
float balKi = 0.1f;
float balKd = 0.02f;
float balMax = 50.0f;
float balDeadband = 1.0f;
float balAbortDeg = 50.0f;

// Leg servo mapping (Change pins to match your setup)
// sign = servo direction that pushes that leg DOWN (+1 or -1)
struct BalLeg { int8_t pin; int8_t fr; int8_t rl; int8_t sign; };
const BalLeg BAL_LEGS[4] = { 
  {4,  1, -1,  1}, // FL (Front-Left)
  {5,  1,  1, -1}, // FR (Front-Right)
  {6, -1, -1, -1}, // RL (Rear-Left)
  {7, -1,  1,  1}  // RR (Rear-Right)
};

// Standing base angles for each leg servo when flat
const float STANDING_ANGLES[4] = { 120.0f, 30.0f, 30.0f, 120.0f }; 

// Objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MPU6050 mpu;
Adafruit_Mahony filter;

// State variables
bool magOk = false;
bool balanceOn = false;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
unsigned long lastFusionUs = 0;
unsigned long lastBalMs = 0;

float tiltPitch = 0, tiltRoll = 0, tiltPitchRate = 0, tiltRollRate = 0;
bool tiltInit = false;
float tiltZeroPitch = 0, tiltZeroRoll = 0;

float balIp = 0, balIr = 0, balSat = 0, balErr0 = 0;
uint32_t balT0 = 0;

// Compass hard-iron offsets
float magOffX = -50;
float magOffY = -100;
float magOffZ = -60;

// Helper function declarations
bool readMag(float &mx, float &my, float &mz);
void calibrateGyro();
void initFusion();
bool updateFusion();
void balanceTick();
void writeServo(int pin, float angle);

void smoothStandUp() {
  Serial.println("Gliding to standing pose...");
  for (int step = 0; step <= 50; step++) {
    float progress = (float)step / 50.0f;
    for (int i = 0; i < 4; i++) {
      float currentAngle = 90.0f + (STANDING_ANGLES[i] - 90.0f) * progress;
      writeServo(BAL_LEGS[i].pin, currentAngle);
    }
    delay(20);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- STANDALONE BALANCE BOOTING ---");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);

  if (!pwm.begin()) {
    Serial.println("ERR: PCA9685 not detected!");
  } else {
    Serial.println("PCA9685 OK");
  }
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // Set servos on pins 0 to 4 to 90 degrees on boot
  int ticks90 = lroundf(SERVOMIN + (90.0f / 180.0f) * (SERVOMAX - SERVOMIN));
  for (int pin = 0; pin <= 4; pin++) {
    pwm.setPWM(pin, 0, ticks90);
  }

  initFusion();
  smoothStandUp();

  delay(1000);
  balanceOn = true;
  balT0 = millis();
  balIp = 0; balIr = 0; balSat = 0;
  balErr0 = fmaxf(fabsf(tiltPitch), fabsf(tiltRoll));
  Serial.println("Balance activated.");
}

void lastBalBead() {
  balIp = 0; balIr = 0; balSat = 0;
  balErr0 = fmaxf(fabsf(tiltPitch), fabsf(tiltRoll));
}

void loop() {
  readSerial(); // Listen for incoming WebSerial GUI commands

  if (updateFusion()) {
    // Sensor fusion tick
  }

  if (millis() - lastBalMs >= 20) {
    lastBalMs = millis();
    balanceTick();
  }
}

// ================= Sensor Fusion Logic =================
bool readMagRaw(int16_t &rx, int16_t &ry, int16_t &rz) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, 6);
  if (Wire.available() < 6) return false;
  uint8_t xl = Wire.read(); uint8_t xh = Wire.read();
  uint8_t yl = Wire.read(); uint8_t yh = Wire.read();
  uint8_t zl = Wire.read(); uint8_t zh = Wire.read();
  rx = (int16_t)(xh << 8 | xl);
  ry = (int16_t)(yh << 8 | yl);
  rz = (int16_t)(zh << 8 | zl);
  return true;
}

bool readMag(float &mx, float &my, float &mz) {
  int16_t rx, ry, rz;
  if (!readMagRaw(rx, ry, rz)) return false;
  mx = (float)rx - magOffX;
  my = -(float)ry - magOffY;
  mz = -(float)rz - magOffZ;
  return true;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro, keep robot completely still...");
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
    delay(10);
  }
  gyroBiasX = sx / 200.0f;
  gyroBiasY = sy / 200.0f;
  gyroBiasZ = sz / 200.0f;
  Serial.println("Gyro calibrated.");
}

void initFusion() {
  if (!mpu.begin()) {
    Serial.println("ERR: MPU6050 not found!");
    return;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0xCF);
  magOk = (Wire.endTransmission() == 0);

  calibrateGyro();
  filter.begin(100);
  lastFusionUs = micros();
}

bool updateFusion() {
  unsigned long now = micros();
  unsigned long dt = now - lastFusionUs;
  if (dt < 10000) return false;
  lastFusionUs = (dt > 50000) ? now : lastFusionUs + 10000;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float gx = (g.gyro.x - gyroBiasX) * 57.2958f;
  float gy = (g.gyro.y - gyroBiasY) * 57.2958f;
  float gz = (g.gyro.z - gyroBiasZ) * 57.2958f;

  float accPitch = atan2f(-a.acceleration.x, sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958f;
  float accRoll = atan2f(a.acceleration.y, a.acceleration.z) * 57.2958f;
  
  if (!tiltInit) { 
    tiltPitch = accPitch; 
    tiltRoll = accRoll; 
    tiltInit = true; 
  }
  
  tiltPitch = 0.98f * (tiltPitch + gy * 0.01f) + 0.02f * accPitch;
  tiltRoll = 0.98f * (tiltRoll + gx * 0.01f) + 0.02f * accRoll;
  tiltPitchRate = gy;
  tiltRollRate = gx;

  float mx, my, mz;
  if (magOk && readMag(mx, my, mz)) {
    filter.update(gx, gyroBiasY, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z, mx, my, mz);
  } else {
    filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);
  }
  return true;
}

// Servo Output Helper
void writeServo(int pin, float angle) {
  int ticks = lroundf(SERVOMIN + (constrain(angle, 0.0f, 180.0f) / 180.0f) * (SERVOMAX - SERVOMIN));
  pwm.setPWM(pin, 0, ticks);
}

// ================= Balance Controller Logic =================
void balanceTick() {
  float pitch = tiltPitch - tiltZeroPitch;
  float roll = tiltRoll - tiltZeroRoll;

  // 1. Emergency safety cutoff if tilted too far
  if (balanceOn && (fabsf(pitch) > balAbortDeg || fabsf(roll) > balAbortDeg)) {
    balanceOn = false;
    Serial.println("Balance ABORTED: Tilt exceeded safety limit.");
    return;
  }

  // 2. Auto-Recovery: If balance is off, check if the user straightened it back up
  if (!balanceOn) {
    if (fabsf(pitch) < 10.0f && fabsf(roll) < 10.0f) {
      balanceOn = true;
      balIp = 0; // Clear integral windup so it doesn't jump
      balIr = 0;
      Serial.println("Balance RESTORED: Robot is upright again.");
    } else {
      return; // Stay off while it's still lying down
    }
  }

  float ep = (fabsf(pitch) <= balDeadband) ? 0.0f : pitch - copysignf(balDeadband, pitch);
  float er = (fabsf(roll) <= balDeadband) ? 0.0f : roll - copysignf(balDeadband, roll);

  balIp = constrain(balIp + balKi * ep * 0.02f, -balMax, balMax);
  balIr = constrain(balIr + balKi * er * 0.02f, -balMax, balMax);

  float uP = constrain(balKp * ep + balIp + balKd * tiltPitchRate, -balMax, balMax);
  float uR = constrain(balKp * er + balIr + balKd * tiltRollRate, -balMax, balMax);

  // Apply balance corrections to the 4 lift leg servos
  for (int i = 0; i < 4; i++) {
    int pin = BAL_LEGS[i].pin;
    float ext = constrain(BAL_LEGS[i].fr * uP + BAL_LEGS[i].rl * uR, -balMax, balMax);
    float targetAngle = STANDING_ANGLES[i] + (BAL_LEGS[i].sign * ext);
    
    writeServo(pin, targetAngle);
  }
}

String serialLine = "";

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      handleSerialLine(serialLine);
      serialLine = "";
    } else if (ch != '\r') {
      serialLine += ch;
      if (serialLine.length() > 100) serialLine = "";
    }
  }
}

void handleSerialLine(String s) {
  s.trim();
  if (s.startsWith("bt ")) { // Receives "bt <kp> <ki> <kd> <max>" from your GUI
    float a, b, c2, d2;
    if (sscanf(s.c_str() + 3, "%f %f %f %f", &a, &b, &c2, &d2) == 4) {
      balKp = constrain(a, 0.0f, 10.0f);
      balKi = constrain(b, 0.0f, 5.0f);
      balKd = constrain(c2, 0.0f, 2.0f);
      balMax = constrain(d2, 2.0f, 45.0f);
      Serial.printf("OK bt updated: Kp=%.2f Ki=%.2f Kd=%.2f Max=%.1f\n", balKp, balKi, balKd, balMax);
    }
  }
}