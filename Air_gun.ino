#include <Wire.h>
#include <BleMouse.h>

BleMouse bleMouse("M Pro", "eTan", 100);

#define MPU6050_ADDR 0x68
#define LEFT_BUTTON 15
#define RIGHT_BUTTON 13

float GYRO_SENSITIVITY = 180.0f;
float FILTER_ALPHA = 0.55f;
int DEADZONE = 20;

// Offsets
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
bool calibrated = false;

// Smoothed
float dx_smooth = 0, dy_smooth = 0;
float angleYaw = 0, anglePitch = 0;

unsigned long lastTime = 0;
bool lastLeftState = HIGH, lastRightState = HIGH;
unsigned long lastLeftTime = 0, lastRightTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// ---------------- MPU FUNCTIONS ----------------
void readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
}

void calibrateMPU() {
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int samples = 1000;
  Serial.println("Calibrating... keep still");

  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    readGyro(gx, gy, gz);
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(3);
  }

  gx_offset = gx_sum / (float)samples;
  gy_offset = gy_sum / (float)samples;
  gz_offset = gz_sum / (float)samples;
  calibrated = true;

  Serial.printf("Calibration done! gx=%.2f gy=%.2f gz=%.2f\n", gx_offset, gy_offset, gz_offset);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);

  bleMouse.begin();
  Serial.println("BLE Air Mouse Ready!");

  calibrateMPU();
  lastTime = millis();
}

// ---------------- LOOP ----------------
void loop() {
  if (bleMouse.isConnected() && calibrated) {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    readGyro(gx, gy, gz);
    readAccel(ax, ay, az);

    float gz_corr = gz - gz_offset;
    float gy_corr = gy - gy_offset;

    // Filter out very slow drifts
    if (abs(gz_corr) < 5) gz_corr = 0;  // suppress slow yaw drift
    if (abs(gy_corr) < DEADZONE) gy_corr = 0;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Gyro integration
    float gyroYawRate = gz_corr / GYRO_SENSITIVITY;
    float gyroPitchRate = gy_corr / GYRO_SENSITIVITY;

    angleYaw += gyroYawRate * dt;
    anglePitch += gyroPitchRate * dt;

    // Natural drift decay (brings yaw back to 0 slowly)
    angleYaw *= 0.995;

    // Accel correction for pitch only
    float accelPitch = atan2(-ax, az) * 180 / PI;
    anglePitch = 0.98 * anglePitch + 0.02 * accelPitch;

    // Movement mapping
    dx_smooth = FILTER_ALPHA * dx_smooth + (1 - FILTER_ALPHA) * (-angleYaw / 2.0);
    dy_smooth = FILTER_ALPHA * dy_smooth + (1 - FILTER_ALPHA) * (-anglePitch / 2.5);

    if ((int)dx_smooth != 0 || (int)dy_smooth != 0)
      bleMouse.move((int)dx_smooth, (int)dy_smooth);

    // --- Buttons ---
    unsigned long currentMillis = millis();

    bool currentLeft = digitalRead(LEFT_BUTTON);
    if (currentLeft != lastLeftState && currentMillis - lastLeftTime > DEBOUNCE_DELAY) {
      lastLeftTime = currentMillis;
      if (currentLeft == LOW) bleMouse.press(MOUSE_LEFT);
      else bleMouse.release(MOUSE_LEFT);
      lastLeftState = currentLeft;
    }

    bool currentRight = digitalRead(RIGHT_BUTTON);
    if (currentRight != lastRightState && currentMillis - lastRightTime > DEBOUNCE_DELAY) {
      lastRightTime = currentMillis;
      if (currentRight == LOW) bleMouse.press(MOUSE_RIGHT);
      else bleMouse.release(MOUSE_RIGHT);
      lastRightState = currentRight;
    }
  }

  delay(10);
}
