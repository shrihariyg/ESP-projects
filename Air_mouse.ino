#include <Wire.h>
#include <BleMouse.h>

BleMouse bleMouse("M Pro", "eTan", 100);

#define MPU6050_ADDR 0x68
#define LEFT_BUTTON 15
#define RIGHT_BUTTON 13

// Settings
float GYRO_SENSITIVITY = 250.0f;  // ↑ less sensitive
float FILTER_ALPHA = 0.7f;        // smoothing
int DEADZONE = 30;

// Gyro offsets
float gx_offset = 0, gy_offset = 0;
bool calibrated = false;

// Smoothed values
float dx_smooth = 0, dy_smooth = 0;

// Complementary filter angles
float angleX = 0, angleY = 0;

// Time tracking
unsigned long lastTime = 0;

// Double-tap middle click
unsigned long lastShakeTime = 0;
bool middleClickActive = false;

// Button debouncing
bool lastLeftState = HIGH, lastRightState = HIGH;
unsigned long lastLeftTime = 0, lastRightTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms

// ---------------- MPU FUNCTIONS ----------------
void readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission((uint8_t)MPU6050_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, true);

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission((uint8_t)MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
}

void calibrateMPU() {
  long gx_sum = 0, gy_sum = 0;
  int samples = 1000;
  Serial.println("Calibrating... keep the sensor still");

  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    readGyro(gx, gy, gz);
    gx_sum += gx;
    gy_sum += gy;
    delay(3);
  }

  gx_offset = gx_sum / (float)samples;
  gy_offset = gy_sum / (float)samples;
  calibrated = true;

  Serial.printf("Calibration done! gx=%.2f gy=%.2f\n", gx_offset, gy_offset);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Wake MPU6050
  Wire.beginTransmission((uint8_t)MPU6050_ADDR);
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

    // Remove offsets
    float gx_corr = gx - gx_offset;
    float gy_corr = gy - gy_offset;

    // Deadzone for small movements
    if (abs(gx_corr) < DEADZONE) gx_corr = 0;
    if (abs(gy_corr) < DEADZONE) gy_corr = 0;

    // Skip movement if both are zero
    if (gx_corr != 0 || gy_corr != 0) {
      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      lastTime = now;

      // Gyro-based angle
      float gyroXrate = gx_corr / GYRO_SENSITIVITY;
      float gyroYrate = gy_corr / GYRO_SENSITIVITY;

      angleX += gyroXrate * dt;
      angleY += gyroYrate * dt;

      // Accel-based angle
      float accelAngleX = atan2(ay, az) * 180 / PI;
      float accelAngleY = atan2(-ax, az) * 180 / PI;

      // Complementary filter
      angleX = 0.98 * angleX + 0.02 * accelAngleX;
      angleY = 0.98 * angleY + 0.02 * accelAngleY;

      // ----------------
      // Movement mapping
      // ----------------
      // Swapped axes + less sensitivity (/4.0 instead of /3.0)
      dx_smooth = FILTER_ALPHA * dx_smooth + (1 - FILTER_ALPHA) * (-angleX / 4.0);  // now X acts as horizontal
      dy_smooth = FILTER_ALPHA * dy_smooth + (1 - FILTER_ALPHA) * (-angleY / 4.0);   // now Y acts as vertical

      // Move cursor if significant
      if ((int)dx_smooth != 0 || (int)dy_smooth != 0)
        bleMouse.move((int)dx_smooth, (int)dy_smooth);
    }

    // --- Buttons ---
    unsigned long currentMillis = millis();

    // Left Button
    bool currentLeft = digitalRead(LEFT_BUTTON);
    if (currentLeft != lastLeftState && currentMillis - lastLeftTime > DEBOUNCE_DELAY) {
      lastLeftTime = currentMillis;
      if (currentLeft == LOW) bleMouse.press(MOUSE_LEFT);
      else bleMouse.release(MOUSE_LEFT);
      lastLeftState = currentLeft;
    }

    // Right Button
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
