#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "zoroFace.h"   // <-- your converted image array

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Wire.begin(21, 22);  // SDA, SCL for ESP32
  display.begin(0x3C, true);
  display.clearDisplay();

  // Draw your image (x, y, image_name, width, height, color)
  display.drawBitmap(0, 0, zoroFace, 128, 64, SH110X_WHITE);
  display.display();
}

void loop() {
  // Nothing here (static image)
}
