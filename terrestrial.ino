#define BLYNK_TEMPLATE_ID "TMPL3VkWhxS-N"
#define BLYNK_TEMPLATE_NAME "Project el"

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(D7, D8);  // GPS TX, RX

// Blynk Virtual Pins for Latitude and Longitude
#define LAT_VPIN V9
#define LNG_VPIN V10


// Motor PINs
#define ENA D0
#define IN1 D1
#define IN2 D2
#define IN3 D3
#define IN4 D4
#define ENB D5

// DHT11 PIN and Type
#define DHTPIN D6
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// MQ-2 Sensor Pin
#define MQ2PIN A0  // Analog pin for MQ-2 sensor

// Mode Variables
bool forward = 0;
bool backward = 0;
bool left = 0;
bool right = 0;
int Speed;
bool lawnmowerMode = false; // False means manual, true means lawnmower pattern

// Blynk credentials
char auth[] = "ZpuDfPpF-goluf2HNGseiLVBmlTt2l70";  // Enter your Blynk app auth token
char ssid[] = "DESKTOP-L08GP2K 6952";  // Enter your WIFI name
char pass[] = "]26m1M16";  // Enter your WIFI password

void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);
  gpsSerial.begin(9600);  
  
  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize MQ-2 sensor pin
  pinMode(MQ2PIN, INPUT);
  
  // Initialize Blynk communication
  Blynk.begin(auth, ssid, pass);

  // Initialize DHT sensor
  dht.begin();
}

// Get values from the widgets in the Blynk app
BLYNK_WRITE(V0) {
  forward = param.asInt();
}

BLYNK_WRITE(V1) {
  backward = param.asInt();
}

BLYNK_WRITE(V2) {
  left = param.asInt();
}

BLYNK_WRITE(V3) {
  right = param.asInt();
}

BLYNK_WRITE(V4) {
  Speed = param.asInt();
}

BLYNK_WRITE(V5) {
  lawnmowerMode = param.asInt(); // Toggle between manual and lawnmower mode
}

// Send DHT11 sensor data to Blynk app
void sendSensorData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  int mq2Value = analogRead(MQ2PIN);  // Read MQ-2 sensor data

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Send temperature and humidity to Blynk
  Blynk.virtualWrite(V6, t);  // Temperature to Virtual Pin V6
  Blynk.virtualWrite(V7, h);  // Humidity to Virtual Pin V7

  // Send MQ-2 sensor data to Blynk
  Blynk.virtualWrite(V8, mq2Value);  // Gas concentration to Virtual Pin V8

  // Print data to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" \xC2\xB0" "C, Humidity: ");
  Serial.print(h);
  Serial.print("%, MQ-2 Sensor Value: ");
  Serial.println(mq2Value);
}

// Check widget values and determine action
void smartcar() {
  if (lawnmowerMode) {
   lawnmowerPattern();
  } else {
    // Manual control mode
    if (forward == 1) {
      Forward();
      Serial.println("Forward");
    } else if (backward == 1) {
      Backward();
      Serial.println("Backward");
    } else if (left == 1) {
      Left();
      Serial.println("Left");
    } else if (right == 1) {
      Right();
      Serial.println("Right");
    } else if (forward == 0 && backward == 0 && left == 0 && right == 0) {
      Stop();
    }
  }
}void gps() {
  Blynk.run();  // Run Blynk

  if (gpsSerial.available()) {
    String gpsData = gpsSerial.readStringUntil('\n');
 // Debugging the GPS data

    if (gpsData.startsWith("$GNGGA") || gpsData.startsWith("$GPGGA")) {
      String fields[15];
      int index = 0;

      // Split the GPS data into fields
      while (gpsData.length() > 0) {
        int commaIndex = gpsData.indexOf(',');
        if (commaIndex == -1) {
          fields[index++] = gpsData;
          break;
        } else {
          fields[index++] = gpsData.substring(0, commaIndex);
          gpsData = gpsData.substring(commaIndex + 1);
        }
      }

      // Debug: Print out all the fields to check data
      for (int i = 0; i < index; i++) {
      
      }

      // Latitude and Longitude extraction
      if (fields[2].length() > 0 && fields[4].length() > 0) {
        float lat = fields[2].toFloat();
        float lon = fields[4].toFloat();



        // Convert latitude and longitude to decimal format
        lat = int(lat / 100) + (lat - int(lat / 100) * 100) / 60;
        lon = int(lon / 100) + (lon - int(lon / 100) * 100) / 60;

        // Adjust based on hemisphere (N/S and E/W)
        if (fields[3] == "S") lat = -lat;
        if (fields[5] == "W") lon = -lon;

        // Debug: Print converted latitude and longitude
        Serial.print("Converted Latitude: ");
        Serial.println(lat, 6);  // Print with 6 decimal places
        Serial.print("Converted Longitude: ");
        Serial.println(lon, 6);  // Print with 6 decimal places

        // Format latitude and longitude to 6 decimal places as strings
        String latStr = String(lat, 6);  // Format latitude with 6 decimals
        String lonStr = String(lon, 6);  // Format longitude with 6 decimals

        // Send formatted latitude and longitude to Blynk
        Blynk.virtualWrite(LAT_VPIN, latStr);  // Send latitude to Virtual Pin V1
        Blynk.virtualWrite(LNG_VPIN, lonStr);  // Send longitude to Virtual Pin V2
      } else {
        Serial.println("Invalid GPS data, no valid latitude/longitude");
      }
    }
  }
}

void loop() {
  // Run the Blynk library
  Blynk.run();
  
  // Execute car movement based on the mode
  smartcar();
 gps();
  // Send DHT11 and MQ-2 data every 2 seconds
  static unsigned long lastDHTRead = 0;
  if (millis() - lastDHTRead > 2000) {
    lastDHTRead = millis();
    sendSensorData();
  }
}

// Motor control functions
void Forward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Backward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Left() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Right() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Lawnmower pattern: Move forward and turn after reaching the end
void lawnmowerPattern() {
  static unsigned long lastMoveTime = 0;
  static bool movingForward = true;
  static bool turnLeft = true;

  unsigned long currentMillis = millis();
  
  if (movingForward) {
    Forward();
    if (currentMillis - lastMoveTime > 2000) {
      Stop();
      lastMoveTime = currentMillis;
      movingForward = false;
    }
  } else {
    if (turnLeft) {
      Left();
    } else {
      Right();
    }
    if (currentMillis - lastMoveTime > 1800) {
      Stop();
      lastMoveTime = currentMillis;
      movingForward = true;
      turnLeft = !turnLeft;
    }
  }
}