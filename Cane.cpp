#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h> 
#include <PulseSensorPlayground.h>

#define TRIG_PIN 13
#define ECHO_PIN 12
#define BUTTON_PIN 14
#define HEART_PIN 34

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // For Neo-M8N
PulseSensorPlayground pulseSensor;

long duration;
float distance;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX for GPS (change as needed)

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  pulseSensor.analogInput(HEART_PIN);
  pulseSensor.begin();
}

void loop() {
  checkUltrasonic();
  checkGyro();
  checkHeartRate();
  checkGPS();
  
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Button pressed! Sending alert...");
    sendEmergencyAlert();
    delay(2000);  // Debounce
  }

  delay(500);
}

void checkUltrasonic() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
}

void checkGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if (abs(a.acceleration.x) > 9 || abs(a.acceleration.y) > 9 || abs(a.acceleration.z) < 2) {
    Serial.println("Fall detected!");
  }
}

void checkHeartRate() {
  int BPM = pulseSensor.getBeatsPerMinute();
  if (pulseSensor.sawStartOfBeat()) {
    Serial.print("Heart rate: "); Serial.print(BPM); Serial.println(" BPM");
  }
}

void checkGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    Serial.print("Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
  }
}

void sendEmergencyAlert() {
  Serial.println("Sending location and heart rate...");  // Replace with real sending logic (e.g., SMS, cloud upload)
  if (gps.location.isValid()) {
    Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print(", Lng: "); Serial.println(gps.location.lng(), 6);
  }
}
