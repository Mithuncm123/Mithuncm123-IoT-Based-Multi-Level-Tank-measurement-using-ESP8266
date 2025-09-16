#define BLYNK_TEMPLATE_ID "TMPL3IfCAp2A5"
#define BLYNK_TEMPLATE_NAME "IOT Based multi tank measurement using Load cell"
#define BLYNK_AUTH_TOKEN "llwnOaFEjFwajpb4uf9O6K3sF_jNABYe"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi credentials
char ssid[] = "CMF by Nothing Phone 1_1744";
char pass[] = "567891234";

// HX711 Pins
#define DT1 15
#define SCK1 2
#define DT2 4
#define SCK2 5
#define DT3 18
#define SCK3 19

// OLED Parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// HX711 Objects
HX711 scale1, scale2, scale3;

// Calibration Factors
float calibration_factor1 = -100000.0;
float calibration_factor2 = -100000.0;
float calibration_factor3 = -100000.0;

// Other Pins
#define TURBIDITY_PIN 34
#define TRIG_PIN 25
#define ECHO_PIN 26
#define RELAY_PIN 32
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// Turbidity calibration
const float TURB_SLOPE = 100.0;
const float TURB_INTERCEPT = 0.0;

// L3 Filtering
float L3_history[10] = {0};
int l3_index = 0;

// Ultrasonic function
long readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  return (duration > 0) ? (duration * 0.0343 / 2.0) : -1;
}

void setup() {
  Serial.begin(115200);

  // Blynk connection
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // OLED setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1); // Stop if OLED fails
  }
  display.clearDisplay();
  display.display();

  // HX711 setup
  scale1.begin(DT1, SCK1);
  scale2.begin(DT2, SCK2);
  scale3.begin(DT3, SCK3);
  delay(500);
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale1.tare();
  scale2.tare();
  scale3.tare();

  // Pins
  pinMode(TURBIDITY_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
}

void loop() {
  Blynk.run();

  // --- Load Cell Readings with Noise Filtering ---
  float L1_kg = fabs(scale1.get_units(5));
  float L2_kg = fabs(scale2.get_units(5));
  float rawL3_kg = fabs(scale3.get_units(5));

  if (L1_kg < 0.2) L1_kg = 0.0;
  if (L2_kg < 0.2) L2_kg = 0.0;
  if (rawL3_kg < 0.2) rawL3_kg = 0.0;

  // Filter for L3
  L3_history[l3_index] = rawL3_kg;
  l3_index = (l3_index + 1) % 10;
  float L3_kg = 0;
  for (int i = 0; i < 10; i++) {
    L3_kg += L3_history[i];
  }
  L3_kg /= 10.0;

  // Convert weight to volume (liters)
  float L1_Liters = L1_kg * 1.0;
  float L2_Liters = L2_kg * 1.0;
  float L3_Liters = L3_kg * 1.0;

  // --- Turbidity Sensor ---
  int turbRaw = analogRead(TURBIDITY_PIN);
  float turbNTU = map(turbRaw, 0, 432, 1000, 0);
  turbNTU = constrain(turbNTU, 0, 1000);

  // --- Motor Control Logic based on L3 Liters ---
  static bool motorOn = false;
  if (L3_Liters < 0.4) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    motorOn = false;
  } else if (L3_Liters >= 0.4 && L3_Liters < 1.0) {
    digitalWrite(RELAY_PIN, RELAY_ON);
    motorOn = true;
  } else {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    motorOn = false;
  }

  // --- Ultrasonic ---
  long dist = readUltrasonicCM();
  if (dist <= 0 || dist > 400) dist = -1;
  String mainStatus;
  if (dist == -1) mainStatus = "Err";
  else if (dist >= 17.3) mainStatus = "Empty";
  else if (dist >= 15.0) mainStatus = "Less";
  else if (dist <= 4.0) mainStatus = "Full";
  else mainStatus = String(dist, 1) + " Medium";

  // --- Send to Blynk ---
  Blynk.virtualWrite(V0, L1_Liters);
  Blynk.virtualWrite(V1, L2_Liters);
  Blynk.virtualWrite(V2, L3_Liters);
  Blynk.virtualWrite(V3, turbNTU);
  Blynk.virtualWrite(V4, mainStatus);
  Blynk.virtualWrite(V5, motorOn ? 1 : 0);

  // --- Serial Debug ---
  Serial.print("L1="); Serial.print(L1_Liters); Serial.print("L ");
  Serial.print("L2="); Serial.print(L2_Liters); Serial.print("L ");
  Serial.print("L3="); Serial.print(L3_Liters); Serial.print("L ");
  Serial.print("Motor="); Serial.print(motorOn ? "ON" : "OFF ");
  Serial.print("Dist="); Serial.println(dist);

  // --- OLED Display ---
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0); display.printf("LC1: %.2fkg %.2fL", L1_kg, L1_Liters);
  display.setCursor(0, 10); display.printf("LC2: %.2fkg %.2fL", L2_kg, L2_Liters);
  display.setCursor(0, 20); display.printf("LC3: %.2fkg %.2fL", L3_kg, L3_Liters);
  display.setCursor(0, 30); display.printf("Turb: %.2f NTU", turbNTU);
  display.setCursor(0, 40); display.print("Main: "); display.print(mainStatus);
  display.setCursor(0, 50); display.print("Motor: "); display.print(motorOn ? "ON" : "OFF");
  display.display();

  delay(300);
}
