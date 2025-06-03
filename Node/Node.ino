/*
 * Board : Arduino Nano Old Bootloader
 */

#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// Pin dan objek sensor
#define DHTPIN A3
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

#define CO2pin 5
#define Buzzer 9
#define Door 0
#define led 2

SoftwareSerial mySerial(4, 3); // RX = 4, TX = 3

BH1750 lightMeter;

float h, h_send;
float t, t_send;
float lux;
int ppm_CO2;
int val = 0;

// Timer untuk sensor
unsigned long lastSensorReadTime = 0;
const unsigned long sensorInterval = 2000; // 2 detik

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  Wire.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 initialized"));
  } else {
    Serial.println(F("Error initializing BH1750"));
  }

  dht.begin();

  pinMode(CO2pin, INPUT);
  pinMode(Door, INPUT_PULLUP);
  pinMode(Buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  h_send = 0;
  t_send = 0;
  lux = 0;
  ppm_CO2 = 0;

  Serial.println("Setup complete.");
}

void loop() {
  // --- 1. Monitor pintu real-time ---
  val = digitalRead(Door);
  if (val == HIGH) {
    Serial.println("Door is open");
    digitalWrite(led, HIGH);
    tone(Buzzer, 1000);
  } else {
    Serial.println("Door is closed");
    digitalWrite(led, LOW);
    noTone(Buzzer);
  }

  // --- 2. Sensor lain setiap 2 detik ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorReadTime >= sensorInterval) {
    lastSensorReadTime = currentMillis;

    // Baca cahaya
    lux = lightMeter.readLightLevel();

    // Baca suhu & kelembaban
    h = dht.readHumidity();
    t = dht.readTemperature();

    // Baca CO2
    ppm_CO2 = gas_concentration_PWM();

    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      t_send = t;
      h_send = h;
      Serial.print("Temp: ");
      Serial.print(t);
      Serial.print(" °C, Humidity: ");
      Serial.println(h);
    }

    Serial.print("Light: ");
    Serial.print(lux);
    Serial.print(" lux, CO2: ");
    Serial.print(ppm_CO2);
    Serial.println(" ppm");

    sendDataToMainboard();
  }
}

int gas_concentration_PWM() {
  unsigned long t0_ms, t1_ms, t2_ms;

  while (digitalRead(CO2pin) == LOW);
  t0_ms = millis();
  while (digitalRead(CO2pin) == HIGH);
  t1_ms = millis();
  while (digitalRead(CO2pin) == LOW);
  t2_ms = millis();

  long tH_ms = t1_ms - t0_ms;
  long tL_ms = t2_ms - t1_ms;

  if ((tH_ms + tL_ms - 4L) == 0) {
    Serial.println("Error: division by zero.");
    return -1;
  }

  if (tH_ms <= 2L) {
    Serial.println("Error: tH too short.");
    return -2;
  }

  if ((tH_ms + tL_ms) <= 4L) {
    Serial.println("Error: total cycle too short.");
    return -3;
  }

  long ppm = 5000L * (tH_ms - 2L) / (tH_ms + tL_ms - 4L);
  return (int)ppm;
}

void sendDataToMainboard() {
  String jsonData = "{";
  jsonData += "\"actTemp\":\"" + String(t_send, 2) + "\", ";
  jsonData += "\"actHum\":\"" + String(h_send, 2) + "\", ";
  jsonData += "\"actLight\":\"" + String(lux, 2) + "\", ";
  jsonData += "\"actPPM\":\"" + String(ppm_CO2) + "\", ";
  jsonData += "\"actDoor\":\"" + String(val) + "\"";
  jsonData += "}";

  mySerial.println(jsonData);
  Serial.println("Data sent: " + jsonData);
}
