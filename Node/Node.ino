#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define DHTPIN A3
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

#define CO2pin 5
#define Buzzer 9
#define Door 0
#define led 2

SoftwareSerial mySerial(4, 3); // RX, TX

BH1750 lightMeter;

float h = 0, t = 0, lux = 0;
int ppm_CO2 = 0, val = 0;
unsigned long lastSensorTime = 0;
unsigned long lastBeepTime = 0;
bool buzzerState = false;

const unsigned long sensorInterval = 2000;
const unsigned long beepInterval = 50;

unsigned long co2_t0 = 0, co2_t1 = 0, co2_t2 = 0;
int co2_step = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Wire.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  dht.begin();

  pinMode(CO2pin, INPUT);
  pinMode(Door, INPUT_PULLUP);
  pinMode(Buzzer, OUTPUT);
  pinMode(led, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  // === 1. Real-time monitor pintu & buzzer ===
  val = digitalRead(Door);
  if (val == HIGH) {
    digitalWrite(led, HIGH);
    if (currentMillis - lastBeepTime >= beepInterval) {
      lastBeepTime = currentMillis;
      buzzerState = !buzzerState;
      if (buzzerState) tone(Buzzer, 1000);
      else noTone(Buzzer);
    }
  } else {
    digitalWrite(led, LOW);
    noTone(Buzzer);
    buzzerState = false;
  }

  // === 2. Sensor suhu, cahaya ===
  if (currentMillis - lastSensorTime >= sensorInterval) {
    lastSensorTime = currentMillis;

    // DHT & BH1750
    float h_val = dht.readHumidity();
    float t_val = dht.readTemperature();
    if (!isnan(h_val)) h = h_val;
    if (!isnan(t_val)) t = t_val;
    lux = lightMeter.readLightLevel();

    Serial.print("Temp: "); Serial.print(t);
    Serial.print(" C, Humidity: "); Serial.println(h);
    Serial.print("Lux: "); Serial.println(lux);
  }

  // === 3. CO2 sensor (non-blocking) ===
  bacaCO2_nonBlocking();

  // === 4. Kirim data ke mainboard ===
  if (currentMillis - lastSensorTime < 50) { // kirim setelah update sensor
    sendDataToMainboard();
  }
}

// Non-blocking CO2 dengan state machine
void bacaCO2_nonBlocking() {
  switch (co2_step) {
    case 0:
      if (digitalRead(CO2pin) == LOW) co2_step = 1;
      break;
    case 1:
      if (digitalRead(CO2pin) == HIGH) {
        co2_t0 = millis();
        co2_step = 2;
      }
      break;
    case 2:
      if (digitalRead(CO2pin) == LOW) {
        co2_t1 = millis();
        co2_step = 3;
      }
      break;
    case 3:
      if (digitalRead(CO2pin) == HIGH) {
        co2_t2 = millis();
        long tH = co2_t1 - co2_t0;
        long tL = co2_t2 - co2_t1;
        if ((tH + tL - 4L) > 0 && tH > 2L && (tH + tL) > 4L) {
          ppm_CO2 = 5000L * (tH - 2L) / (tH + tL - 4L);
          Serial.print("CO2: "); Serial.print(ppm_CO2); Serial.println(" ppm");
        }
        co2_step = 0; // reset
      }
      break;
  }
}

void sendDataToMainboard() {
  String jsonData = "{";
  jsonData += "\"actTemp\":\"" + String(t, 2) + "\", ";
  jsonData += "\"actHum\":\"" + String(h, 2) + "\", ";
  jsonData += "\"actLight\":\"" + String(lux, 2) + "\", ";
  jsonData += "\"actPPM\":\"" + String(ppm_CO2) + "\", ";
  jsonData += "\"actDoor\":\"" + String(val) + "\"";
  jsonData += "}";

  mySerial.println(jsonData);
  Serial.println("Data sent: " + jsonData);
}
