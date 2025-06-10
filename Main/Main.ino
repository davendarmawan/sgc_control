/*
 * Board: Arduino Mega2560
 * Description: Fully automatic environmental controller for a growth chamber.
 * This code manages temperature, humidity, CO2 levels, and a multi-channel LED system.
 * It communicates with a sensor/actuator node and a gateway MiniPC via serial.
 *
 * NOTE: The LED control logic has been updated to a simple, incremental adjustment
 * based on the error between the lux setpoint and the sensor reading.
 */

#include <ArduinoJson.h>
#include <NeoSWSerial.h>

// Define LED Pins
#define PAR_PIN 46  // Timer5 Channel A (OC5A)
#define UV_PIN 44   // Timer5 Channel C (OC5C)
#define IR_PIN 12   // Timer1 Channel B (OC1B)
#define RED_PIN 8   // Timer4 Channel C (OC4C)
#define BLUE_PIN 6  // Timer4 Channel A (OC4A)

// Define Actuator/Sensor Pins
#define PUMP 38
#define VALVE 36
#define CO2_TANK 34
#define HUM 30      // Humidifier power
#define HEATER 19   // PTC Heater
#define SV 21       // Solenoid Valve for Humidifier
#define COMP 23     // Compressor for cooling/dehumidifying

// Constants and Thresholds
#define SENDI 2000          // Interval to send data to gateway (ms)
#define LED_MAX 1599        // Max PWM value for 10kHz with 16MHz clock and no prescaler (ICRx + 1 = 1600)
#define HUMI_TOLERATE 10    // Humidity tolerance band (%)
#define TEMP_TOLERATE 1     // Temperature tolerance band (°C)
#define TIMEOUT 7000        // Timeout for switching serial listen (ms)
#define TUNGGU 300000       // Wait time for temp/hum state machine (5 minutes)

// Software Serial Ports
NeoSWSerial mySerial(50, 48);   // To MiniPC (Gateway)
NeoSWSerial mySerial1(53, 52);  // To Node

// Actuator status (reflects actual hardware state)
bool heater_status = 0;
bool compressor_status = 0;
bool humidifier_status = 0;

// Sensor values from Node
float temp_sensor = 27;
float hum_sensor = 80;
float led_sensor;   // Light sensor (lux)
float door_sensor = 0;
float co2_sensor;

// Manual mode LED percentage variables (Mode 0)
float led_PAR_percent = 0;
float led_UV_percent = 0;
float led_IR_percent = 0;
float led_Blue_percent = 0;
float led_Red_percent = 0;

// Setpoint values from Gateway
float temp_setpoint = 27;
float hum_setpoint = 80;
float co2_setpoint = 500;
int light_mode = 0;       // 0:Manual, 1:PAR, 2:PAR+Red, 3:PAR+Blue, 4:PAR+R+B+UV+IR, 5:Off
float target_lux = 250;   // Target lux for control in modes 1-4
int master_pwm = 0;       // Controller output PWM for active LEDs (0-LED_MAX)
float co2_tolerance = 50;

// Buffers for storing received serial data
String dataMPC;
String dataNode;

// State machine enums
enum state { Idle, Lembabkan, Keringkan, Panaskan, Dinginkan, Tunggu };
enum state CompState = Idle;
enum state HumiState = Idle;

// Flags and Timers
volatile bool stringNodeComplete = false;
volatile bool stringRPIComplete = false;
bool listenNode = true;
bool listenRPI = false;
bool updateKontrol = false;   // Flag to trigger automatic control updates

unsigned long prevMilSend = 0;
unsigned long prevMilTO = 0;
unsigned long prevMilTunggu = 0;

// Forward declarations of all functions
void sendDataToGateway();
void readDataFromGateway();
void readDataFromNode();
void printValue();
void compON();
void compOFF();
void humON();
void humOFF();
void ptcON();
void ptcOFF();
void ValveON();
void ValveOFF();
void PumpON();
void PumpOFF();
void co2TankON();
void co2TankOFF();
void applyLedModeAndIntensity();

// Interrupt service routine for receiving a character from the Gateway (RPI/MiniPC)
static void handleRxRPIChar(uint8_t c) {
  dataMPC += (char)c;
  if (c == '\n') {
    stringRPIComplete = true;
  }
}

// Interrupt service routine for receiving a character from the Node
static void handleRxNodeChar(uint8_t c) {
  dataNode += (char)c;
  if (c == '\n') {
    stringNodeComplete = true;
  }
}

void setup() {
  Serial.begin(9600);
  mySerial.attachInterrupt(handleRxRPIChar);
  mySerial1.attachInterrupt(handleRxNodeChar);

  mySerial.begin(9600);
  mySerial1.begin(9600);

  // Initialize all actuator pins as OUTPUT
  pinMode(HUM, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(SV, OUTPUT);
  pinMode(COMP, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(VALVE, OUTPUT);
  pinMode(CO2_TANK, OUTPUT);

  // Set initial actuator states to OFF
  digitalWrite(HEATER, LOW);
  digitalWrite(SV, LOW);
  digitalWrite(COMP, LOW);
  digitalWrite(HUM, LOW);
  PumpOFF();
  ValveOFF();
  co2TankOFF();

  // Initialize LED pins as OUTPUT
  pinMode(PAR_PIN, OUTPUT);
  pinMode(UV_PIN, OUTPUT);
  pinMode(IR_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // --- Configure Timers for 10kHz Fast PWM ---
  // Timer1 for IR_PIN (Pin 12 - OC1B)
  TCCR1A = (1 << COM1B1) | (1 << WGM11);               // Non-inverting mode for OC1B, Fast PWM mode 14 (ICR1 top)
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);   // Fast PWM mode 14, No prescaler
  ICR1 = LED_MAX;                                      // Sets frequency: 16MHz / (1 * (1599 + 1)) = 10kHz
  OCR1B = 0;                                           // Initial duty cycle for IR LED

  // Timer4 for RED_PIN (Pin 8 - OC4C) and BLUE_PIN (Pin 6 - OC4A)
  TCCR4A = (1 << COM4A1) | (1 << COM4C1) | (1 << WGM41); // Non-inverting for OC4A/OC4C, Fast PWM mode 14
  TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);    // Fast PWM mode 14, No prescaler
  ICR4 = LED_MAX;
  OCR4A = 0; // Blue
  OCR4C = 0; // Red

  // Timer5 for PAR_PIN (Pin 46 - OC5A) and UV_PIN (Pin 44 - OC5C)
  TCCR5A = (1 << COM5A1) | (1 << COM5C1) | (1 << WGM51); // Non-inverting for OC5A/OC5C, Fast PWM mode 14
  TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS50);    // Fast PWM mode 14, No prescaler
  ICR5 = LED_MAX;
  OCR5A = 0; // PAR
  OCR5C = 0; // UV

  Serial.println("Setup complete. System in Automatic Mode. Listening...");
  applyLedModeAndIntensity(); // Apply initial LED state (likely OFF)
  mySerial1.listen();         // Start by listening to the Node
  listenNode = true;
  listenRPI = false;
}

void loop() {
  unsigned long currentMillis = millis();

  // Periodically send sensor and actuator data to the Gateway
  if (currentMillis - prevMilSend >= SENDI) {
    prevMilSend = currentMillis;
    sendDataToGateway();
  }

  // Timeout logic to prevent getting stuck waiting for one serial device
  if (currentMillis - prevMilTO >= TIMEOUT) {
    prevMilTO = currentMillis;
    if (listenNode) {
      Serial.println("Timeout: Switching from Node to Gateway listen");
      dataNode = "";
      mySerial.listen();
      listenNode = false;
      listenRPI = true;
    } else if (listenRPI) {
      Serial.println("Timeout: Switching from Gateway to Node listen");
      dataMPC = "";
      mySerial1.listen();
      listenNode = true;
      listenRPI = false;
    }
  }

  // Process data received from the Sensor Node
  if (stringNodeComplete) {
    stringNodeComplete = false;
    Serial.println("Data received from Node:");
    Serial.println(dataNode);
    readDataFromNode();      // Updates sensor values
    dataNode = "";           // Clear buffer
    mySerial.listen();       // Switch to listen to Gateway
    listenNode = false;
    listenRPI = true;
    prevMilTO = currentMillis; // Reset timeout
    updateKontrol = true;      // Flag to update controls based on new sensor data
  }

  // Process data received from the Gateway (MiniPC)
  if (stringRPIComplete) {
    stringRPIComplete = false;
    Serial.println("Data received from Gateway:");
    Serial.println(dataMPC);
    readDataFromGateway();   // Updates setpoints
    dataMPC = "";            // Clear buffer
    mySerial1.listen();      // Switch to listen to Node
    listenNode = true;
    listenRPI = false;
    prevMilTO = currentMillis; // Reset timeout
    // Immediately apply LED changes from gateway commands
    applyLedModeAndIntensity();
  }

  // If new sensor data arrived, update controlled systems
  if (updateKontrol) {
    if (light_mode >= 1 && light_mode <= 4) { // Only for automated light modes
      applyLedModeAndIntensity();             // Adjust LEDs based on new sensor readings
    }
    updateKontrol = false;
  }

  // --- Humidity Control State Machine ---
  switch (HumiState) {
    case Idle:
      if (hum_sensor > hum_setpoint + HUMI_TOLERATE) {
        HumiState = Keringkan;
      } else if (hum_sensor < hum_setpoint - HUMI_TOLERATE) {
        HumiState = Lembabkan;
      }
      break;
    case Lembabkan: // Humidifying
      humON();
      if (hum_sensor >= hum_setpoint) {
        HumiState = Idle;
        humOFF();
      }
      break;
    case Keringkan: // Dehumidifying
      humOFF(); // Ensure humidifier is off
      // Dehumidification is coupled with the compressor in the Temp FSM
      if (hum_sensor <= hum_setpoint) { HumiState = Idle; }
      break;
    default:;
  }

  // --- Temperature Control State Machine ---
  switch (CompState) {
    case Idle:
      // Turn on cooling if too hot OR if dehumidifying is needed
      if ((temp_sensor >= temp_setpoint + TEMP_TOLERATE) || (HumiState == Keringkan && hum_sensor > hum_setpoint + HUMI_TOLERATE / 2.0)) {
        CompState = Dinginkan;
      } else if (temp_sensor <= temp_setpoint - TEMP_TOLERATE) { // Turn on heating if too cold
        CompState = Panaskan;
      }
      break;
    case Dinginkan:
      compON();
      ptcOFF(); // Ensure heater is off
      CompState = Tunggu;
      prevMilTunggu = millis(); // Start wait timer
      break;
    case Panaskan:
      compOFF(); // Ensure compressor is off
      ptcON();
      CompState = Tunggu;
      prevMilTunggu = millis(); // Start wait timer
      break;
    case Tunggu:
      if ((millis() - prevMilTunggu) >= TUNGGU) { // Check after wait period
        bool temp_in_range = (temp_sensor < temp_setpoint + TEMP_TOLERATE && temp_sensor > temp_setpoint - TEMP_TOLERATE);
        // Turn off compressor if temp is in range AND it's not needed for dehumidification
        if (compressor_status && temp_in_range && !(HumiState == Keringkan && hum_sensor > hum_setpoint + HUMI_TOLERATE / 2.0)) { compOFF(); }
        // Turn off heater if temp is in range
        if (heater_status && temp_in_range) { ptcOFF(); }
        CompState = Idle; // Return to Idle to re-evaluate
      }
      break;
    default:;
  }

  // --- CO2 Control Logic ---
  if ((co2_sensor > co2_setpoint) || co2_sensor >= 1100) { // If CO2 is too high, vent it
    PumpON();
    ValveON();
    co2TankOFF();
  } else if ((co2_sensor < co2_setpoint) && door_sensor == 0) { // If CO2 is low and door is closed, inject it
    PumpOFF();
    ValveOFF();
    co2TankON();
  } else { // Otherwise, do nothing
    PumpOFF();
    ValveOFF();
    co2TankOFF();
  }
}

/**
 * @brief Applies the current LED mode and intensity.
 * For modes 1-4, it uses an incremental adjustment controller to meet target_lux.
 * For mode 0 (Manual), it does nothing, as OCRs are set directly in readDataFromGateway.
 * For mode 5, it turns all LEDs off.
 */
void applyLedModeAndIntensity() {
    if (light_mode == 0) {  // Manual Mode
        // In Mode 0, OCR values are set directly in readDataFromGateway().
        // This function doesn't need to do anything for Mode 0.
        return;
    }

    int previous_pwm = master_pwm; // For logging changes
    int pwm_adjustment = 0; // The value to add to master_pwm

    // For automated modes (1-4) and OFF mode (5)
    if (light_mode >= 1 && light_mode <= 4) { // Automated Control Modes
        
        // --- MODIFIED INCREMENTAL CONTROL LOGIC ---
        float err = target_lux - led_sensor;
        float tolerance = target_lux * 0.02; // Calculate 2% tolerance

        // If error is within the 2% tolerance band, do nothing.
        if (abs(err) <= tolerance) {
            pwm_adjustment = 0;
        } 
        // If error is large, make a big adjustment.
        else if (abs(err) > 500) {
            if (err > 0) {
                pwm_adjustment = 10;
            } else {
                pwm_adjustment = -10;
            }
        } 
        // If error is smaller but outside the tolerance, make a small adjustment.
        else {
            if (err > 0) {
                pwm_adjustment = 1;
            } else { // err < 0
                pwm_adjustment = -1;
            }
        }
        // --- END OF MODIFIED LOGIC ---
        master_pwm += pwm_adjustment;

        // Log if there was a change in master_pwm
        if (pwm_adjustment != 0 || master_pwm != previous_pwm) {
            Serial.print("Light Control: Mode=");
            Serial.print(light_mode);
            Serial.print(", Target=");
            Serial.print(target_lux);
            Serial.print(", Current=");
            Serial.print(led_sensor);
            Serial.print(", Error=");
            Serial.print(err);
            Serial.print(", PWM_Adj=");
            Serial.print(pwm_adjustment);
            Serial.print(" -> NewPWM=");
            Serial.println(master_pwm);
        }

        // --- Apply master_pwm to relevant LEDs for the current mode ---
        // First, reset all, then turn on the selected ones.
        OCR5A = 0; OCR5C = 0; OCR1B = 0; OCR4C = 0; OCR4A = 0;

        switch (light_mode) {
            case 1: // PAR only
                OCR5A = master_pwm;
                break;
            case 2: // PAR + Red
                OCR5A = master_pwm;
                OCR4C = master_pwm;
                break;
            case 3: // PAR + Blue
                OCR5A = master_pwm;
                OCR4A = master_pwm;
                break;
            case 4: // PAR + Red + Blue + UV + IR
                OCR5A = master_pwm; // PAR
                OCR4C = master_pwm; // Red
                OCR4A = master_pwm; // Blue
                OCR5C = master_pwm; // UV
                OCR1B = master_pwm; // IR
                break;
        }
    } else if (light_mode == 5) { // All LEDs Off Mode
        if (master_pwm != 0) { // Log only if a change was needed
             Serial.println("Applied Mode 5: Turning All LEDs Off.");
        }
        master_pwm = 0;
        OCR5A = 0; OCR5C = 0; OCR1B = 0; OCR4C = 0; OCR4A = 0;
    } else {
        // Safety case for unknown mode
        if (light_mode != 0) {
            Serial.print("Warning: Unknown light_mode '");
            Serial.print(light_mode);
            Serial.println("'. Turning LEDs off as a precaution.");
        }
        master_pwm = 0;
        OCR5A = 0; OCR5C = 0; OCR1B = 0; OCR4C = 0; OCR4A = 0;
    }
}


/**
 * @brief Serializes sensor data into a JSON object and sends it to the Gateway.
 */
void sendDataToGateway() {
  StaticJsonDocument<256> doc;
  doc["actTemp"] = String(temp_sensor, 2);
  doc["actHum"] = String(hum_sensor, 2);
  doc["actLight"] = String(led_sensor, 2);
  doc["actPPM"] = String(co2_sensor, 0);
  doc["heaterStatus"] = heater_status;
  doc["compStatus"] = compressor_status;
  doc["humStatus"] = humidifier_status;
  doc["doorStatus"] = String(door_sensor, 0);

  String output;
  serializeJson(doc, output);
  mySerial.println(output); // Send to Gateway (MiniPC)
}

/**
 * @brief Reads incoming JSON data from the Gateway to update setpoints and modes.
 */
void readDataFromGateway() {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, dataMPC);

  if (error) {
    Serial.print(F("deserializeJson() from Gateway failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Update setpoints if they exist in the JSON payload
  if (doc.containsKey("SPTemp"))   { temp_setpoint = doc["SPTemp"]; }
  if (doc.containsKey("SPHum"))    { hum_setpoint = doc["SPHum"]; }
  if (doc.containsKey("SPCo2"))    { co2_setpoint = doc["SPCo2"]; }
  if (doc.containsKey("SPLightIntensity")) { target_lux = doc["SPLightIntensity"]; }

  if (doc.containsKey("SPLightMode")) {
    int new_mode = doc["SPLightMode"];
    if (new_mode != light_mode) {
      light_mode = new_mode;
      Serial.print("Light Mode Setpoint Updated: "); Serial.println(light_mode);
      // Reset master_pwm when changing mode to allow recalculation.
      if (light_mode != 0) {
        master_pwm = 0;
        Serial.println("Resetting master_pwm due to mode change.");
      }
    }
  }

  // Handle manual LED control (Mode 0)
  if (light_mode == 0) {
    Serial.println("Gateway: Processing individual LED controls for Manual Mode (0)");
    // Reset all OCRs first to ensure only specified LEDs are on
    OCR5A = 0; OCR5C = 0; OCR1B = 0; OCR4C = 0; OCR4A = 0;

    if (doc.containsKey("SPpar_light")) {
      led_PAR_percent = doc["SPpar_light"];
      OCR5A = map(constrain((long)led_PAR_percent, 0, 100), 0, 100, 42, 255);
    }
    if (doc.containsKey("SPuv_light")) {
      led_UV_percent = doc["SPuv_light"];
      OCR5C = map(constrain((long)led_UV_percent, 0, 100), 0, 100, 42, 255);
    }
    if (doc.containsKey("SPir_light")) {
      led_IR_percent = doc["SPir_light"];
      OCR1B = map(constrain((long)led_IR_percent, 0, 100), 0, 100, 42, 255);
    }
    if (doc.containsKey("SPred_light")) {
      led_Red_percent = doc["SPred_light"];
      OCR4C = map(constrain((long)led_Red_percent, 0, 100), 0, 100, 42, 255);
    }
    if (doc.containsKey("SPblue_light")) {
      led_Blue_percent = doc["SPblue_light"];
      OCR4A = map(constrain((long)led_Blue_percent, 0, 100), 0, 100, 42, 255);
    }
  }
}

/**
 * @brief Reads incoming JSON data from the Node to update sensor values.
 */
void readDataFromNode() {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, dataNode);

  if (error) {
    Serial.print(F("deserializeJson() from Node failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Update sensor values if they exist in the JSON payload
  if (doc.containsKey("actLight")) { led_sensor = doc["actLight"].as<float>() * 2.4; } // Apply calibration
  if (doc.containsKey("actTemp"))  { temp_sensor = doc["actTemp"]; }
  if (doc.containsKey("actHum"))   { hum_sensor = doc["actHum"]; }
  if (doc.containsKey("actDoor"))  { door_sensor = doc["actDoor"]; }
  if (doc.containsKey("actPPM"))   { co2_sensor = doc["actPPM"]; }

  // Safety override: if door is open, turn off CO2 and lights
  if (door_sensor == 1) {
    ValveOFF();
    PumpOFF();
    co2TankOFF();
  }
}

// --- Actuator Control Functions ---
// These functions include status tracking to prevent redundant commands and allow for clear logging.
void compON()  { if (!compressor_status) { digitalWrite(COMP, HIGH); compressor_status = 1; Serial.println(F("Compressor ON")); } }
void compOFF() { if (compressor_status)  { digitalWrite(COMP, LOW);  compressor_status = 0; Serial.println(F("Compressor OFF")); } }
void humON()   { if (!humidifier_status) { digitalWrite(SV, HIGH); digitalWrite(HUM, HIGH); humidifier_status = 1; Serial.println(F("Humidifier ON")); } }
void humOFF()  { if (humidifier_status)  { digitalWrite(SV, LOW);  digitalWrite(HUM, LOW);  humidifier_status = 0; Serial.println(F("Humidifier OFF")); } }
void ptcON()   { if (!heater_status)     { digitalWrite(HEATER, HIGH); heater_status = 1; Serial.println(F("PTC Heater ON")); } }
void ptcOFF()  { if (heater_status)      { digitalWrite(HEATER, LOW);  heater_status = 0; Serial.println(F("PTC Heater OFF")); } }

void ValveON()    { digitalWrite(VALVE, HIGH); }
void ValveOFF()   { digitalWrite(VALVE, LOW); }
void PumpON()     { digitalWrite(PUMP, HIGH); }
void PumpOFF()    { digitalWrite(PUMP, LOW); }
void co2TankON()  { digitalWrite(CO2_TANK, HIGH); }
void co2TankOFF() { digitalWrite(CO2_TANK, LOW); }
