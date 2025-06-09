/*
 * Board Arduino Mega2560
 * Always in Automatic Mode (for temp/humidity, LED mode can be specified)
 */

#include <ArduinoJson.h>
#include <NeoSWSerial.h>

// Define LED Pins
#define PAR_PIN 46  // OC5A
#define UV_PIN 44   // OC5C
#define IR_PIN 12   // OC1B
#define RED_PIN 8   // OC4C
#define BLUE_PIN 6  // OC4A

#define PUMP 38
#define VALVE 36
#define CO2_TANK 34
#define SENDI 2000
#define HUM 30
#define HEATER 19
#define SV 21
#define COMP 23
#define LED_MAX 1599  // Max PWM value for 10kHz with 16MHz clock and no prescaler (ICRx + 1 = 1600)
#define HUMI_TOLERATE 10
#define TEMP_TOLERATE 1
#define TIMEOUT 7000
#define TUNGGU 300000  //menunggu 5 menit 300000

NeoSWSerial mySerial(50, 48);   // To MiniPC (Gateway)
NeoSWSerial mySerial1(53, 52);  // To Node

//actuator status (reflects actual hardware state)
bool heater_status = 0;
bool compressor_status = 0;
bool humidifier_status = 0;

//sensor value
float temp_sensor = 27;
float hum_sensor = 80;
float led_sensor;  // Light sensor (lux) from Node
float door_sensor = 0;
float co2_sensor;

//pwm value variables for individual manual control (Mode 0)
float led_PAR_percent = 0;
float led_UV_percent = 0;
float led_IR_percent = 0;
float led_Blue_percent = 0;
float led_Red_percent = 0;

//setpoint value
float temp_setpoint = 27;
float hum_setpoint = 80;
float co2_setpoint = 500;
int light_mode = 0;      // 0:Manual, 1:PAR, 2:PAR+Red, 3:PAR+Blue, 4:PAR+R+B+UV+IR, 5:Off
float target_lux = 250;  // Target lux for PID control in modes 1-4 (default)
int master_pwm = 0;      // PID output PWM for active LEDs (0-LED_MAX)
float co2_tolerance = 50;

//string for store received payload
String dataMPC;
String dataNode;

//state to control temperature and humidity
enum state { Idle,
             Lembabkan,
             Keringkan,
             Panaskan,
             Dinginkan,
             Tunggu };
enum state CompState = Idle;
enum state HumiState = Idle;

volatile bool stringNodeComplete = false;
volatile bool stringRPIComplete = false;  // RPI here means MiniPC/Gateway
bool listenNode = true;
bool listenRPI = false;
bool updateKontrol = false;  // Flag to trigger automatic control updates

unsigned long prevMilSend = 0;
unsigned long prevMilTO = 0;
unsigned long prevMilTunggu = 0;

// Forward declarations
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

static void handleRxRPIChar(uint8_t c) {
  dataMPC += (char)c;
  if (c == '\n') {
    stringRPIComplete = true;
  }
}

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

  pinMode(HUM, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(SV, OUTPUT);
  pinMode(COMP, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(VALVE, OUTPUT);
  pinMode(CO2_TANK, OUTPUT);

  digitalWrite(HEATER, LOW);
  digitalWrite(SV, LOW);
  digitalWrite(COMP, LOW);
  digitalWrite(HUM, LOW);
  PumpOFF();
  ValveOFF();
  co2TankOFF();

  pinMode(PAR_PIN, OUTPUT);
  pinMode(UV_PIN, OUTPUT);
  pinMode(IR_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Timer1 for IR_PIN (Pin 12 - OC1B)
  TCCR1A = (1 << COM1B1) | (1 << WGM11);              // Non-inverting mode for OC1B, Fast PWM mode 14 (ICR1 top)
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // Fast PWM mode 14, No prescaler
  ICR1 = LED_MAX;                                     // Sets the top value for the timer (determines frequency)
  OCR1B = 0;                                          // Initial duty cycle for IR LED

  // Timer4 for RED_PIN (Pin 8 - OC4C) and BLUE_PIN (Pin 6 - OC4A)
  TCCR4A = (1 << COM4A1) | (1 << COM4C1) | (1 << WGM41);  // Non-inverting for OC4A/OC4C, Fast PWM mode 14
  TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);     // Fast PWM mode 14, No prescaler
  ICR4 = LED_MAX;
  OCR4A = 0;  // Blue
  OCR4C = 0;  // Red

  // Timer5 for PAR_PIN (Pin 46 - OC5A) and UV_PIN (Pin 44 - OC5C)
  TCCR5A = (1 << COM5A1) | (1 << COM5C1) | (1 << WGM51);  // Non-inverting for OC5A/OC5C, Fast PWM mode 14
  TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS50);     // Fast PWM mode 14, No prescaler
  ICR5 = LED_MAX;
  OCR5A = 0;  // PAR
  OCR5C = 0;  // UV

  Serial.println("Setup complete. System in Automatic Mode. Listening...");
  applyLedModeAndIntensity();  // Apply initial LED state (likely OFF or based on defaults)
  mySerial1.listen();          // Start by listening to the Node
  listenNode = true;
  listenRPI = false;
}

void loop() {
  unsigned long currentMillis = millis();

  // Periodically send data to the Gateway
  if (currentMillis - prevMilSend >= SENDI) {
    prevMilSend = currentMillis;
    sendDataToGateway();
  }

  // Timeout for switching serial listen if no data received
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

  // Process data received from Node
  if (stringNodeComplete) {
    stringNodeComplete = false;
    Serial.println("Data received from Node:");
    Serial.println(dataNode);
    readDataFromNode();  // Updates sensor values
    dataNode = "";       // Clear buffer
    mySerial.listen();   // Switch to listen to Gateway
    listenNode = false;
    listenRPI = true;
    prevMilTO = currentMillis;  // Reset timeout
    updateKontrol = true;       // Flag to update controls based on new sensor data
  }

  // Process data received from Gateway (MiniPC)
  if (stringRPIComplete) {
    stringRPIComplete = false;
    Serial.println("Data received from Gateway:");
    Serial.println(dataMPC);
    readDataFromGateway();  // Updates setpoints, potentially light_mode and master_pwm
    dataMPC = "";           // Clear buffer
    mySerial1.listen();     // Switch to listen to Node
    listenNode = true;
    listenRPI = false;
    prevMilTO = currentMillis;  // Reset timeout
    // Apply LED changes immediately if mode or setpoints from gateway changed them
    // This is important because readDataFromGateway might change light_mode or manual LED values.
    applyLedModeAndIntensity();
  }

  // Update controls if flagged (e.g., after Node data for PID modes)
  if (updateKontrol) {
    if (light_mode >= 1 && light_mode <= 4) {  // Only for PID controlled light modes
      applyLedModeAndIntensity();              // Adjust LEDs based on new sensor readings
    }
    updateKontrol = false;
  }

  // Humidity control state machine
  switch (HumiState) {
    case Idle:
      if (hum_sensor > hum_setpoint + HUMI_TOLERATE) {
        HumiState = Keringkan;
      } else if (hum_sensor < hum_setpoint - HUMI_TOLERATE) {
        HumiState = Lembabkan;
      }
      break;
    case Lembabkan:
      humON();
      if (hum_sensor >= hum_setpoint) {
        HumiState = Idle;
        humOFF();
      }
      break;
    case Keringkan:  // Dehumidification may involve compressor
      humOFF();      // Ensure humidifier is off
      // Compressor for dehumidification is handled in Temp FSM if temp is also high
      if (hum_sensor <= hum_setpoint) { HumiState = Idle; }
      break;
    default:;
  }

  // Temperature and Compressor control state machine
  switch (CompState) {
    case Idle:
      // Turn on cooling if too hot OR if dehumidifying and humidity is still a bit high
      if ((temp_sensor >= temp_setpoint + TEMP_TOLERATE) || (HumiState == Keringkan && hum_sensor > hum_setpoint + HUMI_TOLERATE / 2.0)) {
        CompState = Dinginkan;
      } else if (temp_sensor <= temp_setpoint - TEMP_TOLERATE) {  // Turn on heating if too cold
        CompState = Panaskan;
      }
      break;
    case Dinginkan:
      compON();
      ptcOFF();  // Ensure heater is off
      CompState = Tunggu;
      prevMilTunggu = millis();  // Start wait timer
      break;
    case Panaskan:
      compOFF();  // Ensure compressor is off
      ptcON();
      CompState = Tunggu;
      prevMilTunggu = millis();  // Start wait timer
      break;
    case Tunggu:
      if ((millis() - prevMilTunggu) >= TUNGGU) {  // Check after wait period
        bool temp_in_range = (temp_sensor < temp_setpoint + TEMP_TOLERATE && temp_sensor > temp_setpoint - TEMP_TOLERATE);
        // Turn off compressor if temp is in range AND it's not needed for dehumidification
        if (compressor_status && temp_in_range && !(HumiState == Keringkan && hum_sensor > hum_setpoint + HUMI_TOLERATE / 2.0)) { compOFF(); }
        // Turn off heater if temp is in range
        if (heater_status && temp_in_range) { ptcOFF(); }
        CompState = Idle;  // Return to Idle to re-evaluate
      }
      break;
    default:;
  }

  // float co2_lower = co2_setpoint - (0.1 * co2_setpoint);
  // float co2_upper = co2_setpoint + (0.1 * co2_setpoint);

  if ((co2_sensor > co2_setpoint) || co2_sensor >= 1100) {
    PumpON();
    ValveON();
    co2TankOFF();
  } else if ((co2_sensor < co2_setpoint) && door_sensor == 0) {
    PumpOFF();
    ValveOFF();
    co2TankON();
  } else {
    PumpOFF();
    ValveOFF();
    co2TankOFF();
  }
}


void applyLedModeAndIntensity() {
  if (light_mode == 0) {  // Manual Mode
    // In Mode 0, OCR values are set directly in readDataFromGateway() when new SP values arrive.
    // This function doesn't need to do anything for Mode 0, as manual values persist.
    // Serial.println("LED Mode 0: Manual. No changes by applyLedModeAndIntensity.");
    return;
  }

  int previous_master_pwm_for_logging = master_pwm;  // For logging changes

  // For PID modes (1-4) and OFF mode (5)
  if (light_mode >= 1 && light_mode <= 4) {  // PID Controlled Modes
    float error_lux = target_lux - led_sensor;
    int pwm_adjustment = 0;

    // --- P-Controller Logic (More Aggressive) ---
    if (target_lux <= 1) {  // If target is effectively off (e.g. 0 or 1 lux), ensure lights go off
      master_pwm = 0;
      // pwm_adjustment will remain 0 from initialization, master_pwm is set to 0.
    } else if (error_lux > 20) {                  // Significantly too dim
      pwm_adjustment = (error_lux > 100) ? 30 : 10;    // Was 15 : 5
    } else if (error_lux < -20) {                 // Significantly too bright
      pwm_adjustment = (error_lux < -100) ? -20 : -8;  // Was -10 : -3 (made negative adjustments more significant too)
    } else if (error_lux > 5) {                   // Slightly too dim
      pwm_adjustment = 5;                         // Was 3
    } else if (error_lux < -5) {                  // Slightly too bright
      pwm_adjustment = -4;                        // Was -2
    } else {                                      // Within deadband
      pwm_adjustment = 0;
    }

    master_pwm += pwm_adjustment;

    // Constrain master_pwm
    if (master_pwm > LED_MAX) {
      master_pwm = LED_MAX;
    } else if (master_pwm < 0) {
      master_pwm = 0;
    }

    // "Kickstart" if PWM hit zero but shouldn't have
    // If target is on, master_pwm somehow got to 0, and it's still too dim, give it a nudge.
    if (target_lux > 1 && master_pwm == 0 && error_lux > 5) {
      master_pwm = 5;  // Smallest non-zero PWM to allow P-controller to ramp up (was 1, increased for more aggression)
    }

    // Log if there was a change in master_pwm or a significant adjustment attempt
    if (pwm_adjustment != 0 || master_pwm != previous_master_pwm_for_logging) {
      Serial.print("Light PID: Mode=");
      Serial.print(light_mode);
      Serial.print(", Target=");
      Serial.print(target_lux);
      Serial.print(", Current=");
      Serial.print(led_sensor);
      Serial.print(", Error=");
      Serial.print(error_lux);
      Serial.print(", PWM_Adj=");
      Serial.print(pwm_adjustment);
      Serial.print(", OldPWM=");
      Serial.print(previous_master_pwm_for_logging);
      Serial.print(" -> NewPWM=");
      Serial.println(master_pwm);
    }

    // --- Apply master_pwm to relevant LEDs for the current PID mode ---
    // First, ensure all potentially controlled LEDs are off, then turn on the selected ones.
    OCR5A = 0;
    OCR5C = 0;
    OCR1B = 0;
    OCR4C = 0;
    OCR4A = 0;  // Reset all relevant OCRs

    switch (light_mode) {
      case 1:  // PAR only
        OCR5A = master_pwm;
        break;
      case 2:  // PAR + Red
        OCR5A = master_pwm;
        OCR4C = master_pwm;
        break;
      case 3:  // PAR + Blue
        OCR5A = master_pwm;
        OCR4A = master_pwm;
        break;
      case 4:              // PAR + Red + Blue + UV + IR
        OCR5A = master_pwm;  // PAR
        OCR4C = master_pwm;  // Red
        OCR4A = master_pwm;  // Blue
        OCR5C = master_pwm;  // UV
        OCR1B = master_pwm;  // IR
        break;
    }
  } else if (light_mode == 5) {                                                                      // All LEDs Off Mode
    if (master_pwm != 0 || OCR5A != 0 || OCR4C != 0 || OCR4A != 0 || OCR5C != 0 || OCR1B != 0) {  // Log if something was on
      Serial.println("Applied Mode 5: All LEDs Off. MasterPWM and OCRs reset.");
    }
    master_pwm = 0;
    OCR5A = 0;
    OCR5C = 0;
    OCR1B = 0;
    OCR4C = 0;
    OCR4A = 0;  // Ensure all are off
  } else {
    // This case should ideally not be reached if light_mode is always 0-5.
    // If it's an unknown mode, turn everything off as a safety measure.
    // Mode 0 is handled by returning at the start of the function.
    if (light_mode != 0) {  // Avoid printing for mode 0 which is handled.
      Serial.print("Warning: Unknown/Invalid light_mode in applyLedModeAndIntensity: ");
      Serial.println(light_mode);
      Serial.println("All LEDs turned OFF and MasterPWM reset as a precaution.");
    }
    master_pwm = 0;
    OCR5A = 0;
    OCR5C = 0;
    OCR1B = 0;
    OCR4C = 0;
    OCR4A = 0;
  }
}

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
  mySerial.println(output);  // Send to Gateway (MiniPC)
  // Serial.println("Data sent to Gateway: " + output); // Optional: uncomment for debugging
}

void readDataFromGateway() {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, dataMPC);

  if (error) {
    Serial.print(F("deserializeJson() from Gateway failed: "));
    Serial.println(error.f_str());
    return;
  }

  // bool mode_or_lux_target_changed = false; // This variable was not used, removing.

  if (doc.containsKey("SPTemp")) {
    float new_temp_setpoint = doc["SPTemp"];
    if (new_temp_setpoint != temp_setpoint) {
      temp_setpoint = new_temp_setpoint;
      Serial.print("Temp Setpoint Updated: ");
      Serial.println(temp_setpoint);
    }
  }
  if (doc.containsKey("SPHum")) {
    float new_hum_setpoint = doc["SPHum"];
    if (new_hum_setpoint != hum_setpoint) {
      hum_setpoint = new_hum_setpoint;
      Serial.print("Hum Setpoint Updated: ");
      Serial.println(hum_setpoint);
    }
  }
  if (doc.containsKey("SPCo2")) {
    float new_co2_setpoint = doc["SPCo2"];
    if (new_co2_setpoint != co2_setpoint) {
      co2_setpoint = new_co2_setpoint;
      Serial.print("CO2 Setpoint Updated: ");
      Serial.println(co2_setpoint);
    }
  }

  if (doc.containsKey("SPLightMode")) {
    int new_mode = doc["SPLightMode"];
    if (new_mode != light_mode) {
      light_mode = new_mode;
      Serial.print("Light Mode Setpoint Updated: ");
      Serial.println(light_mode);
      // If light mode changes, reset master_pwm.
      // It will be recalculated if PID mode, set to 0 if OFF mode.
      // Manual mode (0) doesn't use master_pwm in the same way, its OCRs are set directly.
      if (light_mode != 0) {  // Don't reset master_pwm if switching TO manual, it's not used then.
        master_pwm = 0;
        Serial.println("Resetting master_pwm due to mode change (not to manual).");
      }
      // mode_or_lux_target_changed = true; // Not strictly needed as applyLedModeAndIntensity is called anyway
    }
  }

  if (doc.containsKey("SPLightIntensity")) {  // This is target_lux
    float new_target_lux = doc["SPLightIntensity"];
    if (new_target_lux != target_lux) {
      target_lux = new_target_lux;
      Serial.print("Target Lux (SPLightIntensity) Updated: ");
      Serial.println(target_lux);
      // mode_or_lux_target_changed = true; // Not strictly needed
    }
  }

  // Manual LED control if light_mode is 0
  // These are applied immediately. applyLedModeAndIntensity() will be called afterwards
  // from the main loop, but it returns early for mode 0.
  if (light_mode == 0) {
    bool manualUpdateOccurred = false;
    Serial.println("Gateway: Processing individual LED controls for Manual Mode (0)");
    // Turn off LEDs not explicitly set in manual mode to avoid them staying on from a previous PID mode
    OCR5A = 0;
    OCR5C = 0;
    OCR1B = 0;
    OCR4C = 0;
    OCR4A = 0;

    if (doc.containsKey("SPpar_light")) {
      led_PAR_percent = doc["SPpar_light"];
      OCR5A = map(constrain((long)led_PAR_percent, 0, 100), 0, 100, 0, LED_MAX);
      Serial.print("MANUAL Mode 0: PAR LED OCR5A: ");
      Serial.println(OCR5A);
      manualUpdateOccurred = true;
    }
    if (doc.containsKey("SPuv_light")) {
      led_UV_percent = doc["SPuv_light"];
      OCR5C = map(constrain((long)led_UV_percent, 0, 100), 0, 100, 0, LED_MAX);
      Serial.print("MANUAL Mode 0: UV LED OCR5C: ");
      Serial.println(OCR5C);
      manualUpdateOccurred = true;
    }
    if (doc.containsKey("SPir_light")) {
      led_IR_percent = doc["SPir_light"];
      OCR1B = map(constrain((long)led_IR_percent, 0, 100), 0, 100, 0, LED_MAX);
      Serial.print("MANUAL Mode 0: IR LED OCR1B: ");
      Serial.println(OCR1B);
      manualUpdateOccurred = true;
    }
    if (doc.containsKey("SPred_light")) {
      led_Red_percent = doc["SPred_light"];
      OCR4C = map(constrain((long)led_Red_percent, 0, 100), 0, 100, 0, LED_MAX);
      Serial.print("MANUAL Mode 0: Red LED OCR4C: ");
      Serial.println(OCR4C);
      manualUpdateOccurred = true;
    }
    if (doc.containsKey("SPblue_light")) {
      led_Blue_percent = doc["SPblue_light"];
      OCR4A = map(constrain((long)led_Blue_percent, 0, 100), 0, 100, 0, LED_MAX);
      Serial.print("MANUAL Mode 0: Blue LED OCR4A: ");
      Serial.println(OCR4A);
      manualUpdateOccurred = true;
    }
    if (manualUpdateOccurred) {
      // printValue(); // Optional: print all values after manual update
    }
  }

  // If mode or target lux changed for PID modes, applyLedModeAndIntensity will be called
  // from the main loop after this function returns, ensuring PID recalculates.
  // No need to set updateKontrol here as applyLedModeAndIntensity is called directly after this
  // in the stringRPIComplete block.
}

void printValue() {
  Serial.println(F("--- Current Values ---"));
  Serial.print(F("Temp Sensor: "));
  Serial.println(temp_sensor);
  Serial.print(F("Hum Sensor: "));
  Serial.println(hum_sensor);
  Serial.print(F("Light Sensor (lux): "));
  Serial.println(led_sensor);
  Serial.print(F("CO2 Sensor: "));
  Serial.println(co2_sensor);
  Serial.print(F("Door Sensor: "));
  Serial.println(door_sensor);

  Serial.println(F("--- Setpoints ---"));
  Serial.print(F("Temp Setpoint: "));
  Serial.println(temp_setpoint);
  Serial.print(F("Hum Setpoint: "));
  Serial.println(hum_setpoint);
  Serial.print(F("CO2 Setpoint: "));
  Serial.println(co2_setpoint);
  Serial.print(F("Light Mode: "));
  Serial.println(light_mode);
  Serial.print(F("Target Lux: "));
  Serial.println(target_lux);

  Serial.println(F("--- Actuator Status ---"));
  Serial.print(F("Heater: "));
  Serial.println(heater_status);
  Serial.print(F("Compressor: "));
  Serial.println(compressor_status);
  Serial.print(F("Humidifier: "));
  Serial.println(humidifier_status);

  Serial.println(F("--- LED Info ---"));
  Serial.print(F("Master PWM (Modes 1-4): "));
  Serial.println(master_pwm);
  Serial.println(F("  --- Manual Mode (0) Target % ---"));
  Serial.print(F("  PAR %: "));
  Serial.println(led_PAR_percent);
  Serial.print(F("  UV %: "));
  Serial.println(led_UV_percent);
  Serial.print(F("  IR %: "));
  Serial.println(led_IR_percent);
  Serial.print(F("  Red %: "));
  Serial.println(led_Red_percent);
  Serial.print(F("  Blue %: "));
  Serial.println(led_Blue_percent);
  Serial.println(F("  --- Actual OCR Values ---"));
  Serial.print(F("  OCR5A (PAR): "));
  Serial.println(OCR5A);
  Serial.print(F("  OCR5C (UV): "));
  Serial.println(OCR5C);
  Serial.print(F("  OCR1B (IR): "));
  Serial.println(OCR1B);
  Serial.print(F("  OCR4C (Red): "));
  Serial.println(OCR4C);
  Serial.print(F("  OCR4A (Blue): "));
  Serial.println(OCR4A);

  Serial.println(F("--- State Machine ---"));
  Serial.print(F("HumiState: "));
  Serial.println(HumiState);
  Serial.print(F("CompState: "));
  Serial.println(CompState);
  Serial.println(F("----------------------"));
}

void readDataFromNode() {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, dataNode);

  if (error) {
    Serial.print(F("deserializeJson() from Node failed: "));
    Serial.println(error.f_str());
    return;
  }

  if (doc.containsKey("actLight")) {
    if (!doc["actLight"].isNull()) {
      float new_led_sensor = doc["actLight"].as<float>() * 2.4;  // Apply calibration/conversion factor
      if (abs(new_led_sensor - led_sensor) > 0.5) {              // Only log if changed significantly
        // Serial.print("Node Light Sensor Updated: "); Serial.println(new_led_sensor);
      }
      led_sensor = new_led_sensor;
    }
  }
  if (doc.containsKey("actTemp")) {
    if (!doc["actTemp"].isNull()) {
      temp_sensor = doc["actTemp"].as<float>();
      // Serial.print("Node Temp Sensor Updated: "); Serial.println(temp_sensor);
    }
  }
  if (doc.containsKey("actHum")) {
    if (!doc["actHum"].isNull()) {
      hum_sensor = doc["actHum"].as<float>();
      // Serial.print("Node Hum Sensor Updated: "); Serial.println(hum_sensor);
    }
  }
  if (doc.containsKey("actDoor")) {
    if (!doc["actDoor"].isNull()) {
      door_sensor = doc["actDoor"].as<float>();
      // Serial.print("Node Door Sensor Updated: "); Serial.println(door_sensor);
    }
  }
  if (doc.containsKey("actPPM")) {
    if (!doc["actPPM"].isNull()) {
      co2_sensor = doc["actPPM"].as<float>();
      // Serial.print("Node CO2 Concentration Updated: "); Serial.println(co2_sensor);
    }
  }
  if (door_sensor == 1) {
    ValveOFF();
    PumpOFF();
    co2TankOFF();
    light_mode = 5; // Turn off all LEDs
    applyLedModeAndIntensity();
  }
}

// Actuator control functions with status tracking
void compON() {
  if (!compressor_status) {
    digitalWrite(COMP, HIGH);
    compressor_status = 1;
    Serial.println(F("Compressor ON"));
  }
}
void compOFF() {
  if (compressor_status) {
    digitalWrite(COMP, LOW);
    compressor_status = 0;
    Serial.println(F("Compressor OFF"));
  }
}
void humON() {
  if (!humidifier_status) {
    digitalWrite(SV, HIGH);   // Solenoid Valve for humidifier
    digitalWrite(HUM, HIGH);  // Humidifier power
    humidifier_status = 1;
    Serial.println(F("Humidifier ON"));
  }
}
void humOFF() {
  if (humidifier_status) {
    digitalWrite(SV, LOW);
    digitalWrite(HUM, LOW);
    humidifier_status = 0;
    Serial.println(F("Humidifier OFF"));
  }
}
void ptcON() {
  if (!heater_status) {
    digitalWrite(HEATER, HIGH);
    heater_status = 1;
    Serial.println(F("PTC Heater ON"));
  }
}
void ptcOFF() {
  if (heater_status) {
    digitalWrite(HEATER, LOW);
    heater_status = 0;
    Serial.println(F("PTC Heater OFF"));
  }
}

void ValveON() {
  digitalWrite(VALVE, HIGH);
}
void ValveOFF() {
  digitalWrite(VALVE, LOW);
}
void PumpON() {
  digitalWrite(PUMP, HIGH);
}
void PumpOFF() {
  digitalWrite(PUMP, LOW);
}
void co2TankON() {
  digitalWrite(CO2_TANK, HIGH);
}
void co2TankOFF() {
  digitalWrite(CO2_TANK, LOW);
}
