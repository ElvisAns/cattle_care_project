#include "Arduino.h"
#include "LM35.h"

// Define pin assignments for sensors and components
#define ADXL335_PIN_XOUT 12
#define ADXL335_PIN_YOUT 13
#define ADXL335_PIN_ZOUT 14
#define HEARTPULSE_PIN_SIG 25
#define LM35_PIN_VOUT 26
#define ADC_REF_V 3.3

#include "ADXL335.h"

ADXL335 accelerometer;

// Setup hardware timer for ESP32
hw_timer_t* sampleTimer = NULL;
portMUX_TYPE sampleTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Define the use of Arduino interrupts for PulseSensor
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

PulseSensorPlayground pulseSensor;
const int THRESHOLD = 685;

// Initialize LM35 temperature sensor
LM35 lm35(LM35_PIN_VOUT);

// Interrupt service routine for the hardware timer
void IRAM_ATTR onSampleTime() {
  portENTER_CRITICAL_ISR(&sampleTimerMux);
  PulseSensorPlayground::OurThis->onSampleTime();
  portEXIT_CRITICAL_ISR(&sampleTimerMux);
}

// Flag to control sending PulseSensor Signal data to serial
boolean sendPulseSignal = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1500);
  Serial.println("start");

  // Configure PulseSensor
  pulseSensor.analogInput(HEARTPULSE_PIN_SIG);
  pulseSensor.setSerial(Serial);
  pulseSensor.setThreshold(THRESHOLD);

  // Initialize accelerometer
  accelerometer.begin(ADXL335_PIN_XOUT, ADXL335_PIN_YOUT, ADXL335_PIN_ZOUT, ADC_REF_V);

  // Start PulseSensor
  if (!pulseSensor.begin()) {
    while (1) {
      Serial.println("Pulse sensor failed to start");
    }
  }
}

void loop() {
  // Send PulseSensor data to serial if the flag is set
  if (sendPulseSignal) {
    delay(20);
    Serial.println(pulseSensor.getLatestSample());
  }

  // Read BPM (Beats Per Minute) from PulseSensor
  String bpm = String(pulseSensor.getBeatsPerMinute());

  // Read temperature from LM35
  float lm35TempC = lm35.getTempC();

  // Read acceleration values from accelerometer
  int x, y, z;
  accelerometer.getXYZ(&x, &y, &z);

  // Read acceleration components
  float ax, ay, az;
  accelerometer.getAcceleration(&ax, &ay, &az);
}
