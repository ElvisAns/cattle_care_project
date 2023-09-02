#include "Arduino.h"
#include "LM35.h"
#include <WiFi.h> // Include the WiFi library

// Define WiFi credentials
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* serverAddress = "http://your_server_endpoint"; // Replace with your server URL

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

// WiFi client
WiFiClient client;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1500);
  Serial.println("start");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

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

  // Create JSON data
  String jsonPayload = "{";
  jsonPayload += "\"bpm\":" + bpm + ",";
  jsonPayload += "\"temperature\":" + String(lm35TempC) + ",";
  jsonPayload += "\"acceleration_x\":" + String(ax) + ",";
  jsonPayload += "\"acceleration_y\":" + String(ay) + ",";
  jsonPayload += "\"acceleration_z\":" + String(az);
  jsonPayload += "}";

  // Send data to server every 1 minute
  static unsigned long previousMillis = 0;
  const unsigned long interval = 60000; // 1 minute

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Send data to server
    if (sendDataToServer(jsonPayload)) {
      Serial.println("Data sent to server successfully");
    } else {
      Serial.println("Failed to send data to server");
    }
  }
}

bool sendDataToServer(String payload) {
  if (client.connect(serverAddress, 80)) {
    // Make an HTTP POST request
    client.println("POST " + String(serverAddress) + " HTTP/1.1");
    client.println("Host: " + String(serverAddress));
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(payload.length());
    client.println();
    client.println(payload);

    // Wait for the server to respond
    delay(1000);

    // Check HTTP status
    while (client.available()) {
      String line = client.readStringUntil('\r');
      if (line == "\n") {
        break;
      }
    }

    // Check for a successful response (HTTP status code 200)
    if (client.find("HTTP/1.1 200 OK")) {
      client.stop();
      return true;
    } else {
      client.stop();
      return false;
    }
  } else {
    // Unable to connect to the server
    client.stop();
    return false;
  }
}
