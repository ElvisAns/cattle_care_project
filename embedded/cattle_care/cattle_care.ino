#include "Arduino.h"
#include "LM35.h"
#include <WiFi.h>  // Include the WiFi library

#include <HTTPClient.h>

#include <WiFiClientSecure.h>
// Define WiFi credentials
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* serverAddress = "https://your_server_endpoint";  // Replace with your server URL

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

const char* rootCACertificate =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIF6TCCA9GgAwIBAgIQBeTcO5Q4qzuFl8umoZhQ4zANBgkqhkiG9w0BAQwFADCB\n"
  "iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl\n"
  "cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV\n"
  "BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTQw\n"
  "OTEyMDAwMDAwWhcNMjQwOTExMjM1OTU5WjBfMQswCQYDVQQGEwJGUjEOMAwGA1UE\n"
  "CBMFUGFyaXMxDjAMBgNVBAcTBVBhcmlzMQ4wDAYDVQQKEwVHYW5kaTEgMB4GA1UE\n"
  "AxMXR2FuZGkgU3RhbmRhcmQgU1NMIENBIDIwggEiMA0GCSqGSIb3DQEBAQUAA4IB\n"
  "DwAwggEKAoIBAQCUBC2meZV0/9UAPPWu2JSxKXzAjwsLibmCg5duNyj1ohrP0pIL\n"
  "m6jTh5RzhBCf3DXLwi2SrCG5yzv8QMHBgyHwv/j2nPqcghDA0I5O5Q1MsJFckLSk\n"
  "QFEW2uSEEi0FXKEfFxkkUap66uEHG4aNAXLy59SDIzme4OFMH2sio7QQZrDtgpbX\n"
  "bmq08j+1QvzdirWrui0dOnWbMdw+naxb00ENbLAb9Tr1eeohovj0M1JLJC0epJmx\n"
  "bUi8uBL+cnB89/sCdfSN3tbawKAyGlLfOGsuRTg/PwSWAP2h9KK71RfWJ3wbWFmV\n"
  "XooS/ZyrgT5SKEhRhWvzkbKGPym1bgNi7tYFAgMBAAGjggF1MIIBcTAfBgNVHSME\n"
  "GDAWgBRTeb9aqitKz1SA4dibwJ3ysgNmyzAdBgNVHQ4EFgQUs5Cn2MmvTs1hPJ98\n"
  "rV1/Qf1pMOowDgYDVR0PAQH/BAQDAgGGMBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYD\n"
  "VR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMCIGA1UdIAQbMBkwDQYLKwYBBAGy\n"
  "MQECAhowCAYGZ4EMAQIBMFAGA1UdHwRJMEcwRaBDoEGGP2h0dHA6Ly9jcmwudXNl\n"
  "cnRydXN0LmNvbS9VU0VSVHJ1c3RSU0FDZXJ0aWZpY2F0aW9uQXV0aG9yaXR5LmNy\n"
  "bDB2BggrBgEFBQcBAQRqMGgwPwYIKwYBBQUHMAKGM2h0dHA6Ly9jcnQudXNlcnRy\n"
  "dXN0LmNvbS9VU0VSVHJ1c3RSU0FBZGRUcnVzdENBLmNydDAlBggrBgEFBQcwAYYZ\n"
  "aHR0cDovL29jc3AudXNlcnRydXN0LmNvbTANBgkqhkiG9w0BAQwFAAOCAgEAWGf9\n"
  "crJq13xhlhl+2UNG0SZ9yFP6ZrBrLafTqlb3OojQO3LJUP33WbKqaPWMcwO7lWUX\n"
  "zi8c3ZgTopHJ7qFAbjyY1lzzsiI8Le4bpOHeICQW8owRc5E69vrOJAKHypPstLbI\n"
  "FhfFcvwnQPYT/pOmnVHvPCvYd1ebjGU6NSU2t7WKY28HJ5OxYI2A25bUeo8tqxyI\n"
  "yW5+1mUfr13KFj8oRtygNeX56eXVlogMT8a3d2dIhCe2H7Bo26y/d7CQuKLJHDJd\n"
  "ArolQ4FCR7vY4Y8MDEZf7kYzawMUgtN+zY+vkNaOJH1AQrRqahfGlZfh8jjNp+20\n"
  "J0CT33KpuMZmYzc4ZCIwojvxuch7yPspOqsactIGEk72gtQjbz7Dk+XYtsDe3CMW\n"
  "1hMwt6CaDixVBgBwAc/qOR2A24j3pSC4W/0xJmmPLQphgzpHphNULB7j7UTKvGof\n"
  "KA5R2d4On3XNDgOVyvnFqSot/kGkoUeuDcL5OWYzSlvhhChZbH2UF3bkRYKtcCD9\n"
  "0m9jqNf6oDP6N8v3smWe2lBvP+Sn845dWDKXcCMu5/3EFZucJ48y7RetWIExKREa\n"
  "m9T8bJUox04FB6b9HbwZ4ui3uRGKLXASUoWNjDNKD/yZkuBjcNqllEdjB+dYxzFf\n"
  "BT02Vf6Dsuimrdfp5gJ0iHRc2jTbkNJtUQoj1iM=\n"
  "-----END CERTIFICATE-----\n";

// Not sure if WiFiClientSecure checks the validity date of the certificate.
// Setting clock just to be sure...
void setClock() {
  configTime(0, 0, "pool.ntp.org");

  Serial.print(F("Waiting for NTP time sync: "));
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2) {
    delay(500);
    Serial.print(F("."));
    yield();
    nowSecs = time(nullptr);
  }

  Serial.println();
  struct tm timeinfo;
  gmtime_r(&nowSecs, &timeinfo);
  Serial.print(F("Current time: "));
  Serial.print(asctime(&timeinfo));
}


// Flag to control sending PulseSensor Signal data to serial
boolean sendPulseSignal = false;


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
  setClock();
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
  const unsigned long interval = 60000;  // 1 minute

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Send data to server
    sendDataToServer(jsonPayload);
  }
}

void sendDataToServer(String payload) {  //serverAddress
  WiFiClientSecure* client = new WiFiClientSecure;
  if (client) {
    client->setCACert(rootCACertificate);
    {
      // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
      HTTPClient https;
      Serial.print("[HTTPS] begin...\n");
      if (https.begin(*client, serverAddress)) {
        Serial.print("[HTTPS] POST...\n");
        https.addHeader("Content-Type", "application/json");
        int httpCode = https.POST(payload);
        // httpCode will be negative on error
        if (httpCode > 0) {
          // HTTP header has been send and Server response header has been handled
          Serial.printf("[HTTPS] GET... code: %d\n", httpCode);

          // file found at server
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = https.getString();
            Serial.println(payload);
          }
        } else {
          Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }
        https.end();
      } else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }
    }  // End extra scoping block
    delete client;
  } else {
    // Unable to connect to the server
    Serial.println("Unable to create client");
  }
}
