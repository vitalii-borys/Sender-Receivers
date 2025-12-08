#include <Arduino.h>
#include "WiFi.h"
#include "esp_now.h"

// Button
int buttonPin = 27;
const int whiteLedPin = 21;
const int mainLedPin = 22;
const int redLedPin = 19;
const int batteryPin = 34;

//State
int currentButtonState = HIGH;
int lastFlickerableState = HIGH;
int lastSteadyState = HIGH;
int redLedState = LOW;

// Red LED blink
const unsigned long redBlinkCycle = 1000;
unsigned long redOnDuration = 1000;

// Battery
float batteryVoltage = 3.9;
const int voltageReadInterval = 1000;
const float voltageMax = 4.3; //4.25 max observed
const float voltageMin = 3.4; //3.5 cutoff

// Timing
unsigned long currentMillis = 0;
unsigned long previousRedMillis = 0;
unsigned long previousVoltageMillis = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long pressDuration = 0;
int shortPressTime = 500;
int longPressTime = 1000; // 1 second for turning on
int startBlinkingPressTime = 2000; // 2 seconds for turning off
unsigned long lastDebounceTime = 0;
const int deBounceDelay = 50;

// White LED
int brightnessLevel = 0;
bool oneSecondTriggered = false;
bool twoSecondTriggered = false;

// Blinking variables
unsigned long previousBlinkMillis = 0;
bool blinkState = HIGH;
unsigned long highDuration = 20;
unsigned long lowDuration = 40;
bool blinkingMode = false;

void changeBrightness() {
  //brightnessLevel = (brightnessLevel + 1) % 11;
  
  // Turn off blinking mode when not in cases 5-10
  if (brightnessLevel < 5 || brightnessLevel > 10) {
    blinkingMode = false;
    analogWrite(whiteLedPin, 0);
    analogWrite(mainLedPin, 0);
  } else {
    blinkingMode = true;
    // Set blink durations based on case
    switch(brightnessLevel) {
      case 5: highDuration = 20; lowDuration = 80; break;
      case 6: highDuration = 40; lowDuration = 160; break;
      case 7: highDuration = 80; lowDuration = 320; break;
      case 8: highDuration = 160; lowDuration = 640; break;
      case 9: highDuration = 320; lowDuration = 1280; break;
      case 10: highDuration = 640; lowDuration = 2560; break;
    }
    blinkState = HIGH;
    previousBlinkMillis = millis();
    //previousBlinkMillis = millis() - highDuration;
  }
  
  // Regular brightness levels for cases 0-4
  if (!blinkingMode) {
    int pwmValue;
    switch(brightnessLevel) {
      case 0: pwmValue = 0; break;
      case 1: pwmValue = 2; break;
      case 2: pwmValue = 25; break;
      case 3: pwmValue = 128; break;
      case 4: pwmValue = 255; break;
      default: pwmValue = 0;
    }
    analogWrite(whiteLedPin, pwmValue);
    analogWrite(mainLedPin, pwmValue);
  }
}

void handleBlinking() {
  currentMillis = millis();

  // Battery read
  if (currentMillis - previousVoltageMillis >= voltageReadInterval) {
    previousVoltageMillis = currentMillis;
    int rawValue = analogRead(batteryPin);
    batteryVoltage = rawValue * 0.001729;
    redOnDuration = map(batteryVoltage * 100, voltageMin * 100, voltageMax * 100, 0, 1000);
  }

  // Red LED blinking
  unsigned long cyclePosition = currentMillis % redBlinkCycle;
  if (cyclePosition < redOnDuration) {
    // ON portion of the cycle
    if (!redLedState) {
      redLedState = HIGH;
      digitalWrite(redLedPin, redLedState);
    }
  } else {
    // OFF portion of the cycle
    if (redLedState) {
      redLedState = LOW;
      digitalWrite(redLedPin, redLedState);
    }
  }

  if (!blinkingMode) return;
  
  //currentMillis = millis();
  unsigned long currentDuration = blinkState ? highDuration : lowDuration;
  
  if (currentMillis - previousBlinkMillis >= currentDuration) {
    /* previousBlinkMillis = currentMillis; */
    // --- CRITICAL FIX: Time-Anchored Logic ---
    previousBlinkMillis += currentDuration; 
    // -----------------------------------------
    blinkState = !blinkState;
    
    if (blinkState) {
      analogWrite(whiteLedPin, 255);
      analogWrite(mainLedPin, 255);
    } else {
      analogWrite(whiteLedPin, 0);
      analogWrite(mainLedPin, 0);
    }
  }
}

struct DataPacket {
  int brightnessLevel;
};

void onDataReceive(const uint8_t *mac, const uint8_t *data, int len) {
  DataPacket packet;
  if (len != sizeof(packet)) {
    return;
  }
  memcpy(&packet, data, sizeof(packet));
  brightnessLevel = packet.brightnessLevel;
  changeBrightness();
  Serial.print("Brightness: ");
  Serial.println(packet.brightnessLevel);
}

void setup() {
  pinMode(whiteLedPin, OUTPUT);
  pinMode(mainLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(batteryPin, INPUT);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  String mac = WiFi.macAddress();
  Serial.println("My MAC Address: " + mac);

  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  esp_now_register_recv_cb(onDataReceive);
  Serial.println("Receiver ready");
}

void loop() {

  // Handle blinking if active
  handleBlinking();
  
  // Button behavior with debouncing
  currentButtonState = digitalRead(buttonPin);
  if (currentButtonState != lastFlickerableState) {
    lastDebounceTime = millis();
    lastFlickerableState = currentButtonState;
  }

  if ((millis() - lastDebounceTime) > deBounceDelay) {
    if (lastSteadyState == HIGH && currentButtonState == LOW) {
      pressedTime = millis();
      oneSecondTriggered = false;
      twoSecondTriggered = false;
    }
    // While button is held down
    if (currentButtonState == LOW) {
      unsigned long holdDuration = millis() - pressedTime;
      // 1-second press detection (turn on to case 5)
      if (holdDuration > longPressTime && !oneSecondTriggered && !twoSecondTriggered) {
        //Serial.println("1 second press detected - turning on to case 5!");
        brightnessLevel = 0;
        changeBrightness();
        oneSecondTriggered = true;
      }
      // 2-second press detection (turn off)
      if (holdDuration > startBlinkingPressTime && !twoSecondTriggered) {
        brightnessLevel = 5;
        changeBrightness();
        //Serial.println("2 second press detected");
        twoSecondTriggered = true;
      }
    } else if (lastSteadyState == LOW && currentButtonState == HIGH) {
      releasedTime = millis();
      pressDuration = releasedTime - pressedTime;
      //Serial.println("The button is released.");
      // Short press detection (only if no long press was triggered)
      if (pressDuration < shortPressTime && !oneSecondTriggered && !twoSecondTriggered) {
        //Serial.println("Short press detected!");
        brightnessLevel = (brightnessLevel + 1) % 11;
        changeBrightness();
      }
    }
    lastSteadyState = currentButtonState;
  }
}
// Master MAC A0:A3:B3:8A:6F:D0
// Receiver White MAC 5C:01:3B:96:95:C0
// Receiver Green MAC A0:B7:65:2C:23:50
// Receiver Blue MAC A0:B7:65:2D:AA:44
// Receiver Yellow MAC EC:E3:34:B4:96:84
//String mac = WiFi.macAddress();
//Serial.println("My MAC Address: " + mac);
  