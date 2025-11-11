#include <Arduino.h>
#include "WiFi.h"
// Receiver One MAC 5C:01:3B:96:95:C0
// Pins
int buttonPin = 27;
const int whiteLedPin = 21;
const int mainLedPin = 22;
int whiteLedState = LOW;
int mainLedState = LOW;
const int redLedPin = 19;
int redLedState = LOW;
const int batteryPin = 34;

// Timing
unsigned long previousRedMillis = 0;
unsigned long previousVoltageMillis = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long pressDuration = 0;
int shortPressTime = 500;
int longPressTime = 1000; // 1 second for turning on
int turnOffPressTime = 2000; // 2 seconds for turning off

// Button
int currentButtonState = HIGH;
int lastFlickerableState = HIGH;
int lastSteadyState = HIGH;
unsigned long lastDebounceTime = 0;
const int deBounceDelay = 50;

// White LED
int brightnessLevel = 0;
bool oneSecondTriggered = false;
bool twoSecondTriggered = false;

// Red LED blink
const unsigned long redBlinkCycle = 1000;
unsigned long redOnDuration = 1000;

// Battery
float batteryVoltage = 3.9;
const int voltageReadInterval = 1000;
const float voltageMax = 4.16; //4.16 max observed
const float voltageMin = 3.5; //3.5 cutoff

// Blinking variables
unsigned long previousBlinkMillis = 0;
bool blinkState = HIGH;
unsigned long highDuration = 20;
unsigned long lowDuration = 40;
bool blinkingMode = false;

void changeBrightness() {
  brightnessLevel = (brightnessLevel + 1) % 11;  // Changed to loop 0→10 for cases 5-10
  
  // Turn off blinking mode when not in cases 5-10
  if (brightnessLevel < 5 || brightnessLevel > 10) {
    blinkingMode = false;
    analogWrite(whiteLedPin, 0);
    analogWrite(mainLedPin, 0);
  } else {
    blinkingMode = true;
    // Set blink durations based on case
    switch(brightnessLevel) {
      case 5: 
        highDuration = 20;
        lowDuration = 40;
        break;
      case 6:
        highDuration = 40;
        lowDuration = 80;
        break;
      case 7:
        highDuration = 80;
        lowDuration = 160;
        break;
      case 8:
        highDuration = 160;
        lowDuration = 320;
        break;
      case 9:
        highDuration = 320;
        lowDuration = 640;
        break;
      case 10:
        highDuration = 640;
        lowDuration = 1280;
        break;
    }
    blinkState = HIGH;
    previousBlinkMillis = millis();
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
  if (!blinkingMode) return;
  
  unsigned long currentMillis = millis();
  unsigned long currentDuration = blinkState ? highDuration : lowDuration;
  
  if (currentMillis - previousBlinkMillis >= currentDuration) {
    previousBlinkMillis = currentMillis;
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

void turnOff() {
  brightnessLevel = 0;
  blinkingMode = false;
  analogWrite(whiteLedPin, 0);
  analogWrite(mainLedPin, 0);
  Serial.println("Turned off by 2-second hold");
}

void showInfo() {
    Serial.print(batteryVoltage);
    Serial.print("V, ");
    Serial.print(redOnDuration);
    Serial.print("ms. Level: ");
    Serial.print(brightnessLevel);
    Serial.print("/10");
    if (blinkingMode) {
      Serial.print(" (Blinking: ");
      Serial.print(highDuration);
      Serial.print("ms high, ");
      Serial.print(lowDuration);
      Serial.print("ms low)");
    }
    Serial.println();
}

void setup() {
  pinMode(whiteLedPin, OUTPUT);
  pinMode(mainLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(batteryPin, INPUT);

  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA); // Set Wi-Fi mode to Station
  Serial.print("ESP MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle blinking if active
  handleBlinking();
  
  // Battery read
  if (currentMillis - previousVoltageMillis >= voltageReadInterval) {
    showInfo();
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
      Serial.println("The button is pressed.");
    }
    // While button is held down
    if (currentButtonState == LOW) {
      unsigned long holdDuration = millis() - pressedTime;
      
      // 1-second press detection (turn on to case 5)
      if (holdDuration > longPressTime && !oneSecondTriggered && !twoSecondTriggered) {
        Serial.println("1 second press detected - turning on to case 5!");
        // Turn on to case 5 (first blinking pattern)
        brightnessLevel = 4; // Set to 4 so next press goes to 5
        changeBrightness(); // This will activate case 5 blinking
        oneSecondTriggered = true;
      }
      
      // 2-second press detection (turn off)
      if (holdDuration > turnOffPressTime && !twoSecondTriggered) {
        turnOff();
        twoSecondTriggered = true;
      }
    } else if (lastSteadyState == LOW && currentButtonState == HIGH) {
      releasedTime = millis();
      Serial.println("The button is released.");
      pressDuration = releasedTime - pressedTime;
      
      // Short press detection (only if no long press was triggered)
      if (pressDuration < shortPressTime && !oneSecondTriggered && !twoSecondTriggered) {
        Serial.println("Short press detected!");
        changeBrightness();
      }
    }
    lastSteadyState = currentButtonState;
  }
}