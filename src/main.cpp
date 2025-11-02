#include <Arduino.h>

// Pins
int buttonPin = 27;
const int whiteLedPin = 21;
int whiteLedState = LOW;
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
int longPressTime = 2000;

// Button
int currentButtonState = HIGH;
int lastFlickerableState = HIGH;
int lastSteadyState = HIGH;
unsigned long lastDebounceTime = 0;
const int deBounceDelay = 50;

// White LED
int brightnessLevel = 0;
int longPressTriggered = false;

// Red LED blink
const unsigned long redBlinkCycle = 1000;
unsigned long redOnDuration = 1000;

// Battery
float batteryVoltage = 3.9;
const int voltageReadInterval = 1000;
const float voltageMax = 4.2;
const float voltageMin = 3.0;

void changeBrightness() {
  brightnessLevel = (brightnessLevel + 1) % 5;  // loop 0→5
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
}

void resetBrightness() {
  brightnessLevel = 0;
  analogWrite(whiteLedPin, 0);
}

void showInfo() {
    Serial.print(batteryVoltage);
    Serial.print("V, ");
    Serial.print(redOnDuration);
    Serial.print("ms. Level: ");
    Serial.print(brightnessLevel);
    Serial.println("/5");
}

void setup() {
  pinMode(whiteLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(batteryPin, INPUT);

  Serial.begin(115200);
  Serial.println("Start");

  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  unsigned long currentMillis = millis();
  
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
      longPressTriggered = false;
      Serial.println("The button is pressed.");
    }
    // While button is held down
    if (currentButtonState == LOW && !longPressTriggered) {
      unsigned long holdDuration = millis() - pressedTime;
      if (holdDuration > longPressTime) {
        Serial.println("Long press detected (during hold)!");
        resetBrightness();
        longPressTriggered = true; // ensure it only triggers once
      }
    } else if (lastSteadyState == LOW && currentButtonState == HIGH) {
      releasedTime = millis();
      Serial.println("The button is released.");
      pressDuration = releasedTime - pressedTime;
      if (pressDuration < shortPressTime && !longPressTriggered) {
        Serial.println("Short press detected!");
        changeBrightness();
      }
    }
    lastSteadyState = currentButtonState;
  }
}