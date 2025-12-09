#include <Arduino.h>
#include "WiFi.h"
#include "esp_now.h"

// Button
int buttonPin = 27;
const int whiteLedPin = 21;
const int mainLedPin = 22;
const int redLedPin = 19;

//State
int currentButtonState = HIGH;
int lastFlickerableState = HIGH;
int lastSteadyState = HIGH;
int redLedState = LOW;
const int batteryPin = 34;

// Timing
unsigned long previousRedMillis = 0;
unsigned long previousVoltageMillis = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long pressDuration = 0;
int shortPressTime = 500;
int longPressTime = 1000;
int turnOffPressTime = 2000;
unsigned long lastDebounceTime = 0;
const int deBounceDelay = 50;
unsigned long lastHeartBreatTimestamp = 0;
unsigned long currentMillis = 0;

// White LED & blinking
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

// Timing
unsigned long previousBlinkMillis = 0;
bool blinkState = HIGH;
unsigned long highDuration = 0;
unsigned long lowDuration = 0;
unsigned long timeToSendNext = 0;
int nextReceiverIndex = 0;
bool sendingMode = false;
bool blinkingMode = false;

void changeBrightness() { 
    previousBlinkMillis = millis();
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
    // Reset phase on change so it feels responsive
    blinkState = HIGH;

    // Set immediate state
    analogWrite(whiteLedPin, 255);
    analogWrite(mainLedPin, 255);
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

// ESP-NOW
uint8_t receiverMACs[][6] = {
  {0x5C, 0x01, 0x3B, 0x96, 0x95, 0xC0}, // White
  {0xA0, 0xB7, 0x65, 0x2C, 0x23, 0x50}, // Green
  {0xA0, 0xB7, 0x65, 0x2D, 0xAA, 0x44}, // Blue
  {0xEC, 0xE3, 0x34, 0xB4, 0x96, 0x84} // Yellow
};

struct DataPacket {
  int brightnessLevel;
};

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

  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  //ESP-NOW
  for (int i = 0; i < sizeof(receiverMACs) / sizeof(receiverMACs[0]); i++) {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverMACs[i], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if(esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    } else {
      Serial.print("Peer added successfully: ");
      Serial.println(i);
    }
  }
}

void loop() {
  // Handle blinking if active
  handleBlinking();

  // Send with interval
  if (currentMillis >= timeToSendNext && sendingMode) {
    struct DataPacket packet;
    packet.brightnessLevel = brightnessLevel;
    esp_err_t result = esp_now_send(receiverMACs[nextReceiverIndex], (uint8_t *) &packet, sizeof(packet));
    if (result == ESP_OK) {
      Serial.print(" Brightness: ");
      Serial.print(brightnessLevel);
    }
    if (nextReceiverIndex < (sizeof(receiverMACs) / sizeof(receiverMACs[0])) - 1) {
      if (brightnessLevel == 0) {
        timeToSendNext = millis() + 1;
      } else {
        timeToSendNext = timeToSendNext + highDuration;
      }
      nextReceiverIndex++;
    } else {
      nextReceiverIndex = 0;
      sendingMode = false;
      Serial.println(" ");
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
      //Serial.println("The button is pressed.");
    }
    // While button is held down
    if (currentButtonState == LOW) {
      unsigned long holdDuration = millis() - pressedTime;
      // 1-second press detection (turn on to case 5)
      if (holdDuration > longPressTime && !oneSecondTriggered && !twoSecondTriggered) {
        // Turn on to case 5 (first blinking pattern)
        brightnessLevel = 0;
        changeBrightness();
        oneSecondTriggered = true;
        sendingMode = true;
        timeToSendNext = millis() + highDuration;
        //Serial.println("1 second press detected | case 5 | data sent.");
      }
      // 2-second press detection (turn off)
      if (holdDuration > turnOffPressTime && !twoSecondTriggered) {
        if (brightnessLevel < 5) {
          brightnessLevel = 5;
        }
        changeBrightness();
        twoSecondTriggered = true;
        sendingMode = true;
        timeToSendNext = millis() + highDuration;
        //Serial.println("2 second press detected and data sent.");
      }
    } else if (lastSteadyState == LOW && currentButtonState == HIGH) {
      releasedTime = millis();
      pressDuration = releasedTime - pressedTime;
      //Serial.println("The button is released.");
      // Short press detection (only if no long press was triggered)
      if (pressDuration < shortPressTime && !oneSecondTriggered && !twoSecondTriggered) {
        brightnessLevel = (brightnessLevel + 1) % 11;
        changeBrightness();
        sendingMode = true;
        timeToSendNext = millis() + highDuration;
        //Serial.println("Short press detected and data sent.");
      }
    }
    lastSteadyState = currentButtonState;
  }
}
// Master MAC A0:A3:B3:8A:6F:D0
//String mac = WiFi.macAddress();
//Serial.println("My MAC Address: " + mac);
  