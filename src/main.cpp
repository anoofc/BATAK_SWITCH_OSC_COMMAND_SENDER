#define DEBUG 0

#define SWITCH_PIN 12
#define LED_PIN 13

#define DEBOUNCE_DELAY 50
#define LONG_PRESS_DELAY 1000

#include <Arduino.h>
#include <OSCMessage.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>


uint32_t lastDebounceTime = 0;
uint32_t lastPressTime = 0;
bool lastState = HIGH;
bool currentState = HIGH;
bool longPressDetected = false;

void readSwitch() {
  bool reading = digitalRead(SWITCH_PIN);
  if (reading != lastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != currentState) {
      currentState = reading;

      if (currentState == LOW) {
        lastPressTime = millis();
        longPressDetected = false;
      } else {
        if (!longPressDetected) {
          if ((millis() - lastPressTime) < LONG_PRESS_DELAY) {
            if (DEBUG){ Serial.println("Short Press Detected");}
            digitalWrite(LED_PIN, HIGH);
          }
        }
      }
    }
  }

  if (currentState == LOW && (millis() - lastPressTime) >= LONG_PRESS_DELAY) {
    if (!longPressDetected) {
      if (DEBUG){ Serial.println("Long Press Detected"); }
      digitalWrite(LED_PIN, LOW);
      longPressDetected = true;
    }
  }
  lastState = reading;
}

void setup() {
  Serial.begin(115200);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
}

void loop(){
  readSwitch();
}