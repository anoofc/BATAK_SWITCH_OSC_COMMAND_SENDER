#define DEBUG 1

#define SWITCH_PIN 12
#define LED_PIN 13

#define NUMPIXELS 14
#define PIN 8

#define DEBOUNCE_DELAY 50
#define LONG_PRESS_DELAY 1000

#define RED strip.Color(255, 0, 0)
#define GREEN strip.Color(0, 255, 0)
#define BLUE strip.Color(0, 0, 255)
#define YELLOW strip.Color(255, 255, 0) 
#define CYAN strip.Color(0, 255, 255)
#define MAGENTA strip.Color(255, 0, 255)

#include <Arduino.h>
#include <OSCMessage.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ETHERNET CONFIGURATION
IPAddress ip(192, 168, 1, 98);                        //the Arduino's IP
const unsigned int inPort = 7001;                     //Arduino's Port

IPAddress outIp(192, 168, 1, 100);                    //destination IP
const unsigned int outPort = 7000;                    //destination Port
byte mac[] = {0x90, 0xA2, 0xDA, 0x0A, 0x2B, 0X1E};    //Arduino's MAC

EthernetUDP Udp;

bool lastState            = HIGH;
bool currentState         = HIGH;
bool longPressDetected    = false;

uint8_t col = 1;                                // Column number
uint8_t longPressState       = 4;
uint8_t shortPressState      = 2;

uint32_t color[] = { RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA };

uint32_t lastDebounceTime = 0;
uint32_t lastPressTime    = 0;

void neoPixel(uint32_t color) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, color);
  } strip.show();
}

void oscSend(const char* address, const char* type, uint8_t column) {
  char fullAddress[50];
  snprintf(fullAddress, sizeof(fullAddress), "%s%d/connect", address, column);
  OSCMessage msg(fullAddress);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void readSwitch1() {
  uint32_t currentMillis = millis();
  bool reading = digitalRead(SWITCH_PIN);

  if (reading != lastState) { lastDebounceTime = currentMillis; }
  if ((currentMillis - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != currentState) {
      currentState = reading;

      if (currentState == LOW) {
        lastPressTime = currentMillis;
        longPressDetected = false;
      } else if (!longPressDetected && (currentMillis - lastPressTime) < LONG_PRESS_DELAY) {
        if (DEBUG) { Serial.println("Short Press Detected"); }
        oscSend("/composition/columns/", "i", shortPressState);
        shortPressState ^= 1;                   // Toggle between 2 and 3
        digitalWrite(LED_PIN, HIGH);
        neoPixel(color[shortPressState]);
      }
    }
  }

  if (currentState == LOW && (currentMillis - lastPressTime) >= LONG_PRESS_DELAY && !longPressDetected) {
    if (DEBUG) { Serial.println("Long Press Detected"); }
    oscSend("/composition/columns/", "i", longPressState);
    longPressState ^= 1;                        // Toggle between 4 and 5
    digitalWrite(LED_PIN, LOW);
    neoPixel(color[longPressState]);
    longPressDetected = true;
  }

  lastState = reading;
}

void readSwitch2(){
  bool reading = digitalRead(SWITCH_PIN);
  if (reading == LOW){
    if (millis() - lastDebounceTime < 250){ return; }
    oscSend("/composition/columns/", "i", col);
    neoPixel(color[col-1]);
    col++;
    if (col > 6) col = 1;
    lastDebounceTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  strip.begin();
  strip.setBrightness(100);
  strip.show();
  
  Ethernet.begin(mac, ip);
  Udp.begin(inPort);
}

void loop() {
  // readSwitch1();
  readSwitch2();
}
