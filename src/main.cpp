#include <Arduino.h>
#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include "UltrasonicSensor.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define TRIG_PIN 33
#define ECHO_PIN 32
#define MAX_DIST 700
#define SOUND_SPEED 343
#define TIMEOUT_COEFF 58.0f
#define TEMPERATURE 25
#define LED_BUILTIN 2

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Distance sensor
uint32_t timeOut = uint32_t(MAX_DIST * TIMEOUT_COEFF); // ~ 40600 µs
int previousDistance = -1;
UltrasonicSensor distSensor(TRIG_PIN, ECHO_PIN);

// Timer 250ms
hw_timer_t* timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool tick250ms = false;
volatile int ledOn = false;

// Prototypes
void measureAndDisplay();
bool i2cAddrTest(uint8_t addr);

void IRAM_ATTR onTimerISR() {
  tick250ms = true;
  ledOn = !ledOn;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // I2C + LCD
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!i2cAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Distance sensor");
  lcd.setCursor(0, 1);
  lcd.print("Dist.: ");

  // Distance sensor
  distSensor.setTemperature(TEMPERATURE);

  // Timer
  timer = timerBegin(0, 120, true);
  timerAttachInterrupt(timer, &onTimerISR, true);
  timerAlarmWrite(timer, 250000, true);
  timerAlarmEnable(timer);
}

void loop() {
  if (tick250ms) {
    portENTER_CRITICAL(&timerMux);
    tick250ms = false;
    portEXIT_CRITICAL(&timerMux);

    measureAndDisplay();
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }
}

void measureAndDisplay() {
  int distance = distSensor.distanceInCentimeters();
  if (distance != previousDistance) {
    lcd.setCursor(7, 1);
    lcd.print("         "); // clear characters after "Dist.: "
    lcd.setCursor(7, 1);
    lcd.print(distance);
    lcd.print("cm");
    previousDistance = distance;
  }
}

bool i2cAddrTest(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}