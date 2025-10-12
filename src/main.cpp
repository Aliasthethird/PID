// Project Name: PID
// Description: Sets depth of propeller based on a MS5837 pressure sensor
// Board/Target: Lolin lite
// Author: Gero Nootz
// Date: 10-11-2025
// Version: 1.1.1

#include <Arduino.h>

#include <cstring> // strchr, strncpy
#include <cstdlib> // atof
#include <utility> // std::pair

#include <Wire.h>
#include <MS5837.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#include "SerialLineReader.h"
#include "PID.h"

#define SCREEN_WIDTH 128    // display width, in pixels
#define SCREEN_HEIGHT 64    // display height, in pixels
#define DISPLAY_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See data sheet for Address of the display
// #define ADC_PIN A10         // e.g., A15 is connected to GPIO 12

#define SIGNIFICANT_DIGITS 5 // Set significant digits output for serial port
const unsigned long DT = 50; // Set PID update time in ms

// adjust as needed for motor control
uint16_t pwmPeriod_Hertz = 400;
int pwmTimeMin_us = 1000;
int pwmTimeMax_us = 2000;

Servo motor1;
int motor1Pin = 12;

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, DISPLAY_RESET);

SerialLineReader serialLineReader;

float averageDepth(uint16_t n);
float depthCall = 0;
void motorArmingSequence(uint8_t time_s);
void alterSP(uint16_t period_ms);

PID pid(DT);

float dSP = 0;
bool alternateSP = false;
bool useSerialkeyValuePair(const KeyVal &kv);

MS5837 pSensor;

void setup()
{
  Serial.begin(115200);
  Serial.println();

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motor1.setPeriodHertz(pwmPeriod_Hertz);
  motor1.attach(motor1Pin, pwmTimeMin_us, pwmTimeMax_us);

  Wire.begin(SDA, SCL);

  // Initialize the OLED display
  if (!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ;
  }

  while (!pSensor.init())
  {
    Serial.println("Could not find MS5837 sensor!");
    delay(5000);
  }
  Serial.println("MS5837 sensor init OK!");
  pSensor.setModel(MS5837::MS5837_02BA);
  pSensor.setFluidDensity(1000); // H2O = 1000 kg/m^3, Air = 1.2 kg/m^3

  depthCall = averageDepth(100);
  Serial.print("Cal Depth = ");
  Serial.println(depthCall);
  delay(1000);

  motorArmingSequence(5);
  Serial.println("Motor armed");
}

void loop()
{
  if (serialLineReader.newLineAvailable())
  {
    auto keyValuePair = parseSerialData(serialLineReader.getLine());
    useSerialkeyValuePair(keyValuePair);
  }

  pSensor.read();
  float depth = pSensor.depth() - depthCall;
  float pidValue = pid.update(depth);

  Serial.print("depth: ");
  if (depth >= 0)
    Serial.print("+");
  Serial.print(depth, SIGNIFICANT_DIGITS);
  Serial.print(", PID: ");
  Serial.print(pidValue, SIGNIFICANT_DIGITS);

  Serial.print(", kp: ");
  Serial.print(pid.getKp(), SIGNIFICANT_DIGITS);
  Serial.print(", ki: ");
  Serial.print(pid.getKi(), SIGNIFICANT_DIGITS);
  Serial.print(", kd: ");
  Serial.print(pid.getKd(), SIGNIFICANT_DIGITS);
  Serial.print(", sp: ");
  Serial.print(pid.getSp(), SIGNIFICANT_DIGITS);
  Serial.print(", LPF: ");
  Serial.println(pid.getLpfGain(), SIGNIFICANT_DIGITS);

  uint16_t pwmTimeHigh_us = map((long)(pidValue * 1000), -100000, 100000, pwmTimeMin_us, pwmTimeMax_us);
  motor1.writeMicroseconds(pwmTimeHigh_us);

  oled.clearDisplay();
  oled.setCursor(0, 20);
  oled.print(pid.getSp(), 3);
  oled.display();

  alterSP(10000);

  pid.paceLoop();
}


/**
 * @brief Applies a parsed key–value pair to update PID parameters.
 * @param kv Pair containing key (first) and value (second).
 * @return true if key was recognized and applied, false otherwise.
 */
bool useSerialkeyValuePair(const KeyVal &kv)
{

  if (strcmp(kv.key, "KP") == 0)
  {
    // Serial.println("In KP");
    pid.setKp(kv.value);
    return true;
  }
  if (strcmp(kv.key, "KI") == 0)
  {
    // Serial.println("In KI");
    pid.setKi(kv.value);
    return true;
  }
  if (strcmp(kv.key, "KD") == 0)
  {
    // Serial.println("In KD");
    pid.setKd(kv.value);
    return true;
  }
  if (strcmp(kv.key, "SP") == 0)
  {
    // Serial.println("In SP");
    pid.setTarget(kv.value);
    return true;
  }
  if (strcmp(kv.key, "LPF") == 0)
  {
    // Serial.println("In SP");
    pid.setLpfGain(kv.value);
    return true;
  }
  if (strcmp(kv.key, "ALT") == 0)
  {
    // Serial.println("In ALT");
    alternateSP = (bool)kv.value;
    return true;
  }
  if (strcmp(kv.key, "dSP") == 0)
  {
    // Serial.println("In ALT");
    dSP = kv.value;
    return true;
  }

  return false;
}

void motorArmingSequence(uint8_t time_s)
{
  uint16_t pwmTimeHigh_us = 1500; // Mid throttle to arm the ESC
  motor1.writeMicroseconds(pwmTimeHigh_us);
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  for (int t = time_s; t >= 0; t--)
  {
    oled.clearDisplay();
    oled.setCursor(0, 10);
    oled.println("Arm ESC");
    oled.print(t);
    oled.println(" s");
    oled.display();
    delay(1000);
  }
}

/**
 * @brief Calculates and displays the average depth from n sensor readings.
 *
 * Used for calibrating the zero-depth reference during the initialization process.
 */
float averageDepth(uint16_t n)
{
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  float value = 0;
  for (int16_t i = n; i >= 0; i--)
  {
    pSensor.read();
    value += pSensor.depth();

    oled.clearDisplay();
    oled.setCursor(0, 10);
    oled.println("Cal Depth");
    oled.print("n: ");
    oled.print(i);
    oled.display();
  }
  value = value / n;

  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("Cal Depth = ");
  oled.print(value);
  oled.display();
  return value;
}

/**
 * @brief Alternates the PID setpoint periodically for demonstration or testing.
 *
 * This function toggles the target setpoint every "period" ms by adding or subtracting
 * a predefined delta (`dSP`) from the current setpoint. The direction alternates each cycle.
 *
 * When the global flag `alternateSP` is enabled, the function:
 * - Increases or decreases the setpoint (`SP`) by `dSP`.
 * - Reverses direction after each interval.
 * - Updates every "period" ms using a non-blocking timing approach.
 *
 * @note Useful for observing system response or tuning the PID controller.
 */
void alterSP(uint16_t period_md)
{
  static unsigned long previousTime = 0;
  static int sign = 1;
  long delayTime = millis() - previousTime;
  if (delayTime >= period_md)
  {
    if (alternateSP)
    {
      pid.setTarget(pid.getSp() + sign * dSP);

      sign = -sign;
    }
    previousTime = millis();
  }
}