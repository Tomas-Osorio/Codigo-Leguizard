#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t pin) {
  _pin = pin;
  _currentSpeed = 0; // Initialize the current speed to 0
}

bool MotorDriver::init() {
  Serial.begin(115200);
  Serial.println("Start");

  ledcAttach(_pin, 50, 12);
  
  delay(100);
  ledcWrite(_pin, 410);
  delay(500);
  ledcWrite(_pin, 205);
  delay(500);
  ledcWrite(_pin, 306);
  delay(500);

  return true;
}

void MotorDriver::drive(int value) {
  value = constrain(value, -100, 100);
  int targetPwmValue = map(value, -100, 100, 205, 410);
  int currentPwmValue = map(_currentSpeed, -100, 100, 205, 410);
  
  // Check if the difference between current speed and target is 50 or more
  if (abs(value - _currentSpeed) >= 50) {
    // Gradually update the speed to the target
    int step = (value > _currentSpeed) ? 1 : -1;
    while (_currentSpeed != value) {
      _currentSpeed += step;
      currentPwmValue = map(_currentSpeed, -100, 100, 205, 410);
      ledcWrite(_pin, currentPwmValue);
      delay(10); // Small delay to smooth the transition
    }
  } else {
    // Directly set the new speed
    _currentSpeed = value;
    ledcWrite(_pin, targetPwmValue);
  }
}

/*
#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t pin) {
  _pin = pin;
}

bool MotorDriver::init() {
  Serial.begin(115200);
  Serial.println("Start");

  ledcAttach(_pin, 50, 12);   
  
  delay(100);
  ledcWrite(_pin, 410); 
  delay(500);
  ledcWrite(_pin, 205);  
  delay(500);
  ledcWrite(_pin, 306);  
  delay(500);

  return true;
}

void MotorDriver::drive(int value) {
  value = constrain(value, -100, 100);
  int pwmValue = map(value, -100, 100, 205, 410);
  ledcWrite(_pin, pwmValue);  
}
*/