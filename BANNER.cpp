#include "BannerSensor.h"

// Constructor to initialize the sensor pin
BannerSensor::BannerSensor(int pin) {
  _pin = pin;
}

// Function to set up the sensor
void BannerSensor::begin() {
  pinMode(_pin, INPUT); // Set the pin as an input
}

// Function to check if an object is detected
bool BannerSensor::isObjectDetected() {
  return digitalRead(_pin) == HIGH; // Return true if the sensor detects an object
}
/*codigo de prueba:
#include "BannerSensor.h"

BannerSensor sensor(2); // Create a sensor object on pin 2

void setup() {
  Serial.begin(9600);
  sensor.begin(); // Initialize the sensor
}

void loop() {
  if (sensor.isObjectDetected()) {
    Serial.println("Object detected");
  } else {
    Serial.println("No object detected");
  }
  
  delay(500); // Wait for half a second before the next reading
}

*/