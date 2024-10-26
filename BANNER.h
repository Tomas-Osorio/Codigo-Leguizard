#ifndef BANNERSENSOR_H
#define BANNERSENSOR_H

#include <Arduino.h>

class BannerSensor {
  public:
    BannerSensor(int pin); // Constructor to initialize the sensor with a pin
    void begin();          // Function to set up the sensor
    bool isObjectDetected(); // Function to check if an object is detected

  private:
    int _pin; // The pin connected to the sensor
};

#endif
