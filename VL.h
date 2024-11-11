#ifndef VL_H
#define VL_H




class VL {
public:
    VL();                // Constructor to initialize measurements array
    void init();         // Initialize sensors and I/O expander
    int* measure();      // Measure distances and return measurements for all sensors

private:
    PCF8574 pcf;               // PCF8574 instance to control power
    Adafruit_VL53L0X sensors[SENSOR_COUNT];  // Array for VL53L0X sensors
    int measurements[SENSOR_COUNT];  // Array to store measurements
    const uint8_t newAddresses[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35}; // New I2C addresses for each sensor

    void initializeSensor(uint8_t index);  // Helper function to initialize each sensor
};

#endif
