#include "VL.h"

VL::VL() : pcf(PCF8574_ADDRESS) {
    // Initialize all sensor measurements to -1 (invalid)
    for (int i = 0; i < SENSOR_COUNT; i++) {
        measurements[i] = -1;  // Default value indicating no measurement
    }
}

void VL::init() {
    // Start I2C communication with custom SDA and SCL pins
    

    // Initialize the PCF8574 I/O expander
    
    // Initially power off all sensors
    
    delay(10);

    // Initialize each sensor
    for (int i = 0; i < SENSOR_COUNT; i++) {
        // Power on the sensor
        pcf.write(i, HIGH);
        delay(10);  // Allow the sensor to power up

        // Initialize the sensor
        if (sensors[i].begin()) {
            sensors[i].setAddress(newAddresses[i]);
            Serial.printf("Sensor %d initialized at address 0x%02X\n", i, newAddresses[i]);
        } else {
            Serial.printf("Sensor %d failed to initialize.\n", i);
        }

        // Power off the sensor after initialization
        pcf.write(i, LOW);
        delay(10);
    }
}

int* VL::measure() {
    VL53L0X_RangingMeasurementData_t measureData;

    // Take measurements from all sensors
    for (int i = 0; i < SENSOR_COUNT; i++) {
        // Power on the sensor
        pcf.write(i, HIGH);
        delay(10);  // Allow the sensor to power up

        // Perform the measurement
        sensors[i].rangingTest(&measureData, false);

        // Check if the measurement is valid
        if (measureData.RangeStatus == 0) {
            measurements[i] = measureData.RangeMilliMeter;  // Store valid distance
        } else {
            measurements[i] = -1;  // Store -1 if the measurement failed
        }

        // Power off the sensor after measurement
        pcf.write(i, LOW);
        delay(10);
    }

    // Return the array of measurements
    return measurements;
}
