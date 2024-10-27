#include "VL.h"

// Constructor
VL::VL() : pcf(PCF8574_ADDRESS) {
    // Initialize the sensorInitialized array to false
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensorInitialized[i] = false;
    }
}

// Initialize the sensors
void VL::init() {
    Wire.begin(SDA_PIN, SCL_PIN);
    pcf.begin();

    // Scan I2C bus to check if the PCF8574 is detected
    Wire.beginTransmission(PCF8574_ADDRESS);
    if (Wire.endTransmission() != 0) {
        Serial.println("PCF8574 not detected on the I2C bus.");
        return;
    }
    Serial.println("PCF8574 detected on the I2C bus.");

    // Step 1: Ensure all sensors are initially turned off
    Serial.println("Turning off all sensors...");
    for (int i = 0; i < SENSOR_COUNT; i++) {
        pcf.write(xshutPins[i], LOW);
    }
    delay(500); // Increase delay to 500ms to ensure sensors are off

    // Step 2: Initialize each sensor one at a time
    for (int i = 0; i < SENSOR_COUNT; i++) {
        // Turn on the current sensor by setting its xshut pin high
        pcf.write(xshutPins[i], HIGH);
        delay(500); // Increase delay to 500ms to ensure the sensor is powered on

        // Try to begin communication with the sensor and set its address
        if (sensors[i].begin(0x29)) {  
            sensors[i].setAddress(newAddresses[i]);
            Serial.printf("Sensor %d initialized at 0x%02X\n", i, newAddresses[i]);
            sensorInitialized[i] = true; // Mark sensor as initialized
        } else {
            Serial.printf("Sensor %d failed to initialize.\n", i);
            sensorInitialized[i] = false; // Mark sensor as not initialized
        }

        // Turn off the sensor after setting its address
        pcf.write(xshutPins[i], LOW);
        delay(500); // Increase delay to ensure proper shutdown
    }

    // Step 3: Re-enable both sensors
    Serial.println("Turning on all sensors...");
    for (int i = 0; i < SENSOR_COUNT; i++) {
        pcf.write(xshutPins[i], HIGH);
        delay(10); // Small delay for stabilization
    }
}

// Measure distances from the sensors
int* VL::measure() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (!sensorInitialized[i]) {
            measurements[i] = -1; // Indicate an error in the measurement
            continue; // Skip to the next sensor
        }

        VL53L0X_RangingMeasurementData_t measureData;
        sensors[i].rangingTest(&measureData, false);  // Measurement for each sensor at its unique address

        if (measureData.RangeStatus == 0) {
            measurements[i] = measureData.RangeMilliMeter;
            Serial.printf("Sensor %d (Address: 0x%02X): %d mm\n", i, newAddresses[i], measurements[i]);
        } else {
            measurements[i] = -1;
            Serial.printf("Sensor %d (Address: 0x%02X): Measurement failed\n", i, newAddresses[i]);
        }
    }
    return measurements;
}
/*#include "VL.h"

VL::VL() : pcf(PCF8574_ADDRESS) {}

void VL::init() {
    Wire.begin(SDA_PIN, SCL_PIN);
    pcf.begin();

    for (int i = 0; i < SENSOR_COUNT; i++) {
        pcf.write(xshutPins[i], LOW);
    }
    delay(10);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        pcf.write(xshutPins[i], HIGH);
        delay(10);

        if (sensors[i].begin(0x29)) {
            sensors[i].setAddress(newAddresses[i]);
            Serial.printf("Sensor %d initialized at 0x%02X\n", i, newAddresses[i]);
        } else {
            Serial.printf("Sensor %d failed to initialize.\n", i);
        }

        pcf.write(xshutPins[i], LOW);
        delay(10);
    }
}

int* VL::measure() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        VL53L0X_RangingMeasurementData_t measureData;
        sensors[i].rangingTest(&measureData, false);

        if (measureData.RangeStatus == 0) {
            measurements[i] = measureData.RangeMilliMeter;
            Serial.printf("Sensor %d (Address: 0x%02X): %d mm\n", i, newAddresses[i], measurements[i]);
        } else {
            measurements[i] = -1;
            Serial.printf("Sensor %d (Address: 0x%02X): Measurement failed\n", i, newAddresses[i]);
        }
    }
    return measurements;
}
*/