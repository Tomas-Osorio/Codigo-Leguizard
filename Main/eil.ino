#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Create instances for each sensor
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2();

// Define custom I2C pins (replace with your own pin numbers)
#define SDA_PIN 21  // example custom SDA pin
#define SCL_PIN 22  // example custom SCL pin

void setup() {
  // Start serial communication
  Serial.begin(9600);
  
  // Initialize custom I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize first sensor
  if (!sensor1.begin()) {
    Serial.println("Failed to initialize sensor 1");
    while (1);
  }
  
  // Initialize second sensor with a different I2C address
  sensor2.setAddress(0x31);  // You can change the address if needed
  if (!sensor2.begin()) {
    Serial.println("Failed to initialize sensor 2");
    while (1);
  }

  Serial.println("Sensors initialized successfully!");
}

void loop() {
  // Measure distance from the first sensor
  VL53L0X_RangingMeasurementData_t measurement1;
  sensor1.rangingTest(&measurement1, false);
  if (measurement1.RangeStatus == 0) {  // 0 means no error
    Serial.print("Sensor 1 Distance (mm): ");
    Serial.println(measurement1.RangeMilliMeter);
  } else {
    Serial.println("Sensor 1 Error");
  }

  // Measure distance from the second sensor
  VL53L0X_RangingMeasurementData_t measurement2;
  sensor2.rangingTest(&measurement2, false);
  if (measurement2.RangeStatus == 0) {  // 0 means no error
    Serial.print("Sensor 2 Distance (mm): ");
    Serial.println(measurement2.RangeMilliMeter);
  } else {
    Serial.println("Sensor 2 Error");
  }

  // Wait a little before taking the next measurement
  delay(100);  // Adjust delay as needed
}
