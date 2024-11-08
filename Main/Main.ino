#include <Wire.h>
#include <VL53L1X.h>
#include <PCF8574.h>

VL53L1X sensor1;
VL53L1X sensor2;
PCF8574 pcf8574(0x20);  // PCF8574 address

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C on custom pins
  Wire.begin(36, 35);  // SDA on 36, SCL on 35
  pcf8574.begin();

  // Step 1: Disable both sensors by setting both XSHUT lines to LOW
  pcf8574.write(6, LOW);  // XSHUT for first sensor
  pcf8574.write(4, LOW);  // XSHUT for second sensor
  delay(10);              // Short delay to ensure both are off

  // Step 2: Enable the first sensor (XSHUT 6 HIGH)
  Serial.println("Enabling first sensor...");
  pcf8574.write(6, HIGH); 
  delay(50);              // Delay for the sensor to power up
  
  // Initialize sensor1 (assuming it’s at address 0x29)
  if (!sensor1.init()) {
    Serial.println("Failed to initialize sensor 1 at 0x29!");
    while (1);
  }
  Serial.println("Sensor 1 initialized at 0x29.");
  sensor1.setDistanceMode(VL53L1X::Long);
  sensor1.setMeasurementTimingBudget(50000);
  sensor1.startContinuous(50);

  // Step 3: Disable the first sensor, enable the second sensor (XSHUT 4 HIGH)
  pcf8574.write(6, LOW);  // Temporarily disable first sensor
  delay(10);              // Ensure it's powered down
  Serial.println("Enabling second sensor...");
  pcf8574.write(4, HIGH); 
  delay(50);              // Delay for the sensor to power up
  
  // Initialize sensor2 (assuming it’s at address 0x30)
  if (!sensor2.init()) {
    Serial.println("Failed to initialize sensor 2 at 0x30!");
    while (1);
  }
  Serial.println("Sensor 2 initialized at 0x30.");
  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(50000);
  sensor2.startContinuous(50);

  Serial.println("Both sensors initialized successfully.");
}

void loop() {
  // Read and print distance from sensor 1
  Serial.print("Sensor 1 Distance (mm): ");
  Serial.println(sensor1.read());

  // Read and print distance from sensor 2
  Serial.print("Sensor 2 Distance (mm): ");
  Serial.println(sensor2.read());

  delay(100);
}


/*#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup() {
  Serial.begin(115200);

  // Initialize I2C on custom pins
  Wire.begin(36, 35);  // SDA on 36, SCL on 35

  // Initialize the sensor
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); // 50ms for each measurement
  sensor.startContinuous(50);
}

void loop() {
  Serial.print("Distance (mm): ");
  Serial.println(sensor.read());
  delay(100);
}
*/
/*codigo pantalla:(no esta adaptado al esp32)
#include <SoftwareSerial.h>

#define RX_PIN 2  // RX to Nextion TX
#define TX_PIN 3  // TX to Nextion RX

SoftwareSerial nextionSerial(RX_PIN, TX_PIN);

int buttonState = 0;  // Variable to toggle

void setup() {
  nextionSerial.begin(9600);
  Serial.begin(9600);
  Serial.println("Starting communication with Nextion...");
}

void loop() {
  // Check if data is coming from Nextion
  if (nextionSerial.available()) {
    // Read the first byte
    if (nextionSerial.read() == 0x23) {
      delay(10);  // Small delay to allow next bytes to arrive

      // Check for the next two bytes in sequence
      if (nextionSerial.available() >= 2) {
        if (nextionSerial.read() == 0x02 && nextionSerial.read() == 0x54) {
          // Toggle the buttonState variable
          buttonState = !buttonState;
          
          // Print the updated state
          Serial.print("Button State toggled to: ");
          Serial.println(buttonState);
        }
      }
    }
  }
}*/
/*#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup() {
  Serial.begin(115200);

  // Initialize I2C on custom pins
  Wire.begin(36, 35);  // SDA on 36, SCL on 35

  // Initialize the sensor
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); // 50ms for each measurement
  sensor.startContinuous(50);
}

void loop() {
  Serial.print("Distance (mm): ");
  Serial.println(sensor.read());
  delay(100);
}
*//*
Este codigo qtr anda:
#include <QTRSensors.h>

QTRSensors qtr;  // Create an instance of QTRSensors

const uint8_t sensorPins[] = {4, 5, 1, 2}; // Pins for Top Left, Bottom Left, Top Right, Bottom Right

void setup() {
    Serial.begin(115200);

    // Initialize the QTR sensors with the specified pins
    qtr.setTypeRC();  // Sets sensor type to RC for digital sensors
    qtr.setSensorPins(sensorPins, sizeof(sensorPins) / sizeof(sensorPins[0]));
}

void loop() {
    // Array to store readings from each sensor
    uint16_t sensorValues[4];
    
    // Read sensor values
    qtr.read(sensorValues);

    // Print the results to the Serial Monitor
    Serial.print("Top Left (Pin 4): ");
    Serial.println(sensorValues[0] > 500 ? "White" : "Black");

    Serial.print("Bottom Left (Pin 5): ");
    Serial.println(sensorValues[1] > 500 ? "White" : "Black");

    Serial.print("Top Right (Pin 1): ");
    Serial.println(sensorValues[2] > 500 ? "White" : "Black");

    Serial.print("Bottom Right (Pin 2): ");
    Serial.println(sensorValues[3] > 500 ? "White" : "Black");

    // Add a delay for readability
    delay(500);
}*//*
MAIN CHETO MAIN CHETO MAIN CHETO MAIN CHETO MAIN CHETO MAIN CHETO MAIN CHETO MAIN CHETO MAIN CHETO MAIN CHETO
#include <MotorDriver.h>
#include <QTR.h>
#include <Logger.h>
#include <OTAPrograming.h>

// Motor instances
MotorDriver motorFL(39);  // Front Left
MotorDriver motorFR(45);  // Front Right
MotorDriver motorBL(47);  // Back Left
MotorDriver motorBR(9);   // Back Right

// QTR sensor instance
QTR qtrSensors;

// Logger and OTA instances
Logger logger;
OTA ota;

// Sensor pins
const int sensorFL = 1; // Front Left
const int sensorFR = 2; // Front Right
const int sensorBL = 4; // Back Left
const int sensorBR = 5; // Back Right

// Modes and logging level
bool devMode = true;
bool Fight = false;
int Log = 0; // 0 = None, 1 = Errors, 2 = All

void setup() {
  Serial.begin(115200);

  // Initialize motors
  motorFL.init();
  motorFR.init();
  motorBL.init();
  motorBR.init();

  // Initialize sensors
  qtrSensors.init(sensorFL, sensorFR, sensorBL, sensorBR);

  // Initialize logger
  logger.init(Fight, devMode, true, "SSID", "PASSWORD");

  // Initialize OTA with Wi-Fi credentials
  const char* ssid = "SSID";
  const char* password = "PASSWORD";
  if (ota.init(Fight, devMode, Log, ssid, password) == 0) {
    Serial.println("OTA Initialized");
  } else {
    Serial.println("OTA Initialization Failed");
  }

  // Indicate ready status with LED
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(250);
  digitalWrite(12, LOW);
  delay(250);
  digitalWrite(12, HIGH);
  delay(250);
}

void loop() {
  // OTA check
  ota.check();
  Serial.printf("OTA Status: %s\n", ota.isConnected() ? "Connected" : "Disconnected");

  // Check each sensor's reading
  bool edgeFL = qtrSensors.medir(sensorFL);  // Front Left
  bool edgeFR = qtrSensors.medir(sensorFR);  // Front Right
  bool edgeBL = qtrSensors.medir(sensorBL);  // Back Left
  bool edgeBR = qtrSensors.medir(sensorBR);  // Back Right

  // Log QTR sensor readings
  Serial.printf("QTR Sensor States - FL: %d, FR: %d, BL: %d, BR: %d\n", edgeFL, edgeFR, edgeBL, edgeBR);
  logger.logMessage(Fight, devMode, Log, 1, "Reading sensors for edge detection");

  // State control based on sensors
  if (!edgeFL && !edgeFR && !edgeBL && !edgeBR) {
    EstadoNormal();
  } else if (!edgeFL && !edgeFR) {
    EstadoQTR_FL_FR_A();  // 180° turn
  } else if (!edgeBL && !edgeBR) {
    EstadoQTR_BL_BR_B();  // 180° turn
  } else if (!edgeFL) {
    EstadoQTR_FL_A();  // 180° turn
  } else if (!edgeFR) {
    EstadoQTR_FR_A();  // 180° turn
  } else if (!edgeBL) {
    EstadoQTR_BL_B();  // 180° turn
  } else if (!edgeBR) {
    EstadoQTR_BR_B();  // 180° turn
  } else {
    EstadoCero();
  }

  // Disable WiFi if in Fight mode or not in devMode
  if (Fight || !devMode) {
    disableWiFi();
  }

  delay(50); // Adjust delay as needed
}

void EstadoNormal() {
  motorFL.drive(50);
  motorFR.drive(50);
  motorBL.drive(50);
  motorBR.drive(50);
  Serial.printf("EstadoNormal - Motors: FL=50, FR=50, BL=50, BR=50\n");
  logger.logMessage(Fight, devMode, Log, 2, "Driving forward - Safe ground detected");
}

void EstadoCero() {
  motorFL.drive(0);
  motorFR.drive(0);
  motorBL.drive(0);
  motorBR.drive(0);
  Serial.printf("EstadoCero - Motors: FL=0, FR=0, BL=0, BR=0\n");
  logger.logMessage(Fight, devMode, Log, 2, "Stopping motors");
}

// Rotate 180° when both Front sensors detect the line
void EstadoQTR_FL_FR_A() {
  motorFL.drive(-50);  // Reverse front-left motor
  motorFR.drive(-50);  // Reverse front-right motor
  motorBL.drive(50);   // Forward back-left motor
  motorBR.drive(50);   // Forward back-right motor
  Serial.printf("EstadoQTR_FL_FR_A - Motors: FL=-50, FR=-50, BL=50, BR=50\n");
  delay(600);
  EstadoNormal();
  logger.logMessage(Fight, devMode, Log, 2, "180º turn - Front edge detected by both front sensors");
}

// Rotate 180° when both Back sensors detect the line
void EstadoQTR_BL_BR_B() {
  motorFL.drive(50);   // Forward front-left motor
  motorFR.drive(50);   // Forward front-right motor
  motorBL.drive(-50);  // Reverse back-left motor
  motorBR.drive(-50);  // Reverse back-right motor
  Serial.printf("EstadoQTR_BL_BR_B - Motors: FL=50, FR=50, BL=-50, BR=-50\n");
  delay(600);
  EstadoNormal();
  logger.logMessage(Fight, devMode, Log, 2, "180º turn - Back edge detected by both back sensors");
}

// Rotate 180° when Front Left sensor detects the line
void EstadoQTR_FL_A() {
  motorFL.drive(-50);
  motorFR.drive(50);
  motorBL.drive(50);
  motorBR.drive(-50);
  Serial.printf("EstadoQTR_FL_A - Motors: FL=-50, FR=50, BL=50, BR=-50\n");
  delay(600);
  EstadoNormal();
  logger.logMessage(Fight, devMode, Log, 2, "180º turn - Edge detected by Front Left sensor");
}

// Rotate 180° when Front Right sensor detects the line
void EstadoQTR_FR_A() {
  motorFL.drive(50);
  motorFR.drive(-50);
  motorBL.drive(-50);
  motorBR.drive(50);
  Serial.printf("EstadoQTR_FR_A - Motors: FL=50, FR=-50, BL=-50, BR=50\n");
  delay(600);
  EstadoNormal();
  logger.logMessage(Fight, devMode, Log, 2, "180º turn - Edge detected by Front Right sensor");
}

// Rotate 180° when Back Left sensor detects the line
void EstadoQTR_BL_B() {
  motorFL.drive(50);
  motorFR.drive(-50);
  motorBL.drive(-50);
  motorBR.drive(50);
  Serial.printf("EstadoQTR_BL_B - Motors: FL=50, FR=-50, BL=-50, BR=50\n");
  delay(600);
  EstadoNormal();
  logger.logMessage(Fight, devMode, Log, 2, "180º turn - Edge detected by Back Left sensor");
}

// Rotate 180° when Back Right sensor detects the line
void EstadoQTR_BR_B() {
  motorFL.drive(-50);
  motorFR.drive(50);
  motorBL.drive(50);
  motorBR.drive(-50);
  Serial.printf("EstadoQTR_BR_B - Motors: FL=-50, FR=50, BL=50, BR=-50\n");
  delay(600);
  EstadoNormal();
  logger.logMessage(Fight, devMode, Log, 2, "180º turn - Edge detected by Back Right sensor");
}

void disableWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.println("WiFi Disabled");
}
*/