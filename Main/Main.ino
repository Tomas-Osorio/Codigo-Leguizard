#include "Logger.h"
#include "OTAPrograming.h"
#include "MotorDriver.h"
#include "QTR.h"

// Motor declarations
MotorDriver M1(9);
MotorDriver M2(47);
MotorDriver M3(45);
MotorDriver M4(39);

// Logger declaration
Logger logger;

// OTA declaration
OTA ota;

// Sensor declaration
QTR sensor;

// Global variables for modes
bool devMode = true;
bool Fight = false;
int Log = 0; // 0 = NADA, 1 = ERRORES, 2 = TODO

// Sensor pin definitions
const int topLeftPin = A0;
const int topRightPin = A1;
const int bottomLeftPin = A2;
const int bottomRightPin = A3;

// Movement state
enum MovementState { FORWARD, BACKWARD };
MovementState currentMovement = FORWARD;

// Function to disable WiFi when not in use
void disableWiFi() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
}

// Function to initialize motors, sensors, logger, and OTA
void initializeComponents() {
    // Initialize motors
    M1.init();
    M2.init();
    M3.init();
    M4.init();

    // Initialize sensors
    sensor.init(topLeftPin, topRightPin, bottomLeftPin, bottomRightPin);

    // Initialize logger
    logger.init(Fight, devMode, true, "SSID", "PASSWORD");

    // Initialize OTA
    const char* ssid = "SSID";
    const char* password = "PASSWORD";
    if (ota.init(Fight, devMode, Log, ssid, password) == 0) {
        Serial.println("OTA Initialized");
    } else {
        Serial.println("OTA Initialization Failed");
    }
}

// Function to control motors based on sensor readings
void controlMotorsBasedOnSensors() {
    // Get sensor readings
    bool topLeftDetected = sensor.medir(topLeftPin);
    bool topRightDetected = sensor.medir(topRightPin);
    bool bottomLeftDetected = sensor.medir(bottomLeftPin);
    bool bottomRightDetected = sensor.medir(bottomRightPin);

    // Update movement state based on any sensor detection
    if (currentMovement == FORWARD && (topLeftDetected || topRightDetected)) {
        currentMovement = BACKWARD;  // Switch to backward if any top sensor detects something
        logger.logMessage(Fight, devMode, Log, 1, "Switching to BACKWARD");
    } else if (currentMovement == BACKWARD && (bottomLeftDetected || bottomRightDetected)) {
        currentMovement = FORWARD;   // Switch to forward if any bottom sensor detects something
        logger.logMessage(Fight, devMode, Log, 1, "Switching to FORWARD");
    }

    // Set motor speed based on current movement state
    int motorValue = (currentMovement == FORWARD) ? 50 : -50;
    M1.drive(motorValue);
    M2.drive(motorValue);
    M3.drive(motorValue);
    M4.drive(motorValue);
}

void setup() {
    Serial.begin(115200);
    initializeComponents();  // Initialize all components
}

void loop() {
    controlMotorsBasedOnSensors();  // Monitor sensors and control motors
    ota.check();                    // Handle OTA updates
    delay(50);                      // Adjust delay as needed
    if (Fight || !devMode) {
        disableWiFi();              // Disable WiFi if in Fight mode or not in devMode
    }
}

/*#include <MotorDriver.h>
#include <QTR.h>

// Motor instances
MotorDriver motorFL(39);  // Front Left
MotorDriver motorFR(45);  // Front Right
MotorDriver motorBL(47);  // Back Left
MotorDriver motorBR(9);   // Back Right

// QTR sensor instance
QTR qtrSensors;

// Sensor pins
const int sensorFL = 1; // Front Left
const int sensorFR = 2; // Front Right
const int sensorBL = 4; // Back Left
const int sensorBR = 5; // Back Right

void setup() {
  Serial.begin(115200);

  // Initialize motors
  motorFL.init();
  motorFR.init();
  motorBL.init();
  motorBR.init();

  // Initialize sensors
  qtrSensors.init(sensorFL, sensorFR, sensorBL, sensorBR);

  // Indicate ready status with LED
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(12, HIGH);
  delay(250);
  digitalWrite(12, LOW);
  delay(250);
  digitalWrite(12, HIGH);
  delay(250);
}

void loop() {
  // Check each sensor's reading
  bool edgeFL = qtrSensors.medir(sensorFL);  // Front Left
  bool edgeFR = qtrSensors.medir(sensorFR);  // Front Right
  bool edgeBL = qtrSensors.medir(sensorBL);  // Back Left
  bool edgeBR = qtrSensors.medir(sensorBR);  // Back Right

  // State control based on sensors
  if (!edgeFL && !edgeFR && !edgeBL && !edgeBR) {
    // All sensors detect safe ground
    EstadoNormal();
  } else if (!edgeFR) {
    EstadoQTR_FR_A();
    delay(400);
    EstadoQTR_FR_B();
  } else if (!edgeFL) {
    EstadoQTR_FL_A();
    delay(400);
    EstadoQTR_FL_B();
  } else if (!edgeBL || !edgeBR) {
    EstadoEnemigoEncontrado();
  } else {
    EstadoCero();
  }
}

void EstadoNormal() {
  motorFL.drive(50);
  motorFR.drive(50);
  motorBL.drive(50);
  motorBR.drive(50);
}

void EstadoCero() {
  motorFL.drive(0);
  motorFR.drive(0);
  motorBL.drive(0);
  motorBR.drive(0);
}

void EstadoEnemigoEncontrado() {
  motorFL.drive(50);
  motorFR.drive(50);
  motorBL.drive(50);
  motorBR.drive(50);
}

void EstadoQTR_FL_A() {
  motorFL.drive(-50);
  motorFR.drive(50);
  motorBL.drive(-50);
  motorBR.drive(50);
}

void EstadoQTR_FL_B() {
  motorFL.drive(50);
  motorFR.drive(-50);
  motorBL.drive(50);
  motorBR.drive(-50);
}

void EstadoQTR_FR_A() {
  motorFL.drive(50);
  motorFR.drive(-50);
  motorBL.drive(50);
  motorBR.drive(-50);
}

void EstadoQTR_FR_B() {
  motorFL.drive(-50);
  motorFR.drive(50);
  motorBL.drive(-50);
  motorBR.drive(50);
}
*/