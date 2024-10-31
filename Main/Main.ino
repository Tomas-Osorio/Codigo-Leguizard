#include "VL.h"

VL vlSensors;

void setup() {
    Serial.begin(115200);
    vlSensors.init();
}

void loop() {
    int* distances = vlSensors.measure();
    Serial.printf("Left sensor distance: %d mm\n", distances[0]);
    Serial.printf("Right sensor distance: %d mm\n", distances[1]);
    delay(500);
}
/*Anda solo un Vl
#include "VL.h"

VL vlSensors;

void setup() {
    Serial.begin(115200);
    vlSensors.init();
}

void loop() {
    int* distances = vlSensors.measure();
    Serial.printf("Left sensor distance: %d mm\n", distances[0]);
    Serial.printf("Right sensor distance: %d mm\n", distances[1]);
    delay(500);
}
*/

/*Expicacion de los vl para chatgpt: i need you to make me a program that search the direction i2c of 2 vl5320x laser sensors.Their xshut pin si conecte to pins out 5 and 6 from a pin expansor called PCF8574.My I2c pins not only are connected to the vl sda and scl they also are conected to this expansor. So i nee a program to read this sensors measures and define the one in the pin 6 right sensor and the one in pin 5 left sensor. I need to know the measures they have specifically its essential. Im working with an esp32 s3 wroom u1 n8r8 and im using this custom libraries:

/*Este codigo qtr anda:
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
}
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
