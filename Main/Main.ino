#include <QTRSensors.h>
#include "MotorDriver.h"
#include <HardwareSerial.h>

// Motor pins
#define LEFT_FRONT_MOTOR_PIN 39
#define RIGHT_FRONT_MOTOR_PIN 9
#define LEFT_BACK_MOTOR_PIN 45
#define RIGHT_BACK_MOTOR_PIN 47

// QTR Sensor pins
const uint8_t sensorPins[] = {4, 5, 1, 2}; // Front Left, Back Left, Front Right, Back Right

// Sensor thresholds
const uint16_t WHITE_THRESHOLD = 500;

// QTR Sensor object
QTRSensors qtr;

// Motor objects
MotorDriver leftFrontMotor(LEFT_FRONT_MOTOR_PIN);
MotorDriver rightFrontMotor(RIGHT_FRONT_MOTOR_PIN);
MotorDriver leftBackMotor(LEFT_BACK_MOTOR_PIN);
MotorDriver rightBackMotor(RIGHT_BACK_MOTOR_PIN);

// Nextion communication pins
#define RX_PIN 44  // RX to Nextion TX
#define TX_PIN 43  // TX to Nextion RX

HardwareSerial nextionSerial(1);

// Button state
volatile int buttonState = 0; // Variable to toggle
bool programRunning = false;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    nextionSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("Starting communication with Nextion...");

    // Initialize QTR sensors
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, sizeof(sensorPins) / sizeof(sensorPins[0]));

    // Initialize motors
    leftFrontMotor.init();
    rightFrontMotor.init();
    leftBackMotor.init();
    rightBackMotor.init();
}

void loop() {
    // Check Nextion data
    if (nextionSerial.available()) {
        handleNextionInput();
    }

    // If program is not running, do nothing
    if (!programRunning) {
        stopMotors();
        return;
    }

    // Array to store sensor readings
    uint16_t sensorValues[4];

    // Read sensor values
    qtr.read(sensorValues);

    // Interpret sensor readings
    bool frontLeft = sensorValues[0] > WHITE_THRESHOLD;
    bool backLeft = sensorValues[1] > WHITE_THRESHOLD;
    bool frontRight = sensorValues[2] > WHITE_THRESHOLD;
    bool backRight = sensorValues[3] > WHITE_THRESHOLD;

    // Line detection and response logic
    if (frontLeft && frontRight) {
        driveMotors(-20, -20, 20, 20); // Sharp clockwise turn
        delay(300);
        driveStraight();
    } else if (backLeft && backRight) {
        driveMotors(20, 20, -20, -20); // Sharp counter-clockwise turn
        delay(300);
        driveStraight();
    } else if (frontLeft) {
        driveMotors(-20, -10, 20, 10); // Rotate clockwise
        delay(300);
        driveStraight();
    } else if (frontRight) {
        driveMotors(-10, -20, 10, 20); // Rotate counter-clockwise
        delay(300);
        driveStraight();
    } else if (backLeft) {
        driveMotors(-20, -10, 20, 10); // Rotate clockwise
        delay(300);
        driveStraight();
    } else if (backRight) {
        driveMotors(-10, -20, 10, 20); // Rotate counter-clockwise
        delay(300);
        driveStraight();
    } else {
        driveStraight();
    }

    // Add a delay for sensor stability
    delay(100);
}

// Handle Nextion touchscreen input
void handleNextionInput() {
    if (nextionSerial.read() == 0x23) {
        delay(10);
        if (nextionSerial.available() >= 2) {
            if (nextionSerial.read() == 0x02 && nextionSerial.read() == 0x54) {
                buttonState = !buttonState;

                if (buttonState) {
                    Serial.println("Button toggled ON. Starting program after 5 seconds...");
                    delay(5000);
                    programRunning = true;
                } else {
                    Serial.println("Button toggled OFF. Stopping program.");
                    programRunning = false;
                }
            }
        }
    }
}

// Function to stop all motors
void stopMotors() {
    driveMotors(0, 0, 0, 0);
}

// Function to drive motors in aggressive forward motion
void driveStraight() {
    driveMotors(20, 20, 20, 20); // All motors forward
}

// Function to drive motors
void driveMotors(int leftFront, int rightFront, int leftBack, int rightBack) {
    leftFrontMotor.drive(leftFront);
    rightFrontMotor.drive(rightFront);
    leftBackMotor.drive(leftBack);
    rightBackMotor.drive(rightBack);
}