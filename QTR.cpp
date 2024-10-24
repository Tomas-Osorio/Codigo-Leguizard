#include "QTR.h"

QTR::QTR() {

}

void QTR::init(int Pin1, int Pin2, int Pin3, int Pin4) {
    pinMode(Pin1, INPUT);
    pinMode(Pin2, INPUT);
    pinMode(Pin3, INPUT);
    pinMode(Pin4, INPUT);
}


bool QTR::medir(int Pin) {
    int var = analogRead(Pin);
    if(var < 551) return false;
    else return true;
} 
/*
#include "QTR.h"

QTR qtr;

const int topLeftPin = 4;
const int bottomLeftPin = 5;
const int topRightPin = 1;
const int bottomRightPin = 2;

void setup() {
    Serial.begin(9600);
    // Initialize the QTR sensors
    qtr.init(topLeftPin, bottomLeftPin, topRightPin, bottomRightPin);
}

void loop() {
    // Read each sensor and check if it sees black or white
    bool topLeft = qtr.medir(topLeftPin);
    bool bottomLeft = qtr.medir(bottomLeftPin);
    bool topRight = qtr.medir(topRightPin);
    bool bottomRight = qtr.medir(bottomRightPin);

    // Print the results to the Serial Monitor
    Serial.print("Top Left (Pin 4): ");
    Serial.println(topLeft ? "White" : "Black");

    Serial.print("Bottom Left (Pin 5): ");
    Serial.println(bottomLeft ? "White" : "Black");

    Serial.print("Top Right (Pin 1): ");
    Serial.println(topRight ? "White" : "Black");

    Serial.print("Bottom Right (Pin 2): ");
    Serial.println(bottomRight ? "White" : "Black");

    // Add a delay for readability
    delay(500);
}

*/

