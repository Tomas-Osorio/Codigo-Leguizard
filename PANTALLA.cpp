// PANTALLA.cpp
#include "PANTALLA.h"

// Color definitions
const char* PANTALLA::ACTIVE_COLOR = "1024";    // Green in RGB565
const char* PANTALLA::INACTIVE_COLOR = "65535"; // White in RGB565

// Constructor
PANTALLA::PANTALLA(Nextion& nextion) : myNextion(nextion) {
    empezarState = false;
    tactica1State = false;
    tactica2State = false;
    tactica3State = false;
}

// Begin method to initialize buttons with inactive color
void PANTALLA::begin() {
    // Set initial colors for buttons
    myNextion.sendCommand("b0.bco=" + String(INACTIVE_COLOR));  // Empezar button initial color
    myNextion.sendCommand("b1.bco=" + String(INACTIVE_COLOR));  // Táctica 1 button initial color
    myNextion.sendCommand("b2.bco=" + String(INACTIVE_COLOR));  // Táctica 2 button initial color
    myNextion.sendCommand("b3.bco=" + String(INACTIVE_COLOR));  // Táctica 3 button initial color
}

// Handle button presses and toggle states
void PANTALLA::handleButtonPress() {
    if (myNextion.listen()) {
        switch (myNextion.lastPressedComponentId) {
            case 0:  // Empezar button
                empezarState = !empezarState;
                updateButtonColor("b0", empezarState);
                Serial.println("Empezar state: " + String(empezarState));
                break;
            case 1:  // Táctica 1 button
                tactica1State = !tactica1State;
                updateButtonColor("b1", tactica1State);
                Serial.println("Táctica 1 state: " + String(tactica1State));
                break;
            case 2:  // Táctica 2 button
                tactica2State = !tactica2State;
                updateButtonColor("b2", tactica2State);
                Serial.println("Táctica 2 state: " + String(tactica2State));
                break;
            case 3:  // Táctica 3 button
                tactica3State = !tactica3State;
                updateButtonColor("b3", tactica3State);
                Serial.println("Táctica 3 state: " + String(tactica3State));
                break;
        }
    }
}

// Update button color based on state
void PANTALLA::updateButtonColor(const char* buttonId, bool state) {
    if (state) {
        myNextion.sendCommand(String(buttonId) + ".bco=" + String(ACTIVE_COLOR));
    } else {
        myNextion.sendCommand(String(buttonId) + ".bco=" + String(INACTIVE_COLOR));
    }
    myNextion.sendCommand("ref " + String(buttonId));  // Refresh button display
}
/*para el main
#include <Nextion.h>
#include "PANTALLA.h"

// Set up the Nextion display
Nextion myNextion(Serial2, 9600);  // Adjust baud rate if necessary

// Set up the PANTALLA control class
PANTALLA pantalla(myNextion);

void setup() {
  Serial2.begin(9600);  // Start serial for Nextion communication
  delay(500);
  myNextion.init();  // Initialize Nextion
  pantalla.begin();  // Initialize button colors
}

void loop() {
  pantalla.handleButtonPress();  // Check for button presses and handle toggling
}
*/