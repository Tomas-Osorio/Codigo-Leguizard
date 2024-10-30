// PANTALLA.h
#ifndef PANTALLA_H
#define PANTALLA_H

#include <Nextion.h> // Include Nextion library

class PANTALLA {
public:
    // Constructor
    PANTALLA(Nextion& nextion);

    // Initialization
    void begin();

    // Handle button presses
    void handleButtonPress();

private:
    // Function to update button color
    void updateButtonColor(const char* buttonId, bool state);

    // Button states
    bool empezarState;
    bool tactica1State;
    bool tactica2State;
    bool tactica3State;

    // Reference to the Nextion display object
    Nextion& myNextion;

    // Color definitions in RGB565 format
    static const char* ACTIVE_COLOR;
    static const char* INACTIVE_COLOR;
};

#endif
