#include "ButtonToggle.h"

ButtonToggle::ButtonToggle(NexButton* button) 
    : _button(button), _state(false) {}

void ButtonToggle::toggleValue() {
    _state = !_state;  // Toggle between 0 and 1
    Serial.print("Button State: ");
    Serial.println(_state);
}

void ButtonToggle::checkTouch() {
    uint16_t x, y;
    if (_button->getTouch(x, y)) {  // Check if the button is pressed
        toggleValue();
    }
}
/*programa de prueba
#include <Nextion.h>
#include "ButtonToggle.h"

// Define the button on the Nextion screen
NexButton button = NexButton(0, 1, "b0");  // Page 0, ID 1, object name "b0"
ButtonToggle buttonToggle(&button);

void setup() {
    Serial.begin(9600);
    nexInit();  // Initialize Nextion
}

void loop() {
    nexLoop(nex_listen_list);  // Listen for button events
    buttonToggle.checkTouch();
}
*/