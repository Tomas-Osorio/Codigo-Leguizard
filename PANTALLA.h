#ifndef ButtonToggle_h
#define ButtonToggle_h

#include <Nextion.h>

class ButtonToggle {
public:
    ButtonToggle(NexButton* button);
    void toggleValue();
    void checkTouch();

private:
    NexButton* _button;
    bool _state;
};

#endif
