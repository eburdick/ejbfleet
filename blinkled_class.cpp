#include <arduino.h>
#include "blinkled_class.hh"


// BlinkLed class member functions 
//
// Constructor. Create a BlinkLed object with and output pin and on and off durations.

BlinkLed::BlinkLed(int pin, long onDuration, long offDuration)
{
    pinNumber = pin;
    pinMode(pinNumber, OUTPUT);

    onTime = onDuration;
    offTime = offDuration;

    state = HIGH;
    enabled = false;
    prevTimeStamp = 0;
}

// Enable member function. Enables the object.
void BlinkLed::Enable()
{
    enabled = true;
}

// Disable member function. Disables the object and turns off the corresponding LED.
void BlinkLed::Disable()
{
    enabled = false;
    digitalWrite(pinNumber, HIGH);

}

// Update member function. This is called in loop() to check for the passing of the on and off
// duration times and appropriately turn the LED on or off at such times.
void BlinkLed::Update()
{
    // check if object is enabled
    if (enabled)
    {

        // update state based on time
        unsigned long currentTime = millis();

        if ((state == HIGH) && (currentTime - prevTimeStamp >= onTime))
        {
            state = LOW;
            prevTimeStamp = currentTime;  // Save time of state change
            digitalWrite(pinNumber, state); // Update hardware
        }
        else if ((state == LOW) && (currentTime - prevTimeStamp >= offTime))
        {
            state = HIGH;
            prevTimeStamp = currentTime;
            digitalWrite(pinNumber, state);
        }
    }
    else
    {
        state = HIGH;
        digitalWrite(pinNumber, state);
    }
}
