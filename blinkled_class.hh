#ifndef blinkled_class_hh
#define blinkled_class_hh

/*
    LED flash class for Arduino. Ed Burdick 1/26/2021

    This provides state and member functions for flashing LEDs based on timers that use
    the millis function, which returns the time (millinseconds) since power-up or reset.

    Constructor (creates an object of type BlinkLed):

    pin is type int
    onTime and offTime are type long
        myBlinkLed = Blinkled(pin, onTime, offTime)

    Member functions:

    Enable and disable turn the blinker on and off
        myBlinkLed.enable()
        myBlinkLed.disable()

    Update should be called in loop(). It causes the object to check the time and update
    the lamp state.
        myBlinkLed.update()
*/

class BlinkLed
{
    private:
    
        // Control variables

        boolean enabled; // enable this lamp
        int pinNumber;   // digital output pin driving this lamp
        long onTime;     // milliseconds
        long offTime;    // milliseconds

        // State
        
        boolean state;  // true = on
        unsigned long prevTimeStamp;    // will store last time LED was updated

    public:
    
        // Member functions declarations. These are defined in blinkled_class.cpp
        
        BlinkLed(int pin, long onDuration, long offDuration);
        void Enable();
        void Disable();
        void Update();
};

#endif
