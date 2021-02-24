Directory contents:

ejbfleet-v1.ino

The primary Arduino code. This defines global state, local functions, and the setup() and 
loop() functions. Note this is not a complete program, but is inserted by the Arduino system
into the hidden main program.

blinkled_class.hh

C++ header file for Blinkled class. This is the declaration of the class that supports
non-blocking LED flashing.

blinkled_class.cpp

The actual C++ code for the member functions of the Blinkled class. This code is compiled
separately and linked with the main program.

timer_class.hh

C++ header file for Timer class. This is the declaration of the class that supports
non-blocking timers.

timer_class.cpp

The actual C++ code for the member functions of the Timer class. This code is compiled
separately and linked with the main program.

button-stuff.h

This is code to support debounced pushbutton transitions and press-release cycles. It is in
a separate file just to reduce the size of the primary code file. There is no separate
compilation unit to go with it.
 