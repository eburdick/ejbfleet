#include <arduino.h>
#include "clean_edge_class.hh"

// Constructors
//
// pin is the digital sensor pin number
//
// suspendTime is the number of milliseconds to ignore the pin after a state change
//
// cycleBaseState is the initial pin state to assume for the CheckCycle function
//

CleanEdge::CleanEdge(int pin, unsigned int suspendTime, int cycleBaseState)
{
    inPort = pin;
    suspendWaitTime = suspendTime;
    cycleInitialState = cycleBaseState;
}

// version defaulting cycleBaseState to HIGH
CleanEdge::CleanEdge(int pin, unsigned int suspendTime)
{
    inPort = pin;
    suspendWaitTime = suspendTime;
    cycleInitialState = HIGH;
}

/****************************************************************************
    Sample function

    This function reads the input port the Sensor is connected to. But if
    the Sensor state changed within suspendWaitTime ago, the function just
    returns the current value.  Once this time has passed, the function
    resumes reading the port. The purpose of this is to bypass physical
    "bounce" in the switch contacts or other mechanical noise during the
    transition.

    Example call (buttonReader is a CleanEdge object):

    ButtonPressed = buttonReader.Sample();
*/

int CleanEdge::Sample()
{
    //local variables
    int pinState;
    unsigned long currentTime;

    //Take a snapshot of milliseconds since program start
    currentTime = millis();
    //
    // If we are not in a transition wait, read the Sensor.
    //
    if (currentTime - suspendWaitTime > SensorSuspendStartTime)
    {
        pinState = digitalRead(inPort); //Read the Sensor input port

        /*
            If the state read from the Sensor is the same as the previous value,
            just return this state
        */
        if (pinState == SensorState)
        {
            return (SensorState);
        }
        else
        {
            /*
                state read from the Sensor is different from the previous value,
                so we are going to change it and then prevent reading the Sensor
                again for suspendWaitTime milliseconds.
            */

            // start the suspend timeout.
            SensorSuspendStartTime = currentTime;

            // flip the Sensor state and return the new Sensor state.
            if (SensorState == LOW)
            {
                SensorState = HIGH;
            }
            else
            {
                SensorState = LOW;
            }

            return (SensorState);
        }
    }
    else // Still in the suspend time out. Just return the current value.
    {
        return (SensorState);
    }
} //end Sample{}

/************************************************************************
        CheckCycle()

        This function looks for a pair of transitions, either LOW to HIGH
        to LOW, or HIGH to LOW to HIGH. The intended location of calls to
        this function is in the main loop. If we are looking for a LOW to
        HIGH to LOW, and we are already in a HIGH state, we will wait
        until the input goes to LOW to start the cycle. This should
        hardly ever happen, and is probably an error condition, but it is
        better to do this than to just ignore it. Of course, we do the
        same thing for a HIGH to LOW to HIGH if we are already in a LOW
        state.
*/
boolean CleanEdge::CheckCycle()
{
    /*
        There are three possible cases here...

        1. We have not detected any of the cycle transitions and the
        sensor is in the correct initial state. In this case, we will
        check if the sensor state has changed. If so, we will set
        waitingForEndingTransition and return false.

        2. We have not detected any of the cycle transitions and the
        sensor is in the wrong initial state. In this case, we will
        check if the sensor state has changed to the right intial
        state and return false. Presumably next time this is called,
        we will be in case 1.

        3. waitingForEndingTransition is true. This means we detected
        the first transition of the cycle and we are waiting for the
        second one. We read the sensor and if it has switched back to
        the initial state, we clear waitingForEndingTransition and
        return true. The next time around, we will most likely be in
        case 1 above.
    */
    boolean SensorState; //local temporary variable

    SensorState = Sample();
    /*
        We initialized waitingForInitialState to true. This means the sensor
        is initially in the wrong state. If this is the case, the initial
        transition from the initial state to the opposite state cannot
        happen, so until the sensor returns the correct initial state, we
        will read it each time CheckCycle is called. Once the correct intial
        state is detected, we will set this variable to false.
    */
    if (waitingForInitialState)
    {
        if (SensorState != cycleInitialState)
        {
            return (false);
        }
        else
        {
            waitingForInitialState = false;
        }
    }
    // Initial state is correct. Look for the two transitions.
    if (waitingForEndingTransition)
    {
        /*
            waiting for ending transition. Sample Sensor to see if
            wait is over. If so, we are going to clear the state and
        */
        if (SensorState == cycleInitialState)
        {
            //Ending transition detected. Return true.
            waitingForEndingTransition = false;
            return (true);
        }
    }
    else
    {
        /*
            Not waiting for ending transition. This means we have not yet
            seen the initial transition. We will check now for it.
        */

        if (SensorState != cycleInitialState)
        {
            //Sensor has just transitioned from initial state. Start
            //waiting for next transition. Return false.
            waitingForEndingTransition = true;
            return (false);
        }
    }
    // By default, we are waiting for the intial transition.
    return (false);
}
