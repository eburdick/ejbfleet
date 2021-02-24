#ifndef clean_edge_class_hh
#define clean_edge_class_hh

/*
    class CleanEdge

    This class is for reading digital sensors like switches or light
    sensors in a way that eliminates edge noise. Edge noise is the natural
    physical uncertainty around a transition. Typical examples are switch
    bounce and the ragged edge of a light/dark boundary on surface. The
    main function of this class, Sample(), adds a time period of ignoring
    the sensor once a sensor input changes. That time period can be set
    for each instance of the class.

    In addition, the function CheckCycle detects a HIGH LOW HIGH or LOW HIGH LOW
    sequence with the same noise cleaning. When you construct the object, you 
    can specify which of the two sequences you want by specifying the initial
    state (e.g. HIGH for a HIGH LOW HIGH sequence, like for a button press-release)
*/
class CleanEdge
{
    private:
        /*
            milliseconds to wait after a sensor state change before reading the
            sensor again
        */
        unsigned int suspendWaitTime;

        // current sensor state.
        int SensorState;

        // starting time of sensor noise delay
        unsigned long SensorSuspendStartTime = 0;

        // object sensor input port
        int inPort;

        //
        // State variables for CheckCycle function
        //

        // The expected initial state for a cycle check.
        int cycleInitialState = HIGH;

        // preset to true. First sensor read will usually clear it.
        boolean waitingForInitialState = true;

        // sensor first transition has happened. Waiting for ending transition.
        boolean waitingForEndingTransition = false;

    public:
        /*
                Constructor

            sensorPin is the pin number of the target sensor
            ignoreTim is the number of milliseconds to ignore the sensor after a state change
            initialState is the expected start up state of the sensor for the CheckCycle function

        */
        CleanEdge(int sensorPin, unsigned int suspendTime, int initialState);

        // alternate constructor with initialState defaulting to HIGH.
        CleanEdge(int sensorPin, unsigned int suspendTime);

        /*
            Sample() function

            This function reads the input port the Sensor is connected to. But if
            the Sensor state changed within suspendWaitTime ago, the function just
            returns the current value.  Once this time has passed, the function
            resumes reading the port. The purpose of this is to bypass physical
            "bounce" in the switch contacts or other mechanical noise during the
            transition.

            Example call:

            SensorPressed = object.Sample();
        */
        int Sample(void);

        /*
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
        boolean CheckCycle(void);
};

#endif
