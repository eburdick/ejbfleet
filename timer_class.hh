#ifndef timer_class_hh
#define timer_class_hh

/*
    Timer class

    This provides a mechanism for delaying without stopping the program. We take a snapshot from Millis(),
    add the requested delay to that, and test for the completion of that delay with each update.

    Timer myTimer = Timer();        //constructor, called outside of setup() or loop(). This is called for
                                    //each timer to be created.

    void myTimer.Start();           //start timer with no specifed delay. Test will always return false.
    long myTimer.TimeSinceStart;    //find out how many milliseconds have passed since start.
    
    void myTimer.Start(long delay); //start timer with desired delay
    boolean myTimer.Test();         //check if there has been delay milliseconds since timer was started.
*/

class Timer
{
        // object variable

        long endTime;              // caller specified end time
        long startTime;            // snapshot of when timer was started
        boolean endTimeExists;     // true if timer started with an end time

        // Constructor
    public:
        Timer (void);
        void Start(void);
        void Start(int delay);
        boolean Test(void);
        long TimeSinceStart(void);
};
#endif
