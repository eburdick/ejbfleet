/*
    ejbfleet-v1.ino

    Arduino code for Red Shift fleet bot challenge

    Following along with students to discover the problems they will be encountering and to
    get real experience I can use to help them.

    Modifications:

    3/1/21 2pm: adding this revision header. Code basically works on my test track.

    Next: Need to test on the real track. Need to add a mechanism for recognising
    the end of the course, assuming the robot gets there. Experiment with reading
    the line sensors as analog signals to provide more control over corrections.

    3/1/21 7pm: added code for test mode that reports line sensor digital and
    analog values when it discovers a digital state change.

    3/2/21 1:30am: Added code for analog sensing with software thresholds for
    the line sensors. The crossing line detection is still using digital
    sensing because it uses the clean edge class. May not change this. Added code
    for line counting and switch to standby at the end. Track
    testing seems to work.

    3/2/21 11pm: Fixed up the formatting some to make the code more readable, including
    ascii art block text. May do more later
*/
#define ANALOGSENSING //using analog outputs of line sensors and software thresholds.

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MPU6050.h>
#include "blinkled_class.hh"    //support code for blinking LEDs
#include "timer_class.hh"       //support code for non-blocking timers
#include "clean_edge_class.hh"  //support code for edge cleaned sensor reads

/****************************************************Tweakable times and thresholds*******/

//Blinking LED on and off times
long runLedOnTime  = 1000;
long runLedOffTime = 1000;
long pauseLedOnTime = 500;
long pauseLedOffTime = 500;

// Button debounce time window
unsigned int buttonDebounceDelay = 50;

// middle line sensor edge cleaning delay
const long middleLineSensorEdgeDelay = 20;

// delay for passing the start/finish line at the end
const long finishLineDelay = 1000;

// line sensor analog thresholds. Remember Dark is higher than light
int thresholdRightDark = 600;
int thresholdRightLight = 600;
int thresholdLeftDark = 600;
int thresholdLeftLight = 600;
int thresholdMiddleDark = 600;
int thresholdMiddleLight = 600;


// double line detection delays

//amount of time to look for second line of double line
//after finding the first line.
const int seekingSecondLineTimeWindow = 300; // .3 second

// amount of time to stay in pause mode
const int pauseModeDuration = 3000;  // 3 seconds

//delay after return from pause to run before seeking line again.
const int seekFirstLineBlockTime = 500;  // Half second

//motor speeds for line following. Negative is reverse, and fastSideSpeed is the
//straight drive speed of the robot.
const int slowSideSpeed = -130;       // fixed slow side speed for turning out of line error.
const int fastSideSpeed = 130;      // fixed high side speed for turning out of line error.

// Light sensor thresholds for turning the headlights on and off.
const int lightsOnThreshold = 3;
const int lightsOffThreshold = 9;


/****************************************************Hardware related constants**********/

// LED port definitions
const int ledPortYellow = 25;  // Standby LED
const int ledPortRed = 27;     // Stop flasher LED
const int ledPortGreen = 29;   // Run flasher LED
const int ledPortWhite = 23;   // Headlight LEDs

const int buttonPort = 52;     // Push button input port

// IR line sensor port definitions

const int digLineSensorPortRight = 53;
const int digLineSensorPortLeft = 49;

const int digLineSensorPortMiddle = 51; // crossing line sensor

// Analog ports (note A4 and A5 are used by I2C, so we can't use them for this.)

const int analogLightSensorPort = A0;    // for tunnel lights

const int analogLineSensorPortLeft = A1;
const int analogLineSensorPortMiddle = A2;
const int analogLineSensorPortRight = A3;

// Readability constants -- so we dont have to remember on, off, light, dark, pressed and unpressed

// LED state definitions
const int ledOn = LOW;
const int ledOff = HIGH;

// Define button states as physical positions
const int pressed = LOW;
const int unpressed = HIGH;

// Line sensor physical values
const int light = LOW;
const int dark = HIGH;

/**************************** C++ Object constructors ***************************************/

// Construct a motor shield object with the default I2C address
// and pointers to the four motors.

Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
Adafruit_DCMotor *RFMotor = MotorShield.getMotor(1);
Adafruit_DCMotor *RRMotor = MotorShield.getMotor(2);
Adafruit_DCMotor *LFMotor = MotorShield.getMotor(3);
Adafruit_DCMotor *LRMotor = MotorShield.getMotor(4);

// create blinking LED objects
BlinkLed runLed(ledPortGreen, runLedOnTime, runLedOffTime);
BlinkLed pauseLed(ledPortRed, pauseLedOnTime, pauseLedOffTime);

// create standby to run timer. This is to delay starting the
// run at the beginning so the user has time to push the
// button and leave the robot alone.
Timer standbyToRunTimer = Timer();

// create finish line timer to keep going a couple of seconds at the end.
Timer finishLineTimer = Timer();

// create pause timer. This is used to time pause mode.
Timer pauseTimer = Timer();

// create timers for crossing line and double line detection
Timer firstLineBlockTimer = Timer(); //timer for getting off of the line after a pause
Timer secondLineTimer = Timer();  //timer for establishing a window for detecting second line.

// CleanEdge object for button. Initial button state unpressed.
CleanEdge buttonReader = CleanEdge(buttonPort, buttonDebounceDelay, unpressed);

// CleanEdge object for the center line detector, used to find and count cross lines,
// avoiding multiple counts at messy edges.
CleanEdge centerLineSensorReader = CleanEdge(digLineSensorPortMiddle, middleLineSensorEdgeDelay, light);

/*******************************************************Working Global State********************/

// Line sensor values. These are used in setup() and loop()
int lineSensorValLeft;
int lineSensorValMiddle;
int lineSensorValRight;
int analogSensorValLeft;
int analogSensorValMiddle;
int analogSensorValRight;

// Previous sensor values. Used when we need to detect a change.
int prevLineSensorValLeft;
int prevLineSensorValMiddle;
int prevLineSensorValRight;

// Individual motor speed state.
int LFMotorSpeed;
int RFMotorSpeed;
int LRMotorSpeed;
int RRMotorSpeed;

// Initial motor speed (all four motors);
int initialMotorSpeed;

// state constants for cross line processing
const int seekingFirstLine = 0;
const int seekingSecondLine = 1;
const int seekingBlocked = 3;

const int totalDoubleLineCrossings = 4; //Crossing starting line twice and two crosswalks

int waitingForFinalStandby;  //set when we are crossing the finish line

int crossLineState;
int crossLineCount;
int doubleLineCount;

// Loop mode state variables
int mode;
const int modeTest = 1;       //test mode for testing stuff on the robot
const int modeStandby = 2;    //standby mode for before and after the timed run
const int modeRun = 3;        //run mode for the timed run
const int modePause = 4;      //pause mode for crosswalk stops while in run mode


/************************************************************Utility Functions******************/

//
// Functions for setting motor speeds on the left and right sides. These functions have a signed argument
// so the motors can be reversed on a negative sign.
//
// Note: We assume both left motors run the same speed and both right motors run the same speed. If this turns out
// not to be the case, some of this code will need to be changed.
//
void SetSpeedLeft(int speed)
{
    if (speed == 0)
    {
        LFMotor->run(RELEASE);
        LRMotor->run(RELEASE);
    }
    else if (speed > 0)
    {
        LFMotor->run(FORWARD);
        LRMotor->run(FORWARD);
    }
    else
    {
        LFMotor->run(BACKWARD);
        LRMotor->run(BACKWARD);
        speed = abs(speed);
    }    LFMotorSpeed = 80;
    LFMotorSpeed = speed;
    LRMotorSpeed = speed;
    LFMotor->setSpeed(LFMotorSpeed);
    LRMotor->setSpeed(LRMotorSpeed);
}

void SetSpeedRight(int speed)
{
    if (speed == 0)
    {
        RFMotor->run(RELEASE);
        RRMotor->run(RELEASE);
    }
    else if (speed > 0)
    {
        RFMotor->run(FORWARD);
        RRMotor->run(FORWARD);
    }
    else
    {
        RFMotor->run(BACKWARD);
        RRMotor->run(BACKWARD);
        speed = abs(speed);
    }
    RFMotorSpeed = speed;
    RRMotorSpeed = speed;
    RFMotor->setSpeed(RFMotorSpeed);
    RRMotor->setSpeed(RRMotorSpeed);
}



/*
    Mode switching functions. When switching into the modes. These make the one time changes at the beginning of
    each mode.
*/

// This function makes the transition to test mode, which does nothing but activate all of the LEDs at this point.
void ModeStartToTest()
{
    // Activate all of the LED indicators to indicate we are in test mode
    mode = modeTest;
    runLed.Enable();
    pauseLed.Enable();
    digitalWrite(ledPortYellow, ledOn);

    //read all of the line sensors to initialize their state and initialize previous state.
    //this is for sampling the analog values when the digital values change.
    lineSensorValLeft = digitalRead(digLineSensorPortLeft);
    lineSensorValMiddle = digitalRead(digLineSensorPortMiddle);
    lineSensorValRight = digitalRead(digLineSensorPortRight);
    analogSensorValLeft = analogRead(analogLineSensorPortLeft);
    analogSensorValMiddle = analogRead(analogLineSensorPortMiddle);
    analogSensorValRight = analogRead(analogLineSensorPortRight);

    prevLineSensorValLeft = lineSensorValLeft;
    prevLineSensorValMiddle = lineSensorValMiddle;
    prevLineSensorValRight = lineSensorValRight;
}

// Mode transition from start to standby. This is normally the starting mode.

void ModeStartToStandby()
{
    mode = modeStandby;
    runLed.Disable();
    pauseLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);

}

// Mode transition from run to standby
void ModeRunToStandby()
{
    mode = modeStandby;
    runLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);
}

// Mode transition from pause to standby
void ModePauseToStandby()
{
    mode = modeStandby;
    pauseLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);
}

// Mode transition from standby to run

void ModeStandbyToRun()
{
    mode = modeRun;
    crossLineState = seekingFirstLine;
    crossLineCount = 0;
    doubleLineCount = 0;
    runLed.Enable();
    digitalWrite(ledPortYellow, ledOff);
    waitingForFinalStandby = false;
}

// Mode transition from pause to run
void ModePauseToRun()
{
    mode = modeRun;
    runLed.Enable();
    pauseLed.Disable();
    firstLineBlockTimer.Start(seekFirstLineBlockTime);
}

// Mode transition from run to pause
void ModeRunToPause()
{
    mode = modePause;
    runLed.Disable();
    pauseLed.Enable();

    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);

    // Pause mode is 3 seconds long. Start timer. The timer will be polled during pause
    // mode in the main loop until the 3 seconds is past, then the transition back to
    // run mode will happen.

    pauseTimer.Start(pauseModeDuration); // set timer to 3 seconds
}

/*
    Line following support state
*/
// correction directons. Signed integer so we can have multiple values if needed.
const int none = 0;
const int right = 1;
const int left = -1;
int inCorrection = none;
int previousCorrection;



/*
      There are four system modes...

        Test mode (modeTest) is for testing sensors, etc. It is entered by holding down the button
        during startup. Details TBD

        Standby mode (modeStandby) is automatically entered at power on if the button is not being held down.
        In standby, the yellow light is on to indicate the mode and all motors are stopped. Pushing the button
        while in standby mode starts run mode.

        Run mode (modeRun) is entered when the button is pushed while in standby mode. In this mode, the green light is
        flashing and the autonomous
        line following challenge is run until the end line is detected or the button is pushed, at which point the
        mode switches back to standby mode. This mode also looks for a double line across the path and if it finds it, we
        enter pause mode for three seconds.

        Pause mode (modePause) is entered from run mode. The motors stop, the red light flashes for .5 seconds on
        and .5 seconds off for 3 seconds, then goes back run mode.

        For transitions between modes, there is a function for each type of transition; standby to run, run to pause,
        run to standby, pause to run, pause to standby. These are called in the main loop when a mode needs to change,
        and they provide a place for any action that needs to take place just once during a transition.

        Mode state variable and constants...
*/
/*                                        __  _ ___     _
                                         (_  |_  | | | |_)
                                         __) |_  | |_| |
*/
//
// Initialize state, set up hardware, prepare starting state
//
void setup()
{
    // Start serial port for debugging
    Serial.begin(115200); // open the serial port at 115200 bps:

    //
    //  set initial motor speed
    //
    initialMotorSpeed = 80;

    //set up the LED ports
    pinMode(ledPortWhite, OUTPUT);
    pinMode(ledPortYellow, OUTPUT);
    pinMode(ledPortGreen, OUTPUT);
    pinMode(ledPortRed, OUTPUT);

    //set up the button port
    pinMode(buttonPort, INPUT_PULLUP);

    // Start motor shield
    MotorShield.begin();

    // initialize inCorrection
    inCorrection = none;
    previousCorrection = none;

    //
    // Read the button. If the button is pressed, set the mode to modeTest.
    // if not pressed, set the mode to modeStandby. The test mode puts the system in test mode, which is for
    // checking out the sensors, motors, and LEDs.
    //
    if (buttonReader.Sample() == pressed)
    {
        //        EnterModeTest();
        ModeStartToTest();
    }
    else
    {
        ModeStartToStandby();
    }
}
/*                                         _   _   _
                                       |  / \ / \ |_)
                                       |_ \_/ \_/ |
*/
void loop()
{
    /*                                      _   _   _  __    _  _   _   _
                          /\  |  |    |\/| / \ | \ |_ (_    /  / \ | \ |_
                         /--\ |_ |_   |  | \_/ |_/ |_ __)   \_ \_/ |_/ |_
    */

    /*
        read IR line sensors, digital version. This just reads the digital value
        from the sensor, which sets its threshold with a trim pot.
    */
#ifndef ANALOGSENSING
    lineSensorValLeft = digitalRead(digLineSensorPortLeft);
    lineSensorValMiddle = digitalRead(digLineSensorPortMiddle);
    lineSensorValRight = digitalRead(digLineSensorPortRight);
#endif

#ifdef ANALOGSENSING
    /*
        read IR line sensor, analog version. This reads the analog value of the
        sensor and compares it with the analog thresholds. There is a threshold
        for dark and a threshold for light, which gives us a chance to create
        hysteresis for each sensor.

        First, read the sensor analog values
    */
    analogSensorValLeft = analogRead(analogLineSensorPortLeft);
    analogSensorValMiddle = analogRead(analogLineSensorPortMiddle);
    analogSensorValRight = analogRead(analogLineSensorPortRight);

    /*
        Test each sensor value against the light threshold and the dark
        threshold. Note that the analog value goes up as the reflected
        light is darker. If the sensor output is higher than the dark
        threshold, set the digital state for the sensor to dark. If it
        is lower than the light threshold, set it to light. If there is
        a gap between the thresholds, there is hysteresis. If the thresholds
        are equal, there will be no hysteresis. If the dark threshold is
        less than the light threshold, then the dark threshold is
        is effectively the only threshold, because if the dark threshold
        test fails, the light threshold will always succeed.
    */
    // left sensor code
    if (analogSensorValLeft > thresholdLeftDark)
    {
        lineSensorValLeft = dark;
    }
    else if (analogSensorValLeft <= thresholdLeftLight)
    {
        lineSensorValLeft = light;
    }
    else
    {
        // no change. Keep the previous value. This is the hysteresis
        // dead zone.
    }

    // middle sensor code
    if (analogSensorValMiddle > thresholdMiddleDark)
    {
        lineSensorValMiddle = dark;
    }
    else if (analogSensorValMiddle <= thresholdMiddleLight)
    {
        lineSensorValMiddle = light;
    }
    else
    {
        // no change. Keep the previous value. This is the hysteresis
        // dead zone.
    }

    // right sensor code
    if (analogSensorValRight > thresholdRightDark)
    {
        lineSensorValRight = dark;
    }
    else if (analogSensorValRight <= thresholdRightLight)
    {
        lineSensorValRight = light;
    }
    else
    {
        // no change. Keep the previous value. This is the hysteresis
        // dead zone.
    }
#endif

    // resd ambient light sensor. Turn on headlights on at the low light threshold and off
    // at the high light threshold
    //
    int lightLevel = analogRead(analogLightSensorPort);

    if (lightLevel < lightsOnThreshold)
    {
        digitalWrite(ledPortWhite, ledOn);
    }

    if (lightLevel > lightsOffThreshold)
    {
        digitalWrite(ledPortWhite, ledOff);
    }

    /*  debug/calibration code
        Serial.print("light level: ");
        Serial.println(lightLevel);
    */

    /*                    _                   _   _   _    _  _   _   _
                         |_) | | |\ |   |\/| / \ | \ |_   /  / \ | \ |_
                         | \ |_| | \|   |  | \_/ |_/ |_   \_ \_/ |_/ |_
    */
    if (mode == modeRun)
    {
        // update LED flasher
        runLed.Update();


        /*
            ************************************Line following algorithm************************

            There are two line sensors, one on the front right
            corner and one on the front left corner. When one of these sensors sees a line,
            it sets the wheels on the opposite side to slow way down or reverse, which causes
            the robot to turn away from the line. When the sensor stops seeing the line,
            those wheels return to normal forward rotation. There are a few special cases...

            - At places where the road crosses itself, there is a line that goes across the
            road. At that point, both line sensors will see a line. When that happens, we do
            not correct direction, but continue to go straight. This depends on the robot not
            being at a significant angle when it encounters a cross line.

            - For cross lines, we have some special cases.
                1. If all three sensors (left right, middle) are dark, we assume we are crossing
                the line. straight on. In this case, we do not correct.
                2. If middle and right are dark, we assume we are hitting the cross line aiming left,
                so we correct by turning right.
                3. if middle and left are dark, we assume we are hitting the cross line aiming right,
                so we correct by turning left.
        */

        if (lineSensorValLeft == dark && lineSensorValRight == dark)
        {
            // assumption is that if we see both sensors dark, then we are at a cross line and the
            // robot is going straight.
            inCorrection = none;
        }
        else if (lineSensorValLeft == dark && lineSensorValMiddle == light)
        {
            // road edge detected. Turn toward the center of the road.
            inCorrection = left;
        }
        else if (lineSensorValRight == dark && lineSensorValMiddle == light) /*|| lineSensorValRightOuter == dark*/
        {
            // road edge detected. Turn toward the center of the road
            inCorrection = right;
        }
        else if (lineSensorValRight == dark && lineSensorValMiddle == dark)
        {
            // crossing line aimed left. correct as if left sensor detected the left road edge
            inCorrection = left;
        }
        else if (lineSensorValLeft == dark && lineSensorValMiddle == dark)
            // crossing line aimed right. Correct as if right sensor detected the right road edge
            inCorrection = right;
        else
        {
            inCorrection = none;
        }


        //respond to error condition
        if (inCorrection == left) // Correcting for left side line detection
        {
            // line has been detected on the left side. We want to turn right to clear the line
            SetSpeedRight(slowSideSpeed);
            SetSpeedLeft(fastSideSpeed);
        }
        else if (inCorrection == right) // Correcting for right side line detection
        {
            // line has been detected on the right side. We want ato turn left to clear the line
            SetSpeedRight (fastSideSpeed);
            SetSpeedLeft (slowSideSpeed);
        }
        else if (inCorrection == none)//no correction needed
        {
            // just go straight
            SetSpeedRight (fastSideSpeed);
            SetSpeedLeft (fastSideSpeed);
        }

        /*
            ****************************Line counting and crosswalk detection***************************

            This code counts the lines crossed by the robot. Where the roadway crosses itself, we see lines
            crossing the road. At these points, we need to ignore the line following sensors, because the
            crossing lines are obscuring the boundaries at that point. At two points on the course, there is
            an extra line before the line that marks the edge of the roadway, spaced one inch away from the
            crossing roadway boundary. These positions are designated as crosswalks and the robot needs to go
            into pause mode for three seconds at these points. There is also a double line at the end of the course,
            where we need to go into standby mode.

            Cross walk detection. We use our cleanEdge object, centerLineSensorReader, to detect a
            light-dark-light cycle, then we look for another light-dark transition within a short time later.
            At this point, we should be just past a double cross line. Sequence of operations...
        */

        /*
            There are three possibilities at this point:

            1...We are looking for a crossing line, either the first of a double line, or a crossing roadway.
            pauseReturnDelay is false
            firstLineDetected is false
            crossLineState = seekingFirstLine

            2...We have found a line and we are checking for a second line close to it.
            firstLineDetected is true
            pauseReturnDelay is false
            crossLineState = seekingSecondLine

            3...We have recently returned from a pause due to finding a double line and need to move off of it.
            firstLineDetected is false
            pauseReturnDelay is true
            crossLineState = seekingBlocked

        */
        if (crossLineState == seekingFirstLine)
        {
            //we are looking for a crossing line. We will check the sensor for a light-dark-light cycle
            if (centerLineSensorReader.CheckCycle())
            {
                // we just detected a light-dark-light sequence. This means we have passed one line.
                // At this point, we set a timer and start looking for the next line until the time
                // is up. The timer gives us enough time to detect a second line close to the first
                // one, but not long enough to detect a line furthur down the road as the second line
                // in a double line

                // count the line
                crossLineCount++;

                // update state to start seeking second line and start the time window
                crossLineState = seekingSecondLine;
                secondLineTimer.Start(seekingSecondLineTimeWindow); //open time window for finding a second line

                //Serial.println("First line detected");
                //Serial.println(crossLineCount);

            }
            else
            {
                // no line detected. Do nothing.
            }
        }
        else if (crossLineState == seekingSecondLine)
        {
            if (!secondLineTimer.Test())
            {
                //second line time window is still open. Read sensor for second line. Just looking for
                //light-dark transition here because we want to switch to pause mode right away if we
                //see it.

                if (centerLineSensorReader.Sample() == dark)
                {
                    crossLineCount++; // count the second line
                    doubleLineCount++; // count the double line

                    if (doubleLineCount == 1)
                    {
                        // Stay in run mode. This is the start of the course. But set the
                        // first line detect timeout to keep from detecting this line again.
                        firstLineBlockTimer.Start(seekFirstLineBlockTime);
                        crossLineState = seekingBlocked;
                    }
                    else if (doubleLineCount == totalDoubleLineCrossings)
                    {
                        // double line is the start/finish line. Set the finish line timer and
                        // the state flag indicating we are at the finish line. When the timer
                        // has expired, we will go into standby mode
                        finishLineTimer.Start(finishLineDelay);
                        waitingForFinalStandby = true;
                        //
                        // Just to make sure we stop looking for crosslines and avoid finding
                        // this one again on the next time around the loop, we block line seeking
                        // and start first line block timer.
                        firstLineBlockTimer.Start(seekFirstLineBlockTime);
                        crossLineState = seekingBlocked;
                    }
                    else if (doubleLineCount < totalDoubleLineCrossings)
                    {
                        // double line is a crosswalk. Go to mode pause
                        ModeRunToPause(); // switch mode to pause. Next call to loop() will be in pause state;

                        crossLineState = seekingBlocked; // once we get back from pause mode, will block the sensor for a while.
                    }


                    //Serial.println("Second line detected");
                    //Serial.println(crossLineCount);
                }
            }
            else
            {
                //second line time window has expired. This means we have failed to find a second line,
                //close enough to the first one, so there was only one line at this location.

                crossLineState = seekingFirstLine; //start looking for the next line

                //Serial.println("Second line not detected");
                //Serial.println(crossLineCount);
            }
        }
        else if (firstLineBlockTimer.Test()) // crossLineState == seekingBlocked
        {
            // here we are not seeking the first or second line, but the timer we started to avoid double
            // counting the second line has expired, so we can start looking for a first line again.
            crossLineState = seekingFirstLine;

            //Serial.println("firstLine Block complete");
        }
        else
        {
            // if we fall through to here, we have left pause mode, but are still waiting to start
            // looking for lines again, so we do nothing. This will keep happening until the timer
            // expires.
        }
        /*
            Code past this point has calls to mode transitions. We put this at the end because the
            transition functions called will return to here before the next mode is started, and
            we don't want to execute any more consequential run mode code after this return.
        */

        /***********************************Button Processing********************************/
        //
        // Look for a button press and release. If we see it, then switch to standby mode.
        //
        if (buttonReader.CheckCycle())      // when the button is pressed and released, we go to standby mode
        {
            //Serial.println("run to standby");
            ModeRunToStandby();
        }

        /***********************************Finish Line Processing***************************/
        // check if we have passed the finish line. and the fiish line timer has expired.
        // We want to run long enough after
        // detecting the finish line to fully pass it before going into pause mode.
        if (waitingForFinalStandby && finishLineTimer.Test())
        {
            //Serial.println("finish line");
            ModeRunToStandby();
        }
    } // end of run
    /*                       ___ _  __ ___         _   _   _    _  _   _   _
                              | |_ (_   |    |\/| / \ | \ |_   /  / \ | \ |_
                              | |_ __)  |    |  | \_/ |_/ |_   \_ \_/ |_/ |_
    */
    else if (mode == modeTest)
    {
        //
        // code for test mode
        //
        /*
            Code for finding analog thresholds of line sensors. The hardware thresholds on the sensors are
            adjusted via trimpots on the boards. This code watches each sensor's digital output and when it sees
            a change, it reads the corresponing analog input and reports the transition and value via the
            serial monitor. All of the sensors will have already been read at this point. We just have to recognise
            that there has been a change since the last read and report both values when there is.
        */
        char stringBuffer[80];  // character array to assemble formatted strings using sprintf()

        // Check each sensor digital state and report changes to the serial terminal along with the
        // analog values.
        if (prevLineSensorValLeft != lineSensorValLeft)
        {
            // report both the left and the right sensor data
            sprintf(stringBuffer, "L Sensor %d -> %d Analog %d", prevLineSensorValLeft, lineSensorValLeft, analogSensorValLeft);
            Serial.println(stringBuffer);
            sprintf(stringBuffer, "R Sensor        %d Analog %d", lineSensorValRight, analogSensorValRight);
            Serial.println(stringBuffer);
            prevLineSensorValLeft = lineSensorValLeft;
        }

        if (prevLineSensorValMiddle != lineSensorValMiddle)
        {
            sprintf(stringBuffer, "M Sensor %d -> %d Analog %d", prevLineSensorValMiddle, lineSensorValMiddle, analogSensorValMiddle);
            Serial.println(stringBuffer);
            prevLineSensorValMiddle = lineSensorValMiddle;
        }

        if (prevLineSensorValRight != lineSensorValRight)
        {
            // report both the left and the right sensor data
            sprintf(stringBuffer, "R Sensor %d -> %d Analog %d", prevLineSensorValRight, lineSensorValRight, analogSensorValRight);
            Serial.println(stringBuffer);
            prevLineSensorValRight = lineSensorValRight;
            sprintf(stringBuffer, "L Sensor        %d Analog %d", lineSensorValLeft, analogSensorValLeft);
            Serial.println(stringBuffer);
        }


    } // end of test
    /*                       __ ___           _    _              _   _   _    _  _   _   _
                            (_   |  /\  |\ | | \  |_) \_/   |\/| / \ | \ |_   /  / \ | \ |_
                            __)  | /--\ | \| |_/  |_)  |    |  | \_/ |_/ |_   \_ \_/ |_/ |_
    */
    else if (mode == modeStandby)
    {
        //
        // Code for standby mode.
        //
        // At this point, the Yellow LED has been set by EnterStandbyMode(), and the motors are off.
        // The only thing this mode does is wait for the button to be cycled (pressed and released,) then it starts
        // a timer. When the timer period is finished, we go to run mode.
        //
        static boolean countdownToRun = false;  //static variable to hold state between calls to loop()
        if (countdownToRun)
        {
            if (standbyToRunTimer.Test())  // poll the timer. When it is finished, enter run mode
            {
                countdownToRun = false;
                //Serial.println("entering run mode");
                ModeStandbyToRun();
            }
        }
        else if (buttonReader.CheckCycle())      // when the button is pressed and released, we go to run mode after a delay.
        {
            // Note: CheckButtonCycle returns true only once per button cycle.
            countdownToRun = true;                    //flag that we are in the countdown to run
            standbyToRunTimer.Start(2000);       //Start pause to run timer for 2 second countdown
            //Serial.println("standby to run start delay");
        }
        else
        {
            // do nothing while we wait for button cycle
        }
    } // end of standby

    /*                         _           __  _         _   _   _    _  _   _   _
                              |_) /\  | | (_  |_   |\/| / \ | \ |_   /  / \ | \ |_
                              |  /--\ |_| __) |_   |  | \_/ |_/ |_   \_ \_/ |_/ |_
    */

    else if (mode == modePause)
    {
        //
        // Pause mode code
        //
        pauseLed.Update();                       // update pause LED flasher object

        boolean pauseDone = pauseTimer.Test();   //poll the pause timer
        if (pauseDone)
        {
            ModePauseToRun();
            //Serial.println("pause to run");
        }
        //
        // For the rare case that the button is pressed during pause mode:
        // Look for a button press and release. If we see it, then switch to standby mode.
        //
        if (buttonReader.CheckCycle())      // when the button is pressed and released, we go to standby mode
        {
            //Serial.println("pause to standby");
            ModePauseToStandby();
        }
    } // end of  pause
} // end of loop()
