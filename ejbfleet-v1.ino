
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MPU6050.h>
#include "blinkled_class.hh"    //support code for blinking LEDs
#include "timer_class.hh"       //support code for non-blocking timers
#include "clean_edge_class.hh"  //support code for edge cleaned sensor reads

// Define button states as physical positions
const int pressed = LOW;
const int unpressed = HIGH;

// Construct a motor shield object with the default I2C address
// and pointers to the four motors.
Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
Adafruit_DCMotor *RFMotor = MotorShield.getMotor(1);
Adafruit_DCMotor *RRMotor = MotorShield.getMotor(2);
Adafruit_DCMotor *LFMotor = MotorShield.getMotor(3);
Adafruit_DCMotor *LRMotor = MotorShield.getMotor(4);

// Individual motor speed state.
int LFMotorSpeed;
int RFMotorSpeed;
int LRMotorSpeed;
int RRMotorSpeed;

// Initial motor speed (all four motors);
int initialMotorSpeed;

// LED port definitions
const int ledPortYellow = 25;  // Standby LED
const int ledPortRed = 27;     // Stop flasher LED
const int ledPortGreen = 29;   // Run flasher LED
const int ledPortWhite = 23;   // Headlight LEDs

// LED state definitions
const int ledOn = LOW;
const int ledOff = HIGH;

// IR line sensor port definitions
const int digLineSensorPortRight = 53;

const int digLineSensorPortLeft = 49;
const int digLineSensorPortRightOuter = 47;
const int digLineSensorPortLeftOuter = 45;
const int algLineSensorPortLeft = A1;
const int algLineSensorPortLeftOuter = A6;
const int algLineSensorPortMiddle = A2;
const int algLineSensorPortRight = A3;
const int algLineSensorPortRightOuter = A7;
// note A4 and A5 are used by I2C, so we can't use them for this.

// Line sensor physical values
const int light = LOW;
const int dark = HIGH;

// Light sensor thresholds.
const int algLightSensorPort = A0;
const int lightsOnThreshold = 3;
const int lightsOffThreshold = 9;

// Line sensor analog thresholds. Each sensor has a dark threshold and a light threshold.
// The dark threshold is the voltage at which we switch from light state to dark state.
// The light threshold is the voltage at which we switch from dark state to light state.
// These are different because when we switch states from light to dark, we don't want to
// switch back with a small change in light level and vice versa. Adding hysteresis through
// this method should prevent this issue.

const int algDarkLineSensorThdRt =  500;
const int algDarkLineSensorThdRtOuter =  500;
const int algDarkLineSensorThdLf =   500;
const int algDarkLineSensorThdLfOuter =   500;
const int algLightLineSensorThdRt =   500;
const int algLightLineSensorThdRtOuter =   500;
const int algLightLineSensorThdLf =   500;
const int algLightLineSensorThdLfOuter =   500;

// create blinking LED objects
BlinkLed runLed(ledPortGreen, 1000, 1000); //two second cycle
BlinkLed pauseLed(ledPortRed, 500, 500);   //one half second on, one half second off


// create pause timer. This is used to time pause mode.
Timer pauseTimer = Timer();

// create timers for crossing line and double line detection
Timer firstLineBlockTimer = Timer(); //timer for getting off of the line after a pause
Timer secondLineTimer = Timer();  //timer for establishing a window for detecting second line.
Timer correctionTimer = Timer();  //timer for extending short line sensor corrections.
const int correctionMinTime = 0;

// state constants for cross line processing
const int seekingFirstLine = 0;
const int seekingSecondLine = 1;
const int seekingBlocked = 3;
int crossLineState;
int crossLineCount;
const int seekingSecondLineTimeWindow = 200; //amount of time to look for second line of double line.
const int seekFirstLineBlockTime = 1000; //delay after return from pause to run before seeking line again.

// create standby to run timer. This is to delay the start of the run until after the user's
// hand is clear of the button
// Timer pauseToRunTimer = Timer();

// create standby to run timer. This is to delay starting the run at the beginning so the user has time to push the
// button and leave the robot alone.
Timer standbyToRunTimer = Timer();

const int buttonPort = 52;     // Push button input port
// CleanEdge object for button. 50mS for debounce time, initial button state unpressed.
CleanEdge buttonReader = CleanEdge(buttonPort, 50, unpressed);

const int digLineSensorPortMiddle = 51;
// CleanEdge object for the center line detector, used to find and count cross lines
CleanEdge centerLineSensorReader = CleanEdge(digLineSensorPortMiddle, 20, light);
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
int mode;
const int modeTest = 1;       //test mode for testing stuff on the robot
const int modeStandby = 2;    //standby mode for before and after the timed run
const int modeRun = 3;        //run mode for the timed run
const int modePause = 4;      //pause mode for crosswalk stops while in run mode
/*
    Mode switching functions. When switching into the modes. These make the one time changes at the beginning of
    each mode.
*/

// This function makes the transition to test mode, which does nothing but activate all of the LEDs at this point.
void ModeStartToTest()
{
    mode = modeTest;
    runLed.Enable();
    pauseLed.Enable();
    digitalWrite(ledPortYellow, ledOn);
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
    runLed.Enable();
    digitalWrite(ledPortYellow, ledOff);
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
    pauseTimer.Start(3000); // set timer to 3 seconds
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

const int slowSideSpeed = -130;       // fixed slow side speed for turning out of line error.
const int fastSideSpeed = 130;      // fixed high side speed for turning out of line error.



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
        ModeStartToStandby();
    }
    else
    {
        ModeStartToStandby();
    }
}

// Main loop. Poll sensors, execute mode specific code.
//
void loop()
{
    static unsigned oldTime;
    unsigned currentTime = millis();
    //Serial.println(currentTime - oldTime);
    oldTime = currentTime;
    //
    // Code for all modes reads the sensors, and turns on the headlights if it is dark.
    //
    // read IR line sensors
    //
    int digSensorValLeft = digitalRead(digLineSensorPortLeft);
    int digSensorValLeftOuter = digitalRead(digLineSensorPortLeftOuter);
    int digSensorValMiddle = digitalRead(digLineSensorPortMiddle);
    int digSensorValRight = digitalRead(digLineSensorPortRight);
    int digSensorValRightOuter = digitalRead(digLineSensorPortRightOuter);
    int algSensorValLeft = analogRead(algLineSensorPortLeft);
    int algSensorValLeftOuter = analogRead(algLineSensorPortLeftOuter);
    int algSensorValMiddle = analogRead(algLineSensorPortMiddle);
    int algSensorValRight = analogRead(algLineSensorPortRight);
    int algSensorValRightOuter = analogRead(algLineSensorPortRightOuter);

    /*  debug code
        Serial.print("left digital ");
        Serial.println(digSensorValLeft);
        Serial.print("mid digital ");
        Serial.println(digSensorValMiddle);
        Serial.print("right digital ");
        Serial.println(digSensorValRight);
        Serial.println("");
        Serial.print("left analog ");
        Serial.println(algSensorValLeft);
        Serial.print("mid analog ");
        Serial.println(algSensorValMiddle);
        Serial.print("right analog ");
        Serial.println(algSensorValRight);
        delay(500);
    */

    // resd ambient light sensor. Turn on headlights on at the low light threshold and off
    // at the high light threshold
    //
    int lightLevel = analogRead(algLightSensorPort);

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

    //
    // Run mode code.
    //
    if (mode == modeRun)
    {
        // update LED flasher
        runLed.Update();

        /*
            Line following algorithm v1; There are two line sensors, one on the front right
            corner and one on the front left corner. When one of these sensors sees a line,
            it sets the wheels on the opposite side to slow way down or reverse, which causes
            the robot to turn away from the line. When the sensor stops seeing the line,
            those wheels return to normal forward rotation. There are a few special cases...

            - At places where the road crosses itself, there is a line that goes across the
            road. At that point, both line sensors will see a line. When that happens, we do
            not correct direction, but continue to go straight.

            - We also do not correct if the
            sensor in the center between the right and left sensors sees a line.

            How do we handle the case where the robot crosses the road boundary because the
            line sensor crosses the line at too steep a boundary and doesn't have time to
            correct? This case limits the top speed of the robot, because once the sensor is
            on the other side of the line, there is no chance to get back via the original
            algorithm. Detecting this case:

            - the crossing is fast, so the sensor will see dark for a shorter time than
            the normal case.

            - the center sensor will see the line shortly after the crossing, because the
            line will be at an angle to the front of the robot.

            Possible recovery algorithm:

            - detect the single short blip followed by the center sensor hit and reverse the
            robot until the line is detected passing under (light-dark-light cycle) the same
            corner sensor, then resume normal operation. The robot will immediatly correct
            because it will be going slow.

            Alternate recovery algorithm:

            -set a minumum correction time. This makes the line look wider and pulls the
            sensor back to the real line. Whether this works will depend on whether it
            results in over correction in line grazing situations.
        */
        //Serial.println(correctionTimer.Test());
        //Serial.println(correctionTimer.TimeSinceStart());
        //Serial.println(test);
        //        if (correctionTimer.Test())
        //        {
        // detect path edge error condition
        //
        if (digSensorValLeft == dark && digSensorValRight == dark)
        {
            // if both line sensors detect a line, we are at a cross line. No correction necessary.
            inCorrection = none;
        }
        else if (digSensorValLeft == dark && digSensorValMiddle == light)
        {
            inCorrection = left;
        }
        else if (digSensorValRight == dark && digSensorValMiddle == light) /*|| digSensorValRightOuter == dark*/
        {
            inCorrection = right;
        }
        else
        {
            inCorrection = none;
        }
        //     }
        // if the correction value has changed, start the correction timer. While the timer
        // is running, we will not look at the sensor values. This is specifically to extend
        // very short line detections, especially when the robot fully crosses a roadway
        // boundary.
        /*
            if (previousCorrection != inCorrection)
            {
            //Serial.println("changed Correction");
            //Serial.print("Prev:");
            //Serial.print(previousCorrection);
            //Serial.print("New: ");
            //Serial.println(inCorrection);
            correctionTimer.Start(correctionMinTime);
            previousCorrection = inCorrection;
            }
        */


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
            Line counting and crosswalk detection

            This code counts the lines crossed by the robot. Where the roadway crosses itself, we see lines
            crossing the road. At these points, we need to ignore the line following sensors, because the
            crossing lines are obscuring the boundaries at that point. At two points on the course, there is
            an extra line before the line that marks the edge of the roadway, spaced one inch away from the
            crossing roadway boundary. These positions are designated as crosswalks and the robot needs to go
            into pause mode for three seconds at these points. There is also a double line at the end of the course,
            where we need to go into standby mode.

            The first crosswalk is in the middle of a straight section of the road, but the second one is at the end
            of a tight curve, so the robot will be doing course corrections, so we cannot assume it is going the same
            speed as on a straight road.

            Candidate for how to detect a double line, assuming there is a center sensor:

            Time the crossing time for the first line and look for another crossing within a similar amount of time. This only
            works if course corrections do not result in entry and exit on the leading edges of the lines. This could probably be
            mitigated with some dead time after the initial detection of each edge.

            We also want to count line crossing so we know where we are on the course, especially at the end, though we could
            just count crosswalks instead.



            /*
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
                // one, but not long enough to detect a line future down the road as the second line
                // in a double line
                crossLineState = seekingSecondLine;
                secondLineTimer.Start(seekingSecondLineTimeWindow); //open time window for finding a second line
                crossLineCount++;
                Serial.println("First line detected");
                Serial.println(crossLineCount);

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
                //light-dark transition here because we want to switch to pause mode right away.

                if (centerLineSensorReader.Sample() == dark)
                {
                    ModeRunToPause(); // switch mode to pause. Next call to loop() will be in pause state;
                    crossLineState = seekingBlocked; // once we get back from pause mode, will block the sensor for a while.
                    crossLineCount++;
                    Serial.println("Second line detected");
                    Serial.println(crossLineCount);

                }
            }
            else
            {
                //second line time window has expired. This means we have failed to find a second line,
                //so there was only one line at this location.
                crossLineState = seekingFirstLine; //start looking for the next line
                Serial.println("Second line not detected");
                Serial.println(crossLineCount);
            }
        }
        else if (firstLineBlockTimer.Test()) // crossLineState == seekingBlocked
        {
            crossLineState = seekingFirstLine; //block timer has expired, so we start looking for a line again
            Serial.println("firstLine Block complete");
        }



        //
        // Look for a button press and release. If we see it, then switch to standby mode.
        //
        if (buttonReader.CheckCycle())      // when the button is pressed and released, we go to standby mode
        {
            Serial.println("run to standby");
            ModeRunToStandby();
        }
    }

    else if (mode == modeTest)
    {
        //
        // code for test mode empty for now
        //
    }

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
                Serial.println("entering run mode");
                ModeStandbyToRun();
            }
        }
        else if (buttonReader.CheckCycle())      // when the button is pressed and released, we go to run mode after a delay.
        {
            // Note: CheckButtonCycle returns true only once per button cycle.
            countdownToRun = true;                    //flag that we are in the countdown to run
            standbyToRunTimer.Start(2000);       //Start pause to run timer for 2 second countdown
            Serial.println("standby to run start delay");
        }
        else
        {
            // do nothing while we wait for button cycle
        }
    }
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
            Serial.println("pause to run");
        }
        //
        // For the rare case that the button is pressed during pause mode:
        // Look for a button press and release. If we see it, then switch to standby mode.
        //
        if (buttonReader.CheckCycle())      // when the button is pressed and released, we go to standby mode
        {
            Serial.println("pause to standby");
            ModePauseToStandby();
        }
    } // end of  pause
} // end of loop()
