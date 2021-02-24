
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
const int digLineSensorPortMiddle = 51;
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

// create cross walk timer. This is used to allow time to leave the crosswalk after a pause.
Timer crossWalkTimer = Timer();

// create standby to run timer. This is to delay the start of the run until after the user's
// hand is clear of the button
// Timer pauseToRunTimer = Timer();

// create standby to run timer. This is to delay starting the run at the beginning so the user has time to push the
// button and leave the robot alone.
Timer standbyToRunTimer = Timer();

const int buttonPort = 52;     // Push button input port

// CleanEdge object for button. 50mS for debounce time, initial button state unpressed.
CleanEdge buttonReader = CleanEdge(buttonPort, 50, unpressed);

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
void StartModeTest()
{
    mode = modeTest;
    runLed.Enable();
    pauseLed.Enable();
    digitalWrite(ledPortYellow, ledOn);
}

// These functiona make the transition to standby mode, setting the mode state variable to standby,
// turning off all flashing LEDs and lighting the standby LED. Then it stops the motors.

void StartModeStandby()
{
    mode = modeStandby;
    runLed.Disable();
    pauseLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);
}

void ModeRunToStandby()
{
    mode = modeStandby;
    runLed.Disable();
    pauseLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);
}

// This function makes the transition to run mode, setting the mode state variable to run, turning off the
// standby LED, disbling the pause flasher, and starting the run flasher. TEMPORARY: It also starts the crosswalk timer to
// disable looking for crosswalks for a couple of seconds.
// This function make the transition to pause mode, setting up the LEDs for the mode, stopping the motors, and starting
// the pause mode timer to three seconds. The timer will be polled in the main loop pause code.

/*
void EnterModePause()
{
    mode = modePause;
    runLed.Disable();
    pauseLed.Enable();
    digitalWrite(ledPortYellow, ledOff);

    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);

    // Pause mode is 3 seconds long. Start timer.
    pauseTimer.Start(3000); // set timer to 3 seconds
}
*/
void ModeRunToPause()
{
    mode = modePause;
    runLed.Disable();
    pauseLed.Enable();
    digitalWrite(ledPortYellow, ledOff);

    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);

    // Pause mode is 3 seconds long. Start timer.
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

    //
    // Read the button. If the button is pressed, set the mode to modeTest.
    // if not pressed, set the mode to modeStandby. The test mode puts the system in test mode, which is for
    // checking out the sensors, motors, and LEDs.
    //
    if (buttonReader.Sample() == pressed)
    {
        StartModeTest();
    }
    else
    {
        StartModeStandby();
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
            Line following algoritm 1: Very simple on/off turns. Just to see how tight a turn this can deal with.
            Run all wheels at initial speed. When a line is detected via the digial sensor output, slow down the wheels
            on the opposite side to a fixed value and keep running at that speed until the sensor clears the line. Experiment
            with the speed values. I expect this will work as tuned up to the harder turns. There also needs to be
            an exception when a crossing line is detected, which will be with the center sensor an inch or so before the side
            sensors see it, so we can try just ignoring those until the cross lines are behind us. If this happens in a turn,
            this may not work.
        */

        //detect path edge error condition
        //
        if (digSensorValLeft == dark)
        {
            inCorrection = left;
        }
        else if (digSensorValRight == dark /*|| digSensorValRightOuter == dark*/)
        {
            inCorrection = right;
        }
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

        */
        //
        // Temporary cross walk detection. This just looks for a black line with the center sensor
        // and switches to pause mode after testing whether the test is enabled (resumeCrossWalkTest).
        // When control comes back here from pause mode, a timer is set to give the robot time to
        // move on past the black line that triggered the switch into pause mode. We test that timer
        // here until it expires, then re enable the cross walk test.
        //
        boolean resumeCrossWalkTest = crossWalkTimer.Test();   //poll the cross walk timer

        if (resumeCrossWalkTest)
        {
            if (digSensorValMiddle == dark)
            {
                Serial.println("run to pause");
                ModeRunToPause();
            }
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
            EnterModeRun();
            Serial.println("pause to run");
        }
        //
        // For the rare case that the button is pressed during pause mode:
        // Look for a button press and release. If we see it, then switch to standby mode.
        //
        if (buttonReader.CheckCycle())      // when the button is pressed and released, we go to standby mode
        {
            Serial.println("pause to standby");
            EnterModeStandby();
        }
    } // end of  pause
} // end of loop()
