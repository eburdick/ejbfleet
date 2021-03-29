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

    3/10/21 9:30pm: Added more ascii art block text for sections of the code. Made
    modifications to to support the addition of a data logger shield, changing the
    I2C address of the MPU-6050, moving the button to pin 50 from 52, adding includes
    for the real time clock and the SD Card functions.

    3/11/21 9:30pm: Added code for the real time clock and the SD card interface for
    data logging. Added real time clock test to the test mode code. None of this is
    doing anything yet, but today was devoted to trying out the libraries. Next step:
    add some data logging code to capture the average angular velocity of the robot
    and its tilt going up and down the ramps. The goal is to determine where we are
    on the track by recording the turns, so we can prepare for upcoming difficult
    turns during a run and speed up in the easy parts of the course. Using line count
    and turn count should get us an accurate idea of where we are on the course.

    3/12/21 9:50pm: moved digital inputs 50-53 down to 45-49 to eliminate conflicts
    with the ICSP pins used for SPI communication with the data logger shield.

    3/15/21 Switched to a different library for MPU6050 IMU and removed most code written
    to the old one. Reinstalled RTClib. Not sure what happened to it.

    3/27/21 Added code to support a two digit seven segment LED display

    3/28/21 Code cleanup, add call to update seven segment display with cross line
    count. Next step, add turn detection and course segment code, so that robot speed
    and turn speed can be customized for each turn. This will have to be tuned on 
    the real course.

*/
#define ANALOGSENSING //using analog outputs of line sensors and software thresholds.

#include <Wire.h>               // I2C library
#include <SD.h>                 // SD card library
#include <SPI.h>                // Serial Peripheral Interface library
#include <RTClib.h>             // Real Time Clock library
//#include <EEPROM.h>           // EEPROM library
#include <Adafruit_MotorShield.h>
#include <MPU6050.h>   // IMU library (accelerometers, gyros) from https://github.com/jarzebski/Arduino-MPU6050
#include "blinkled_class.hh"    //support code for blinking LEDs
#include "timer_class.hh"       //support code for non-blocking timers
#include "clean_edge_class.hh"  //support code for edge cleaned sensor reads

/*
    Global variables and constants. Most of these could be put into functions functions
    as local static variables, but it is easier to keep track of them as globals.
*/
/*  ___      __ __           __   ___     __  __ __     __     __  __
    | ||\/||_ (_    /\ |\ ||  \   | |__||__)|_ (_ |__|/  \|  |  \(_
    | ||  ||____)  /--\| \||__/   | |  || \ |____)|  |\__/|__|__/__)
*/

//Blinking LED on and off times
const long runLedOnTime  = 1000;
const long runLedOffTime = 1000;
const long pauseLedOnTime = 500;
const long pauseLedOffTime = 500;

// Button debounce time window
const unsigned int buttonDebounceDelay = 50;

// middle line sensor edge cleaning delay
const long middleLineSensorEdgeDelay = 20;

// delay for stopping after passing the start/finish line at the end
const long finishLineDelay = 1000;

// line sensor analog thresholds. Remember Dark is higher than light
const int thresholdRightDark = 600;
const int thresholdRightLight = 600;
const int thresholdLeftDark = 600;
const int thresholdLeftLight = 600;
const int thresholdMiddleDark = 600;
const int thresholdMiddleLight = 600;

//amount of time to look for second line of double line
//after finding the first line.
const int seekingSecondLineTimeWindow = 300; // .3 second

//delay after return from pause to run before seeking line again.
const int seekFirstLineBlockTime = 500;  // Half second

// amount of time to stay in pause mode
const int pauseModeDuration = 3000;  // 3 seconds

//motor speeds for line following. Negative is reverse, and fastSideSpeed is the
//straight drive speed of the robot.
const int slowSideSpeed = -130;       // fixed slow side speed for turning out of line error.
const int fastSideSpeed = 130;      // fixed high side speed for turning out of line error.

// Light sensor thresholds for turning the headlights on and off.
const int lightsOnThreshold = 3;
const int lightsOffThreshold = 9;

// The seven segment number display can display only one digit at a time,
// so we switch back and forth between them every displayRefreshPeriod milliseconds
const int displayRefreshPeriod = 13; //how often display switches between ones and tens place


/*           __  __          __  __   __ __      _____        ___ __
    |__| /\ |__)|  \|  | /\ |__)|_   /  /  \|\ |(_  |  /\ |\ | | (_
    |  |/--\| \ |__/|/\|/--\| \ |__  \__\__/| \|__) | /--\| \| | __)
*/
// I2C address for MPU-6050 is 0x69 to avoid a conflict with 0x68 used by the real time clock
// of the data logging shield. In the hardware, this is done by tying AD0 of the MPU-6050
// HIGH. In the software, we pass this address to the imu.begin method.
const uint8_t imuI2Caddress = 0x69;

// LED port definitions
const int ledPortYellow = 25;  // Standby LED
const int ledPortRed = 27;     // Stop flasher LED
const int ledPortGreen = 29;   // Run flasher LED
const int ledPortWhite = 23;   // Headlight LEDs

const int ledPort7SegTop = 36;        //    --
const int ledPort7SegUpperLeft = 34;  //  |
const int ledPort7SegUpperRight = 35; //       |
const int ledPort7SegCenter = 40;     //    --
const int ledPort7SegLowerLeft = 39;  //  |
const int ledPort7SegLowerRight = 37; //       |
const int ledPort7SegBottom = 41;     //    --
const int ledPort7SegPoint = 38;      //           .

const int ledPort7SegAnodeOnes = 32;
const int ledPort7SegAnodeTens = 33;

const int buttonPort = 48;     // Push button input port

// IR line sensor port definitions

const int digLineSensorPortRight = 49;
const int digLineSensorPortLeft = 45;
const int digLineSensorPortMiddle = 47; //crossing line sensor

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

/*   __  __     __ _ ___   __ __      _____ __      _ ___ __  __  __
    /  \|__)  ||_ /   |   /  /  \|\ |(_  | |__)/  \/   | /  \|__)(_
    \__/|__)__)|__\__ |   \__\__/| \|__) | | \ \__/\__ | \__/| \ __)
*/

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

// Global Timers. We could re-use some of these, but they are very light weight, so
// the code is more readable if we have one for each purpose. Note there are some
// static local timers in functions that use them exclusively. These are global because
// they are used in loop() and in mode transition functions.

// create pause timer. This is used to time pause mode.
Timer pauseTimer = Timer();

// create standby to run timer. This is to delay starting the
// run at the beginning so the user has time to push the
// button and leave the robot alone.
Timer standbyToRunTimer = Timer();

// timer for real time clock test code
Timer rtcTestTimer = Timer();

// general purpose timer for test code
Timer testTimer = Timer();

// create timers for crossing line and double line detection
Timer firstLineBlockTimer = Timer(); //timer for getting off of the line after a pause
Timer secondLineTimer = Timer();  //timer for establishing a window for detecting second line.

// Timer for refresh timing
Timer displayRefreshTimer = Timer();

// create finish line timer to keep going a second at the end.
Timer finishLineTimer = Timer();

// IMU gyro sample timer
Timer imuGyroTimer = Timer();

// CleanEdge object for button. Initial button state unpressed.
CleanEdge buttonReader = CleanEdge(buttonPort, buttonDebounceDelay, unpressed);

// CleanEdge object for the center line detector, used to find and count cross lines,
// avoiding multiple counts at messy edges.
CleanEdge centerLineSensorReader = CleanEdge(digLineSensorPortMiddle, middleLineSensorEdgeDelay, light);

// Accelerometer/gyro (IMU) interfaces
MPU6050 imu;

//Real time clock
RTC_PCF8523 realTimeClock;

/*       __  __          __    __     __  __           _____   ___ __
    |  |/  \|__)|_/||\ |/ _   / _ |  /  \|__) /\ |    (_  |  /\ | |_
    |/\|\__/| \ | \|| \|\__)  \__)|__\__/|__)/--\|__  __) | /--\| |__
*/

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

// Line following support state

// correction directons. Signed integer so we can have multiple values if needed.
const int none = 0;
const int right = 1;
const int left = -1;
int inCorrection = none;
int previousCorrection;

// test stuff
int count = 0;

// IMU globals (accelerometer, gyro)

float timeStep = 0.01;      //Sample time for gyro rate incremental integration in seconds
long imuTimeStep = timeStep * 1000; //timeStep for sample timer in milliseconds
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

/*      ___     ___      __         _____  __      __
    /  \ | ||  | | \_/  |_ /  \|\ |/   | |/  \|\ |(_
    \__/ | ||__| |  |   |  \__/| \|\__ | |\__/| \|__)
*/

/*
    Display a number between 0 and 99 or 0xFF on the two digit seven segment display. This function is
    designed to be called every time around the loop when it is in use. between calls, the display
    will be static, displaying the most recent result. Because this function alternates between the
    digits for numbers greating than 9 or 0xF, not calling it regularly will result in only one digit
    being displayed. 
*/
void DisplayCount(int num)
{
    int radix = 10;
    int radixSquared = 100;
    // uncomment to switch to base 16
    //int radix = 16;
    //int radixSquared = 256;
    int digit;
    static boolean showOnes = true; //display ones place if true, else display tens place

    /*
        Set array to define which segments to turn on for each digit. A zero in the array
        corresponds to a segment being lit. Entry 17 of the array turns off all segments.
            0
          1   2
            3
          4   5
            6

    */

    uint8_t segments[17][7] =
    {
        {0, 0, 0, 1, 0, 0, 0},  //0
        {1, 1, 0, 1, 1, 0, 1},  //1
        {0, 1, 0, 0, 0, 1, 0},  //2
        {0, 1, 0, 0, 1, 0, 0},  //3
        {1, 0, 0, 0, 1, 0, 1},  //4
        {0, 0, 1, 0, 1, 0, 0},  //5
        {0, 0, 1, 0, 0, 0, 0},  //6
        {0, 1, 0, 1, 1, 0, 1},  //7
        {0, 0, 0, 0, 0, 0, 0},  //8
        {0, 0, 0, 0, 1, 0, 0},  //9
        {0, 0, 0, 0, 0, 0, 1},  //A
        {1, 0, 1, 0, 0, 0, 0},  //b
        {0, 0, 1, 1, 0, 1, 0},  //C
        {1, 1, 0, 0, 0, 0, 0},  //d
        {0, 0, 1, 0, 0, 1, 0},  //E
        {0, 0, 1, 0, 0, 1, 1},  //F
        {1, 1, 1, 1, 1, 1, 1}   //blank
    };

    // deal with illegal values. We just take the absolute value and truncate to two digits,
    // so the value will always be between zero and 99.
    num  = abs(num % radixSquared);

    // If the number is only one digit, force showOnes to true because the tens
    // digit will always be blank.
    if (num <= (radix-1))
    {
        showOnes = true;
    }
    // We update the display and then skip the update for some length of time, using the
    // display time. When the timer expires, we update the other digit and wait again.
    if (displayRefreshTimer.Test())
    {
        // The display is a common anode device. The anodes are pulled up by PNP transistors
        // driven low.
        //
        if (showOnes)
        {
            digit = num % radix; // base radix integer remainder (modulus)
            digitalWrite(ledPort7SegAnodeOnes, ledOn);
            digitalWrite(ledPort7SegAnodeTens, ledOff);
            showOnes = false; //set up for tens place next time
        }
        else
        {
            digit = num / radix;
            if (digit == 0)
            {
                digit = 17; // causes blank display (no leading zero)
            }
            digitalWrite(ledPort7SegAnodeOnes, ledOff);
            digitalWrite(ledPort7SegAnodeTens, ledOn);
            showOnes = true; //set up for ones place next time
        }
        // Use the segments array to set segment port values.
        digitalWrite(ledPort7SegTop, segments[digit][0]);
        digitalWrite(ledPort7SegUpperLeft, segments[digit][1]);
        digitalWrite(ledPort7SegUpperRight, segments[digit][2]);
        digitalWrite(ledPort7SegCenter, segments[digit][3]);
        digitalWrite(ledPort7SegLowerLeft, segments[digit][4]);
        digitalWrite(ledPort7SegLowerRight, segments[digit][5]);
        digitalWrite(ledPort7SegBottom, segments[digit][6]);
        digitalWrite(ledPort7SegPoint, ledOff); // decimal point always off

        //start the refresh timer. No ports will be written until it times out.
        displayRefreshTimer.Start(displayRefreshPeriod);
    }
}
/*********************************************************************************************/
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

/****************************************************************************************

    Mode switching functions. When switching into the modes. These make the one
    time changes at the transition from one mode to the next.
*/

// This function makes the transition to test mode, which runs test code for code
// debug and calibraton. It activates all leds to indicate it is in test mode.
void ModeStartToTest()
{
    // Activate all of the LED indicators to indicate we are in test mode
    mode = modeTest;
    runLed.Enable();
    pauseLed.Enable();
    digitalWrite(ledPortYellow, ledOn);

    //read all of the line sensors to initialize their state and initialize previous state.
    //this is for sampling the analog values when the digital values change, part of
    // calibration in test mode
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

// Mode transition from run to standby, which happens when you push the
// button in run mode or encounter the start/finish ine
void ModeRunToStandby()
{
    mode = modeStandby;
    runLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);
}

// Mode transition from pause to standby. Handles a button push during pause.
void ModePauseToStandby()
{
    mode = modeStandby;
    pauseLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft (0);
}

// Mode transition from standby to run. This transition happens after a delay after
// you push the button in standby mode

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

// Mode transition from pause to run. This happens when the pause timer expires.
void ModePauseToRun()
{
    mode = modeRun;
    runLed.Enable();
    pauseLed.Disable();
    firstLineBlockTimer.Start(seekFirstLineBlockTime);
}

// Mode transition from run to pause. This happens when a double cross line is detected.
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
      There are four system modes...

        Test mode (modeTest) is for testing sensors, calibrating, testing new code, etc. 
        It is entered by holding down the button during startup.

        Standby mode (modeStandby) is automatically entered at power on if the button is 
        not being held down. In standby, the yellow light is on to indicate the mode and 
        all motors are stopped. Pushing the button while in standby mode starts run mode.

        Run mode (modeRun) is entered when the button is pushed while in standby mode. 
        In this mode, the green light is flashing and the autonomous line following challenge  
        is run until the end line is detected or the button is pushed, at which point the
        mode switches back to standby mode. This mode also looks for a double line across 
        the path and if it finds it, we enter pause mode for three seconds.

        Pause mode (modePause) is entered from run mode. The motors stop, the red light 
        flashes for .5 seconds on and .5 seconds off for 3 seconds, then goes back run mode.

        For transitions between modes, there is a function for each type of transition;
        standby to run, run to pause, run to standby, pause to run, pause to standby. 
        These are called in the main loop when a mode needs to change, and they provide a 
        place for any action that needs to take place just once during a transition.
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

    // Start real time clock
    realTimeClock.begin(); // connect real time clock to I2C bus
    realTimeClock.start(); // clear the stop bit. Normally not necessary, but doesn't hurt.

    //
    //  set initial motor speed
    //
    initialMotorSpeed = 80;

    //set up the LED ports
    pinMode(ledPortWhite, OUTPUT);
    pinMode(ledPortYellow, OUTPUT);
    pinMode(ledPortGreen, OUTPUT);
    pinMode(ledPortRed, OUTPUT);

    pinMode(ledPort7SegTop, OUTPUT);
    pinMode(ledPort7SegUpperLeft, OUTPUT);
    pinMode(ledPort7SegUpperRight, OUTPUT);
    pinMode(ledPort7SegCenter, OUTPUT);
    pinMode(ledPort7SegLowerLeft, OUTPUT);
    pinMode(ledPort7SegLowerRight, OUTPUT);
    pinMode(ledPort7SegBottom, OUTPUT);
    pinMode(ledPort7SegPoint, OUTPUT);
    pinMode(ledPort7SegAnodeOnes, OUTPUT);
    pinMode(ledPort7SegAnodeTens, OUTPUT);

    //set up input ports. Note the line sensors ports default to the mode we are
    //setting, but I like to set them anyway.
    pinMode(buttonPort, INPUT_PULLUP);
    pinMode(digLineSensorPortLeft, INPUT);
    pinMode(digLineSensorPortMiddle, INPUT);
    pinMode(digLineSensorPortRight, INPUT);
    
    // Start motor shield
    MotorShield.begin();

    // Start IMU (Inertial Measurement Unit) with scale and range settings, for I2C address 0x69
    imu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x69);
    imu.calibrateGyro();    // Calibrate gyroscope. The calibration must be at rest.
    imu.setThreshold(1);    // Set threshold sensitivty. Default = 3.

    // initialize inCorrection. These are used by the line following code
    inCorrection = none;
    previousCorrection = none;

    // turn off seven segment display by deactivating both anodes
    digitalWrite(ledPort7SegAnodeOnes, ledOff);
    digitalWrite(ledPort7SegAnodeTens, ledOff);

    //
    // Read the button. If the button is pressed, set the mode to modeTest.
    // if not pressed, set the mode to modeStandby. The test mode puts the 
    // system in test mode, which is for checking out the sensors, motors, and LEDs,
    // calibration procedures, and testing new code.
    //
    if (buttonReader.Sample() == pressed)
    {
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

    /*                    _                   _   _   _    _  _   _   _
                         |_) | | |\ |   |\/| / \ | \ |_   /  / \ | \ |_
                         | \ |_| | \|   |  | \_/ |_/ |_   \_ \_/ |_/ |_
    */
    if (mode == modeRun)
    {
        // update LED flasher
        runLed.Update();

        /*************************************Line following algorithm************************

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

        // end of line detection.  Result is setting inCorrection. The next code responds
        // to that value

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
                }
            }
            else
            {
                //second line time window has expired. This means we have failed to find a second line,
                //close enough to the first one, so there was only one line at this location.

                crossLineState = seekingFirstLine; //start looking for the next line
            }
        }
        else if (firstLineBlockTimer.Test()) // crossLineState == seekingBlocked
        {
            // here we are not seeking the first or second line, but the timer we started to avoid double
            // counting the second line has expired, so we can start looking for a first line again.
            crossLineState = seekingFirstLine;
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
            ModeRunToStandby();
        }
        DisplayCount(crossLineCount);
    } // end of run mode code
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
        /*
                // For line sensor calibration
                // Check each line sensor digital state and report changes to the serial terminal along with the
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

                if (rtcTestTimer.Test())
                {
                    rtcTestTimer.Start(5000); // set timer to 5 seconds
                    char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
                    //Serial.println("Real Time Clock check...");
                    DateTime now = realTimeClock.now();
                    Serial.print(now.year(), DEC);
                    Serial.print('/');
                    Serial.print(now.month(), DEC);
                    Serial.print('/');
                    Serial.print(now.day(), DEC);
                    Serial.print(" (");
                    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
                    Serial.print(") ");
                    Serial.print(now.hour(), DEC);
                    Serial.print(':');
                    Serial.print(now.minute(), DEC);
                    Serial.print(':');
                    Serial.print(now.second(), DEC);
                    Serial.println();
            }
        */
        /*
            Gyro test code. We sample the gyro every timeStep seconds. The value we sample is
            an angular rate. We multiply that by the timeStep to turn the rate into an angle
            change, and add that to an accumulating value, which starts out at zero. Roughly
            speaking, we are integrating the angular velocity over time to yield an angular
            displacement. This will drift over time due to cumulative rounding error, but we
            will try it out and see how well it works.
        */
        /*
                if (imuGyroTimer.Test()) // check if timer has reached its limit
                {
                    // Read normalized values
                    Vector norm = imu.readNormalizeGyro(); //vector to receive 3D gyro readings.

                    // Calculate Pitch, Roll and Yaw (rough incremental rate integration)
                    //pitch = pitch + norm.YAxis * timeStep;
                    //roll = roll + norm.XAxis * timeStep;
                    yaw = yaw + norm.ZAxis * timeStep;

                    // Output raw
                    //Serial.print(" Pitch = ");
                    //Serial.print(pitch);
                    //Serial.print(" Roll = ");
                    //Serial.print(roll);
                    Serial.print(" Yaw = ");
                    Serial.println(yaw);

                    // Wait to full timeStep period
                    //delay((timeStep * 1000) - (millis() - timer));
                    imuGyroTimer.Start(imuTimeStep); // start timer with time step delay
                }
        */
        /*
            Test seven segment display. DisplayCount is called every cycle, but the
            number to display is changed every second or so. Note DisplayCount has its
            own timer for its refresh rate, so a lot of times when it is called, it does
            nothing but maintain the current display state.
        */

        static int count = 0;
        DisplayCount(count);
        if (testTimer.Test())
        {
            testTimer.Start(1000);
            count++;
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

        //boolean pauseDone = pauseTimer.Test();   //poll the pause timer
        if (pauseTimer.Test())
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
