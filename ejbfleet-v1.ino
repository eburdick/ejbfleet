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

    3/30/21 Added code to detect boundaries between turns and straight sections just
    using the correction variables in the line following code. For my little track, using
    a .5 second time window and a 30 count threshold in that .5 second to determine in
    a turn or not, I am getting five sections, which is what I would expect with a straight
    section, then a left 90, another straight section, then a left 180, then a final
    straight section. The real track will not work so well, because some transitions between
    right and left gentle turns might count wrong. So I think I will use crossing lines and
    slopes to syncronize the count, dividing the track into subsections. 1: start to the
    first ramp. Nothing major to deal with there except maybe for the turn onto the up ramp.
    2: the ramp, detected by the Y accelerometer, and the turn at the top. 3: the down ramp,
    again detected by the Y accelerometer, the turn at the bottom, which is problematic
    because of speeding up on the down slope (slow down there) and the long curve through
    the tunnel to the first crosswalk. 4: from the first crosswalk to the second crosswalk.
    5: from the second crosswalk through the lower part of the figure 8, ending at the first
    crossing line. 6: from that first crossing line, through the second one, and past the
    problematic abrupt right turn up to the next crossing line, just outside of the tunnel.
    We can slow down approaching that nasty abrupt turn, detect the turn, and then speed
    back up. 7: the rest of the course. We can detect that last three 90 right turns and
    maybe sprint on the tilted part and the stretch to the finish line. All of these sections
    can use a combination of line counting, slope detection, and turn detection. I think
    specialized code for each of these seven sections make sense with some fine grain speed
    and turn tuning.

    4/16/21 Started working on a more robust way of subdividing the course, a refinement
    of the discussion above. At this point, constants for speed, correction differentials
    and delays for straight section sprints have been added assuming the track is divided
    into eight sections with some subsections. Just starting to work on code to identify
    and sequence the sections. Once that is done, the constants can be tuned on the real
    track. The sections are pretty close to those described above, with section 7
    divided arbitrarily into sections 7, ending at the beginning of the banked section,
    and section 8, from there to the finish line.

    4/22/2021 Finished course section recognition and tracking, cleaned up code in general.
    code compiles clean. Ready to test and tune (speeds and sprint delays, maybe turn tracking
    time slot and thresholds)

    4/28/2021 After some testing on the track, added leftRightBias to skew the straight
    sections to the right or left. Lots of time debugging that because I overflowed
    the 8 bit unsigned integer that goes to the motor shield. Started tuning the speeds
    and times, but a ways to go yet. May need to use the gyro to validate the turns, but
    this would not be that hard given the yaw test code that is already in the test part
    of this code. Main issue is premature sequencing of track sections because of the
    simplistic detection of curve completion and depending too much on timers when
    there are spots in the course, especially at the bottom and top of the up ramp, where
    the robot slows down because of losing traction.

    4/29/2021 Added some gyro code using the adafruit mpu4050 library and deleting the
    old pitch, roll and yaw stuff. The test code works for tracking rotation about the
    Z axis with ranges and filter values set for what I think we want. Also experimented
    with compensation for a small amount of offset when there is no rotation. This works
    well enough. Started designing a better method for detecting and tracking turns using
    the gyro and an improved method for tracking course sections. Not a huge change, but
    should be much more reliable. The design is detailed in a comment block after the
    current turn tracking code.

    5/2/2021 1:00 AM Added and tested code in loop() to take a snapshot of zRotation and xAcceleration
    every set amount of time. Started adding code to use it for turn tracking. Checking in to
    snapshot this.

    5/2/2021 10:30 PM Finished new turn tracking code based on zRotation discussed above.
    It compiles, but is not tested. Next: add some simple test code to adjust the thresholds
    for turn starts and ends, debug, then modify the track section code to utilize the new
    approach, including adding post sprint speeds for straight sections.

    5/3/2021 Modified new turn tracking code to track from right turns to left turns and left turns
    right turns. Added functions to handle most straight and turn sections. Started modifying section
    code to use this new stuff. No compile today because more stuff has to be updated.

    5/6/2021 Continued modifications of turn tracking code to use straight and turn sections. Added some
    sections for the left side of the figure 8 because of the strict alternation between curved and
    straight sections. More modifications to go, including constants to support this scheme,
    so no compiles yet.

    5/7/2021 Finished modifications of turn tracking code and supporting global constants and worked out
    compilation errors. Code now compiles without errors. Next steps: bench tests, track tests, tuning.

    5/8/2021 Finished bench debugging. Still confused about why everything but the 21 to 22 transition gets
    stuck in idle tracking without an extra statement to restart tracking, but all sections are updating as
    expected when the robot is just manually turned in the expected directions. Debugging print statements
    display the progress through the sequence for now. Next step is testing on the track and tuning the
    speeds and delays and maybe the gyro thresholds. And I would like to solve the 21 to 22 mystery.

    5/9/2021 added conditionals for print statements

    5/10/2021 Testing on the track:

    Added new batteries. Code changes as follows...

    1...Tweaked turn detection thresholds, but not all turns need the same values, so I added new
    constants for wide, normal, and tight turns. But for the long (wide) turn into the tunnel, the
    transition from the turn to the straight section after it is too subtle to accurately
    distinguish. Change for this: make the detection more sensitive just for that turn, but use
    the darkness of the tunnel to detect the end of the turn. This happens a bit before the turn
    ends, but should not be a problem, because we take this turn fast.

            New code written: Use ambient light reduction to switch from section 43 to 44.

    2...Tweaked sprint times and agressiveness of corrections at the beginnings of turns. A lot of
    playing with speeds.  More to be done here to make turns more reliable and speed up the run.

            Coding changes: created turn detection thresholds for specific turns that need help
            instead of lumping everything into tight, normal and wide. Problem turns I know about
            are section 43, which is very difficult to tweak for both start and end, section 52, which
            is a pretty tight turn, and was missing in testing, and section 81
            at the top of the banked section. It is a pretty tight turn, so we should be able to
            detect it by reducing the threshold. 
            
            Possible if needed: add code to detect the bank and force the end of the section 81 turn.

    3...The turn onto the banked section is not working most of the time, so the section count is off.
    Tweaking the turn detect sensitively and the speed of the turn may solve it, but we can also sense
    the bank itself to provide some redundancy. I sampled the Y accelerometer value for the bank and
    starting line to support this. While I was at it I took samples for X acceration on the ramps...
       - Y Tilt on banked section: > 1.5 ( ~ 0.6 at starting line)
       - X tilt on upramp > 1.7 ( ~ 0.7 ??? at starting line)
       - X tilt on downramp < 0.5 at top, < 0.8 at middle

    Other coding stuff that could be done: use the X tilt value to detect the bump at the top of
    the up ramp to deal with the stalling issue, maybe with wiggle or a quick pivot to the right.

    Removed conditional compile for analog sensing. Removed digital sensing code.

*/
//#define DEBUGTURNTRACK
//#define DEBUGTURNSENSE

#include <arduino.h>
#include <Adafruit_MPU6050.h> //Adafruit library for for inertial measurement unit
#include <Adafruit_Sensor.h> //Adafruit sensor library (for reading MPU6050 sensors
#include <Adafruit_MotorShield.h>
#include <Wire.h>    // I2C library
#include <SD.h>      // SD card library
#include <SPI.h>     // Serial Peripheral Interface library
#include <RTClib.h>  // Real Time Clock library
#include "blinkled_class.hh"    //support code for blinking LEDs
#include "timer_class.hh"       //support code for non-blocking timers
#include "clean_edge_class.hh"  //support code for edge cleaned physical sensor reads (button, line sensor)


/*
    Global variables and constants. Most of these could be put into functions functions
    as local static variables, but it is easier to keep track of them as globals.
*/
/*  ___      __ __           __   ___     __  __ __     __     __  __
     | ||\/||_ (_    /\ |\ ||  \   | |__||__)|_ (_ |__|/  \|  |  \(_
     | ||  ||____)  /--\| \||__/   | |  || \ |____)|  |\__/|__|__/__)
*/

//Blinking LED on and off times
const long runLedOnTime = 1000;
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
const int seekingSecondLineTimeWindow = 300;  // .3 second

//delay after return from pause to run before seeking line again.
const int seekFirstLineBlockTime = 500;  // Half second

// amount of time to stay in pause mode
const int pauseModeDuration = 3000;  // 3 seconds

// Light sensor thresholds for turning the headlights on and off.
const int lightsOnThreshold = 3;
const int lightsOffThreshold = 9;

// The seven segment number display can display only one digit at a time,
// so we switch back and forth between them every displayRefreshPeriod milliseconds
const int displayRefreshPeriod = 13;

// The turn time sample interval is the period of time we sample turn data before making a
// decision on whether to advance the section counter. This should be happening several
// times a second. The longer this period, the better smoothing we will have to avoid
// local minima. Too long will make the section boundaries too inaccurate.
const int turnSampleInterval = 300;

// turn thresholds are the total value of the sum of rotation values we count per time slot to
// define entering and exiting a turn. This number will go up with the size of the time interval.
float turnStartThreshold;
float turnEndThreshold;
const float normalTurnStartThreshold = 15.0;
const float sec52TurnStartThreshold = 12.0;
const float sec81TurnStartThreshold = 12.0;
const float sec43TurnStartThreshold = 10.0;  // hard turn to detect. Make it more sensitive. Turn end
// detected by tunnel entrance.
const float normalTurnEndThreshold = 7.0;
const float sec52TurnEndThreshold = 7.0;
const float sec81TurnEndThreshold = 7.0;

int courseSection; //state variable stores which section we are in. Initialized in setup()
//
// Course specific constants and state. Define constants for the course sections, and their
// subsections. The sections are numbered with two digits. The
// tens place is the main section number, and the ones place is the sub-section number.
// In straight sections, we have sec##FastSideSpeed1tants specify motor speeds beingtimes for straight sections.
//
// Notes:
//
// Motor speeds to the motor shield are unsigned 8 bit integers, so they range from 0 to 255.
// If there is a leftRightBias set, then the sum of the speed
// number and the bias number must be in this range.
//
// FastSideSpeed is the speed the robot moves when it is not turning. LeftRightBias is added
// to the left side and subtracted from the right side to turn straight runs into constant
// curves. If the bias is positive, it will curve to the right, and if it is negative, it will
// curve to the left.
//
// SprintTime is the number of milliseconds to run with *FastSideSpeed1 and *SlowSideSpeed1 in
// a straight run. After that time, we run with *FastSideSpeed2 and *SlowSideSpeed2 until an
// event moves us to the next track subsection, which is either at a turn or at a crosswalk.
// Too long a sprint time may cause a turn to be

//first straight section. Sprint to just before the first turn.
const int sec11FastSideSpeed1 = 240;
const int sec11SlowSideSpeed1 = 50;    // gentle correction
const int sec11FastSideSpeed2 = 130;   // post sprint speed
const int sec11SlowSideSpeed2 = -180;    // Post sprint correction speed
const int sec11SprintTime = 2400;     // duration of sprint time before starting turn
const int sec11LeftRightBias = 0;

// first turn (right)
const int sec12FastSideSpeed = 130;   // fast side speed for first turn right 60 degrees
const int sec12SlowSideSpeed = -50;  // slow side speed for first turn

// straight section
const int sec13FastSideSpeed1  = 200;  // speed for short sprint between right turns
const int sec13SlowSideSpeed1 = 50;    // gentle correction
const int sec13FastSideSpeed2 = 130;   // post sprint speed
const int sec13SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec13SprintTime = 500;      // duration of sprint before second turn
const int sec13LeftRightBias = 0;

// second turn (right)
const int sec14FastSideSpeed = 130;   // fast side speed for second turn right 80 degrees
const int sec14SlowSideSpeed = -50;  // slow side speed for gentle turns

// third turn (left)
const int sec15FastSideSpeed = 130;   // fast side speed for third turn left 80 degrees
const int sec15SlowSideSpeed = -50;  // slow side speed for third turn

// courseSection 2 constants (turn onto ramp to turn at the top of the ramp)
const int sec21FastSideSpeed = 150;   // fast side speed for turn onto ramp right 140 degrees
const int sec21SlowSideSpeed = -100;  // slow side speed for turn onto ramp

//straight section to turn at top of ramp. End at start of turn.
const int sec22FastSideSpeed1 = 240;   // speed up the ramp
const int sec22SlowSideSpeed1 = 50;    // gentle correction
const int sec22FastSideSpeed2 = 130;   // post sprint speed
const int sec22SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec22SprintTime = 2500;      // duration of sprint time before ramp turn
const int sec22LeftRightBias = 0;     // stay to right to avoid premature right turn

// courseSection 3 contants (turn onto down ramp to turn onto tunnel approach)
const int sec31FastSideSpeed = 130;  // turn at top of ramp right 90 degrees
const int sec31SlowSideSpeed = -130;

//straight section to turn at bottom of ramp. End at start of turn.
const int sec32FastSideSpeed1 = 200;  // speed down the ramp
const int sec32SlowSideSpeed1 = 0;    // gentle correction
const int sec32FastSideSpeed2 = 100;   // post sprint speed
const int sec32SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec32SprintTime = 1600;     // duration of sprint time before turn at bottom
const int sec32LeftRightBias = 20;    // stay to right to avoid premature right turn

// courseSection 4 constants (turn onto tunnel approach, through tunnel to first crosswalk
const int sec41FastSideSpeed = 130;  // first turn at bottom of ramp right 90 degrees
const int sec41SlowSideSpeed = -130;

//Straight section to tunnel approach turn (right). End at start of turn.
const int sec42FastSideSpeed1 = 200;  // short sprint after turn
const int sec42SlowSideSpeed1 = 50;   // gentle correction
const int sec42FastSideSpeed2 = 130;   // post sprint speed
const int sec42SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec42SprintTime = 300;
const int sec42LeftRightBias = 0;

//wide right turn into tunnel. track turn until it ends
const int sec43FastSideSpeed = 220;  // tunnel approach curve right 180 degrees
const int sec43SlowSideSpeed = 70;

//straight section to crosswalk
const int sec44FastSideSpeed1 = 220;  // short sprint to first crosswalk
const int sec44SlowSideSpeed1 = 50;   // gentle correction
const int sec44FastSideSpeed2 = 130;   // post sprint speed
const int sec44SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec44SprintTime = 200;
const int sec44LeftRightBias = 0;

//Straight section from first crosswalk to right turn into the figure 8. Ends with beginning of turn
const int sec51FastSideSpeed1 = 200;  // short sprint to turn
const int sec51SlowSideSpeed1 = 50;   // gentle correction
const int sec51FastSideSpeed2 = 130;   // post sprint speed
const int sec51SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec51SprintTime = 200;
const int sec51LeftRightBias = 0;

//Right turn toward figure 8. Track the turn until it ends.
const int sec52FastSideSpeed = 130;  // first turn right 90 degrees
const int sec52SlowSideSpeed = -100;

//Short straight section before a right dogleg. Ends when the dogleg starts.
const int sec53FastSideSpeed1 = 130;   // sprint speed
const int sec53SlowSideSpeed1 = 0;    // sprint correction speed
const int sec53FastSideSpeed2 = 130;   // post sprint speed
const int sec53SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec53SprintTime = 200;
const int sec53LeftRightBias = 0;

//Right dogleg in figure 8. Ends when turn finishes.
const int sec54FastSideSpeed = 130;  // dogleg right 45 degrees
const int sec54SlowSideSpeed = -130;

//Straight section before second crosswalk
const int sec55FastSideSpeed1 = 130;  // sprint to first turn
const int sec55SlowSideSpeed1 = 50;   // gentle correction
const int sec55FastSideSpeed2 = 130;   // post sprint speed
const int sec55SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec55SprintTime = 500;
const int sec55LeftRightBias = 0;

// courseSection 6 constants (bottom of figure 8 to second dogleg, passing three cross lines)
const int sec61FastSideSpeed1 = 130;  // sprint to first turn
const int sec61SlowSideSpeed1 = 50;   // gentle correction
const int sec61FastSideSpeed2 = 100;   // post sprint speed
const int sec61SlowSideSpeed2 = -150;    // Post sprint correction speed
const int sec61SprintTime = 100;
const int sec61LeftRightBias = 0;

// first turn at bottom of fig 8 left 120 degrees
const int sec62FastSideSpeed = 130;
const int sec62SlowSideSpeed = -130;

//Straight section at the bottom of the figure 8. Ends at left turn out of figure 8
const int sec63FastSideSpeed1 = 200;  // sprint to second turn
const int sec63SlowSideSpeed1 = 50;   // gentle correction
const int sec63FastSideSpeed2 = 130;   // post sprint speed
const int sec63SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec63SprintTime = 300;
const int sec63LeftRightBias = 0;

//Left turn toward the figure 8 crossing point. Track the turn until it ends.
const int sec64FastSideSpeed = 130;  // second turn at bottom of fig 8 left 160 degrees
const int sec64SlowSideSpeed = -130;

//Straight section coming out of the bottom of the figure 8. Ends with dogleg right turn.
const int sec65FastSideSpeed1 = 200;  // sprint to dogleg
const int sec65SlowSideSpeed1 = 50;   // gentle correction
const int sec65FastSideSpeed2 = 100;   // post sprint speed
const int sec65SlowSideSpeed2 = -180;    // Post sprint correction speed
const int sec65SprintTime = 800;
const int sec65LeftRightBias = 0;

// courseSection 7 constants (detecting dogleg and turning right, passing two cross lines, then 90 degree right,
// straight sprint up to turn onto banked section

//Right dogleg turn after crossing center of figure 8 . Ends when turn ends.
const int sec71FastSideSpeed = 100;   // turning at dogleg
const int sec71SlowSideSpeed = -200;  // agressive turn

//Straight section exiting the figure 8, ending right turn
const int sec72FastSideSpeed1 = 200;   // sprint to 90 degree right
const int sec72SlowSideSpeed1 = 50;    // gentle correction
const int sec72FastSideSpeed2 = 130;   // post sprint speed
const int sec72SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec72SprintTime = 1000;
const int sec72LeftRightBias = 0;

//Right turn to horizontal section at top of field. Ends when turn ends.
const int sec73FastSideSpeed = 130;  // 90 degree right turn to straight section along course top edge
const int sec73SlowSideSpeed = -130;

//Straight section to right turn onto banked section. Ends at right turn
const int sec74FastSideSpeed1 = 200;  // sprint to 90 degree right onto banked section
const int sec74SlowSideSpeed1 = 50;   // gentle correction
const int sec74FastSideSpeed2 = 130;   // post sprint speed
const int sec74SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec74SprintTime = 500;
const int sec74LeftRightBias = 0;

//Right turn to banked section. Ends when turn ends.
const int sec81FastSideSpeed = 130;  // turning onto banked section
const int sec81SlowSideSpeed = -130;

//Straight banked section to right turn to home stretch section.
const int sec82FastSideSpeed1 = 240;  // sprint on banked section
const int sec82SlowSideSpeed1 = 50;   // gentle correction
const int sec82FastSideSpeed2 = 130;   // post sprint speed
const int sec82SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec82SprintTime = 2000;    // long section
const int sec82LeftRightBias = 0;

//Right turn to home stretch. Ends when turn ends.
const int sec83FastSideSpeed = 130;  // final turn
const int sec83SlowSideSpeed = -130;

//Straight section to finish line.
const int sec84FastSideSpeed1 = 200;  // sprint to finish line
const int sec84SlowSideSpeed1 = 50;   // gentle correction
const int sec84FastSideSpeed2 = 130;   // post sprint speed
const int sec84SlowSideSpeed2 = -130;    // Post sprint correction speed
const int sec84SprintTime = 2000;    // this sprint ends at the finish line, so sprint time probably not needed.
const int sec84LeftRightBias = 0;


/*           __  __          __  __   __ __      _____        ___ __
    |__| /\ |__)|  \|  | /\ |__)|_   /  /  \|\ |(_  |  /\ |\ | | (_
    |  |/--\| \ |__/|/\|/--\| \ |__  \__\__/| \|__) | /--\| \| | __)
*/
// I2C address for MPU-6050 is 0x69 to avoid a conflict with 0x68 used by the real time clock
// of the data logging shield. In the hardware, this is done by tying AD0 of the MPU-6050
// HIGH. In the software, we pass this address to the imu.begin method.
const uint8_t imuI2Caddress = 0x69;

// gyro drift rate is how much the gyro is off. This particular gyro is off by
// the following number based on just watching the numbers change on the serial
// monitor as they are read. This should be good enough.
const double imuGyroZDrift = -0.00143;

// event target veriables for the imu. These will be used when we read the values from
// the imu in main loop.
sensors_event_t accelerationsXYZ,  rotationRatesXYZ,  temperature;


// LED port definitions
const int ledPortYellow = 25;  // Standby LED
const int ledPortRed = 27;     // Stop flasher LED
const int ledPortGreen = 29;   // Run flasher LED
const int ledPortWhite = 23;   // Headlight LEDs

const int ledPort7SegTop = 36;         //    --
const int ledPort7SegUpperLeft = 34;   //  |
const int ledPort7SegUpperRight = 35;  //       |
const int ledPort7SegCenter = 40;      //    --
const int ledPort7SegLowerLeft = 39;   //  |
const int ledPort7SegLowerRight = 37;  //       |
const int ledPort7SegBottom = 41;      //    --
const int ledPort7SegPoint = 38;       //           .

const int ledPort7SegAnodeOnes = 32;
const int ledPort7SegAnodeTens = 33;

const int buttonPort = 48;  // Push button input port

// IR line sensor port definitions

const int digLineSensorPortRight = 49;
const int digLineSensorPortLeft = 45;
const int digLineSensorPortMiddle = 47;  //crossing line sensor

// Analog ports (note A4 and A5 are used by I2C, so we can't use them for this.)

const int analogLightSensorPort = A0;  // for tunnel lights

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
// the code is more readable if we have one for each purpose. Note some of these are only
// used in one function, so they could be created there, but since we are using global
// variables anyway, it is good to keep them all together for consistency.

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
Timer firstLineBlockTimer = Timer();  //timer for getting off of the line after a pause
Timer secondLineTimer = Timer();      //timer for establishing a window for detecting second line.

// Timer for refresh timing
Timer displayRefreshTimer = Timer();

// create finish line timer to keep going a second at the end.
Timer finishLineTimer = Timer();

// IMU gyro sample timer
Timer imuGyroTimer = Timer();

// Turn timer. This is for creating time slots for sampling rotation data
// during turns.
Timer turnTimer = Timer();

// sprint timer. This is for setting times for sprints in the straight sections
Timer sprintTimer = Timer();

// CleanEdge object for button. Initial button state unpressed.
CleanEdge buttonReader = CleanEdge(buttonPort, buttonDebounceDelay, unpressed);

// CleanEdge object for the center line detector, used to find and count cross lines,
// avoiding multiple counts at messy edges.
CleanEdge centerLineSensorReader = CleanEdge(digLineSensorPortMiddle, middleLineSensorEdgeDelay, light);

// Accelerometer/gyro (IMU) interfaces
Adafruit_MPU6050 imu;

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

// Motor left right bias. This value is added to the left motor speed and subtracted from
// the right motor speed to bias the direction to the left (leftRightBias < 0) or to the right
// (leftRightBias > 0). It is for when we want to hug the right or left in a straight run.
int leftRightBias;

//motor speeds for line following. Negative is reverse, and fastSideSpeed is the
//straight drive speed of the robot. These are assigned from the motor speeds
//in the course section sequencing code.
int slowSideSpeed = -130;  // fixed slow side speed for turning out of line error.
int fastSideSpeed = 130;   // fixed high side speed for turning out of line error.

// state constants for cross line processing
const int seekingFirstLine = 0;
const int seekingSecondLine = 1;
const int seekingBlocked = 3;

const int totalDoubleLineCrossings = 4;  //Crossing starting line twice and two crosswalks

int waitingForFinalStandby;  //set when we are crossing the finish line

int crossLineState;
int crossLineCount;
int doubleLineCount;

// Loop mode state variables
int mode;
const int modeTest = 1;     //test mode for testing stuff on the robot
const int modeStandby = 2;  //standby mode for before and after the timed run
const int modeRun = 3;      //run mode for the timed run
const int modePause = 4;    //pause mode for crosswalk stops while in run mode

// Line following support state

// correction directons and turn status constants. Signed integer so we can have multiple values if needed.
const int none = 0;
const int right = 1;
const int left = -1;
const int unknown = 2;
const int idle = 5;
const int waitingForRightTurn = 6;
const int waitingForLeftTurn = 7;
const int tracking = 8;

int inCorrection = none;
int previousCorrection;
int turnState;
int turnDirection;
int turnTrackState;
int prevTurnTrackState; //for debug prints
int inTurn;
int trackTurnTo;
int prevTrackTurnTo; //for debug prints

// Robot rotation rate around Z axis and accleration in the X axis.
float zRotationRate;
float xAcceleration;

// variable to accumulate rotation rate for averaging.
float rotationAccum;

boolean finishingCrosswalk; // to let course section code know crosswalk pause is finished

int leftCorrectionCount;
int rightCorrectionCount;

// test stuff
int count = 0;

// IMU globals (accelerometer, gyro)

//Sample time for gyro rate incremental integration in seconds
unsigned long imuTimeStep = 10;  //timeStep for sample timer in milliseconds


/*      ___     ___      __         _____  __      __
    /  \ | ||  | | \_/  |_ /  \|\ |/   | |/  \|\ |(_
    \__/ | ||__| |  |   |  \__/| \|\__ | |\__/| \|__)
*/

/*
    this function sets up a timed sprint followed by a (usually) slower speed leading up the the next
    course section, which is always a turn. It is mostly to make the course section sequencing code
    more compact, and only works for straight track sections that end with a turn. The sequence is...

    start the curve tracking code looking for a turn in the direction of the next curve.

    The assumption is that the sprint timer has already been started during the track section transition.
    test the sprint timer. If it has not expired, run at sprint speed, otherwise run at post
    sprint speed.
*/
void ProcessStraightSection
(
    int sprintFastSideSpeed,
    int sprintSlowSideSpeed,
    int postSprintFastSideSpeed,
    int postSprintSlowSideSpeed,
    int nextSection,
    int nextSectionTurn
)
{

    // Since we are in a straight section, there will only be small corrections leading up to
    // the turn in the next section, so we start looking for the turn here.
    if (nextSectionTurn == right)
    {
        turnTrackState = waitingForRightTurn;
    }
    else
    {
        turnTrackState = waitingForLeftTurn;
    }

    // run at sprint speed during the sprint time. This should end before the next track section just
    // to avoid going too fast to start the turn.
    if (sprintTimer.Test())
    {
        fastSideSpeed = postSprintFastSideSpeed;
        slowSideSpeed = postSprintSlowSideSpeed;
    }
    else
    {
        fastSideSpeed = sprintFastSideSpeed;
        slowSideSpeed = sprintSlowSideSpeed;
    }
    if (inTurn == nextSectionTurn)
    {
        courseSection = nextSection;
    }
}
/*
    This function processes a track section that is a turn. It will be called after a waitingForRightTurn
    or waitingForLeftTurn has transitioned to tracking a turn, which is the event that put us into this
    section.
*/
void ProcessTurnSection
(
    int turnFastSideSpeed,
    int turnSlowSideSpeed,
    int nextSection,
    int nextSectionSprintTime,
    int nextSectionLeftRightBias
)
{
    // set speeds for the turn
    fastSideSpeed = turnFastSideSpeed;
    slowSideSpeed = turnSlowSideSpeed;
    if (inTurn == none)
    {
        courseSection = nextSection;
        // end turn tracking
        turnTrackState = idle;
        sprintTimer.Start(nextSectionSprintTime);
        leftRightBias = nextSectionLeftRightBias;
    }
}


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
    static boolean showOnes = true;  //display ones place if true, else display tens place

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
        { 0, 0, 0, 1, 0, 0, 0 },  //0
        { 1, 1, 0, 1, 1, 0, 1 },  //1
        { 0, 1, 0, 0, 0, 1, 0 },  //2
        { 0, 1, 0, 0, 1, 0, 0 },  //3
        { 1, 0, 0, 0, 1, 0, 1 },  //4
        { 0, 0, 1, 0, 1, 0, 0 },  //5
        { 0, 0, 1, 0, 0, 0, 0 },  //6
        { 0, 1, 0, 1, 1, 0, 1 },  //7
        { 0, 0, 0, 0, 0, 0, 0 },  //8
        { 0, 0, 0, 0, 1, 0, 0 },  //9
        { 0, 0, 0, 0, 0, 0, 1 },  //A
        { 1, 0, 1, 0, 0, 0, 0 },  //b
        { 0, 0, 1, 1, 0, 1, 0 },  //C
        { 1, 1, 0, 0, 0, 0, 0 },  //d
        { 0, 0, 1, 0, 0, 1, 0 },  //E
        { 0, 0, 1, 0, 0, 1, 1 },  //F
        { 1, 1, 1, 1, 1, 1, 1 }   //blank
    };

    // deal with illegal values. We just take the absolute value and truncate to two digits,
    // so the value will always be between zero and 99.
    num = abs(num % radixSquared);

    // If the number is only one digit, force showOnes to true because the tens
    // digit will always be blank.
    if (num <= (radix - 1))
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
            digit = num % radix;  // base radix integer remainder (modulus)
            digitalWrite(ledPort7SegAnodeOnes, ledOn);
            digitalWrite(ledPort7SegAnodeTens, ledOff);
            showOnes = false;  //set up for tens place next time
        }
        else
        {
            digit = num / radix;
            if (digit == 0)
            {
                digit = 17;  // causes blank display (no leading zero)
            }
            digitalWrite(ledPort7SegAnodeOnes, ledOff);
            digitalWrite(ledPort7SegAnodeTens, ledOn);
            showOnes = true;  //set up for ones place next time
        }
        // Use the segments array to set segment port values.
        digitalWrite(ledPort7SegTop, segments[digit][0]);
        digitalWrite(ledPort7SegUpperLeft, segments[digit][1]);
        digitalWrite(ledPort7SegUpperRight, segments[digit][2]);
        digitalWrite(ledPort7SegCenter, segments[digit][3]);
        digitalWrite(ledPort7SegLowerLeft, segments[digit][4]);
        digitalWrite(ledPort7SegLowerRight, segments[digit][5]);
        digitalWrite(ledPort7SegBottom, segments[digit][6]);
        digitalWrite(ledPort7SegPoint, ledOff);  // decimal point always off

        //start the refresh timer. No ports will be written until it times out.
        displayRefreshTimer.Start(displayRefreshPeriod);
    }
}// ******************end of void DisplayCount(int num)

// function to update the seven segment display. This just calls the display function
// above with a given variable. It is called at the beginning of the code for each non-test
// mode, so we put it here so we can change what it displays in one place.

void update7SegDisplay()
{
    // update the courseSection display
    DisplayCount(courseSection);
}
/*********************************************************************************************/

//
// Functions for setting motor speeds on the left and right sides. These functions have a signed argument
// so the motors can be reversed on a negative sign.
//
// Note: We assume both left motors run the same speed and both right motors run the same speed. If this turns out
// not to be the case, some of this code will need to be changed.
//
// The motor speeds are modified by the global leftRightBias, which skews the direction to the right or left, mosly
// for straight sections where we want to be hugging the right or left side.
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
        // To skew right, we add a little speed to the left side. (bias > 0)
        // To skew left, we subtract a little speed from the left side. (bias < 0)
        LFMotorSpeed = speed + leftRightBias;
        LRMotorSpeed = speed + leftRightBias;
    }
    else //speed < 0
    {
        // Reverse the motors and take the absolute value of the speed to make it positive.
        LFMotor->run(BACKWARD);
        LRMotor->run(BACKWARD);
        speed = abs(speed);
        // Wheels are turning in reverse. The skew right, we want to slow them down, and to
        // skew left, we want to speed them up, so the sign is reversed from the forward case.
        LFMotorSpeed = speed - leftRightBias;
        LRMotorSpeed = speed - leftRightBias;
    }
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
        // To skew right, we subtract a little speed from the right side. (bias > 0)
        // To skew left, we add a little speed to the right side. (bias < 0)
        RFMotorSpeed = speed - leftRightBias;
        RRMotorSpeed = speed - leftRightBias;
    }
    else
    {
        RFMotor->run(BACKWARD);
        RRMotor->run(BACKWARD);
        speed = abs(speed);
        // Wheels are turning in reverse. The skew right, we want to speed them up, and to
        // skew left, we want to slow them down, so the sign is reversed from the forward case.
        RFMotorSpeed = speed + leftRightBias;
        RRMotorSpeed = speed + leftRightBias;

    }
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
    SetSpeedLeft(0);
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
    SetSpeedLeft(0);
    // set course section back to zero
    courseSection = 0;
}

// Mode transition from pause to standby. Handles a button push during pause.
void ModePauseToStandby()
{
    mode = modeStandby;
    pauseLed.Disable();
    digitalWrite(ledPortYellow, ledOn);
    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft(0);
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

// Mode transition from pause to run. This happens when the pause timer expires
// and we want to resume our run.
void ModePauseToRun()
{
    mode = modeRun;
    runLed.Enable();
    pauseLed.Disable();
    //When we leave pause mode and go back to run, we may still be over the second
    // line of the double line we paused for. To avoid double counting that line,
    // we set the firstLineBlockTimer and block line seeking until it expires.
    firstLineBlockTimer.Start(seekFirstLineBlockTime);
    crossLineState = seekingBlocked;
    // tell section tracking code crosswalk is done
    finishingCrosswalk = true;
}

// Mode transition from run to pause. This happens when a double cross line is detected.
void ModeRunToPause()
{
    mode = modePause;
    runLed.Disable();
    pauseLed.Enable();

    // stop motors
    SetSpeedRight(0);
    SetSpeedLeft(0);

    // Pause mode is 3 seconds long. Start timer. The timer will be polled during pause
    // mode in the main loop until the 3 seconds is past, then the transition back to
    // run mode will happen.

    pauseTimer.Start(pauseModeDuration);  // set timer to 3 seconds
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
                                         (_  |_  | | | |_) /  \
                                         __) |_  | |_| |   \  /
*/
//
// Initialize state, set up hardware, prepare starting state
//
void setup()
{
    // Start serial port for debugging
    Serial.begin(115200);  // open the serial port at 115200 bps:

    // Start real time clock
    realTimeClock.begin();  // connect real time clock to I2C bus
    realTimeClock.start();  // clear the stop bit. Normally not necessary, but doesn't hurt.

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
    imu.begin(0x69);
    imu.setGyroRange(MPU6050_RANGE_250_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    // initialize inCorrection. This is used by the line following code
    inCorrection = none;

    // turn off seven segment display by deactivating both anodes
    digitalWrite(ledPort7SegAnodeOnes, ledOff);
    digitalWrite(ledPort7SegAnodeTens, ledOff);

    // initialize course section and turn tracking state
    courseSection = 0;
    turnTrackState = idle;
    finishingCrosswalk = false;
    leftRightBias = 0;
    trackTurnTo = none;

    turnStartThreshold = normalTurnStartThreshold;
    turnEndThreshold = normalTurnEndThreshold;


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
                                       |  / \ / \ |_) /  \
                                       |_ \_/ \_/ |   \  /
*/
void loop()
{
    /*                                        _   _   _  __    _  _   _   _
                            /\  |  |    |\/| / \ | \ |_ (_    /  / \ | \ |_
                           /--\ |_ |_   |  | \_/ |_/ |_ __)   \_ \_/ |_/ |_

          read IR line sensors, digital version. This just reads the digital value
          from the sensor, which sets its threshold with a trim pot.
    */
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

    // resd ambient light sensor. Turn on headlights on at the low light threshold and off
    // at the high light threshold

    int lightLevel = analogRead(analogLightSensorPort);

    if (lightLevel < lightsOnThreshold)
    {
        digitalWrite(ledPortWhite, ledOn);
    }

    if (lightLevel > lightsOffThreshold)
    {
        digitalWrite(ledPortWhite, ledOff);
    }

    /*
        Sample the Z axis gyro rotation rate and the X axis acceleration.
    */

    /*
        Sample the z-axis gyro, putting the value into the global variable zRotation. After the first read,
        we actually read the gyro sensor every imuTimeStep milliseconds. Between these reads, we keep the
        previous value. We also sample the X axis acceleration here, so we can use it to detect the ramps.
    */
    if (imuGyroTimer.Test()) // check if timer has reached its limit. The first time through, this test will succeed.
    {
        //mpu6050 sensor events.

        imu.getEvent  (&accelerationsXYZ, &rotationRatesXYZ, &temperature);
        zRotationRate = rotationRatesXYZ.gyro.z - imuGyroZDrift;
        xAcceleration = accelerationsXYZ.acceleration.x;

        // We do not read the IMU values in every loop cycle, because it is not necessary for our
        // application. Set the timer for tne next read.
        imuGyroTimer.Start(imuTimeStep);
    }
#ifdef DEBUGTURNSENSE
    Serial.print("Z rotation: ");
    Serial.print(zRotationRate);
    Serial.print ("X accel: ");
    Serial.println(xAcceleration);
#endif

    /*                      _                   _   _   _    _  _   _   _
                           |_) | | |\ |   |\/| / \ | \ |_   /  / \ | \ |_
                           | \ |_| | \|   |  | \_/ |_/ |_   \_ \_/ |_/ |_
    */
    if (mode == modeRun)
    {
        runLed.Update();  // Update Run flasher.
        DisplayCount(courseSection);



        /*******************************Using gyro to track turns*****************************
            We want to know when turns begin and when they end, and we know when to expect each turn. By using
            the gyro, we can directly detect the turns and the ends of the turns and leave the sprint
            timers to the job of controlling speed, not track section switching.

            1. Don't use the sprint timer for switching track sections. Instead, use the timers just to
            control the speed through most of a straight section. Add another speed for after the
            timer expires to avoid overruns at the beginning of turns, though biasing the straight section
            toward the outside of turn might be adequate for everything except sudden changes in direction
            like doglegs.

            2. Track the turn using the gyro. The gyro detects the rate of the turn in radians per
            second. Because the turn is taken as a series of corrections, we need a way of smoothing
            out the choppiness of the corrections. A running average or equivalent should work for this.
            We do this by just accumulating the turn rates for a fixed amount of time and then test
            the sum rather than doing a floating point divide to get the real average.
            When the sum drops below a threshold, the turn is finished and we go on the
            the next section.

            Implementation:
            turnTrackState tells us where we are in detection and tracking a turn...
                - idle means we are not tracking because we don't need to
                - waitingForRightTurn means we expect a right turn, but we are not looking for
                  a left edge. Instead, we are waiting for the gyro to detect the beginning
                  of a right turn, so when we are in this state, we look for a sustained rate
                  of rotation to the right.
                - waitingForLeftTurn means we expect a left turn. Same deal as above for right turns.
                - tracking means we are in a turn and waiting for the turn to end.
            inTurn which has meaning when turnTrackState == tracking
                - left means a left turn is in progress
                - right means a right turn is in progress
                - none means we are not detecting a turn, usually signaling the end of a turn.
                - unknown means we are not tracking turns (turnTrackState == idle)

            We sample the gyro and accelerometer at the beginning of loop(), updating the global
            variabes zRotationRate and xAcceleration. We can directly use zRotationRate to determine
            the current dynamics of the robot. xAcceleration can be used to detect the ramps.

            To determine if we are in a turn, we want a kind of average of the zRotationRate over time,
            so we don't get false positives from short term events, and we want to track the beginning
            and end of the turn. We know when we are looking for the beginning of the turn because
            turnTrackState is either waitingForLeftTurn or waitingForRightTurn, and we know that a
            turn in that direction is next on the track, so it is a matter of when the turn starts,
            not whether there is a turn. The smooth out the measurements, we add up the rotation rates
            over a period of time, on the order one or two tenths of a second, then we test the sum
            against a threshold. If the turn started near the end of such a time slice, this sum may be
            too small, but will will get it in the next time slice and change our tracking state to "tracking"
            and restart the timer.

            During turnTrackState == tracking, we are looking for the end of the turn, and we use the same
            summing approach, this time looking for a |sum| below a threshold. Again, if the turn ends near
            the end of a time slot, we will catch in in the next one. then set turnTrackState back to idle.
        */

        if (turnTrackState == idle)
        {
            inTurn = unknown;
            rotationAccum = 0.0; // accumulator for gyro values
            // We start the timer every time here so that when turn detection starts, this time
            // will be in sync.
            turnTimer.Start(turnSampleInterval);

#ifdef DEBUGTURNTRACK
            if (prevTurnTrackState != turnTrackState)
            {
                Serial.print(millis());
                Serial.println(" idle");
                prevTurnTrackState = idle;
            }
#endif

        }
        else if (turnTrackState == waitingForRightTurn)
        {
            if (turnTimer.Test())
            {
                //Serial.println(rotationAccum);

#ifdef DEBUGTURNTRACK
                if (prevTurnTrackState != turnTrackState)
                {
                    Serial.print(millis());
                    Serial.println(" waitingForRightTurn");
                    prevTurnTrackState = waitingForRightTurn;
                }
#endif

                // timer has expired. Test the accumulated turn events to see if we have detected
                // the existence of a right turn. If so, change turnTrackState to tracking.
                if (rotationAccum < -turnStartThreshold) //right turn is negative
                {
                    // turn detected. Switch to tracking state
                    turnTrackState = tracking;
                    inTurn = right;
                }
                // whether or not we are changing state, we want to start the timer
                // again and zero the accumulator
                turnTimer.Start(turnSampleInterval);
                rotationAccum = 0.0;
            }
            else
            {
                rotationAccum += zRotationRate;
            }
        }
        else if (turnTrackState == waitingForLeftTurn)
        {
            if (turnTimer.Test())
            {
                //Serial.println(rotationAccum);

#ifdef DEBUGTURNTRACK
                if (prevTurnTrackState != turnTrackState)
                {
                    Serial.print(millis());
                    Serial.println(" waitingForLeftTurn");
                    prevTurnTrackState = waitingForLeftTurn;
                }
#endif

                // timer has expired. Test the accumulated turn events to see if we have detected
                // the existence of a left turn. If so, change turnTrackState to tracking.
                if (rotationAccum > turnStartThreshold) //left rotation is positive
                {
                    // turn detected. Switch to tracking state
                    turnTrackState = tracking;
                    inTurn = left;
                }
                // whether or not we are changing state, we want to start the timer
                // again and zero the accumulator
                turnTimer.Start(turnSampleInterval);
                rotationAccum = 0.0;
            }
            else
            {
                rotationAccum += zRotationRate;
            }
        }
        if (turnTrackState == tracking)
            // When we are tracking a turn, we have already started the turn and we are going to track it until it ends,
            // either by becoming a turn in the opposite direction, or no turn. We use the variable trackTurnTo to specify
            // what end state we want to use. The default value if trackTurnTo == none, because most of our turns end in
            // a straight section. In any of these cases, we stop the tracking when the end case is detected, so if we are
            // looking for a left to right transition or a right to left transition, we need to restart the tracking
            // sequence with turnTrackState = waitingForRightTurn or waitingForLeftTurn even though we will already be in that
            // turn.
        {
            if (turnTimer.Test())
            {
                //Serial.println(rotationAccum);

#ifdef DEBUGTURNTRACK
                if (prevTurnTrackState != turnTrackState)
                {
                    Serial.print(millis());
                    Serial.println(" tracking");
                    prevTurnTrackState = tracking;
                }
#endif
                // timer has expired. Test the accumulated turn events to see if we have detected
                // the specified trackTurnTo target value. If so, change turnTrackState to idle.

                if (trackTurnTo == left)
                {
#ifdef DEBUGTURNTRACK
                    if (prevTrackTurnTo != trackTurnTo)
                    {
                        Serial.println("   tracking until left");
                        prevTrackTurnTo = trackTurnTo;
                    }
#endif
                    if (rotationAccum > turnStartThreshold) //left rotation is positive
                    {
                        // accumulated rotation is > the turn start threshold, so we are now in a left turn
                        // which is what we were seeking. This will happen when we are tracking from a right
                        // turn directly into a left turn.
                        turnTrackState = idle;
                        inTurn = left;
                        trackTurnTo = none; // resetting to the default
#ifdef DEBUGTURNTRACK
                        Serial.println("     left turn detected. inTurn==left");
#endif


                    }
                }
                else if (trackTurnTo == right)
                {
                    // DEBUG
#ifdef DEBUGTURNTRACK
                    if (prevTrackTurnTo != trackTurnTo)
                    {

                        Serial.println("   tracking until right");
                        prevTrackTurnTo = trackTurnTo;
                    }
#endif
                    if (rotationAccum < -turnStartThreshold)
                    {
                        // right turn rotation is negative, so we tested against -threshold. The test is true,
                        // so we are in a right turn. This will happen when we are tracking from a left turn
                        // directly into a right turn.
                        turnTrackState = idle;
                        inTurn = right;
                        trackTurnTo = none; // resetting to the default
#ifdef DEBUGTURNTRACK
                        Serial.println("     right turn detected. inTurn==right");
#endif

                    }
                }

                else //if (trackTurnTo == none)
                {
#ifdef DEBUGTURNTRACK
                    if (prevTrackTurnTo != trackTurnTo)
                    {
                        Serial.println("   tracking to none");
                        prevTrackTurnTo = trackTurnTo;
                    }
#endif
                    if (abs(rotationAccum) < turnEndThreshold) // absolute value of accumulated rotation below the
                    {
                        // Absolute value of accumulated rotation is below the "not in turn" threshold.
                        // Turn is finished.
                        turnTrackState = idle;
                        inTurn = none;
#ifdef DEBUGTURNTRACK
                        Serial.println("     End of turn detected. inTurn==none");
#endif
                    }
                }

                // whether or not we are changing state, we want to start the timer
                // again and zero the accumulator
                turnTimer.Start(turnSampleInterval);
                rotationAccum = 0.0;
            }
            else
            {
                rotationAccum += zRotationRate;
            }
        }

        /*******************************Detect Course Segment*********************************

                We start in segment 11 at the beginning of the course. Each section transition has a criterion
                for advancing to the next section. For the straight sections, this is a timer, which will
                need to track the time it takes to the next turn. For the turns, it is detection of the
                opposite road edge, eg a right turn ends when the right edge of the roadway is detected.
                Some transitions are detected when a cross line is detected, like the first crosswalk.

                Turns are tracked from the first time we detect the outside edge of the road until the time
                we see a small number or correction events. The turn tracking code will start by looking for
                the specified correction (e.g. turnTrackState = waitingForRightTurn) As it tracks, it periodically
                sets the global variable inTurn. To detect the end of a turn, we look for inTurn == none, at which
                point we advance to the next track section. There are places where we will bias our straight runs
                to the right of left to avoid falsely detecting a curve starting edge. E.G. when we expect a right
                turn, we might bias to the right to avoid any left edge encounters. This becomes important if there
                is a mechanical hesitation at the beginning of a timed run, and the timer expires significantly
                before the expected turn, resulting in seeing the initiating correction followed by the inTurn == none.
                (Another way of dealing with this might be using the gyro to detect the real turn before looking for
                the end of the turn)

        */
        switch (courseSection)
        {
            case 0:
                /*
                    Section zero is just the beginning of the course. It kicks off section 11 by starting the timer
                    for the first sprint.
                */
                courseSection = 11;
                sprintTimer.Start(sec11SprintTime);
                leftRightBias = sec11LeftRightBias;

            case 11:
                ProcessStraightSection  //first straight section. Sprint to just before the first turn.
                (
                    sec11FastSideSpeed1,
                    sec11SlowSideSpeed1,
                    sec11FastSideSpeed2,
                    sec11SlowSideSpeed2,
                    12,
                    right
                );
                break;

            case 12:
                ProcessTurnSection  //first right turn. Section endss when the turn ends.
                (
                    sec12FastSideSpeed,
                    sec12SlowSideSpeed,
                    13,
                    sec13SprintTime,
                    sec13LeftRightBias
                );
                break;

            case 13:
                ProcessStraightSection  //straight section right after the first right turn.
                (
                    sec13FastSideSpeed1,
                    sec13SlowSideSpeed1,
                    sec13FastSideSpeed2,
                    sec13SlowSideSpeed2,
                    14,
                    right
                );
                break;

            case 14:
                /*
                    Second right turn. This is not followed by a straight section, so we need to do something
                    different. The previous section transition set the turn tracking state to waitForRightTurn,
                    and we are now in that right turn. We are going to let the turn tracking go all the way to
                    the point where it detects a left turn. To do this, we have to tell the turn tracking code
                    to look for a left turn, not a non-turn, so we set trackTurnTo = left;
                */
                fastSideSpeed = sec14FastSideSpeed;
                slowSideSpeed = sec14SlowSideSpeed;
                trackTurnTo = left;

                if (inTurn == left)
                {
                    courseSection = 15;
                    turnTrackState = waitingForLeftTurn; //restart turn tracking for the left turn
                }
                break;

            case 15:
                /*
                    ***** gentle left turn just before the turn onto the ramp. we advance to the
                    next section when we detect a shift to a right turn.
                */
                fastSideSpeed = sec15FastSideSpeed;
                slowSideSpeed = sec15SlowSideSpeed;
                trackTurnTo = right;

                if (inTurn == right)
                {
                    courseSection = 21;
                    turnTrackState = tracking;//waitingForRightTurn; //restart turn tracking.
                }
                break;

            case 21:

                turnTrackState = tracking;

                ProcessTurnSection  //right turn onto up ramp. track turn until it ends
                (
                    sec21FastSideSpeed,
                    sec21SlowSideSpeed,
                    22,
                    sec22SprintTime,
                    sec22LeftRightBias
                );
                break;

            case 22:


                ProcessStraightSection  //straight section to turn at top of ramp. End at start of turn.
                (
                    sec22FastSideSpeed1,
                    sec22SlowSideSpeed1,
                    sec22FastSideSpeed2,
                    sec22SlowSideSpeed2,
                    31,
                    right
                );
                break;

            case 31:
                ProcessTurnSection  //right turn onto down ramp. track turn until it ends
                (
                    sec31FastSideSpeed,
                    sec31SlowSideSpeed,
                    32,
                    sec32SprintTime,
                    sec32LeftRightBias
                );
                break;

            case 32:
                ProcessStraightSection  //straight section to turn at bottom of ramp. End at start of turn.
                (
                    sec32FastSideSpeed1,
                    sec32SlowSideSpeed1,
                    sec32FastSideSpeed2,
                    sec32SlowSideSpeed2,
                    41,
                    right
                );
                break;

            case 41:
                ProcessTurnSection  //right turn off of down ramp. track turn until it ends
                (
                    sec41FastSideSpeed,
                    sec41SlowSideSpeed,
                    42,
                    sec42SprintTime,
                    sec42LeftRightBias
                );
                break;

            case 42:
                turnStartThreshold = sec43TurnStartThreshold; //increase turn sensitivity for the next turn
                ProcessStraightSection  //Straight section to tunnel approach turn (right). End at start of turn.
                (
                    sec42FastSideSpeed1,
                    sec42SlowSideSpeed1,
                    sec42FastSideSpeed2,
                    sec42SlowSideSpeed2,
                    43,
                    right
                );
                break;

            case 43:
                // Because this is a wide turn, detecting both its beginning and end with the gyro
                // is not reliable. Because the turn ends in the tunnel, we just detect the darkness
                // inside the tunnel, which is a little early, but should not be a problem.

                fastSideSpeed = sec43FastSideSpeed;
                slowSideSpeed = sec43SlowSideSpeed;
                if (lightLevel < lightsOnThreshold)
                {

                    courseSection = 44;
                    sprintTimer.Start(sec44SprintTime);
                    leftRightBias = sec44LeftRightBias;
                    turnStartThreshold = normalTurnStartThreshold; //return to normal turn sensitivity
                }
                break;

            case 44:

                // run at sprint speed during the sprint time. We want to slow down just before the crosswalk so
                // we have time to stop before running into the road.
                if (sprintTimer.Test())
                {
                    fastSideSpeed = sec44FastSideSpeed1;
                    slowSideSpeed = sec44SlowSideSpeed1;
                }
                else
                {
                    fastSideSpeed = sec44FastSideSpeed2;
                    slowSideSpeed = sec44SlowSideSpeed2;
                }

                // This section ends at the end of the crosswalk pause, which we detect
                // by testing the finishingCrosswalk flag, which is set just as the pause finishes.
                // This section is followed by the rest of this straight part, so we set up section
                // 51 as a straight section.

                if (finishingCrosswalk)
                {
                    finishingCrosswalk = false; //consume the flag
                    courseSection = 51;
                    sprintTimer.Start(sec51SprintTime);
                    leftRightBias = sec51LeftRightBias;
                }
                break;

            case 51:
                // set turn thresholds for the next turn
                turnStartThreshold = sec52TurnStartThreshold;
                turnEndThreshold = sec52TurnEndThreshold;
                ProcessStraightSection  //Straight section from first crosswalk to right turn into the figure 8. Ends with beginning of turn
                (
                    sec51FastSideSpeed1,
                    sec51SlowSideSpeed1,
                    sec51FastSideSpeed2,
                    sec51SlowSideSpeed2,
                    52,
                    right
                );
                break;

            case 52:
                ProcessTurnSection  //Right turn toward figure 8. Track the turn until it ends.
                (
                    sec52FastSideSpeed,
                    sec52SlowSideSpeed,
                    53,
                    sec53SprintTime,
                    sec53LeftRightBias
                );
                turnStartThreshold = normalTurnStartThreshold;
                turnEndThreshold = normalTurnEndThreshold;

                break;

            case 53:
                ProcessStraightSection  //Short straight section before a right dogleg. Ends whe the dogleg starts.
                (
                    sec53FastSideSpeed1,
                    sec53SlowSideSpeed1,
                    sec53FastSideSpeed2,
                    sec53SlowSideSpeed2,
                    54,
                    right
                );
                break;

            case 54:
                ProcessTurnSection  //Right dogleg in figure 8. Ends when turn finishes.
                (
                    sec54FastSideSpeed,
                    sec54SlowSideSpeed,
                    55,
                    sec55SprintTime,
                    sec55LeftRightBias
                );
                break;
            case 55:

                // run at sprint speed during the sprint time. We want to slow down just before the crosswalk so
                // we have time to stop before running into the road.
                if (sprintTimer.Test())
                {
                    fastSideSpeed = sec55FastSideSpeed1;
                    slowSideSpeed = sec55SlowSideSpeed1;
                }
                else
                {
                    fastSideSpeed = sec55FastSideSpeed2;
                    slowSideSpeed = sec55SlowSideSpeed2;
                }

                // This section ends at the end of the crosswalk pause, which we detect
                // by testing the finishingCrosswalk flag, which is set just as the pause finishes.
                // This section is followed by the rest of this straight part, so we set up section
                // 61 as a straight section.

                if (finishingCrosswalk)
                {
                    finishingCrosswalk = false; //consume the flag
                    courseSection = 61;
                    sprintTimer.Start(sec61SprintTime);
                    leftRightBias = sec61LeftRightBias;
                }
                break;

            case 61:
                ProcessStraightSection  //Straight section from second crosswalk to first sharp left turn in figure 8
                (
                    sec61FastSideSpeed1,
                    sec61SlowSideSpeed1,
                    sec61FastSideSpeed2,
                    sec61SlowSideSpeed2,
                    62,
                    left
                );
                break;

            case 62:
                ProcessTurnSection  //Left turn into bottom of figure 8. Track the turn until it ends.
                (
                    sec62FastSideSpeed,
                    sec62SlowSideSpeed,
                    63,
                    sec63SprintTime,
                    sec63LeftRightBias
                );
                break;

            case 63:
                ProcessStraightSection  //Straight section at the bottom of the figure 8. Ends at left turn out of figure 8
                (
                    sec63FastSideSpeed1,
                    sec63SlowSideSpeed1,
                    sec63FastSideSpeed2,
                    sec63SlowSideSpeed2,
                    64,
                    left
                );
                break;

            case 64:
                ProcessTurnSection  //Left turn toward the figure 8 crossing point. Track the turn until it ends.
                (
                    sec64FastSideSpeed,
                    sec64SlowSideSpeed,
                    65,
                    sec65SprintTime,
                    sec65LeftRightBias
                );
                break;

            case 65:
                ProcessStraightSection  //Straight section coming out of the bottom of the figure 8. Ends with dogleg right turn.
                (
                    sec65FastSideSpeed1,
                    sec65SlowSideSpeed1,
                    sec65FastSideSpeed2,
                    sec65SlowSideSpeed2,
                    71,
                    right
                );
                break;

            case 71:
                ProcessTurnSection  //Right dogleg turn after crossing center of figure 8 . Ends when turn ends.
                (
                    sec71FastSideSpeed,
                    sec71SlowSideSpeed,
                    72,
                    sec72SprintTime,
                    sec72LeftRightBias
                );
                break;

            case 72:
                ProcessStraightSection  //Straight section exiting the figure 8, ending right turn
                (
                    sec72FastSideSpeed1,
                    sec72SlowSideSpeed1,
                    sec72FastSideSpeed2,
                    sec72SlowSideSpeed2,
                    73,
                    right
                );
                break;

            case 73:
                ProcessTurnSection  //Right turn to horizontal section at top of field. Ends when turn ends.
                (
                    sec73FastSideSpeed,
                    sec73SlowSideSpeed,
                    74,
                    sec74SprintTime,
                    sec74LeftRightBias
                );
                break;

            case 74:
                turnStartThreshold = sec81TurnStartThreshold;
                turnEndThreshold = sec81TurnEndThreshold;

                ProcessStraightSection  //Straight section to right turn onto banked section. Ends at right turn
                (
                    sec74FastSideSpeed1,
                    sec74SlowSideSpeed1,
                    sec74FastSideSpeed2,
                    sec74SlowSideSpeed2,
                    81,
                    right
                );
                break;

            case 81:
                ProcessTurnSection  //Right turn to banked section. Ends when turn ends.
                (
                    sec81FastSideSpeed,
                    sec81SlowSideSpeed,
                    82,
                    sec82SprintTime,
                    sec82LeftRightBias
                );
                turnStartThreshold = normalTurnStartThreshold;
                turnEndThreshold = normalTurnEndThreshold;

                break;

            case 82:
                ProcessStraightSection  //Straight banked section to right turn to home stretch section. Ends at right turn
                (
                    sec82FastSideSpeed1,
                    sec82SlowSideSpeed1,
                    sec82FastSideSpeed2,
                    sec82SlowSideSpeed2,
                    83,
                    right
                );
                break;

            case 83:
                ProcessTurnSection  //Right turn to home stretch. Ends when turn ends.
                (
                    sec83FastSideSpeed,
                    sec83SlowSideSpeed,
                    84,
                    sec84SprintTime,
                    sec84LeftRightBias
                );
                break;

            case 84:
                /*
                    Straight section to finish line. Because the finish line is a double line, the double
                    line detection code will know it is the finish line and do its finish line routine
                    (run past the finish line on a timer, then switch to standby mode.)
                */
                ProcessStraightSection
                (
                    sec84FastSideSpeed1,
                    sec84SlowSideSpeed1,
                    sec84FastSideSpeed2,
                    sec84SlowSideSpeed2,
                    85,
                    right               // this state should never end because the robot will go to
                    // standby mode first, but if that fails to happen, we will
                    // detect the first turn after the start/end line and stop in
                    // section 85, below.
                );
                break;

            case 85:
                // End of run. The code will get here only if we reach the right turn after the finish line,
                // which should never happen. We just stop if this turn is detected
                fastSideSpeed = 0;
                slowSideSpeed = 0;
                break;

            default:
                break;

        }  // *************************End of course section id code****************************


        /*************************************Line following algorithm************************

                There are two line sensors, one on the front right
                corner and one on the front left corner. When one of these sensors sees a line,
                it sets the wheels on the opposite side to slow way down or reverse, which causes
                the robot to turn away from the line. When the sensor stops seeing the line,
                those wheels return to normal forward rotation. There are a few special cases...

                - At places where the road crosses itself, there is a line that goes across the
                road. At that point, both line sensors will see a line. When that happens, we do
                not correct direction, but continue to go straight. This depends on the robot not
                being at a significant angle when it encounters a cross line. Since all of the cross lines
                are in straight sections, this should always work unless we have overcorrected just before
                a cross line.

                - For cross lines, we have some special cases.
                    1. If all three sensors (left right, middle) are dark, we assume we are crossing
                    the line. straight on. In this case, we do not correct.
                    2. If middle and right are dark, we assume we are hitting the cross line aiming left,
                    so we correct by turning right. We may want to adjust relative left and right speeds
                    if this overcorrects.
                    3. if middle and left are dark, we assume we are hitting the cross line aiming right,
                    so we correct by turning left. See caveat above.
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
        else if (lineSensorValRight == dark && lineSensorValMiddle == light)   /*|| lineSensorValRightOuter == dark*/
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

        if (inCorrection == left)  // Correcting for left side line detection
        {
            // line has been detected on the left side. We want to turn right to clear the line
            SetSpeedRight(slowSideSpeed);
            SetSpeedLeft(fastSideSpeed);
        }
        else if (inCorrection == right)    // Correcting for right side line detection
        {
            // line has been detected on the right side. We want ato turn left to clear the line
            SetSpeedRight(fastSideSpeed);
            SetSpeedLeft(slowSideSpeed);
        }
        else if (inCorrection == none)    //no correction needed
        {
            // just go straight
            SetSpeedRight(fastSideSpeed);
            SetSpeedLeft(fastSideSpeed);
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
                secondLineTimer.Start(seekingSecondLineTimeWindow);  //open time window for finding a second line
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
                // Second line time window is still open. Read sensor for second line.
                // Just looking for light-dark transition here because we want to switch
                // to pause mode right away if we see it.

                if (centerLineSensorReader.Sample() == dark)
                {
                    // increment the cross line counts.
                    crossLineCount++;
                    doubleLineCount++;

                    // set next mode based on the new line count...

                    if (doubleLineCount == 1)
                    {
                        // Stay in run mode. This is the start of the course. But start the
                        // first line detect timeout to keep from detecting this line again.
                        firstLineBlockTimer.Start(seekFirstLineBlockTime);
                        crossLineState = seekingBlocked;
                    }
                    else if (doubleLineCount == totalDoubleLineCrossings)
                    {
                        // double line is the start/finish line. Set the finish line timer and
                        // the state flag indicating we are at the finish line. When the timer
                        // has expired, we will stop the robot and go into standby mode
                        finishLineTimer.Start(finishLineDelay);
                        waitingForFinalStandby = true;

                        //
                        // Just to make sure we stop looking for crosslines and avoid finding
                        // this one again on the next time around the loop, we block line seeking
                        // and start first line block timer. This code is probably not needed.
                        firstLineBlockTimer.Start(seekFirstLineBlockTime);
                        crossLineState = seekingBlocked;
                    }
                    else if (doubleLineCount < totalDoubleLineCrossings)
                    {
                        // double line is a crosswalk. Transition to pause mode. The next call
                        // to loop() will go to the pause mode code.
                        ModeRunToPause();
                    }
                }
            }
            else
            {
                // second line time window has expired. This means we have failed to find a
                // second line, close enough to the first one, so there was only one line at
                // this location and we can start looking for the next one.
                crossLineState = seekingFirstLine;  //start looking for the next line
            }
        }
        else if (firstLineBlockTimer.Test())    // crossLineState == seekingBlocked
        {
            // here we are not seeking the first or second line, but the timer we started to avoid double
            // counting the second line has expired, so we can start looking for a first line again.
            crossLineState = seekingFirstLine;
        }
        else
        {
            // if we fall through to here, we have left pause mode, but are still waiting to start
            // looking for lines again, so we do nothing. This will keep happening until the
            // firstLineBlockTimer expires.
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
        if (buttonReader.CheckCycle())  // when the button is pressed and released, we go to standby mode
        {
            ModeRunToStandby();
        }

        /***********************************Finish Line Processing***************************/
        // check if we have passed the finish line. and the finish line timer has expired.
        // We want to run long enough after
        // detecting the finish line to fully pass it before going into pause mode.
        if (waitingForFinalStandby && finishLineTimer.Test())
        {
            ModeRunToStandby();
        }
    }  // end of run mode code
    /*                         ___ _  __ ___         _   _   _    _  _   _   _
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

        // sample the gyro every imuTimeStep milliseconds
        //

        //if (imuGyroTimer.Test()) // check if timer has reached its limit
        {
            //mpu6050 sensor events. We only care about the gyro, but the library only supports reading all of them.
            sensors_event_t accelerationsXYZ,  rotationRatesXYZ,  temperature;
            imu.getEvent  (&accelerationsXYZ, &rotationRatesXYZ, &temperature); // get the acceleration, rotation and temperature values
            static float cumulative = 0;

            float xTilt = accelerationsXYZ.acceleration.x;
            float yTilt = accelerationsXYZ.acceleration.y;
            float zRotationRate = rotationRatesXYZ.gyro.z - imuGyroZDrift;
            cumulative += zRotationRate;
            //Y Tilt on banked section: >1,5 (.6 or so at starting line)
            //X tilt on upramp >1.7 (~.7 at starting line)
            //downramp <.5 at top <.8 at middle
            Serial.print(", Z: ");
            Serial.print(zRotationRate);
            Serial.print(" rad/s ");
            Serial.print("cumulative ");
            Serial.print(cumulative);
            Serial.print(" X tilt: ");
            Serial.print(xTilt);
            Serial.print(" Y tilt: ");
            Serial.println(yTilt);

            // imuGyroTimer.Start(imuTimeStep); // start timer with time step delay
        }

        /*
                Test seven segment display. DisplayCount is called every cycle, but the
                number to display is changed every second or so. Note DisplayCount has its
                own timer for its refresh rate, so a lot of times when it is called, it does
                nothing but maintain the current display state.
        */
        /*
            static int count = 0;
            DisplayCount(count);
            if (testTimer.Test())
            {
                testTimer.Start(1000);
                count++;
            }
        */
    }  // end of test
    /*                         __ ___           _    _              _   _   _    _  _   _   _
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

        // refresh 7 segment display with whichever value it is set to display
        update7SegDisplay();

        static boolean countdownToRun = false;  //static variable to hold state between calls to loop()
        if (countdownToRun)
        {
            if (standbyToRunTimer.Test())  // poll the timer. When it is finished, enter run mode
            {
                countdownToRun = false;
                ModeStandbyToRun();
            }
        }
        else if (buttonReader.CheckCycle())    // when the button is pressed and released, we go to run mode after a delay.
        {
            // Note: CheckButtonCycle returns true only once per button cycle.
            countdownToRun = true;          //flag that we are in the countdown to run
            standbyToRunTimer.Start(2000);  //Start pause to run timer for 2 second countdown
        }
        else
        {
            // do nothing while we wait for button cycle
        }
    }  // end of standby

    /*                           _           __  _         _   _   _    _  _   _   _
                                |_) /\  | | (_  |_   |\/| / \ | \ |_   /  / \ | \ |_
                                |  /--\ |_| __) |_   |  | \_/ |_/ |_   \_ \_/ |_/ |_
    */

    else if (mode == modePause)
    {
        //
        // Pause mode code
        //
        pauseLed.Update();  // update pause LED flasher object

        // update the 7 segment display with whatever value it is set to display
        update7SegDisplay();

        //boolean pauseDone = pauseTimer.Test();   //poll the pause timer
        if (pauseTimer.Test())
        {
            ModePauseToRun();
        }
        //
        // For the rare case that the button is pressed during pause mode:
        // Look for a button press and release. If we see it, then switch to standby mode.
        //
        if (buttonReader.CheckCycle())  // when the button is pressed and released, we go to standby mode
        {
            ModePauseToStandby();
        }
    }  // end of  pause
}  // end of loop()
