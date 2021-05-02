/*
  STAIRCASE

  This sketch supports automatic lighting for a staircase with the following characteristics:
  
  * An (analogue) motion sensor signal indicates the presence of a body
  * Pressure pads at the top and bottom of the staircase determine the stairs are being climbed
  * An (analogue) light sensor indicates the ambient light level
  * A PWM-controlled LED light illuminates the staircase

  In the presence of a body, the light will be partially illuminated, fading up to the required level
  and fading down when the presence is removed.

  When/if the stairs are climbed, the lights will be fully illuminated, fading up to the required level
  and fading down after a fixed period.

  The operation described can be suppressed while the ambient light exceeds a certain threshold.

  The threshold for determining that presence has been sensed and the threshold for the ambient light level
  are set by analog voltages (from potentiometers adjustable by the user).

  Implementation details:

    To avoid the need for calibration, an exponential moving average is calcuated of the sensor readings so 
    gradual changes in room temperature or short term changes in ambient lighting should not affect the
    reliability of operation.

    The threshold value for motion detection is scaled down to a smaller range as it will typically be a small
    fraction of the full range of the ADC and it allows finer adjustment of the potentiomenter.

    The motion sensor is unstable for around 10-30 seconds after power is first applied, so we ignore motion
    events during this period.

    Turning the lights on obviously effects the light sensor value, but can also falsely trigger the motion 
    sensor. Consequently, we don't update the ambient light calculation when the lights are on and also 
    we suppress events from the motion sensor for 10 seconds after the lights have faded down.

    We use Timer 1 to generate interrupts sufficiently fast to adjust the PWM signal to smoothly fade the lights.
    A simple "Finite State Machine" handles the bulk of the logic.
*/

#include <Arduino.h>
#include "TimerOne.h"

//#define DEBUG 1


/****************************************************/
/*                 Controller PINS                  */
/****************************************************/

const int pinPad = 4;                                                   // Pressure pad to D4
const int pinPWM = 3;                                                   // PWM output on D3
const int pinMotion = A0;                                               // Motion sensor analog input pin
const int pinLight = A1;                                                // Light sensor analog pin
const int pinThresholdMotion = A2;                                      // Pin to which motion threshold pot is attached
const int pinThresholdLight = A3;                                       // Pin to which light threshold pot is attached

const int thresholdMotionShift = 4;                                     // Number of bits to discard in motion threshold value -
                                                                        // gives finer adjustment as the actual threshold will be small

/****************************************************/
/*                 Timing Constants                 */
/****************************************************/

const long timerTick = 50;                                              // Desired timer tick in mSec
const long timerPeriod = timerTick * 1000L;                             // Equivalent period in uSec for timer
const int loopDelay = 250;                                              // Delay period (mS) in each iteration of main loop

/*
 * The following time periods must exceed the 1.5 seconds which is the fade-up/down time
 */
const int timePresence = 20;                                            // Time for which "presence" state should remain active (Sec)
const int timeTrigger = 20;                                             // Time for which ive"trigger" state should remain active (Sec)
const int timeResetInitial = 30;                                        // Time for which initial "reset" state persists (Sec)
const int timeResetSubsequent = 10;                                     // Time for which subsequent "reset" state persists (Sec)
const long ticksPresence = timePresence * 1000 / timerTick;             // Number of timer intervals to hold "presence" condition
const long ticksTrigger = timeTrigger * 1000 / timerTick;               // Number of timer intervals to hold "trigger" condition
const long ticksResetInitial = timeResetInitial * 1000 / timerTick;     // Number of timer intervals to hold "reset" condition initially
const long ticksResetSubsequent = timeResetSubsequent * 1000 / timerTick;// Number of timer intervals to hold "reset" condition initially

/****************************************************/
/*                 Lighting Constants               */
/****************************************************/

const int targetPresence = 40;                                          // Target PWM value for lighting in "presence" state (0..255)
const int targetTrigger = 255;                                          // Target PWM value for lighting in "trigger" state (0..255)
const int outputSteps = 1500 / timerTick;                               // Number of steps in which PWM value will be increased/decreased (number of ticks in 1.5 seconds)

/****************************************************/
/*                Sensitivity                       */
/****************************************************/

const int presenceConsecutivePositivesRequired = 2;                     // Number of successive motion detection samples that must exceed threshold for presence to be detected

/****************************************************/
/*               Events & States                    */
/****************************************************/

enum event 
      { 
      eventPresence,                                                    // Presence has been detected (motion sensor)
      eventTrigger                                                      // Trigger has been detected (pressure pad)
      };
      
enum state : byte
      { 
      stateReset,                                                       // Initializing - events ignored
      stateIdle,                                                        // No events detected
      statePresence,                                                    // A presence event has been detected
      stateTrigger                                                      // A trigger event has been detected
      };

/****************************************************/
/*               Exponential Moving Average         */
/****************************************************/

long meanMotion;                                                        // The rolling average of the motion sensor value 
long meanLight;                                                         // The rolling average of the light sensor value

/* 
 *  The exponential rolling average is used to avoid having to calibrate the motion sensor and to ignore short term
 *  variations in the light sensor. If the previous mean value is M and the current sample value is S, this is 
 *  normally calculated by taking a fractonal weighting (e) of the present sample to give the new mean as:
 *  
 *  M = (1 - e)*M + e*S
 *  
 *  As the Arduino doesn't divide and multiplication is slow we take a shortcut and choose e to be a fraction 
 *  where the numerator is 1 and the denominator is a power of 2. The fractional calculation can then be achieved 
 *  with simple shift operations. The size of the denominator can be changed below to weight the samples over a
 *  longer period. The values stored in meanMotion and meanLight have an implicit binary point with the following number
 *  of binary places:
 *  
 */

const int meanExponentialShift = 6;                                     // e is 1/64


/****************************************************/
/*               Volatile state shared with ISR     */
/****************************************************/

volatile int outputTarget = 0;                                          // Target PWM value for light output
volatile int outputCurrent = 0;                                         // Current PWM value of light output
volatile int outputStep = 0;                                            // Change in PWM value required for each step to target
volatile int remainingTicksPresence = 0;                                // Number of timer ticks left to maintain presence state
volatile int remainingTicksTrigger = 0;                                 // Number of timer ticks left to maintain trigger state
volatile int remainingTicksReset = 0;                                   // Number of timer tickls left to maintain reset state
volatile state stateCurrent = stateReset;                               // Current state of the state machine

/****************************************************/
/*               Miscellaneous data                 */
/****************************************************/

int presenceConsecutivePositivesActual = 0;                             // The number of consecutive motion samples exceeding the threshold

/****************************************************/
/*               SETUP                              */
/****************************************************/
void setup() 
  {
  pinMode(LED_BUILTIN, OUTPUT);                                         // Initialise on-board LED
  pinMode(pinPWM, OUTPUT);                                              // Set the PWM pin to output
  pinMode(pinPad, INPUT_PULLUP);                                        // Set PULLUP on pressure pad which closes to ground

  meanMotion = 512;                                                     // Set initial values of rolling mean to middle of A/D converter range
  meanMotion = meanMotion << meanExponentialShift;                      // .. initial sensor value may be wildly off, so 512 is a better initial estimate

  meanLight = analogRead(pinLight);                                     // Set initial values of rolling mean to current sensor value          
  meanLight = meanLight << meanExponentialShift;                        // .. we can't guess what this will be and sensor should be stable

  analogWrite (pinPWM, 0);                                              // Set lighting off during startup (should be off by default, but...)
 
  stateCurrent = stateReset;                                            // We start in the reset state, ignoring motion detection
  remainingTicksReset = ticksResetInitial;                              // ... for this period
  setTimer();                                                           // Start counting reset period

#ifdef DEBUG
  Serial.begin(19200); 
#endif
  }
  
/****************************************************/
/*               MAIN LOOP                          */
/****************************************************/
void loop() 
  {
  long tempMotion;
  long tempLight;

  /* READ SENSORS */

  int motionValue = analogRead(pinMotion);                              // Current value of motion sensor
  int lightValue = analogRead(pinLight);                                // Current value of light sensor

  /* READ SETTINGS FROM TRIM POTS */
  
  int thresholdMotion = analogRead(pinThresholdMotion) >>
                    thresholdMotionShift;                               // Motion threshold value (scaled)
  int thresholdLight = analogRead(pinThresholdLight) ;                  // Light threshold value

  /* COMPUTE EMA */

  tempMotion = meanMotion >> meanExponentialShift;                      // Scale motion EMA to sensor range
  meanMotion = meanMotion - tempMotion + motionValue;                   // Compute new EMA
  
  tempLight = meanLight >> meanExponentialShift;                        // Scale light EMA to sensor range

  if (outputCurrent == 0)                                               // Compute new EMA if lights are not on...
      {
      meanLight = meanLight - tempLight + lightValue;                   // ... or they may upset ambient reading
      }
      
#ifdef DEBUG
  logThreshold(thresholdMotion, thresholdLight);
  logRead(motionValue, lightValue);
#endif

  /* CHECK FOR MOTION OR PRESSURE EVENTS */

  if (stateCurrent == stateIdle)                                        // Only process motion events if idle state
    {
     if ((abs(tempMotion - motionValue) > thresholdMotion) && (tempLight > thresholdLight))
      {
      digitalWrite(LED_BUILTIN, HIGH);                                  // Turn on on-board LED

      if (++presenceConsecutivePositivesActual > presenceConsecutivePositivesRequired)
        {
        presenceConsecutivePositivesActual = 0;
        newEvent(eventPresence);
        }
      }
    else
      {
      digitalWrite(LED_BUILTIN, LOW);                                   // Turn off on-board LED
      presenceConsecutivePositivesActual = 0;
      }
    }

  if ((stateCurrent == stateIdle) || (stateCurrent == statePresence))   // Only process pressure pad in Idle or Presence states
    {
    if ((digitalRead(pinPad) == LOW)  && (tempLight > thresholdLight))
      {
      newEvent(eventTrigger);
      }
    }

  delay(loopDelay);
  }

/****************************************************/
/*               STATE MACHINE                      */
/****************************************************/

void newEvent(event e)
  {
#ifdef DEBUG
  Serial.print ("Event: ");
  Serial.print (e);
  Serial.print ("; state: ");
  Serial.println (stateCurrent);
#endif
  
  switch (stateCurrent)
    {
    case stateIdle:                                                     // We are idle                
      {
      if (e == eventPresence)                                           // Presence detected
        {
        outputTarget = targetPresence;                                  // Set the final target for PWM value
        outputStep = (outputTarget - outputCurrent) / outputSteps;      // Calculate the PWM step size
        if (outputStep < 1)
          outputStep = 1;
        stateCurrent = statePresence;                                   // New state
        remainingTicksPresence = ticksPresence;                         // Timer ticks to maintain current state
        setTimer();                                                     // Start the timer
        }
      else if (e == eventTrigger)                                       // Pressure pad triggered
        {
        outputTarget = targetTrigger;                                   // Set the final target for PWM value
        outputStep = (outputTarget - outputCurrent) / outputSteps;      // Calculate the PWM step size
        if (outputStep < 1)
          outputStep = 1;
     
        stateCurrent = stateTrigger;                                    // New state
        remainingTicksPresence = 0;
        remainingTicksTrigger = ticksTrigger;                           // Timer ticks to maintain current state
        setTimer();                                                     // Start the timer (if not already started)
        }
      break;
      }

    case statePresence:                                                 // Presence already detected
      {
      if (e == eventPresence)                                           // And has been detected again
        {
        remainingTicksPresence = ticksPresence;                         // Restart the clock
        }
      else if (e == eventTrigger)                                       // Pressure pad triggered
        {
        outputTarget = targetTrigger;                                   // Do as above
        outputStep = (outputTarget - outputCurrent) / outputSteps;
        if (outputStep < 1)
          outputStep = 1;
    
        stateCurrent = stateTrigger;
        remainingTicksPresence = 0;
        remainingTicksTrigger = ticksTrigger;
        }
        
      break;
      }

    case stateTrigger:                                                  // Pressure pad has been triggered
      {  
      if (e == eventTrigger)                                            // And has been detected again
        {
        remainingTicksTrigger = ticksTrigger;                           // Restart the clock
        }
      break;
      }
    }
  }

/****************************************************/
/*               START TIMER                        */
/****************************************************/

void setTimer()
  {
  Timer1.initialize(timerPeriod);
  Timer1.attachInterrupt(timerInterrupt);
  }

/****************************************************/
/*               TIMER INTERRUPT                    */
/****************************************************/
void timerInterrupt(void)
  {
  /*
   * NB: This is effectively the "timer event" part of the State Machine, however
   * we keep it separate to emphasis it is called in interrupt context and may 
   * execute while "newEvent" is in progress.
   * 
   */
  switch (stateCurrent)
    {
    /*
     * The "reset" state actually does double duty. When the device starts up, it serves to prevent
     * certain events being processed while the motion sensor is settling down and the ambient 
     * light level is being adjusted.
     * 
     * Later, it serves to prevent motion events being recognised for a short period after the lights
     * have been faded back down as this can falsely trigger the motion sensor.
     * 
     */
    case stateReset:
      {                                                                 // No events in procgress
      if (outputCurrent > outputTarget)                                 // If light is on, continue to dim to 0
        {
        if (outputCurrent < outputStep)
          outputCurrent = 0;
        else
          outputCurrent -= outputStep;
        }

      if (remainingTicksReset > 0)                                      // Count down the duration of the reset state
        {
        remainingTicksReset--;
        }
      else
        {
        stateCurrent = stateIdle;                                       // If it gets to 0, transition to idle state
        outputCurrent = 0;                                              // Output level should be zero if Reset time > fade time, but...
        outputTarget = 0;                                               // ... as should target level
        Timer1.stop();                                                  // Stop the timer when we're idle
        }
      break;
      }

    case statePresence:                                                 // Presence detected
      if (outputCurrent < outputTarget)                                 // If light not at target value, continue to increase
        {
        outputCurrent += outputStep;
        }
      if (outputCurrent > outputTarget)
        {
        outputCurrent = outputTarget;
        }

      if (remainingTicksPresence > 0)                                   // Count down the duration of the presence state
        {
        remainingTicksPresence--;
        }
      else
        {
        stateCurrent = stateReset;                                      // If it gets to 0, transition to Reset state
        remainingTicksReset = ticksResetSubsequent;                     // Stay in that state for appointed time
        outputTarget = 0;                                               // ... setting new PWM target of 0
        outputStep = outputCurrent / outputSteps;                       // ... and calculating new step value
        if (outputStep < 1)
          outputStep = 1;
         }

      break;
        
   case stateTrigger:                                                   // Pressure pad detected
      if (outputCurrent < outputTarget)                                 // If light not at target value, continue to increase
        {
        outputCurrent += outputStep;
        }
      if (outputCurrent > outputTarget)
        {
        outputCurrent = outputTarget;
        }

      if (remainingTicksTrigger > 0)                                    // Count down the duration of the trigger state
        remainingTicksTrigger--;
      else
        {
        stateCurrent = stateReset;                                      // If it gets to 0, transition to Reset state
        remainingTicksReset = ticksResetSubsequent;                     // Stay in that state for appointed time
        outputTarget = 0;                                               // ... setting new PWM target of 0
        outputStep = outputCurrent / outputSteps;                       // ... and calculating new step value
        if (outputStep < 1)
          outputStep = 1;
        }
  
      break;
    }
    
  analogWrite (pinPWM, outputCurrent);                                  // Set the PWM output
  }

#ifdef DEBUG
/****************************************************/
/*               DEBUGGING                          */
/****************************************************/

void logThreshold(int thresholdMotion, int thresholdLight)
  {
    Serial.print("Th: ");
    Serial.print(thresholdMotion);
    Serial.print(",");
    Serial.println(thresholdLight);
  }
  
void logRead(int motionValue, int lightValue)
  {
    Serial.print("M: ");
    Serial.print(motionValue);
    Serial.print(" (");
    Serial.print(meanMotion >> meanExponentialShift);
    Serial.print(") L: ");
    Serial.print(lightValue);
    Serial.print(" (");
    Serial.print(meanLight >> meanExponentialShift);
    Serial.println(")");
  
    Serial.print("S: ");
    Serial.print(stateCurrent);
    Serial.print("; T: ");
    Serial.print(outputTarget);
    Serial.print("; C: ");
    Serial.println(outputCurrent);
   }
#endif
