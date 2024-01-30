#pragma once

// #define STALL_VALUE -64 // [-64..63]

// #define EN_PIN    38 
// #define DIR_PIN   55
// #define STEP_PIN  54
// #define CS_PIN    63

// UNO with Geoff S. TMC2130 proto shield:
#define EN_PIN     7 
#define DIR_PIN    8
#define STEP_PIN   9
#define CS_PIN    10

#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13

#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>
TMC2130Stepper driver = TMC2130Stepper(CS_PIN);
#include "stepperDriver.h"

//   500 = 14.648 kHz (too high for TMC2100/motor)
//   700 = 10.738 kHz
//   900 =  8.463
//   930 =  8.203
//   935 =  8.1675
//   936 =  8.154
//   937 =  8.154
//  1000 =  7.650 kHz
//  5000 =  1.586 kHz
//                                Direct Drive
//                               --------------
// 30000 = 0.267 kHz = 267   Hz = 12 s/rotation
// 35000 = 0.228 kHz = 228   Hz = 14 s/rot
// 40000 = 0.200 kHz = 200   Hz = 16 s/rot
// 50000 =           = 159.8 Hz = 20 s/rot

// 7.75:1 Reduction Drive (124T/16T):
//  6452 =           = ~1000 Hz = 20 s/rot

// 20.74s per rotation (measured):
// unsigned int delayCount = 1500; //6452; //round(50000/reductionRatio);
unsigned int delayCount = 3000;

unsigned int TCNT1_PRESET = 0Xffff - delayCount;

void initInterrupt()
{
  // initialize Timer1
  cli();         // disable global interrupts
  TCCR1A = 0;    // set entire TCCR1A register to 0
  TCCR1B = 0;    // set entire TCCR1A register to 0

  // enable Timer1 overflow interrupt:
  TIMSK1 |= (1 << TOIE1);
  
  TCNT1 = TCNT1_PRESET;
  
  // Set CS10 bit so timer runs at clock speed: (no prescaling)
  TCCR1B |= (1 << CS10); // Sets bit CS10 in TCCR1B
  
  // enable global interrupts:
  sei();
}

#define FREE_RUNNING_MODE 0
#define TARGET_RUNNING_MODE 1

#define RUNNING_DIRECTION_POS 0
#define RUNNING_DIRECTION_NEG 1

// inRunning, counterTarget, and counter need to be "volatile" since 
// they are set in the main loop for pulsing control:
volatile byte runningMode = FREE_RUNNING_MODE;
volatile byte runningDirection = RUNNING_DIRECTION_POS;
volatile byte isRunning = 0;
volatile byte updateCounterTarget = 0;
volatile byte dontReadCounter = 0;
volatile byte dontReadCounterTarget = 0;
volatile long newCounterTargetValue = 0;
volatile long counterTarget = 0;
volatile long counter = counterTarget;

// pulseFlag does not need to be volatile because it's just used
// internally by the interrupt routine.
byte pulseFlag = 0;

void stepPos()
{
  runningDirection = RUNNING_DIRECTION_POS;
  digitalWrite(DIR_PIN, LOW); // TODO direct register write.
  dontReadCounter = 1;
  counter++;
  dontReadCounter = 0;
}

void stepNeg()
{
  runningDirection = RUNNING_DIRECTION_NEG;
  digitalWrite(DIR_PIN, HIGH); // TODO direct register write.
  dontReadCounter = 1;
  counter--;
  dontReadCounter = 0;
}

void doFreeRunning()
{
  if(isRunning == 0)
  {
    isRunning = 0;
  }
  else
  {
    if(runningDirection == RUNNING_DIRECTION_POS)
    {
      stepPos();
    }
    else // runningDirection == RUNNING_DIRECTION_NEG
    {
      stepNeg();
    }
  }
}

void doTargetRunning()
{
  // Check to see if an update to the counterTarget has been requested:
  // We can do that here because we know that the only ISR accesses to 
  // counterTarget happend below.
  if(updateCounterTarget == 1)
  {
    dontReadCounterTarget = 1;
    counterTarget = newCounterTargetValue;
    updateCounterTarget = 0;
    dontReadCounterTarget = 0;
  }

  dontReadCounterTarget = 1;
  if (isRunning == 0 || counter == counterTarget)
  {
    isRunning = 0;
  }
  else
  {
    isRunning = 1;
    if(counter < counterTarget) 
    {
      stepPos();
    }
    else // counter > counterTarget
    {
      stepNeg();
    }
  }
  
    dontReadCounterTarget = 0;
}

void doPulsing()
{
  if(isRunning == 0) 
  {
    digitalWrite(STEP_PIN,  LOW); // TODO direct register write.;
    return;
  }

  if (pulseFlag == 0)
  {
    digitalWrite(STEP_PIN, HIGH); // TODO direct register write.;
    pulseFlag = 1;
  }
  else
  {
    digitalWrite(STEP_PIN,  LOW); // TODO direct register write.;
    pulseFlag = 0;
  }
  TCNT1 = TCNT1_PRESET;
}

// The Interrupt service routine:
ISR(TIMER1_OVF_vect)
{
  if(runningMode == TARGET_RUNNING_MODE)
  {
    doTargetRunning();
  }
  else // runningMode == FREE_RUNNING_MODE
  {
    doFreeRunning();
  }

  doPulsing();
}

void enableDriver()
{
  digitalWrite(EN_PIN, LOW);
}

void disableDriver()
{
  digitalWrite(EN_PIN, HIGH); // Disable
}

bool vsense;

void initStepper(int16_t rmsCurrent_ma, int8_t stallValue)
{
  disableDriver();

  // pinMode(TMC2100_DIR_PIN, OUTPUT);
  // digitalWrite(TMC2100_DIR_PIN, LOW);
  // pinMode(TMC2100_STEP_PIN, OUTPUT);
  // digitalWrite(TMC2100_STEP_PIN, LOW);
  
  // Init pins and SPI
  {
    pinMode(EN_PIN, OUTPUT);
    disableDriver();
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW); //LOW or HIGH
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();
    pinMode(MISO, INPUT_PULLUP);
  }

  //set TMC2130 config
  {
    driver.push();
    driver.toff(3);
    driver.tbl(1);
    driver.hysteresis_start(4);
    driver.hysteresis_end(-2);
    driver.rms_current(rmsCurrent_ma); // mA
    driver.microsteps(16);
    driver.diag1_stall(1);
    driver.diag1_active_high(1);
    driver.coolstep_min_speed(0xFFFFF); // 20bit max
    driver.THIGH(0);
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.sg_stall_value(stallValue);

    driver.stealthChop(1); 	// Enable extremely quiet stepping
  }
  
  initInterrupt();
  delay(100); // Let the ISR get running.

  enableDriver();

  vsense = driver.vsense();
}

long getCurrentCounter()
{
  while(dontReadCounter == 1);
  return counter;
}

void zeroCounter()
{
  while(dontReadCounter == 1);
  counter = 0;
}

long getCurrentCounterTarget()
{
  while(dontReadCounterTarget == 1);
  return counterTarget;
}

void rotateToAsync(long newTargetCounts)
{
  // isRunning *must* be 0 since we're using it as a semaphone
  // to avoid concurrent access to the multi-byte counterTarget in the ISR and here:
  while(isRunning == 1);

  // Now we can update counterTarget:
  counterTarget = newTargetCounts;
  
  // Tell the ISR it is OK to access counterTarget again:
  runningMode = TARGET_RUNNING_MODE;
  isRunning = 1;
}

void updateTargetCounts(long newTargetCounts)
{
  if(updateCounterTarget != 0) return;

  if(isRunning == 0)
  {
    rotateToAsync(newTargetCounts);
  }
  else
  {
    newCounterTargetValue = newTargetCounts;
    runningMode = TARGET_RUNNING_MODE;
    updateCounterTarget = 1;
  }

  // // Now we can update counterTarget:
  // counterTarget = newTargetCounts;
  
  // // Tell the ISR it is OK to access counterTarget again:
  // runningMode = TARGET_RUNNING_MODE;
  // isRunning = 1;
}

void stop()
{
  isRunning = 0;
}

void rotateTo(long newTargetCounts)
{
  rotateToAsync(newTargetCounts);

  // Wait until the rotation is done:
  while(isRunning == 1);
}

void freeRotatePos()
{
  // isRunning *must* be 0 since we're using it as a semaphone
  // to avoid concurrent access to the multiple variables in the ISR:
  while(isRunning == 1);

  runningMode = FREE_RUNNING_MODE;
  runningDirection = RUNNING_DIRECTION_POS;
  isRunning = 1;
}

void freeRotateNeg()
{
  // isRunning *must* be 0 since we're using it as a semaphone
  // to avoid concurrent access to the multiple variables in the ISR:
  while(isRunning == 1);

  runningMode = FREE_RUNNING_MODE;
  runningDirection = RUNNING_DIRECTION_NEG;
  isRunning = 1;
}
