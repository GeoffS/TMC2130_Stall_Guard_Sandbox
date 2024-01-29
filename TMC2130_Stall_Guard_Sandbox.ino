#include "stepperDriver.h"

// TMC2130 Stall Pin:
// #define STALL_GUARD_PIN 6
// #define STALL_DETECTED 0
// #define INITIAL_ROTATION freeRotateNeg()
// #define REVERSE_COUNT 87305

// Inductive Sensor:
#define STALL_GUARD_PIN A5
#define STALL_DETECTED 1
#define INITIAL_ROTATION freeRotateNeg()
// #define REVERSE_COUNT -87305
#define REVERSE_COUNT 88112
// #define REVERSE_COUNT -88305

long numCounts = 20000;
long programStartTime_ms = 0;

uint32_t last_time_status = 0;
uint32_t enableStallCheckingWait_ms = 0;
bool enableStallChecking = false;
bool enableStallCheckingDelay = false;

void setup() 
{
  programStartTime_ms = millis();

  Serial.begin(250000); //init serial port and set baudrate
  Serial.println("\nStart...");

  pinMode(STALL_GUARD_PIN, INPUT);

  initStepper(1000, 64);

  stop();
  // freeRotateNeg();
  delay(1000);
  // rotateToAsync(20000);
}

uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

bool isFirstLoop = true;


void loop() 
{
  if(isFirstLoop)
  {
    isFirstLoop = false;
    enableStallChecking = false;
    enableStallCheckingDelay = true;
    Serial.println("First loop!\n");
    INITIAL_ROTATION;
  }

  uint32_t ms = millis();

  if(enableStallChecking)
  {
    if(digitalRead(STALL_GUARD_PIN) != STALL_DETECTED)
    {
      disableDriver();
      stop();
      
      Serial.print("STALL_GUARD_PIN = ");
      Serial.println(digitalRead(STALL_GUARD_PIN));

      zeroCounter();

      delay(1000);

      enableStallChecking = false;
      enableStallCheckingDelay = false;

      enableDriver();
      enableStallCheckingWait_ms = millis();
      rotateToAsync(REVERSE_COUNT);
      // enableStallCheckingWait_ms = millis();
      // rotateToAsync(90000);
    }
  }
  
  if((enableStallCheckingDelay == true) && (enableStallChecking == false) && ((ms-enableStallCheckingWait_ms) > 100))
  {
    enableStallCheckingWait_ms = ms;
    Serial.println("Set enableStallChecking = true\n");
    enableStallChecking = true;
  }

  if((ms-last_time_status) > 1000) //run every 1s
  {
    last_time_status = ms;

    Serial.print("enableStallChecking = ");
    Serial.println(enableStallChecking);

    Serial.print("STALL_GUARD_PIN = ");
    Serial.println(digitalRead(STALL_GUARD_PIN));

    Serial.print("Running time: ");
    Serial.print(ms-programStartTime_ms);
    Serial.println("ms");

    uint32_t drv_status = driver.DRV_STATUS();
    Serial.print("0 ");
    uint32_t sgValue = (drv_status & SG_RESULT_bm)>>SG_RESULT_bp;

    Serial.print(sgValue, DEC);
    Serial.print(" ");
    Serial.println(rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp), DEC);

    Serial.print("currCounter = ");
    Serial.println(getCurrentCounter());

    Serial.println();
  }
}
