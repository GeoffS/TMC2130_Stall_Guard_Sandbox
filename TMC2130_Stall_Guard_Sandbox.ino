#include "stepperDriver.h"

// Inductive Sensor:
#define STALL_GUARD_PIN 6
#define STALL_DETECTED 1
#define FORMARD_ROTATION freeRotatePos()
#define BACKUP_COUNT -20000
#define POST_STALL_COUNT -20000

long programStartTime_ms = 0;

uint32_t last_time_status = 0;

void setup() 
{
  programStartTime_ms = millis();

  Serial.begin(250000); //init serial port and set baudrate
  Serial.println("\nStart...");

  pinMode(STALL_GUARD_PIN, INPUT);

  initStepper(400, 5);

  stop();
  delay(1000);
}

uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

int state = 0;

void loop() 
{
  uint32_t curr_ms = millis();

  switch(state)
  {
    case 0:
      if((curr_ms - programStartTime_ms) > 3000) state = 1;
      break;

    case 1:
      Serial.println("Backing up a bit...");
        Serial.println();
      rotateToAsync(BACKUP_COUNT);
      state = 2;
      break;

    case 2:
      if(isRunning == 0) 
      {
        Serial.println("Done backing up!");
        Serial.println();
        delay(2000);
        state = 3;
      }
      break;

    case 3:
      Serial.println("Starting forward!");
        Serial.println();
      FORMARD_ROTATION;
      delay(20);
      state = 4;
      break;

    case 4:
      if(digitalRead(STALL_GUARD_PIN) == STALL_DETECTED)
      {
        disableDriver();
        stop();
        
        Serial.print("Stall dectected with STALL_GUARD_PIN = ");
        Serial.println(digitalRead(STALL_GUARD_PIN));
        Serial.println();

        zeroCounter();

        delay(1000);

        enableDriver();
        rotateToAsync(POST_STALL_COUNT);
        Serial.println("Backing away from the stop!");
        Serial.println();
        state = 5;
      }
      break;

    case 5:
      if(isRunning == 0) 
      {
        Serial.println("Done.");
        state = 6;
      }
      break;

    case 6:
      // Do nothing...
      break;
  }

  
  if((curr_ms-last_time_status) > 1000) //run every 1s
  {
    last_time_status = curr_ms;

    Serial.print("state = ");
    Serial.println(state);

    Serial.print("STALL_GUARD_PIN = ");
    Serial.println(digitalRead(STALL_GUARD_PIN));

    Serial.print("Running time: ");
    Serial.print(curr_ms-programStartTime_ms);
    Serial.println("ms");

    uint32_t drv_status = driver.DRV_STATUS();

    Serial.print("sgValue = ");
    uint32_t sgValue = (drv_status & SG_RESULT_bm)>>SG_RESULT_bp;
    Serial.println(sgValue, DEC);

    Serial.print("Irms = ");
    Serial.println(rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp), DEC);

    Serial.print("currCounter = ");
    Serial.println(getCurrentCounter());

    Serial.println();
  }
}
