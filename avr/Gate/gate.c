/*
 * transceiver.c
 *
 *  Created on: 17 pa? 2019
 *      Author: cj037y
 */

#include <util/delay.h>
#include "gate.h"
#include "main.h"
#include "../Uart/uart.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

void setRelay(struct gate * gatePtr, uint8_t level)
{
  if(&gate1 == gatePtr)
  {
    if(HIGH == level) REL_1_HI;
    else REL_1_LO;
  }
  else if(&gate2 == gatePtr)
  {
    if(HIGH == level) REL_1_HI;
    else REL_1_LO;
  }
}

void gateInit(struct gate * gatePtr)
{
  gatePtr->isRunning = false;
  gatePtr->pwm = 0;
  gatePtr->state = STOP;
  gatePtr->slowDownTime = SLOW_DOWN_TIME;
  gatePtr->ptr = 0;
  gatePtr->currentThreshold = 80;

  for (uint8_t i = 0; i < 6; i++)
  {
    gatePtr->currentBuff[i] = 0;
  }

}

void gateLogic(struct gate * gatePtr)
{
  char name1[] = "Gate1: ";
  char name2[] = "Gate2: ";
  char * namePtr;
  if(&gate1 == gatePtr) namePtr = name1;
  if(&gate2 == gatePtr) namePtr = name2;

  if(INIT == gatePtr->state)
  {
    gatePtr->runTime = 0;
    gatePtr->isRunning = true;
    gatePtr->pwm = 0;
    // if(DIR_CLOSE == gatePtr->dir) setRelay(gatePtr, HIGH);
    gatePtr->cnt = 4;
    gatePtr->state = DELAY_FOR_RELAY;
    Serial.println("i");
  }
  else if(DELAY_FOR_RELAY == gatePtr->state)
  {
    if(gatePtr->cnt > 0) gatePtr->cnt --;
    else 
    {
      gatePtr->pwm = INIT_PWM;
      gatePtr->state = INRUSH;
    }
    Serial.println("d");
  }
  else if(INRUSH == gatePtr->state)
  {
    Serial.println("e");
    // 255-50 = 250*10ms = 2,5s
    if(gatePtr->pwm < MAX_PWM) (gatePtr->pwm) ++;
    else gatePtr->state = RUN;
  }
  else if(RUN == gatePtr->state)
  {
    Serial.println("r");
    if(gatePtr->runTime > gatePtr->slowDownTime)
    {
      gatePtr->state = SLOW_DOWN;
    }
  }
  else if(SLOW_DOWN == gatePtr->state)
  {
    if(gatePtr->pwm > LOW_SPEED_PWM) gatePtr->pwm -= 2;
    else 
    {
      gatePtr->pwm = LOW_SPEED_PWM;
      gatePtr->state = LOW_SPEED;
    }
  }
  else if(LOW_SPEED == gatePtr->state)
  {
    //wait for overcurrent
  }
  else if(SLOW_TO_ZERO == gatePtr->state)
  {
    if(gatePtr->pwm >= 3) gatePtr->pwm -= 3;
    else 
    {
      gatePtr->pwm = 0, 
      gatePtr->cnt = 100;
      gatePtr->state = WAIT_FOR_RELAY;
    }
  }
  else if(WAIT_FOR_RELAY == gatePtr->state)
  {
    if(0 < gatePtr->cnt) gatePtr->cnt--;
    else gatePtr->state = END;
  }
  else if(END == gatePtr->state)
  {
    // REL_1_LO;
    gatePtr->isRunning = false;

    gatePtr->state = STOP;

    if(DIR_CLOSE == gatePtr->dir) {
      gatePtr->pos = POS_CLOSED;
      Serial.print(namePtr);
      Serial.println("status CLOSED");
    }
    if(DIR_OPEN == gatePtr->dir) {
      gatePtr->pos = POS_OPEN;
      Serial.print(namePtr);
      Serial.println("status OPEN");
    }

    if(DRIVER_STATE_OPEN_BOTH == gateDriver.state) gateDriver.pos = DRIVER_POS_OPEN;
    if(DRIVER_STATE_CLOSE_BOTH == gateDriver.state) gateDriver.pos = DRIVER_POS_CLOSED;
    gateDriver.state = DRIVER_STATE_IDLE;
  }


  if(true == gatePtr->isRunning)
  {
    gatePtr->runTime += 10;
    if(gatePtr->runTime > RUN_TIME_MAX)
    {
      gatePtr->runTime = 0;
      gatePtr->state = SLOW_TO_ZERO;
      Serial.print(namePtr);
      Serial.println("stop reason - run time");
    }
  }

  // analogWrite(PWM_GATE_1, 255 - gate1.pwm);
}

void driverLogic(void)
{
  if(DRIVER_STATE_IDLE == gateDriver.state)
  {
    if(COMMAND_BOTH == gateDriver.cmd)
    {
      gateDriver.cmd = COMMAND_NONE;
      gateDriver.state = DRIVER_STATE_PREPARE;
      gateDriver.runMode = DRIVER_RUN_MODE_BOTH;
      gateDriver.isSecondGateRunning = false;
      gateDriver.runTime = 0;
      gateDriver.isRunning = true;
      REL_MAIN_HI;
    }
  }
  else if(DRIVER_STATE_PREPARE == gateDriver.state)
  {
    gateDriver.state = DRIVER_STATE_RUN;

    if(DRIVER_POS_CLOSED == gateDriver.pos)
    {
      Serial.println("Driver: start opening");
      gateDriver.dir = DRIVER_DIR_OPENING;
      gate1.state = INIT;
      gate1.dir = DIR_OPEN;
      gateDriver.pos = DRIVER_POS_OPEN;
    }
    else if(DRIVER_POS_OPEN == gateDriver.pos)
    {
      Serial.println("Driver: start closing");
      gateDriver.dir = DRIVER_DIR_CLOSING;
      gate2.state = INIT;
      gate2.dir = DIR_CLOSE;
      gateDriver.pos = DRIVER_POS_CLOSED;
    }
  }
  else if(DRIVER_STATE_END_PREPARE == gateDriver.state)
  {
    gateDriver.state = DRIVER_STATE_END;
    REL_MAIN_LO;
    gateDriver.isRunning = false;
    Serial.println("Driver: stop");
  }
  else if(DRIVER_STATE_END == gateDriver.state)
  {
    gateDriver.state = DRIVER_STATE_IDLE;
  }

  if(true == gateDriver.isRunning)
  {
    gateDriver.runTime += 10;

    if((DRIVER_RUN_MODE_BOTH == gateDriver.runMode) && (false == gateDriver.isSecondGateRunning))
    {
      if(gateDriver.runTime > gateDriver.timeToStartGate2)
      {
        gateDriver.isSecondGateRunning = true;
        Serial.println("Driver: start second gate");
        // if(DRIVER_DIR_OPENING == gateDriver.dir)
        // {
        //   gate2.state = INIT;
        //   gate2.dir = DIR_OPEN;
        // }
        // else
        // {
        //   gate1.state = INIT;
        //   gate1.dir = DIR_CLOSE;          
        // }
      }
    }
  }
}

void gateDataPrint(struct gate * gatePtr)
{
  Serial.println("************************************");
  if(gatePtr == &gate1) Serial.println("gate1 data:");
  if(gatePtr == &gate2) Serial.println("gate2 data:");

  Serial.print("runtime: ");
  Serial.println(gatePtr->runTime);
  Serial.print("pwm: ");
  Serial.println(gatePtr->pwm);
  Serial.print("current: ");
  Serial.println(gatePtr->current);
  Serial.print("isRunning: ");
  Serial.println(gatePtr->isRunning);
  Serial.print("state: ");
  Serial.println(gatePtr->state);
  Serial.print("pos: ");
  Serial.println(gatePtr->pos);

  Serial.println("************************************");
}


void gateCurrentControll(struct gate * gatePtr)
{
  if(true == gatePtr->adcComplete)
  {
    //co 16/3ms ok 5ms
    gatePtr->adcComplete = false;

    if(gatePtr->runTime > 2000)
    {
      //jeśli w śród ostatnich 6 próbek 4 były overcurrent to koniec
      gatePtr->currentBuff[gatePtr->ptr] = gatePtr->current;
      gatePtr->ptr ++;
      if(gatePtr->ptr >= 6) gatePtr->ptr = 0;

      if(gatePtr->isRunning)
      {
        if(checkIfOvercurrent(gatePtr))
        {
          gatePtr->pwm = 0;
          analogWrite(gatePtr->pwmPin, 255 - gatePtr->pwm);
          gatePtr->isRunning = false;
          gatePtr->state = END;
          if(gatePtr == &gate1) Serial.println("gate1: overcurrent");
          if(gatePtr == &gate2) Serial.println("gate2: overcurrent");
        }
      }
    }
    else
    {

    }

    //jeśli silnik jedzie i ma włączoną flage obserwacji prądu

  }
}

bool checkIfOvercurrent(struct gate * gatePtr)
{
  int cnt = 0;
  bool ret = false;

  for (uint8_t i = 0; i < 6; i++)
  {
    if(gatePtr->currentBuff[i] > gatePtr->currentThreshold) cnt++;
  }
  
  if(cnt >= 4) ret = true;

  return ret;
}

KEY_T getKey(void)
{
  static int drgTab[2] = {0};
  int drgIdx = 0;
  int deb = 4;
  KEY_T key = KEY_NONE;

  drgIdx = 0;
  if(KEY_BOTH_LOW)
  {
    if(drgTab[drgIdx] < deb) drgTab[drgIdx]++;
    if(drgTab[drgIdx] == deb) {
      drgTab[drgIdx]++;
      key = KEY_BOTH;
    }
  }
  else drgTab[drgIdx] = 0;

  drgIdx = 1;
  if(KEY_ONE_LOW)
  {
    if(drgTab[drgIdx] < deb) drgTab[drgIdx]++;
    if(drgTab[drgIdx] == deb) {
      drgTab[drgIdx]++;
      key = KEY_ONE;
    }
  }
  else drgTab[drgIdx] = 0;

  return key;
}
