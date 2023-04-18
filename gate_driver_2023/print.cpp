#include "print.h"

#include <Arduino.h>

extern struct gate gate1;
extern struct gate gate2;
extern struct driver gateDriver;

const char* DRIVER_POS_T_STR[] = {"DRIVER_POS_ONE_OPEN", "DRIVER_POS_BOTH_OPEN", "DRIVER_POS_BOTH_CLOSED"};
const char* GATE_POS_T_STR[] = {"POS_OPEN", "POS_CLOSED", "POS_MID"};
const char* GATE_STATE_T_STR[] = {"INIT", "DELAY_FOR_RELAY", "INRUSH", "RUN", "SLOW_DOWN", "LOW_SPEED", "SLOW_TO_ZERO", "WAIT_FOR_RELAY", "END", "STOP"};

void gateDataPrint(struct gate* gatePtr)
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

void driverDataPrint(void)
{
    Serial.print("runtime: ");
    Serial.print(gateDriver.runTime);
    Serial.print(", I1: ");
    Serial.print(gate1.current);
    Serial.print(", I2: ");
    Serial.println(gate2.current);
}

void runSummaryPrint(void)
{
    Serial.println("************************************");
    Serial.println("Summary:");
    Serial.println("Driver");
    Serial.print("state: ");
    Serial.println(gateDriver.state);
    Serial.print("pos: ");
    Serial.println(DRIVER_POS_T_STR[gateDriver.pos]);

    Serial.println("------------");
    Serial.println("Gate1");

    Serial.print("runtime: ");
    Serial.println(gate1.lastRunTime);
    Serial.print("max current: ");
    Serial.println(gate1.maxCurrent);
    // Serial.print("state: ");
    // Serial.println(GATE_STATE_T_STR[gate1.state]);
    Serial.print("pos: ");
    Serial.println(GATE_POS_T_STR[gate1.pos]);
    Serial.print("pos2: ");
    Serial.println(getPositionInRev(gate1.position));

    Serial.println("------------");
    Serial.println("Gate2");

    Serial.print("runtime: ");
    Serial.println(gate2.lastRunTime);
    Serial.print("max current: ");
    Serial.println(gate2.maxCurrent);
    // Serial.print("state: ");
    // Serial.println(GATE_STATE_T_STR[gate2.state]);
    Serial.print("pos: ");
    Serial.println(GATE_POS_T_STR[gate2.pos]);
    Serial.print("pos2: ");
    Serial.println(getPositionInRev(gate2.position));

    Serial.println("************************************");
}
