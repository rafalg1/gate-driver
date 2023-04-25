#include <avr/io.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>

#include "gate_driver.h"
#include "lowLevel.h"
#include "print.h"

struct gate gate1, gate2;
struct adc_struct adc;
struct driver gateDriver;

volatile uint16_t task_1000ms = 1000;
volatile uint16_t task_300ms = 300;
volatile uint16_t task_200ms = 200;
volatile uint16_t task_10ms = 10;

// uint8_t overloadTab[OVERLOAD_TAB_ENTRIES][3] =
//     {
//         {125, 2, 3},
//         {120, 4, 5},
//         {110, 8, 10},
//         {100, 18, 20},
//         {95, 38, 40}};

// uint8_t overloadTab[OVERLOAD_TAB_ENTRIES][3] =
//     {
//         {50, 2, 3},
//         {35, 4, 5},
//         {30, 8, 10},
//         {28, 18, 20},
//         {25, 38, 40}};

uint8_t overloadTabInrush[OVERLOAD_TAB_ENTRIES][3] =
    {
        {140, 2, 3},
        {130, 3, 4},
        {120, 3, 4},
        {100, 3, 4},
        {90, 4, 6}};

// uint8_t overloadTab[OVERLOAD_TAB_ENTRIES][3] =
//     {
//         {100, 1, 2},
//         {70, 2, 3},
//         {60, 3, 3},
//         {55, 3, 4},
//         {50, 4, 4}};

uint8_t overloadTab[OVERLOAD_TAB_ENTRIES][3] =
    {
        {35, 1, 2},
        {30, 2, 3},
        {25, 3, 3},
        {23, 3, 4},
        {20, 4, 4}};

void gateInit(struct gate* gatePtr)
{
    setPwm(gatePtr, 0);

    gatePtr->isRunning = false;
    gatePtr->currentMonitorEnableOvercurrent = true;
    gatePtr->currentMonitorEnableInrush = false;
    gatePtr->stopType = STOP_TYPE_NONE;
    gatePtr->distance = 0;
    gatePtr->pwm = 0;
    gatePtr->state = STOP;
    // gatePtr->slowDownTime = SLOW_DOWN_TIME;
    gatePtr->ptr = 0;
    // gatePtr->currentThreshold = CURRENT_THRESHOLD;
    gatePtr->maxCurrent = 0;

    for(uint8_t i = 0; i < CURRENT_BUFF_SIZE; i++)
    {
        gatePtr->currentBuff[i] = 0;
    }
}

void setup()
{
    PORTD = SW1_PIN | SW2_PIN;
    DDRD = REL_1_PIN | REL_2_PIN | REL_MAIN_PIN;
    DDRB = LED_PIN | GATE1_PIN | GATE2_PIN;

    REL_1_OFF;
    REL_2_OFF;
    REL_MAIN_OFF;

    GATE1_HI;
    GATE2_HI;

    delay(50);

    gateInit(&gate1);
    gateInit(&gate2);

    // gate1.pwmPin = PWM_GATE_1;
    // gate2.pwmPin = PWM_GATE_2;

    gateDriver.state = DRIVER_STATE_IDLE;
    gateDriver.pos = DRIVER_POS_BOTH_CLOSED;
    gateDriver.timeToStartGate2 = 3000;
    gateDriver.cmd = COMMAND_NONE;
    gateDriver.canAcceptCommand = true;

    Serial.begin(115200);
    Serial.println("Start driver");

    // TIMER2: 1ms
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = 0;
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22);  // 16MHz/64/(30+1)
    TIMSK2 |= (1 << OCIE2A);
    OCR2A = 249;

    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // preskaler 128 -> 16/128 = 125kHz, sample rate: 9kHz
    ADMUX = (1 << REFS0);
    ADCSRA |= (1 << ADSC);

    wdt_enable(WDTO_500MS);
}

void loop()
{
    if(!task_1000ms)
    {
        task_1000ms = 1000;
    }

    if(Serial.available() > 0)
    {
        char incomingByte = Serial.read();
        if('1' == incomingByte) handleCommand(COMMAND_BOTH);
        else if('2' == incomingByte) handleCommand(COMMAND_ONE);
        // else if('8' == incomingByte) handleCommand(COMMAND_ONE);
        else if('3' == incomingByte) gateDataPrint(&gate1);
        else if('4' == incomingByte) gateDataPrint(&gate2);
        // else if('5' == incomingByte)
        // {
        //     gate1.position = 0;
        //     setPwm(gate1, 30);
        // }
        // else if('6' == incomingByte)
        // {
        //     gate1.position = 0;
        //      setPwm(gate1, 0);
        // }
        else if('7' == incomingByte)
            while(1)
            {
            };
    }

    if(!task_10ms)
    {
        task_10ms = 10;
        KEY_T key = getKey();

        if(KEY_BOTH == key) handleCommand(COMMAND_BOTH);
        else if(KEY_ONE == key) handleCommand(COMMAND_ONE);

        // setPwm(PWM_GATE_1, gate1.pwm);
        // setPwm(PWM_GATE_2, gate2.pwm);
    }

    if(gateDriver.pwmPeriodFlag == true)
    {
        gateDriver.pwmPeriodFlag = false;

        // calculate position:
        calculatePosition();

        // check overcurrent:
        gateCurrentControl(&gate1);
        gateCurrentControl(&gate2);

        driverLogic();

        gateLogic(&gate1);
        gateLogic(&gate2);

        static int cntPrint = 0;
        // static int cntPrint = 1;

        // #define PRINT_2_DATA

        if(gateDriver.isRunning == true)
        {
            Serial.print("i1: ");
            Serial.print(gate1.current);
#if defined(PRINT_2_DATA)
            Serial.print(" i2: ");
            Serial.print(gate2.current);
#endif
            Serial.print(" w1: ");
            Serial.print(gate1.pwm);
            Serial.print(" u: ");
            Serial.print(gateDriver.batteryVoltage);

            cntPrint++;
            if(5 == cntPrint)
            {
                cntPrint = 0;
                Serial.print(" p1: ");
                Serial.print(getPositionInRev(gate1.position));
#if defined(PRINT_2_DATA)
                Serial.print(" p2: ");
                Serial.print(getPositionInRev(gate2.position));
#endif
                Serial.print(" d1: ");
                Serial.print(getPositionInRev(gate1.distance));
#if defined(PRINT_2_DATA)
                Serial.print(" d2: ");
                Serial.print(getPositionInRev(gate2.distance));
#endif
            }
            Serial.println("");
        }
    }

    if(!task_200ms)
    {
        task_200ms = 200;
        wdt_reset();
    }
    if(!task_300ms)
    {
        task_300ms = 300;
    }
}

void handleError(ERROR_T type)
{
    Serial.println("Error");
}

void handleCommand(COMMAND_T cmd)
{
    if(true == gateDriver.canAcceptCommand)
    {
        gateDriver.canAcceptCommand = false;
        if(COMMAND_BOTH == cmd) gateDriver.cmd = COMMAND_BOTH;
        else if(COMMAND_ONE == cmd) gateDriver.cmd = COMMAND_ONE;
        else handleError(ERROR_NOT_HANDLED_CASE);
    }
    else
    {
        Serial.println("Cant Accept Command");
    }
}

void stopGate(struct gate* gatePtr, STOP_TYPE_T stopType)
{
    setPwm(gatePtr, 0);
    if(STOP_TYPE_TIME == stopType)
    {
        gatePtr->state = END;
        Serial.println("stop reason - run time");
    }
    else if(STOP_TYPE_MAX_DISTANCE == stopType)
    {
        gatePtr->state = END;
        Serial.println("stop reason - distance");
    }
    else if(STOP_TYPE_CMD == stopType)
    {
        gatePtr->state = END;
        Serial.println("stop reason - cmd");
    }
}

// void stopFromCmd(void)
// {
//     stopGate(&gate1, STOP_TYPE_CMD);
//     stopGate(&gate2, STOP_TYPE_CMD);
// }

// void stopDriverFromTimeout(void) { gateDriver.isRunning = false; }

void prepareDriverForRun(void)
{
    gateDriver.isSecondGateRunning = false;
    gateDriver.runTime = 0;
    gateDriver.isRunning = true;
    gateInit(&gate1);
    gateInit(&gate2);
    checkPositionBeforeStart();
    REL_MAIN_HI;
}

void driverLogic(void)
{
    // resolve command
    if(gateDriver.cmd != COMMAND_NONE)
    {
        if(COMMAND_BOTH == gateDriver.cmd)
        {
            // jeśli jest iddle
            if(DRIVER_STATE_IDLE == gateDriver.state)
            {
                if(DRIVER_POS_BOTH_CLOSED == gateDriver.pos)
                {
                    prepareDriverForRun();
                    gateDriver.runMode = DRIVER_RUN_MODE_BOTH;
                    gateDriver.isRunningG1 = true;
                    gateDriver.isRunningG2 = true;
                    Serial.println("Driver: start opening both");
                    gateDriver.dir = DRIVER_DIR_OPENING;
                    gateDriver.pos = DRIVER_POS_BOTH_OPEN;
                    gate1.state = INIT;
                    gate1.dir = DIR_OPEN;
                }
                else if(DRIVER_POS_BOTH_OPEN == gateDriver.pos)
                {
                    prepareDriverForRun();
                    gateDriver.runMode = DRIVER_RUN_MODE_BOTH;
                    gateDriver.isRunningG1 = true;
                    gateDriver.isRunningG2 = true;
                    Serial.println("Driver: start closing both");
                    gateDriver.dir = DRIVER_DIR_CLOSING;
                    gateDriver.pos = DRIVER_POS_BOTH_CLOSED;
                    gate2.state = INIT;
                    gate2.dir = DIR_CLOSE;
                }
                else
                {
                    handleError(ERROR_NOT_HANDLED_CASE);
                }
                gateDriver.state = DRIVER_STATE_RUN;
            }
            else if(DRIVER_RUN_MODE_BOTH == gateDriver.runMode)
            {
                // stopFromCmd();
                stopGate(&gate1, STOP_TYPE_CMD);
                stopGate(&gate2, STOP_TYPE_CMD);
            }
            else
            {
                handleError(ERROR_NOT_HANDLED_CASE);
            }
        }
        else if(COMMAND_ONE == gateDriver.cmd)
        {
            // jeśli jest iddle
            //  jeśli są obie zamknięte to otwiera jedną
            if(DRIVER_STATE_IDLE == gateDriver.state)
            {
                if(DRIVER_POS_BOTH_CLOSED == gateDriver.pos)
                {
                    // open one gate
                    prepareDriverForRun();
                    gateDriver.state = DRIVER_STATE_RUN;
                    gateDriver.runMode = DRIVER_RUN_MODE_ONE;
                    gateDriver.isRunningG1 = true;
                    gateDriver.pos = DRIVER_POS_ONE_OPEN;
                    gate1.state = INIT;
                    gate1.dir = DIR_OPEN;
                    Serial.println("Driver: start opening one");
                }
                else if(DRIVER_POS_ONE_OPEN == gateDriver.pos)
                {
                    // close one gate
                    prepareDriverForRun();
                    gateDriver.state = DRIVER_STATE_RUN;
                    gateDriver.runMode = DRIVER_RUN_MODE_ONE;
                    gateDriver.isRunningG1 = true;
                    gateDriver.pos = DRIVER_POS_BOTH_CLOSED;
                    gate1.state = INIT;
                    gate1.dir = DIR_CLOSE;
                    Serial.println("Driver: start closing one");
                }
                else
                {
                    handleError(ERROR_NOT_HANDLED_CASE);
                }
            }
            else if(DRIVER_RUN_MODE_ONE == gateDriver.runMode)
            {
                Serial.println("stop in run mode one");
                stopGate(&gate1, STOP_TYPE_CMD);
            }
            else
            {
                handleError(ERROR_NOT_HANDLED_CASE);
            }
        }

        gateDriver.cmd = COMMAND_NONE;
    }

    if(DRIVER_STATE_IDLE == gateDriver.state)
    {
    }
    else if(DRIVER_STATE_PREPARE == gateDriver.state)
    {
        gateDriver.state = DRIVER_STATE_RUN;
    }
    else if(DRIVER_STATE_END_PREPARE == gateDriver.state)
    {
        gateDriver.state = DRIVER_STATE_END;
        REL_MAIN_LO;
        setRelay(&gate1, LOW);
        setRelay(&gate2, LOW);
        gateDriver.isRunning = false;
        Serial.println("Driver: stop");
    }
    else if(DRIVER_STATE_END == gateDriver.state)
    {
        checkPositionAfterStop();
        runSummaryPrint();
        gateDriver.canAcceptCommand = true;
        gateDriver.state = DRIVER_STATE_IDLE;
    }

    if(false == gateDriver.canAcceptCommand)
    {
        gateDriver.timeCommand += DRIVER_LOGIC_TASK_PERIOD;
        if(gateDriver.timeCommand > ACCEPT_COMMAND_TIME)
        {
            gateDriver.timeCommand = 0;
            gateDriver.canAcceptCommand = true;
        }
    }

    if(true == gateDriver.isRunning)
    {
        gateDriver.runTime += DRIVER_LOGIC_TASK_PERIOD;

        if((false == gateDriver.isRunningG1) &&
           (false == gateDriver.isRunningG2))
        {
            gateDriver.state = DRIVER_STATE_END_PREPARE;
        }

        if((DRIVER_RUN_MODE_BOTH == gateDriver.runMode) &&
           (false == gateDriver.isSecondGateRunning))
        {
            if(gateDriver.runTime > gateDriver.timeToStartGate2)
            {
                gateDriver.isSecondGateRunning = true;
                Serial.println("Driver: start second gate");
                if(DRIVER_DIR_OPENING == gateDriver.dir)
                {
                    gate2.state = INIT;
                    gate2.dir = DIR_OPEN;
                }
                else
                {
                    gate1.state = INIT;
                    gate1.dir = DIR_CLOSE;
                }
            }
        }
    }
}

void gateLogic(struct gate* gatePtr)
{
    char name1[] = "Gate1: ";
    char name2[] = "Gate2: ";
    char* namePtr;
    if(&gate1 == gatePtr) namePtr = name1;
    if(&gate2 == gatePtr) namePtr = name2;

    if(INIT == gatePtr->state)
    {
        gatePtr->runTime = 0;
        gatePtr->isRunning = true;
        setPwm(gatePtr, 0);
        if(DIR_CLOSE == gatePtr->dir) setRelay(gatePtr, HIGH);
        else setRelay(gatePtr, LOW);
        // gatePtr->cnt = 4;
        gatePtr->state = DELAY_FOR_RELAY;
        // Serial.println("INIT");
    }
    else if(DELAY_FOR_RELAY == gatePtr->state)
    {
        setPwm(gatePtr, INIT_PWM);
        gatePtr->state = RUN_2;
    }
    // else if(INRUSH_1 == gatePtr->state)
    // {
    //   if(gatePtr->pwm < LEVEL_1_PWM) (gatePtr->pwm) ++;
    //   else gatePtr->state = RUN_1;
    // }
    // else if(RUN_1 == gatePtr->state)
    // {
    //   if(gatePtr->runTime > RUN_1_TIME) gatePtr->state = INRUSH_2;
    // }
    // else if(INRUSH_2 == gatePtr->state)
    // {
    //   if(gatePtr->pwm < LEVEL_2_PWM) (gatePtr->pwm) ++;
    //   else gatePtr->state = RUN_2;
    // }
    else if(RUN_2 == gatePtr->state)
    {
        if((DIR_OPEN == gatePtr->dir) && (gatePtr->position > POSITION_SLOW_OPEN))
        {
            setPwm(gatePtr, LOW_SPEED_PWM);
            gatePtr->state = LOW_SPEED;
        }
        if((DIR_CLOSE == gatePtr->dir) && (gatePtr->position < POSITION_SLOW_CLOSE))
        {
            setPwm(gatePtr, LOW_SPEED_PWM);
            gatePtr->state = LOW_SPEED;
        }
    }
    // else if(SLOW_DOWN == gatePtr->state)
    // {
    //     if(gatePtr->pwm > LOW_SPEED_PWM) gatePtr->pwm--;
    //     else
    //     {
    //         gatePtr->pwm = LOW_SPEED_PWM;
    //         gatePtr->state = LOW_SPEED;
    //     }
    // }
    else if(LOW_SPEED == gatePtr->state)
    {
        // wait for overcurrent
    }
    // else if(SLOW_TO_ZERO == gatePtr->state)
    // {
    //     if(gatePtr->pwm >= 3) gatePtr->pwm -= 3;
    //     else
    //     {
    //         gatePtr->pwm = 0, gatePtr->cnt = 100;
    //         gatePtr->state = WAIT_FOR_RELAY;
    //         gatePtr->lastRunTime = gatePtr->runTime;
    //     }
    // }
    else if(OVERCURRENT_DETECTED == gatePtr->state)
    {
        gateDriver.canAcceptCommand = false;
        gatePtr->state = DELAY_FOR_RELAY_2;
    }
    else if(DELAY_FOR_RELAY_2 == gatePtr->state)
    {
        if(DIR_OPEN == gatePtr->dir) setRelay(gatePtr, HIGH);
        else setRelay(gatePtr, LOW);
        gatePtr->state = DELAY_FOR_RELAY_3;
        gatePtr->cnt = BREAK_BEFORE_KICKBACK;
    }
    else if(DELAY_FOR_RELAY_3 == gatePtr->state)
    {
        if(0 < gatePtr->cnt) gatePtr->cnt--;
        else
        {
            setPwm(gatePtr, MAX_PWM);
            gatePtr->cnt = KICKBACK_TIME;
            gatePtr->state = BACK_MOTOR;
        }
    }
    else if(BACK_MOTOR == gatePtr->state)
    {
        if(0 < gatePtr->cnt) gatePtr->cnt--;
        else
        {
            setPwm(gatePtr, 0);
            gatePtr->state = END;
        }
    }
    else if(END == gatePtr->state)
    {
        gatePtr->isRunning = false;
        setRelay(gatePtr, LOW);

        if(DIR_CLOSE == gatePtr->dir)
        {
            gatePtr->pos = POS_CLOSED;
        }
        if(DIR_OPEN == gatePtr->dir)
        {
            gatePtr->pos = POS_OPEN;
        }

        if(&gate1 == gatePtr) gateDriver.isRunningG1 = false;
        if(&gate2 == gatePtr) gateDriver.isRunningG2 = false;

        gatePtr->lastRunTime = gatePtr->runTime;
        gatePtr->state = STOP;
    }

    if(true == gatePtr->isRunning)
    {
        gatePtr->runTime += GATE_LOGIC_TASK_PERIOD;
        if(gatePtr->runTime > INRUSH_DISABLE_MONITOR_TIME) gatePtr->currentMonitorEnableInrush = true;

        if(STOP_TYPE_NONE == gatePtr->stopType)
        {
            if(gatePtr->runTime > RUN_TIME_MAX)
            {
                stopGate(gatePtr, STOP_TYPE_TIME);
            }
            if(gatePtr->distance > MAX_DISTANCE)
            {
                stopGate(gatePtr, STOP_TYPE_MAX_DISTANCE);
            }
        }
    }
}

void gateCurrentControl(struct gate* gatePtr)
{
    if(gatePtr->current > gatePtr->maxCurrent)
        gatePtr->maxCurrent = gatePtr->current;

    if((true == gatePtr->currentMonitorEnableInrush) && (true == gatePtr->currentMonitorEnableOvercurrent))
    {
        gatePtr->currentBuff[gatePtr->ptr] = gatePtr->current;
        gatePtr->ptr++;
        if(gatePtr->ptr >= CURRENT_BUFF_SIZE) gatePtr->ptr = 0;

        if(true == overcurrentDetected(gatePtr))
        {
            setPwm(gatePtr, 0);
            gateDriver.canAcceptCommand = false;
            gatePtr->currentMonitorEnableOvercurrent = false;
            // gatePtr->isRunning = false;
            gatePtr->state = OVERCURRENT_DETECTED;
            if(gatePtr == &gate1) Serial.println("gate1: overcurrent");
            if(gatePtr == &gate2) Serial.println("gate2: overcurrent");
        }
    }
    else
    {
    }
}

uint8_t currentThresholdCalculate(uint8_t current, uint8_t pwm)
{
    uint16_t tmp = current;
    tmp = tmp * pwm / 200 + current / 2;
    return (uint8_t)tmp;
}

bool overcurrentDetected(struct gate* gatePtr)
{
    int cnt = 0;
    bool ret = false;

    static int currThreshold;
    static int lastIndex;
    static int mVal;
    static int nVal;
    static int aboveCnt;
    static int actIndex;

    lastIndex = gatePtr->ptr;

    for(uint8_t i = 0; i < OVERLOAD_TAB_ENTRIES; i++)
    {
        if(false == ret)
        {
            actIndex = lastIndex;
            aboveCnt = 0;
            currThreshold = overloadTab[i][0];
            currThreshold = currentThresholdCalculate(currThreshold, gatePtr->pwm);

            mVal = overloadTab[i][1];
            nVal = overloadTab[i][2];

            for(uint8_t j = 0; j < nVal; j++)
            {
                if(gatePtr->currentBuff[actIndex] >= currThreshold) aboveCnt++;
                if(actIndex == 0) actIndex = CURRENT_BUFF_SIZE - 1;
                else actIndex--;
            }
            if(aboveCnt >= mVal)
            {
                ret = true;
                Serial.print("type: ");
                Serial.println(i);
            }
        }
    }

    return ret;
}