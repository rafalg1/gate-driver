#include "lowLevel.h"

#include <Arduino.h>

extern struct gate gate1;
extern struct gate gate2;
extern struct adc_struct adc;
extern struct driver gateDriver;

volatile uint8_t pwmCnt = 0;
volatile uint8_t pwmG1 = 0;
volatile uint8_t pwmG2 = 0;
volatile uint8_t pwmG1req = 0;
volatile uint8_t pwmG2req = 0;

volatile uint8_t adc_inputs[3] = {0, 1, 2};
volatile uint8_t adc_cnt = 0;
volatile uint16_t adcIntCnt = 0;

extern volatile uint16_t task_1000ms;
extern volatile uint16_t task_300ms;
extern volatile uint16_t task_200ms;
extern volatile uint16_t task_10ms;

#define TAB_SIZE 8
// 12 15 20
uint16_t currentTab_100[TAB_SIZE] = {8, 11, 15, 20, 25, 30, 40, 50};
uint16_t speedTab_100[TAB_SIZE] = {587, 575, 561, 544, 529, 515, 490, 380};

uint16_t getMap2D(uint16_t *argPtr, uint16_t *valuePtr, uint16_t val)
{
    if(val > argPtr[TAB_SIZE - 1]) val = argPtr[TAB_SIZE - 1];
    if(val < argPtr[0]) val = argPtr[0];

    uint8_t x = 0;

    for(int i = 0; i < TAB_SIZE; i++)
    {
        if(val >= argPtr[i]) x++;
        else break;
    }

    uint32_t x1, x2, y1, y2, result;

    if(x > TAB_SIZE - 1)
    {
        result = valuePtr[TAB_SIZE - 1];
    }
    else
    {
        //		A	B

        x2 = argPtr[x];
        x1 = argPtr[x - 1];

        y2 = valuePtr[x];
        y1 = valuePtr[x - 1];

        // Serial.print("x1: ");
        // Serial.print(x1);
        // Serial.print("  x2: ");
        // Serial.println(x2);
        // Serial.print("y1: ");
        // Serial.print(y1);
        // Serial.print("  y2: ");
        // Serial.println(y2);
        result = y1 - (y1 - y2) * (val - x1) / (x2 - x1);
    }
    // Serial.print("  result: ");
    // Serial.println(result);
    return (uint16_t)result;
}

void calculatePosition(void)
{
    uint16_t periodDistance = 0;

    if(gate1.motorInRun)
    {
        periodDistance = getMap2D(currentTab_100, speedTab_100, gate1.current) / 20;  // 50ms
        // Serial.print("dist: ");
        // Serial.println(periodDistance);

        gate1.distance += periodDistance;
        if(DIR_OPEN == gate1.relayState) gate1.position += periodDistance;
        else gate1.position -= periodDistance;
    }

    if(gate2.motorInRun)
    {
        periodDistance = getMap2D(currentTab_100, speedTab_100, gate2.current) / 20;  // 50ms

        gate2.distance += periodDistance;
        if(DIR_OPEN == gate2.relayState) gate2.position += periodDistance;
        else gate2.position -= periodDistance;
    }
}

void checkPositionAfterStop(void)
{
    if(gate1.position > POSITION_MAX) gate1.position = POSITION_MAX;
    if(gate1.position < POSITION_MIN) gate1.position = POSITION_MIN;
    if(gate2.position > POSITION_MAX) gate2.position = POSITION_MAX;
    if(gate2.position < POSITION_MIN) gate2.position = POSITION_MIN;
}

void checkPositionBeforeStart(void)
{
    gate1.fastRideAllowed = false;
    gate2.fastRideAllowed = false;

    if(DIR_OPEN == gate1.dir && gate1.position < POSITION_FAST_RIDE_OPEN) gate1.fastRideAllowed = true;
    if(DIR_CLOSE == gate1.dir && gate1.position > POSITION_FAST_RIDE_CLOSE) gate1.fastRideAllowed = true;

    if(DIR_OPEN == gate2.dir && gate2.position < POSITION_FAST_RIDE_OPEN) gate2.fastRideAllowed = true;
    if(DIR_CLOSE == gate2.dir && gate2.position > POSITION_FAST_RIDE_CLOSE) gate2.fastRideAllowed = true;

    Serial.print("fast ride 1: ");
    Serial.print(gate1.fastRideAllowed);
    Serial.print(" fast ride 2: ");
    Serial.println(gate1.fastRideAllowed);
}

void setPwm(int gatePtr, uint8_t val)
{
    uint8_t sol = val / 5;

    cli();

    if(&gate1 == gatePtr)
    {
        gate1.pwm = val;
        pwmG1req = sol;
        if(0 == pwmG1req) pwmG1 = 0, GATE1_HI, gate1.motorInRun = false;
        else gate1.motorInRun = true;
    }
    if(&gate2 == gatePtr)
    {
        gate2.pwm = val;
        pwmG2req = sol;
        if(0 == pwmG2req) pwmG2 = 0, GATE2_HI, gate2.motorInRun = false;
        else gate2.motorInRun = true;
    }

    sei();
}

ISR(TIMER2_COMPA_vect)
{
    uint16_t avgResult;

    if(task_1000ms > 0) task_1000ms--;
    if(task_300ms > 0) task_300ms--;
    if(task_200ms > 0) task_200ms--;
    if(task_10ms > 0) task_10ms--;

    pwmCnt++;
    if(pwmCnt >= 20)
    {
        pwmCnt = 0;

        pwmG1 = pwmG1req;
        pwmG2 = pwmG2req;

        avgResult = adc.gate1_sum / adc.gate1_sumCnt;
        gate1.current = 1000UL * avgResult / 3072;

        avgResult = adc.gate2_sum / adc.gate2_sumCnt;
        gate2.current = 1000UL * avgResult / 3072;

        avgResult = adc.battery_sum / adc.battery_sumCnt;
        gateDriver.batteryVoltage = 292UL * avgResult / 100;

        adc.gate1_sum = 0;
        adc.gate1_sumCnt = 0;
        adc.gate2_sum = 0;
        adc.gate2_sumCnt = 0;
        adc.battery_sum = 0;
        adc.battery_sumCnt = 0;

        gate1.adcComplete = true;
        gate2.adcComplete = true;

        gateDriver.printCurrent = true;
        gateDriver.pwmPeriodFlag = true;
    }

    if(pwmG1 > pwmCnt) GATE1_LO;
    else GATE1_HI;

    if(pwmG2 > pwmCnt) GATE2_LO;
    else GATE2_HI;
}

// ok 9KHz: 9pomiarów/ms, 3 kanały
ISR(ADC_vect)
{
    adcIntCnt++;
    uint16_t result = ADC;

    if(0 == adc_cnt)
    {
        adc.gate1_sum += result;
        adc.gate1_sumCnt++;
    }

    if(1 == adc_cnt)
    {
        adc.gate2_sum += result;
        adc.gate2_sumCnt++;
    }
    if(2 == adc_cnt)
    {
        adc.battery_sum += result;
        adc.battery_sumCnt++;
    }

    adc_cnt++;
    if(adc_cnt >= 3) adc_cnt = 0;
    ADMUX = ((1 << REFS0) | (adc_inputs[adc_cnt] & 0x0F));
    ADCSRA |= (1 << ADSC);
}

void setRelay(struct gate *gatePtr, uint8_t level)
{
    if(&gate1 == gatePtr)
    {
        if(HIGH == level) REL_1_ON, gate1.relayState = DIR_CLOSE;
        else REL_1_OFF, gate1.relayState = DIR_OPEN;
    }
    else if(&gate2 == gatePtr)
    {
        if(HIGH == level) REL_2_ON, gate2.relayState = DIR_CLOSE;
        else REL_2_OFF, gate2.relayState = DIR_OPEN;
    }
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
        if(drgTab[drgIdx] == deb)
        {
            drgTab[drgIdx]++;
            key = KEY_BOTH;
        }
    }
    else drgTab[drgIdx] = 0;

    drgIdx = 1;
    if(KEY_ONE_LOW)
    {
        if(drgTab[drgIdx] < deb) drgTab[drgIdx]++;
        if(drgTab[drgIdx] == deb)
        {
            drgTab[drgIdx]++;
            key = KEY_ONE;
        }
    }
    else drgTab[drgIdx] = 0;

    return key;
}
