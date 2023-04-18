#include <Arduino.h>


#include "lowLevel.h"

extern struct gate gate1;
extern struct gate gate2;

extern struct driver gateDriver;

volatile uint8_t adc_inputs[3] = {0,1,2}; 
volatile uint8_t adc_cnt = 0;

volatile uint16_t adcIntCnt = 0;

//ok 9KHz: 9pomiarów/ms, 3 kanały,uśrednianie 16, flaga co ok 5ms
ISR(ADC_vect)
{
  adcIntCnt++;
  uint16_t result = ADC;
  uint16_t avgResult; 
    
  if(0 == adc_cnt)
  {
    gate1.sum += result;
    gate1.sumCnt++;
    // if(gate1.sumCnt >= 16)
    // {
    //   avgResult = gate1.sum/16;
    //   uint32_t current = 1000UL*avgResult/3072;
    //   gate1.current = current;
    //   gate1.sum = 0;
    //   gate1.sumCnt = 0;
    //   gate1.adcComplete = true;
    // }
  }

  if(1 == adc_cnt)
  {
    gate2.sum += result;
    gate2.sumCnt++;
    // if(gate2.sumCnt >= 16)
    // {
    //   avgResult = gate2.sum/16;
    //   uint32_t current = 1000UL*avgResult/3072;
    //   gate2.current = current;
    //   gate2.sum = 0;
    //   gate2.sumCnt = 0;
    //   gate2.adcComplete = true;
    // }
  }

  if(2 == adc_cnt)
  {
    gateDriver.sum += result;
    gateDriver.sumCnt++;
    if(gateDriver.sumCnt >= 16)
    {
      avgResult = gateDriver.sum/16;
      uint32_t voltage = 1000UL*avgResult/3072;
      gateDriver.batteryVoltage = voltage;
      gateDriver.sum = 0;
      gateDriver.sumCnt = 0;
    }
  } 

  adc_cnt++;
  if(adc_cnt >= 3) adc_cnt=0;
  ADMUX = ((1<<REFS0) | (adc_inputs[adc_cnt] & 0x0F));
  ADCSRA |= (1<<ADSC);
}

void setRelay(struct gate * gatePtr, uint8_t level)
{
  if(&gate1 == gatePtr)
  {
    if(HIGH == level) REL_1_HI;
    else REL_1_LO;
  }
  else if(&gate2 == gatePtr)
  {
    if(HIGH == level) REL_2_HI;
    else REL_2_LO;
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
