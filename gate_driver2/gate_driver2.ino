#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/wdt.h> 

#include "gate_driver.h"




volatile uint16_t task_1000ms = 1000;
volatile uint16_t task_300ms = 300;
volatile uint16_t task_200ms = 200;
volatile uint16_t task_10ms = 10;
volatile uint16_t task_1ms = 1;

volatile uint8_t pwmCnt = 0;
volatile uint8_t pwmG1 = 0;
volatile uint8_t pwmG2 = 0;

ISR(TIMER2_COMPA_vect)
{
  if(task_1000ms > 0) task_1000ms--;
  if(task_300ms > 0) task_300ms--;
  if(task_200ms > 0) task_200ms--;
  if(task_10ms > 0) task_10ms--;
  if(task_1ms > 0) task_1ms--;

  pwmCnt++;
  if(pwmCnt >= 50) {
    pwmCnt = 0;
  }

  if(pwmG1 > pwmCnt) GATE1_LO;
  else GATE1_HI;

}

// próbka co 5ms, 15próbek prądu powyżej max przerywa
void setup()
{
  
  PORTD = SW1_PIN | SW2_PIN;
  DDRD = REL_1_PIN | REL_2_PIN | REL_MAIN_PIN;
  DDRB = LED_PIN | GATE1_PIN;

  GATE1_HI;

  Serial.begin(115200);

  Serial.println("Start driver");

  REL_MAIN_HI;

  // TIMER2: 1ms
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;
  TCCR2A |= (1<<WGM21);
  TCCR2B |= (1<<CS22); // 16MHz/64/(30+1)
  TIMSK2 |= (1<<OCIE2A);
  OCR2A = 249;

  delay(2000);
}


void incdec(uint8_t* ptr, int val)
{
  int act;
  act = *ptr;

  act += val;
  if(act>50) act=50;
  if(act<0) act=0;
  *ptr = act;
  
  
}

void loop()
{
  
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if('1' == incomingByte) incdec(&pwmG1, -1);
    else if ('2' == incomingByte) incdec(&pwmG1, 1);
    Serial.println(pwmG1);
  }

  if(!task_1000ms)
  {
    task_1000ms = 5000;

  
  }

 
}
