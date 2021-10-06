#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include "main.h"
#include "Uart/uart.h"
#include "Gate/gate.h"



#define ADC_FILTER(input, alpha, prior) (((uint32_t)input * (256 - alpha) + ((uint32_t)prior * alpha))) >> 8
// extern struct stepper stepTab[2];

// AVR.source.compiler = "C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avr-c++.exe"
// avr-gcc -mmcu=atmega8 -Wall -Os -DF_CPU=8000000UL -o main.elf main.c Uart/uart.c Transceiver/transceiver.c Stepper/stepper.c -w
// avr-objcopy -j .text -j .data -O ihex main.elf main.hex
// avrdude -c usbasp -p m8 -U flash:w:"main.hex":a 


struct gate gate1, gate2;
struct driver gateDriver;


uint16_t flag_30ms = 3;
uint16_t flag_300ms = 300;

volatile uint16_t task_1000ms = 1000;
volatile uint16_t task_300ms = 300;
volatile uint16_t task_200ms = 200;
volatile uint16_t task_10ms = 10;

volatile uint8_t adc_inputs[3] = {0,1,2}; 
volatile uint8_t adc_cnt = 0;


volatile uint16_t adcIntCnt = 0;
uint8_t resetCause;


void handleCommand(COMMAND_T cmd)
{
  if(DRIVER_STATE_IDLE == gateDriver.state)
  {
    if(COMMAND_BOTH == cmd)
    {
      gateDriver.cmd = COMMAND_BOTH; 
    }
    else
    {
    }
  }
}

int main(void)
{
	_delay_ms(100);

	resetCause = MCUSR;
	analogWrite(PWM_GATE_1, 255);
	analogWrite(PWM_GATE_2, 255);
	
	PORTD = SW1_PIN | SW2_PIN;
	DDRD = REL_1_PIN | REL_2_PIN | REL_MAIN_PIN;
	DDRB = LED_PIN;

	// Serial.begin(115200);
    USART_Init(__UBRR);
	// Serial.println("Start driver");
	// Serial.println(resetCause);
	

	gateInit(&gate1);
	gateInit(&gate2);

	gate1.pwmPin = PWM_GATE_1;
	gate2.pwmPin = PWM_GATE_2;

	gateDriver.state = DRIVER_STATE_IDLE;
	gateDriver.pos = DRIVER_POS_CLOSED;
	gateDriver.timeToStartGate2 = 3000;
	gateDriver.cmd = COMMAND_NONE;;

	REL_1_LO;
	REL_2_LO;
	REL_MAIN_LO;

	// TIMER2: 1ms
	TCCR2A = 0;
	TCCR2B = 0;
	TIMSK2 = 0;
	TCCR2A |= (1<<WGM21);
	TCCR2B |= (1<<CS22); // 16MHz/64/(30+1)
	TIMSK2 |= (1<<OCIE2A);
	OCR2A = 249;

	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); //preskaler 128 -> 16/128 = 125kHz, sample rate: 9kHz
	ADMUX  = (1<<REFS0);
	ADCSRA |= (1<<ADSC);

	wdt_enable(WDTO_500MS);

	while(1)
	{
		if(!task_1000ms)
		{
			task_1000ms = 1000;
			adcIntCnt=0;
		}

		if (Serial.available() > 0) {
			char incomingByte = Serial.read();
			if('1' == incomingByte) handleCommand(COMMAND_BOTH);
			else if ('2' == incomingByte) handleCommand(COMMAND_ONE);
			else if ('3' == incomingByte) gateDataPrint(&gate1);
			else if ('4' == incomingByte) gateDataPrint(&gate2);
			else if ('7' == incomingByte) 
			{
			while(1)
			{

			}
			}
			else if ('6' == incomingByte) {
				Serial.println(MCUSR);
			}
			else if ('5' == incomingByte) 
			{
				REL_MAIN_HI;
				uint16_t tcnt = 0;
				while(tcnt < 2000)
				{
					tcnt++;
					delay(10);
					gate2.pwm ++;
					analogWrite(PWM_GATE_2, 255 - gate2.pwm);
				}
				gate2.pwm = 0;
				analogWrite(PWM_GATE_2, 255 - gate2.pwm);
				REL_MAIN_LO;
			}
		}

		if(!task_10ms)
		{
			task_10ms = 10;
			KEY_T key = getKey();

			if(KEY_BOTH == key) handleCommand(COMMAND_BOTH);
			else if(KEY_ONE == key) handleCommand(COMMAND_ONE);

			driverLogic();

			gateLogic(&gate1);
			gateLogic(&gate2);

			analogWrite(PWM_GATE_1, 255 - gate1.pwm);
			analogWrite(PWM_GATE_2, 255 - gate2.pwm);
		}

		gateCurrentControll(&gate1);
		gateCurrentControll(&gate2);

		if(!task_200ms)
		{
			task_200ms = 200;
			wdt_reset();
		}

	}
	
}


ISR(ADC_vect)
{
  adcIntCnt++;
  uint16_t result = ADC;
  uint16_t avgResult; 
    
  if(0 == adc_cnt)
  {
    gate1.sum += result;
    gate1.sumCnt++;
    if(gate1.sumCnt >= 16)
    {
      avgResult = gate1.sum/16;
      uint32_t current = 1000UL*avgResult/3072;
      gate1.current = current;
      gate1.sum = 0;
      gate1.sumCnt = 0;
      gate1.adcComplete = true;
    }
  }

  if(1 == adc_cnt)
  {
    gate2.sum += result;
    gate2.sumCnt++;
    if(gate2.sumCnt >= 16)
    {
      avgResult = gate2.sum/16;
      uint32_t current = 1000UL*avgResult/3072;
      gate2.current = current;
      gate2.sum = 0;
      gate2.sumCnt = 0;
      gate2.adcComplete = true;
    }
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

ISR(TIMER2_COMPA_vect)
{
  if(task_1000ms > 0) task_1000ms--;
  if(task_300ms > 0) task_300ms--;
  if(task_200ms > 0) task_200ms--;
  if(task_10ms > 0) task_10ms--;
}

