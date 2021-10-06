#ifndef REFRAKTO_H_
#define REFRAKTO_H_

#include <stdbool.h>

#define ADC_FILTER(input, alpha, prior) (((uint32_t)input * (256 - alpha) + ((uint32_t)prior * alpha))) >> 8

#define LED_PIN (1<<PB5)
#define LED_HI PORTB |= LED_PIN
#define LED_LO PORTB &= ~LED_PIN
#define LED_TOG PORTB ^= LED_PIN

#define REL_1_PIN (1<<PD4)
#define REL_1_HI PORTD |= REL_1_PIN
#define REL_1_LO PORTD &= ~REL_1_PIN

#define REL_2_PIN (1<<PD5)
#define REL_2_HI PORTD |= REL_2_PIN
#define REL_2_LO PORTD &= ~REL_2_PIN

#define REL_MAIN_PIN (1<<PD6)
#define REL_MAIN_HI PORTD |= REL_MAIN_PIN
#define REL_MAIN_LO PORTD &= ~REL_MAIN_PIN

#define SW1_PIN (1<<PD2)
#define SW2_PIN (1<<PD3)

#define KEY_BOTH_LOW (!(PIND &= SW1_PIN))
#define KEY_ONE_LOW (!(PIND &= SW2_PIN))

#define ANALOG_GATE_1 A0
#define ANALOG_GATE_2 A1
#define ANALOG_VBAT   A2

#define PWM_GATE_1 9
#define PWM_GATE_2 10

#define MAX_PWM 255
#define INIT_PWM 50
#define LOW_SPEED_PWM 160

#define RUN_TIME_MAX (11*1000)
#define SLOW_DOWN_TIME (7*1000)

typedef enum
{
  KEY_BOTH,
  KEY_ONE,
  KEY_NONE
} KEY_T;


typedef enum
{
  COMMAND_BOTH,
  COMMAND_ONE,
  COMMAND_NONE
} COMMAND_T;

typedef enum
{
  OPEN,
  CLOSED,
  OPENING,
  CLOSING,
  INIT, 
  DELAY_FOR_RELAY, 
  INRUSH,
  RUN,
  SLOW_DOWN,
  LOW_SPEED,
  SLOW_TO_ZERO,
  WAIT_FOR_RELAY,
  END,
  STOP,
} GATE_STATE_T;

typedef enum
{
  GATE_ERROR_1

} GATE_ERROR_T;

typedef enum
{
  POS_OPEN,
  POS_CLOSED,
  POS_MID,
} GATE_POS_T;

typedef enum
{
  DIR_OPEN,
  DIR_CLOSE,
} GATE_DIR_T;

struct gate {
  uint8_t pwmPin;
  uint16_t currentThreshold;
  uint16_t cnt;
  uint8_t pwm;
  uint16_t runTime;
  uint16_t slowDownTime;
  volatile uint16_t current;
  volatile uint16_t sum;
  volatile uint8_t sumCnt;
  volatile bool adcComplete;
  bool isRunning;
  bool overcurrentFlag;
  bool currentControll;
  GATE_STATE_T state;
  GATE_DIR_T dir;
  GATE_POS_T pos;
  GATE_ERROR_T error;
  uint8_t ptr;
  uint16_t currentBuff[6];
};

struct adcBuff {
  uint8_t ptr;
  uint16_t buff[6];
};

typedef enum
{
  DRIVER_POS_OPEN,
  DRIVER_POS_CLOSED,
  DRIVER_POS_MID,
} DRIVER_POS_T;

typedef enum
{
  DRIVER_STATE_IDLE,
  DRIVER_STATE_PREPARE,
  DRIVER_STATE_RUN,
  DRIVER_STATE_END_PREPARE,
  DRIVER_STATE_END,
  DRIVER_STATE_OPEN_ONE,
  DRIVER_STATE_OPEN_BOTH,
  DRIVER_STATE_CLOSE_ONE,
  DRIVER_STATE_CLOSE_BOTH,
} DRIVER_STATE_T;

typedef enum
{
  DRIVER_RUN_MODE_ONE,
  DRIVER_RUN_MODE_BOTH,
} DRIVER_RUN_MODE_T;
typedef enum
{
  DRIVER_DIR_OPENING,
  DRIVER_DIR_CLOSING,
} DRIVER_DIR_T;

struct driver {
  uint16_t timeToStartGate2;
  bool isRunning;
  bool isSecondGateRunning;
  uint16_t runTime;
  uint16_t error;
  volatile uint16_t batteryVoltage;
  volatile uint16_t sum;
  volatile uint8_t sumCnt;
  DRIVER_STATE_T state;
  DRIVER_POS_T pos;
  DRIVER_RUN_MODE_T runMode;
  COMMAND_T cmd;
  DRIVER_DIR_T dir;
};


#endif /* REFRAKTO_H_ */