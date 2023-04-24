#ifndef GATE_DRIVER_H
#define GATE_DRIVER_H

#include <stdint.h>

#define LED_PIN (1 << PB5)
#define LED_HI PORTB |= LED_PIN
#define LED_LO PORTB &= ~LED_PIN
#define LED_TOG PORTB ^= LED_PIN

#define GATE1_PIN (1 << PB1)
#define GATE1_HI PORTB |= GATE1_PIN
#define GATE1_LO PORTB &= ~GATE1_PIN

#define GATE2_PIN (1 << PB2)
#define GATE2_HI PORTB |= GATE2_PIN
#define GATE2_LO PORTB &= ~GATE2_PIN

#define REL_1_PIN (1 << PD4)
#define REL_1_HI PORTD |= REL_1_PIN
#define REL_1_LO PORTD &= ~REL_1_PIN
#define REL_1_OFF REL_1_HI
#define REL_1_ON REL_1_LO

#define REL_2_PIN (1 << PD5)
#define REL_2_HI PORTD |= REL_2_PIN
#define REL_2_LO PORTD &= ~REL_2_PIN
#define REL_2_OFF REL_2_HI
#define REL_2_ON REL_2_LO

#define REL_MAIN_PIN (1 << PD6)
#define REL_MAIN_HI PORTD |= REL_MAIN_PIN
#define REL_MAIN_LO PORTD &= ~REL_MAIN_PIN
#define REL_MAIN_ON REL_MAIN_HI
#define REL_MAIN_OFF REL_MAIN_LO

#define SW1_PIN (1 << PD2)
#define SW2_PIN (1 << PD3)
#define FOTO_PIN (1 << PD7)

#define KEY_BOTH_LOW (!(PIND & SW1_PIN))
#define KEY_ONE_LOW (!(PIND & SW2_PIN))
#define FOTO_LOW (!(PIND & FOTO_PIN))

#define ANALOG_GATE_1 A0
#define ANALOG_GATE_2 A1
#define ANALOG_VBAT A2

#define PWM_GATE_1 9
#define PWM_GATE_2 10

#define MAX_PWM 100
#define INIT_PWM 35
#define LEVEL_1_PWM 60
#define LEVEL_2_PWM 100
#define LOW_SPEED_PWM 45

#define RUN_1_TIME (2 * 1000)
#define RUN_TIME_MAX (40 * 1000)
#define SLOW_DOWN_TIME (19 * 1000)
#define THREAD_PITCH (5)                  // 5mm skok gwintu
#define MAX_DISTANCE (45 * THREAD_PITCH * 100)  // w obrotach

#define AVG_RUN_TIME (26 * 1000)

#define OVERLOAD_TAB_ENTRIES (5)

// 8 próbek z 10
#define CURRENT_THRESHOLD 140  // 12.0A
#define CURRENT_BUFF_SIZE 40   // 50ms*40 = 2s
#define OVERCURRENT_NUM 2

// pozycja wyrażona w liczbie obrotów
#define POSITION_MAX (175 * 100)
#define POSITION_MIN (0)

#define REVS_LEFT_TO_FAST_RIDE (60 * 100) // 12cm
#define SLOW_REVS (40 * 100) // 40 rev -> 8cm
#define POSITION_SLOW_OPEN (POSITION_MAX - SLOW_REVS)
#define POSITION_SLOW_CLOSE (SLOW_REVS)

#define POSITION_FAST_RIDE_OPEN (POSITION_MAX - REVS_LEFT_TO_FAST_RIDE)
#define POSITION_FAST_RIDE_CLOSE (REVS_LEFT_TO_FAST_RIDE)

#define getPositionInRev(x) ((x) / 100)


#define GATE_LOGIC_TASK_PERIOD (50)
#define DrIVER_LOGIC_TASK_PERIOD (50)




typedef enum
{
    STOP_TYPE_TIME,
    STOP_TYPE_MAX_DISTANCE,
    STOP_TYPE_OVERCURRENT,
    STOP_TYPE_CMD,
    STOP_TYPE_NONE
} STOP_TYPE_T;

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
    INIT,
    DELAY_FOR_RELAY,
    INRUSH_1,
    INRUSH_2,
    RUN_1,
    RUN_2,
    SLOW_DOWN,
    LOW_SPEED,
    SLOW_TO_ZERO,
    WAIT_FOR_RELAY,
    DELAY_FOR_RELAY_2,
    BACK_MOTOR,
    OVERCURRENT_DETECTED,
    DELAY_FOR_RELAY_3,
    END,
    STOP
} GATE_STATE_T;

typedef enum
{
    GATE_ERROR_1
} GATE_ERROR_T;

typedef enum
{
    POS_OPEN,
    POS_CLOSED,
    POS_MID
} GATE_POS_T;

typedef enum
{
    DIR_OPEN,
    DIR_CLOSE,
} GATE_DIR_T;

struct adcBuff
{
    uint8_t ptr;
    uint16_t buff[6];
};

typedef enum
{
    DRIVER_POS_ONE_OPEN,
    DRIVER_POS_BOTH_OPEN,
    DRIVER_POS_BOTH_CLOSED,
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

struct gate
{
    uint8_t pwmPin;
    uint16_t positionRaw;
    long position;      // 1.10 obrotu
    // int startPosition;  // 1.1 obrotu *10
    long distance;
    uint16_t lastRunTime;
    uint16_t cnt;
    uint8_t pwm;
    uint8_t relayState;
    uint16_t runTime;
    uint16_t slowDownTime;
    uint16_t currentThreshold;
    uint16_t maxCurrent;
    uint16_t maxCurrentInrush;
    volatile uint16_t current;
    // volatile uint32_t sum;
    // volatile uint16_t sumCnt;
    volatile bool adcComplete;
    bool isRunning;
    bool motorInRun;
    bool currentMonitorEnableOvercurrent;
    bool currentMonitorEnableInrush;
    bool fastRideAllowed;

    bool overcurrentFlag;
    bool currentControll;
    STOP_TYPE_T stopType;
    GATE_STATE_T state;
    GATE_DIR_T dir;
    GATE_POS_T pos;
    GATE_ERROR_T error;
    uint8_t ptr;
    uint16_t currentBuff[CURRENT_BUFF_SIZE];
};

struct adc_struct
{
    volatile uint32_t gate1_sum;
    volatile uint16_t gate1_sumCnt;
    volatile uint32_t gate2_sum;
    volatile uint16_t gate2_sumCnt;
    volatile uint32_t battery_sum;
    volatile uint16_t battery_sumCnt;
};

struct driver
{
    uint16_t timeToStartGate2;
    uint16_t timeCommand;
    volatile bool pwmPeriodFlag;
    bool canAcceptCommand;
    bool printCurrent;
    bool isRunning;
    bool isRunningG1;
    bool isRunningG2;
    bool isSecondGateRunning;
    uint16_t runTime;
    uint16_t error;
    volatile uint16_t batteryVoltage;
    // volatile uint16_t sum;
    // volatile uint8_t sumCnt;
    DRIVER_STATE_T state;
    DRIVER_POS_T pos;
    DRIVER_RUN_MODE_T runMode;
    COMMAND_T cmd;
    DRIVER_DIR_T dir;
};

#endif /* #ifndef GATE_DRIVER_H */
