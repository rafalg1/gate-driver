#ifndef LOW_LEVEL_H
#define LOW_LEVEL_H

#include "gate_driver.h"

void setPwm(int gate, uint8_t val);
void checkPositionAfterStop(void);
void checkPositionBeforeStart(void);
void calculatePosition(void);
KEY_T getKey(void);
void setRelay(struct gate* gatePtr, uint8_t level);

#endif /* #ifndef LOW_LEVEL_H */