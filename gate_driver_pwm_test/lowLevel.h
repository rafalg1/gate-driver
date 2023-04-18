#ifndef LOW_LEVEL_H
#define LOW_LEVEL_H

#include "gate_driver.h"

KEY_T getKey(void);
void setRelay(struct gate * gatePtr, uint8_t level);

#endif /* #ifndef LOW_LEVEL_H */