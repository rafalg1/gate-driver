#ifndef PRINT_H
#define PRINT_H

#include "gate_driver.h"

void runSummaryPrint(void);
void gateDataPrint(struct gate* gatePtr);
void driverDataPrint(void);
void printGateName(struct gate* gatePtr);

#endif /* #ifndef PRINT_H */