#ifndef AML_MPUSensor_H
#define AML_MPUSensor_H

#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
// #include "stm32f4xx_hal_uart.h"

void AML_MPUSensor_Setup(void);
void AML_MPUSensor_ResetAngle(void);
double AML_MPUSensor_GetAngle(void);

#endif /* test_h */