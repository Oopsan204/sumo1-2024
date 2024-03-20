#ifndef AML_DEBUGDEVICE_H
#define AML_DEBUGDEVICE_H

#include "stm32f1xx_hal.h"
#include "main.h"

// Led number
// #define LEDPORT GPIOC



void AML_DebugDevice_BuzzBeep(uint16_t delay);
void AML_DebugDevice_BuzzBeep_TitTit(uint8_t turn);

#endif // AML_DEBUGDEVICE_H
