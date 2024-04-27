#ifndef AML_KEY_ADC_H
#define AML_KEY_ADC_H
#include "main.h"

int AML_keyboard_getKey(int inputValue);
int AML_keyboard_readKeyLoop(int);
void AML_Keyboard_readResetKey(void);

#endif
