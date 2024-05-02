#ifndef AML_CONTROLL_MOTOR_H
#define AML_CONTROLL_MOTOR_H
#include "tim_pwm.h"
#include "main.h"
#include "parameter.h"

void AML_motor_stop(void);

void AML_motor_forward(void);

void AML_motor_backward(void);

void AML_motor_left(void);

void AML_motor_right(void);

void AML_motor_turn_left(void);

void AML_motor_turn_right(void);

void AML_motor_rote(void);
#endif
