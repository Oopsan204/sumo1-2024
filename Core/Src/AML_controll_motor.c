#include "AML_controll_motor.h"
extern TIM_HandleTypeDef htim2;

void AML_motor_stop()
{
    PWM_Start(&htim2, LPWM1);
    PWM_Start(&htim2, LPWM2);
    PWM_Start(&htim2, RPWM1);
    PWM_Start(&htim2, RPWM2);
    PWM_Write(&htim2, LPWM1, 0);
    PWM_Write(&htim2, LPWM2, 0);
    PWM_Write(&htim2, RPWM1, 0);
    PWM_Write(&htim2, RPWM2, 0);
}
void AML_motor_forward(uint8_t speed)
{
    PWM_Start(&htim2, RPWM1);
    PWM_Start(&htim2, RPWM2);
    PWM_Write(&htim2, RPWM1, speed);
    PWM_Write(&htim2, RPWM2, speed);
}
void AML_motor_lef(uint8_t speed_L, uint8_t speed_R)
{
    PWM_Start(&htim2, RPWM2);
    PWM_Start(&htim2, LPWM1);
    PWM_Write(&htim2, RPWM2, speed_L - 50);
    PWM_Write(&htim2, LPWM1, speed_R);
}
void AML_motor_right(uint8_t speed_L, uint8_t speed_R)
{
    PWM_Start(&htim2, LPWM2);
    PWM_Start(&htim2, RPWM1);
    PWM_Write(&htim2, LPWM2, speed_L);
    PWM_Write(&htim2, RPWM1, speed_R - 50);
}
void AML_motor_back(uint8_t speed)
{
    PWM_Start(&htim2, LPWM1);
    PWM_Start(&htim2, LPWM2);
    PWM_Write(&htim2, LPWM1, speed);
    PWM_Write(&htim2, LPWM2, speed);
}
void AML_motor_rote(uint8_t speed_L, uint8_t speed_R)
{
    PWM_Start(&htim2, LPWM1);
    PWM_Start(&htim2, RPWM2);
    PWM_Write(&htim2, LPWM1, speed_L);
    PWM_Write(&htim2, RPWM2, speed_R);
}