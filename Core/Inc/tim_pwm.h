#ifndef __TIMER_PWM_H__
#define __TIMER_PWM_H__

#include "main.h"
/*
_TIM_ :ex: &htim1
CHANNEL: ex: CH1
_PRESCALER_:tan so
_MAX_:gia tri dem max
_VAR_ :do rong xung 0-100%

*/
#define CH1 TIM_CHANNEL_1
#define CH2 TIM_CHANNEL_2
#define CH3 TIM_CHANNEL_3
#define CH4 TIM_CHANNEL_4

#define PWM_Get(_TIM_,CHANNEL)                  __HAL_TIM_GET_COMPARE(_TIM_,CHANNEL)
#define PWM_Start(_TIM_,CHANNEL)                HAL_TIM_PWM_Start(_TIM_,CHANNEL)
#define TIM_SET(_TIM_,_PRESCALER_,_MAX_)        PWM_SET(_TIM_,_PRESCALER_,_MAX_)
 void TIM_SET(TIM_HandleTypeDef* _TIM_,uint32_t_PRESCALER_,uint32_t_PERIOD_)
 {
 __HAL_TIM_SET_AUTORELOAD(_TIM_,_PERIOD_);
 }
