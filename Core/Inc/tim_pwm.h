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
#define LPWM1 TIM_CHANNEL_1
#define RPWM1 TIM_CHANNEL_2
#define LPWM2 TIM_CHANNEL_3
#define RPWM2 TIM_CHANNEL_4

#define PWM_Get(_TIM_,CHANNEL)                  __HAL_TIM_GET_COMPARE(_TIM_,CHANNEL)
#define PWM_Start(_TIM_,CHANNEL)                HAL_TIM_PWM_Start(_TIM_,CHANNEL)
#define TIM_SET(_TIM_,_PRESCALER_,_MAX_)        PWM_SET(_TIM_,_PRESCALER_,_MAX_)
#define TIM_GET(_TIM_,CHANNEL)					__HAL_TIM_SET_COUNTER(_TIM_,CHANNEL)
 void TIM_SET(TIM_HandleTypeDef* _TIM_,uint32_t _PRESCALER_,uint32_t _PERIOD_)
 {
 __HAL_TIM_SET_AUTORELOAD(_TIM_,_PERIOD_);
 __HAL_TIM_SET_PRESCALER(_TIM_,_PRESCALER_);
 }
 uint16_t PWM_Write(TIM_HandleTypeDef* __HANDLE__,uint32_t CHANNEL,uint8_t _VAR_)
 {
 uint32_t COMPARE = (((__HANDLE__)->Instance->ARR/100)*(_VAR_));
	 __HAL_TIM_SET_COMPARE(__HANDLE__,CHANNEL,COMPARE);
	 return COMPARE;
 }
 #endif
