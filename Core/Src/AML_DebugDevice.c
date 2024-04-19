#include "AML_DebugDevice.h"

extern TIM_HandleTypeDef htim2;

void AML_DebugDevice_BuzzBeep(uint16_t delay)
{
    uint32_t InitTime = HAL_GetTick();

    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);

    while (HAL_GetTick() - InitTime < (uint32_t)delay)
        ;

    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_SET);
}

void AML_DebugDevice_BuzzBeep_TitTit(uint8_t turn)
{

    for (int i = 0; i <= turn; i++)
    {
        HAL_GPIO_WritePin(Buzz_GPIO_Port,Buzz_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Led_GPIO_Port,Led_Pin,GPIO_PIN_RESET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(Buzz_GPIO_Port,Buzz_Pin,GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(Led_GPIO_Port,Led_Pin,GPIO_PIN_SET);
    }
}

void AML_DebugDevice_Handle()
{
    // if (LedIndexFlag)
    // {
    //     AML_DebugDevice_ToggleLED(LedIndex++);

    //     if (LedIndex == 8)
    //     {
    //         LedIndex = 7;
    //         LedIndexFlag = 0;
    //     }
    // }
    // else
    // {
    //     AML_DebugDevice_ToggleLED(LedIndex--);

    //     if (LedIndex == 0)
    //     {
    //         LedIndex = 0;
    //         LedIndexFlag = 1;
    //     }
    // }

    // if (LedIndexFlag)
    // {
    //     Led
    // }
    // else
    // {
    //     AML_DebugDevice_TurnOffLED(LedIndex--);

    //     if (LedIndex == 0)
    //     {
    //         LedIndex = 0;
    //         LedIndexFlag = 1;
    //     }
    // }
}

//void AML_DebugDevice_TurnOnIT()
//{
//    HAL_TIM_Base_Start_IT(&htim3);
//}

//void AML_DebugDevice_TurnOffIT()
//{
//    HAL_TIM_Base_Stop_IT(&htim3);
//}
