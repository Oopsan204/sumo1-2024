#include "AML_Key_ADC.h"
#include <stdbool.h>
#include "parameter.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// ADCkeyboar Module

uint16_t adc_key_val[Array_Size_Button] = {0, 127, 2141, 475, 307};
uint8_t NUM_KEY = 5;
uint8_t adc_key_in;
uint8_t oldkey = -1;
uint8_t keyValue; // luu nut bam
uint8_t key = -1;

void (*resetFunc)(void) = 0;

uint16_t AML_keyboard_getKey(uint8_t inputValue)
{
    uint8_t k;
    for (k = 0; k < NUM_KEY; k++)
    {
        if (inputValue < adc_key_val[k])
        {
            return k;
        }
    }
    if (k >= NUM_KEY)
        k = -1;
    return k;
}

// doc nut cho den khi phat hien bam nut
uint16_t AML_keyboard_readKeyLoop()
{
    bool flagPressButton = false;
    adc_key_in = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_key_val, 1);
    key = AML_keyboard_getKey(adc_key_in);
    if (key != oldkey)
    {
        HAL_Delay(50);
        adc_key_in = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_key_val, 1);
        key = AML_keyboard_getKey(adc_key_in);
        if (key != oldkey)
        {
            oldkey = key;
            if (key >= 0)
            {
                switch (key)
                {
                case 0:
                    keyValue = 0;
                    break;
                case 1:
                    keyValue = 1;
                    break;
                case 2:
                    keyValue = 2;
                    break;
                case 3:
                    keyValue = 3;
                    break;
                case 4:
                    keyValue = 4;
                    break;
                }
                flagPressButton = true;
            }
        }
    }
    if (!flagPressButton)
    {
        AML_keyboard_readKeyLoop();
    }
    else
    {
        return keyValue;
    }
}

// Đọc nút reset
void AML_Keyboard_readResetKey()
{
    adc_key_in = HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_key_val, 1); // read the value from the sensor pin A0
    key = AML_keyboard_getKey(adc_key_in);                              // convert into key press

    if (key != oldkey) // if keypress is detected
    {
        HAL_Delay(25);                                        // wait for debounce time
        adc_key_in = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_key_val, 1); // read the value from the sensor
        key = AML_keyboard_getKey(adc_key_in);                // convert into key press
        if (key != oldkey)
        {
            oldkey = key;
            if (key >= 0)
            {
                switch (key)
                {
                case 0:
                    keyValue = 0;
                    break;
                case 1:
                    keyValue = 1;
                    break;
                case 2:
                    keyValue = 2;
                    break;
                case 3:
                    keyValue = 3;
                    break;
                case 4:
                    keyValue = 4;
                    break;
                }

                if (keyValue == 1) // Nút B
                {
                    resetFunc(); // Lệnh reset
                }
            }
        }
    }
}
