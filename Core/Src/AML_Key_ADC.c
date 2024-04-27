#include "AML_Key_ADC.h"
#include <stdbool.h>
#include "parameter.h"


extern ADC_HandleTypeDef hadc1;
// extern DMA_HandleTypeDef hdma_adc1;

// Biến lưu trữ giá trị analog cũ

// Hàm đọc giá trị analog
int16_t read_analog_value(int new_value)
{
    // Nếu giá trị mới là 4095, bỏ qua
    if (new_value == 4095)
    {
        return old_analog_value;
    }

    // Lưu giá trị cũ
    int temp = old_analog_value;

    // Cập nhật giá trị cũ
    old_analog_value = new_value;

    // Tính toán sự thay đổi
    int difference = new_value - temp;

    return difference;
}

int main()
{
    // Giả sử bạn đã đọc giá trị analog từ ADC
    int new_analog_value = 4095; // Giá trị mới

    // Gọi hàm đọc giá trị analog
    int change = read_analog_value(new_analog_value);

    if (change != 0)
    {
    }
    else
    {
    }

    return 0;
}
void readADCStore()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)button, 1);
    adcvalue = button[0];
    if (adcvalue == 0)
    {
        // HAL_Delay(3000);
        plan_begin();
        HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    }
    else if (adcvalue > 400 && adcvalue < 500)
    {
        AML_motor_stop();
        HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    }
    else if (adcvalue < 700 && adcvalue > 600)
    {
        plan = &search2;
        HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    }
    else if (adcvalue < 2800 && adcvalue > 2700)
    {
        HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    }
    else
    {
        AML_motor_stop();
    }
}