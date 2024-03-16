#include "AML_LaserSensor.h"

// uint8_t LaserSensorAddress[] = {0x29, 0x59, 0x60, 0x32, 0x57};
uint8_t LaserSensorAddress[] = {0x32, 0x57, 0x60, 0x29, 0x59};
// define the name laser
//enum LaserPosition { BL1,BR1,FF1,FL1,FR1,R1,L1};
// define the laser anddresses
#define NUM_LASERS 7
uint8_t LaserSensorAddresses[NUM_LASERS] = {0x52, 0x52, 0x52, 0x52, 0x52, 0x52, 0x52};

// Define the GPIO pins
GPIO_TypeDef *XSHUT_GPIO_Ports[NUM_LASERS] = {XSHUT_BL_GPIO_Port, XSHUT_FF_GPIO_Port, XSHUT_BR_GPIO_Port, XSHUT_FL_GPIO_Port, XSHUT_FR_GPIO_Port, XSHUT_R_GPIO_Port, XSHUT_L_GPIO_Port};
uint16_t XSHUT_Pins[NUM_LASERS] = {XSHUT_BL_Pin, XSHUT_FF_Pin, XSHUT_BR_Pin, XSHUT_FL_Pin, XSHUT_FR_Pin, XSHUT_R_Pin, XSHUT_L_Pin};

SimpleKalmanFilter KalmanFilter[7];
extern I2C_HandleTypeDef hi2c1;

VL53L0X_RangingMeasurementData_t SensorValue[7];
VL53L0X_Dev_t Dev_Val[7];
VL53L0X_DEV Laser[7];

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

void AML_LaserSensor_Init(uint8_t i)
{
    VL53L0X_WaitDeviceBooted(Laser[i]);
    VL53L0X_DataInit(Laser[i]);
    VL53L0X_StaticInit(Laser[i]);
    VL53L0X_PerformRefCalibration(Laser[i], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Laser[i], &refSpadCount, &isApertureSpads);

    // VL53L0X_SetDeviceMode(Laser[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);

    VL53L0X_SetDeviceMode(Laser[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

    // Enable/Disable Sigma and Signal check
    VL53L0X_SetLimitCheckEnable(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);

    // VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
    // VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32 * 65536));
    // VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Laser[i], 20000);

    // if (i == BL || i == BR)
    // {
    VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
    VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Laser[i], 20000);
    // }
    // else
    // {
    // VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    // VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    // VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Laser[i], 33000);
    // }

    VL53L0X_SetVcselPulsePeriod(Laser[i], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(Laser[i], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

    VL53L0X_StartMeasurement(Laser[i]);
}

// Function to initialize all lasers
void InitAllLasers(void)
{
		uint8_t DelayTime = 70;
    for (int i = 0; i < NUM_LASERS; i++)
    {
        // Enable laser and init
        HAL_GPIO_WritePin(XSHUT_GPIO_Ports[i], XSHUT_Pins[i], GPIO_PIN_SET);
        HAL_Delay(DelayTime);
        Laser[i] = &Dev_Val[i];
        Laser[i]->I2cHandle = &hi2c1;
        Laser[i]->I2cDevAddr = LaserSensorAddresses[i];
        VL53L0X_SetDeviceAddress(Laser[i], LaserSensorAddresses[i]);
        Laser[i]->I2cDevAddr = LaserSensorAddresses[i];
        AML_LaserSensor_Init(i);
    }
}
void AML_LaserSensor_Setup(void)
{
    uint8_t DelayTime = 70;
    // disable all laser
    HAL_GPIO_WritePin(XSHUT_FL_GPIO_Port, XSHUT_FL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FF_GPIO_Port, XSHUT_FF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FR_GPIO_Port, XSHUT_FR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_BR_GPIO_Port, XSHUT_BR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_BL_GPIO_Port, XSHUT_BL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_L_GPIO_Port, XSHUT_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_R_GPIO_Port, XSHUT_R_Pin, GPIO_PIN_RESET);

    HAL_Delay(DelayTime);

    InitAllLasers();
     for (uint8_t i = 0; i < 7; i++)
    {
        SimpleKalmanFilter_Init(&KalmanFilter[i], 0.07, 0.01, 0.001);
    }
    /*
        thu tu: R, x, Q

        Bộ lọc Kalman sử dụng ba hệ số chính: Q (hệ số nhiễu quá trình), R (hệ số nhiễu đo lường) và x (giá trị ban đầu). Sự thay đổi của các hệ số này sẽ ảnh hưởng đến hiệu năng và độ chính xác của bộ lọc.

         Hệ số nhiễu quá trình Q: Hệ số này đại diện cho sự không chắc chắn trong mô hình quá trình của bạn. Nếu Q lớn, bộ lọc sẽ tin tưởng vào các đo lường hơn là dự đoán của nó. Điều này có thể làm giảm độ chính xác nếu nhiễu đo lường lớn.

        Hệ số nhiễu đo lường R: Hệ số này đại diện cho sự không chắc chắn trong các đo lường của bạn. Nếu R lớn, bộ lọc sẽ tin tưởng vào dự đoán của nó hơn là các đo lường. Điều này có thể làm giảm độ chính xác nếu mô hình quá trình của bạn không chính xác.

        Giá trị ban đầu x: Giá trị này là ước lượng ban đầu của trạng thái. Nếu giá trị này không chính xác, bộ lọc có thể mất thời gian để "hội tụ" với giá trị thực sự.
    */
}

void AML_LaserSensor_ReadAll(void)
{
    uint8_t i = 0;

    VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);

    if (SensorValue[i].RangeMilliMeter < 2000 && SensorValue[i].RangeMilliMeter > 10) // 2000 is the maximum range of the sensor
    {
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }

    i++;

    VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);

    if (SensorValue[i].RangeMilliMeter < 2000 && SensorValue[i].RangeMilliMeter > 10) // 2000 is the maximum range of the sensor
    {
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }

    i++;

    VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);

    if (SensorValue[i].RangeMilliMeter < 2000 && SensorValue[i].RangeMilliMeter > 10) // 2000 is the maximum range of the sensor
    {
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }

    i++;

    VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);

    if (SensorValue[i].RangeMilliMeter < 2000 && SensorValue[i].RangeMilliMeter > 10) // 2000 is the maximum range of the sensor
    {
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }

    i++;

    VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);

    if (SensorValue[i].RangeMilliMeter < 2000 && SensorValue[i].RangeMilliMeter > 10) // 2000 is the maximum range of the sensor
    {
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }
    i++;
    VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);

    if (SensorValue[i].RangeMilliMeter < 2000 && SensorValue[i].RangeMilliMeter > 10) // 2000 is the maximum range of the sensor
    {
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }
    i++;
    VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);

    if (SensorValue[i].RangeMilliMeter < 2000 && SensorValue[i].RangeMilliMeter > 10) // 2000 is the maximum range of the sensor
    {
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }
}

int32_t AML_LaserSensor_ReadSingleWithFillter(uint8_t name)
{
    // VL53L0X_GetRangingMeasurementData(Laser[name], &SensorValue[name]);

    // if (SensorValue[name].RangeMilliMeter < 2000) // 2000 is the maximum range of the sensor
    // {
    //     SensorValue[name].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[name], SensorValue[name].RangeMilliMeter);
    // }
    // VL53L0X_GetRangingMeasurementData(Laser[name], &SensorValue[name]);

    // if (SensorValue[name].RangeMilliMeter < 2000) // 2000 is the maximum range of the sensor
    // {
    //     SensorValue[name].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[name], SensorValue[name].RangeMilliMeter);
    // }

    // return (int32_t)SensorValue[name].RangeMilliMeter;
    return (int32_t)SensorValue[name].RangeMilliMeter;
}

int32_t AML_LaserSensor_ReadSingleWithoutFillter(uint8_t name)
{
    VL53L0X_GetRangingMeasurementData(Laser[name], &SensorValue[name]);

    if (SensorValue[name].RangeMilliMeter > 25)
    {
        return (int32_t)SensorValue[name].RangeMilliMeter;
    }
    else
    {
        return 2000;
    }
    // AML_LaserSensor_ReadAll();
    // return (int32_t)SensorValue[name].RangeMilliMeter;
}

//void AML_LaserSensor_TestLaser(void)
//{
//    int32_t t0 = AML_LaserSensor_ReadSingleWithoutFillter(FL);
//    int32_t t1 = AML_LaserSensor_ReadSingleWithoutFillter(FF);
//    int32_t t2 = AML_LaserSensor_ReadSingleWithoutFillter(FR);
//    int32_t t3 = AML_LaserSensor_ReadSingleWithoutFillter(BR);
//    int32_t t4 = AML_LaserSensor_ReadSingleWithoutFillter(BL);
//    int32_t t5 = AML_LaserSensor_ReadSingleWithoutFillter(R);
//    int32_t t6 = AML_LaserSensor_ReadSingleWithoutFillter(L);

//    for (int8_t i = 0; i < 7; i++)
//    {
//        if ((AML_LaserSensor_ReadSingleWithFillter(FL) - t0) != 0)
//        {
//        }
//        else
//        {
//        }

//        if ((AML_LaserSensor_ReadSingleWithFillter(FF) - t1) != 0)
//        {
//            AML_DebugDevice_TurnOnLED(1);
//        }
//        else
//        {
//            AML_DebugDevice_TurnOffLED(1);
//        }

//        if ((AML_LaserSensor_ReadSingleWithFillter(FR) - t2) != 0)
//        {
//            AML_DebugDevice_TurnOnLED(2);
//        }
//        else
//        {
//            AML_DebugDevice_TurnOffLED(2);
//        }

//        if ((AML_LaserSensor_ReadSingleWithFillter(BR) - t3) != 0)
//        {
//            AML_DebugDevice_TurnOnLED(3);
//        }
//        else
//        {
//            AML_DebugDevice_TurnOffLED(3);
//        }

//        if ((AML_LaserSensor_ReadSingleWithFillter(BL) - t4) != 0)
//        {
//            AML_DebugDevice_TurnOnLED(4);
//        }
//        else
//        {
//            AML_DebugDevice_TurnOffLED(4);
//        }

//        HAL_Delay(500);
//    }
//}
