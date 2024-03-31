#ifndef AML_LASERSENSOR_H
#define AML_LASERSENSOR_H

#include "stm32f1xx.h"
#include "vl53l0x_api.h"
#include "main.h"
#include "KalmanFilter.h"

typedef enum
{
    FL,
    FF,
    FR,
    BR,
    BL,
    R,
    L
} LaserName;

void AML_LaserSensor_Setup(void);
void AML_LaserSensor_ReadAll(void);
void print_sensorvalue(void);
void AML_LaserSensor_TestLaser(void);
int32_t AML_LaserSensor_ReadSingleWithFillter(uint8_t name);
int32_t AML_LaserSensor_ReadSingleWithoutFillter(uint8_t name);


#endif
