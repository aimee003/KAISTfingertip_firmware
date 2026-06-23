#include "sensor_interface.h"
#include <stdio.h>
#include <stdint.h>
#include "string.h"
#include "main.h"

VL53L4CD_Object_t SensorObjs[TOF_SENSOR_COUNT];
static VL53L4CD_IO_t SensorIO;

extern I2C_HandleTypeDef hi2c1;

/* --- Private I2C Wrappers (Not in .h file) --- */
static int32_t CUSTOM_I2C_WriteReg(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
    return HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, Length, 100);
}

static int32_t CUSTOM_I2C_ReadReg(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
    return HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, Length, 100);
}

static int32_t CUSTOM_GetTick(void) {
    return HAL_GetTick();
}

static int32_t CUSTOM_Init(void) {
    return 0;
}

static int32_t CUSTOM_DeInit(void) {
    return 0;
}

/* --- Helper Function --- */
static int32_t Init_One_Sensor(int index, uint16_t new_addr) {
    int32_t status;
    uint32_t sensorId;
    VL53L4CD_Object_t *pObj = &SensorObjs[index];

    /* 1. Setup IO Struct for Registration (initially at default address) */
    SensorIO.Address = 0x52; 
    SensorIO.Init = CUSTOM_Init;
    SensorIO.DeInit = CUSTOM_DeInit;
    SensorIO.WriteReg = CUSTOM_I2C_WriteReg;
    SensorIO.ReadReg = CUSTOM_I2C_ReadReg;
    SensorIO.GetTick = CUSTOM_GetTick;

    /* 2. Register Bus IO */
    status = VL53L4CD_RegisterBusIO(pObj, &SensorIO);
    if (status != VL53L4CD_OK) return status;

    /* 3. Check ID */
    status = VL53L4CD_ReadID(pObj, &sensorId);
    if (status != VL53L4CD_OK) {
        // printf("Sensor %d I2C fail\n\r", index);
        return -10;
    }
    if (sensorId != VL53L4CD_ID) {
        // printf("Sensor %d Wrong ID: 0x%x\n\r", index, (unsigned int)sensorId);
        return -20;
    }

    /* 4. Set New Address */
    status = VL53L4CD_SetAddress(pObj, new_addr);
    if (status != VL53L4CD_OK) return -30;

    /* 5. Init Sensor */
    status = VL53L4CD_Init(pObj);
    if (status != VL53L4CD_OK) return -40;

    /* 6. Configure Profile */
    VL53L4CD_ProfileConfig_t Profile;
    Profile.RangingProfile = VL53L4CD_PROFILE_AUTONOMOUS;
    Profile.TimingBudget = 20;
    Profile.Frequency = 10;
    Profile.EnableAmbient = 1;
    Profile.EnableSignal = 1;

    status = VL53L4CD_ConfigProfile(pObj, &Profile);
    if (status != VL53L4CD_OK) return -50;

    /* 7. Start */
    return VL53L4CD_Start(pObj, VL53L4CD_MODE_ASYNC_CONTINUOUS);
}

/* --- Public Functions --- */

int32_t TOF_Init(void) {
    /* 1. Reset all sensors */
//    HAL_GPIO_WritePin(TOF_XSHUT_1_GPIO_Port, TOF_XSHUT_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF_XSHUT_2_GPIO_Port, TOF_XSHUT_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF_XSHUT_3_GPIO_Port, TOF_XSHUT_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

//    /* 2. Init Sensor 1 (Addr 0x54) */
//    HAL_GPIO_WritePin(TOF_XSHUT_1_GPIO_Port, TOF_XSHUT_1_Pin, GPIO_PIN_SET);
//    HAL_Delay(5);
//    if (Init_One_Sensor(0, 0x54) != 0) return -1;

    /* 3. Init Sensor 2 (Addr 0x56) */
    HAL_GPIO_WritePin(TOF_XSHUT_2_GPIO_Port, TOF_XSHUT_2_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    if (Init_One_Sensor(1, 0x56) != 0) return -2;

    /* 4. Init Sensor 3 (Addr 0x58) */
    HAL_GPIO_WritePin(TOF_XSHUT_3_GPIO_Port, TOF_XSHUT_3_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    if (Init_One_Sensor(2, 0x58) != 0) return -3;

    return 0;
}
