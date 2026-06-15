/* sensor_interface.h */
#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "vl53l4cd.h"  /* Include the component driver */
#include "../../Drivers/vl53l4cd/vl53l4cd.h"

#define TOF_SENSOR_COUNT 3
extern VL53L4CD_Object_t SensorObjs[TOF_SENSOR_COUNT];


int32_t TOF_Init(void);
void TOF_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_INTERFACE_H */
