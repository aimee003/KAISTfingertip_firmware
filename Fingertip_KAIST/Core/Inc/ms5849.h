/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	ms5849.h
 * @date	05 Apr 2018
 * @version	1.1.0
 * @brief
 *
 */

/*! @file ms5849.h
    @brief Sensor driver for MS5849 sensor */
/*!
 * @defgroup MS5849 SENSOR API
 * @{*/
#ifndef MS5849_H_
#define MS5849_H_

/* Header includes */
#include "ms5849_defs.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/*!
 *  @brief This API is the entry point.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id and calibration data of the sensor.
 *
 *  @param[in,out] dev : Structure instance of ms5849_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t ms5849_init(struct ms5849_dev *dev);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of ms5849_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t ms5849_soft_reset(const struct ms5849_dev *dev);

/*!
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the ms5849_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | MS5849_PRESS
 *     2       | MS5849_TEMP
 *     3       | MS5849_ALL
 *
 * @param[out] data : Structure instance of ms5849_data.
 * @param[in] dev : Structure instance of ms5849_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t ms5849_start_press_conversion(struct ms5849_dev *dev);
int8_t ms5849_start_temp_conversion(struct ms5849_dev *dev);
int8_t ms5849_read_press_raw(struct ms5849_dev *dev, uint32_t *pressure);
int8_t ms5849_read_temp_raw(struct ms5849_dev *dev, uint32_t *temperature);
void ms5849_compensate(struct ms5849_dev *dev, uint32_t d1, uint32_t d2,
                       struct ms5849_data *comp_data);
int8_t ms5849_get_sensor_data(uint8_t sensor_comp, struct ms5849_data *data, struct ms5849_dev *dev);


/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len : No of bytes of data to write..
 * @param[in] dev : Structure instance of ms5849_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t ms5849_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct ms5849_dev *dev);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] length : No of bytes of data to be read.
 * @param[in] dev : Structure instance of ms5849_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t ms5849_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, const struct ms5849_dev *dev);


int8_t ms5849_write_config(struct ms5849_dev *dev);
int8_t ms5849_read_config(struct ms5849_dev *dev, uint8_t sel_cfg);

/*!
 * @brief Sanity-check the calibration coefficients previously read from
 * PROM by ms5849_init(). Catches failed PROM reads (all-zero / all-ones
 * coefficient words) that would otherwise silently corrupt every
 * compensated pressure sample.
 *
 * @param[in] dev : Structure instance of ms5849_dev.
 * @return MS5849_OK if coefficients look valid, error code otherwise.
 */
int8_t ms5849_check_calib(const struct ms5849_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* MS5849_H_ */
/** @}*/



