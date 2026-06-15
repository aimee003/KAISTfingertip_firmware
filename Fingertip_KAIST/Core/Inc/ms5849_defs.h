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
 * @file	ms5849_defs.h
 * @date	05 Apr 2018
 * @version	1.1.0
 * @brief
 *
 */

/*! @file ms5849_defs.h
    @brief Sensor driver for MS5849 sensor */
/*!
 * @defgroup MS5849 SENSOR API
 * @brief
 * @{*/
#ifndef MS5849_DEFS_H_
#define MS5849_DEFS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/********************************************************/
/* header includes */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/*************************** Common macros   *****************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)       S8_C(x)
#define UINT8_C(x)      U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)      S16_C(x)
#define UINT16_C(x)     U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)      S32_C(x)
#define UINT32_C(x)     U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)      S64_C(x)
#define UINT64_C(x)     U64_C(x)
#endif

/**@}*/

/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE                UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE               UINT8_C(0)
#endif

/********************************************************/
/**\name Compiler switch macros */
/**\name Uncomment the below line to use floating-point compensation */
#ifndef BMP3_DOUBLE_PRECISION_COMPENSATION
#define BMP3_DOUBLE_PRECISION_COMPENSATION  // LE uncommented
#endif

/********************************************************/
/**\name Macro definitions */

/**\name I2C addresses */
#define BMP3_I2C_ADDR_PRIM	UINT8_C(0x76)
#define BMP3_I2C_ADDR_SEC	UINT8_C(0x77)

/**\name BMP3 chip identifier */
#define BMP3_CHIP_ID  UINT8_C(0x50)
/**\name BMP3 pressure settling time (micro secs)*/
#define BMP3_PRESS_SETTLE_TIME	UINT16_C(392)
/**\name BMP3 temperature settling time (micro secs) */
#define BMP3_TEMP_SETTLE_TIME		UINT16_C(313)
/**\name BMP3 adc conversion time (micro secs) */
#define BMP3_ADC_CONV_TIME		UINT16_C(2000)

/**\name Register Address */
#define BMP3_CHIP_ID_ADDR		UINT8_C(0x00)
#define BMP3_ERR_REG_ADDR		UINT8_C(0x02)
#define BMP3_SENS_STATUS_REG_ADDR	UINT8_C(0x03)
#define BMP3_DATA_ADDR		UINT8_C(0x04)
#define BMP3_EVENT_ADDR		UINT8_C(0x10)
#define BMP3_INT_STATUS_REG_ADDR	UINT8_C(0x11)
#define BMP3_FIFO_LENGTH_ADDR		UINT8_C(0x12)
#define BMP3_FIFO_DATA_ADDR		UINT8_C(0x14)
#define BMP3_FIFO_WM_ADDR		UINT8_C(0x15)
#define BMP3_FIFO_CONFIG_1_ADDR	UINT8_C(0x17)
#define BMP3_FIFO_CONFIG_2_ADDR	UINT8_C(0x18)
#define BMP3_INT_CTRL_ADDR		UINT8_C(0x19)
#define BMP3_IF_CONF_ADDR		UINT8_C(0x1A)
#define BMP3_PWR_CTRL_ADDR		UINT8_C(0x1B)
#define BMP3_OSR_ADDR			UINT8_C(0X1C)
#define BMP3_CALIB_DATA_ADDR		UINT8_C(0x31)
#define BMP3_CMD_ADDR			UINT8_C(0x7E)

/**\name Error status macros */
#define BMP3_FATAL_ERR	UINT8_C(0x01)
#define BMP3_CMD_ERR		UINT8_C(0x02)
#define BMP3_CONF_ERR		UINT8_C(0x04)

/**\name Status macros */
#define BMP3_CMD_RDY		UINT8_C(0x10)
#define BMP3_DRDY_PRESS	UINT8_C(0x20)
#define BMP3_DRDY_TEMP	UINT8_C(0x40)

/**\name Power mode macros */
#define BMP3_SLEEP_MODE		UINT8_C(0x00)
#define BMP3_FORCED_MODE		UINT8_C(0x01)
#define BMP3_NORMAL_MODE		UINT8_C(0x03)

/**\name FIFO related macros */
/**\name FIFO enable  */
#define BMP3_ENABLE			UINT8_C(0x01)
#define BMP3_DISABLE			UINT8_C(0x00)

/**\name Interrupt pin configuration macros */
/**\name Open drain */
#define BMP3_INT_PIN_OPEN_DRAIN		UINT8_C(0x01)
#define BMP3_INT_PIN_PUSH_PULL		UINT8_C(0x00)

/**\name Level */
#define BMP3_INT_PIN_ACTIVE_HIGH		UINT8_C(0x01)
#define BMP3_INT_PIN_ACTIVE_LOW		UINT8_C(0x00)

/**\name Latch */
#define BMP3_INT_PIN_LATCH			UINT8_C(0x01)
#define BMP3_INT_PIN_NON_LATCH		UINT8_C(0x00)

/**\name Advance settings  */
/**\name I2c watch dog timer period selection */
#define BMP3_I2C_WDT_SHORT_1_25_MS		UINT8_C(0x00)
#define BMP3_I2C_WDT_LONG_40_MS		UINT8_C(0x01)

/**\name FIFO Sub-sampling macros */
#define BMP3_FIFO_NO_SUBSAMPLING		UINT8_C(0x00)
#define BMP3_FIFO_SUBSAMPLING_2X		UINT8_C(0x01)
#define BMP3_FIFO_SUBSAMPLING_4X		UINT8_C(0x02)
#define BMP3_FIFO_SUBSAMPLING_8X		UINT8_C(0x03)
#define BMP3_FIFO_SUBSAMPLING_16X		UINT8_C(0x04)
#define BMP3_FIFO_SUBSAMPLING_32X		UINT8_C(0x05)
#define BMP3_FIFO_SUBSAMPLING_64X		UINT8_C(0x06)
#define BMP3_FIFO_SUBSAMPLING_128X		UINT8_C(0x07)

/**\name Over sampling macros */
#define BMP3_NO_OVERSAMPLING		UINT8_C(0x00)
#define BMP3_OVERSAMPLING_2X		UINT8_C(0x01)
#define BMP3_OVERSAMPLING_4X		UINT8_C(0x02)
#define BMP3_OVERSAMPLING_8X		UINT8_C(0x03)
#define BMP3_OVERSAMPLING_16X		UINT8_C(0x04)
#define BMP3_OVERSAMPLING_32X		UINT8_C(0x05)

/**\name Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE		UINT8_C(0x00)
#define BMP3_IIR_FILTER_COEFF_1		UINT8_C(0x01)
#define BMP3_IIR_FILTER_COEFF_3		UINT8_C(0x02)
#define BMP3_IIR_FILTER_COEFF_7		UINT8_C(0x03)
#define BMP3_IIR_FILTER_COEFF_15		UINT8_C(0x04)
#define BMP3_IIR_FILTER_COEFF_31		UINT8_C(0x05)
#define BMP3_IIR_FILTER_COEFF_63		UINT8_C(0x06)
#define BMP3_IIR_FILTER_COEFF_127		UINT8_C(0x07)

/**\name Odr setting macros */
#define BMP3_ODR_200_HZ		UINT8_C(0x00)
#define BMP3_ODR_100_HZ		UINT8_C(0x01)
#define BMP3_ODR_50_HZ		UINT8_C(0x02)
#define BMP3_ODR_25_HZ		UINT8_C(0x03)
#define BMP3_ODR_12_5_HZ		UINT8_C(0x04)
#define BMP3_ODR_6_25_HZ		UINT8_C(0x05)
#define BMP3_ODR_3_1_HZ		UINT8_C(0x06)
#define BMP3_ODR_1_5_HZ		UINT8_C(0x07)
#define BMP3_ODR_0_78_HZ		UINT8_C(0x08)
#define BMP3_ODR_0_39_HZ		UINT8_C(0x09)
#define BMP3_ODR_0_2_HZ		UINT8_C(0x0A)
#define BMP3_ODR_0_1_HZ		UINT8_C(0x0B)
#define BMP3_ODR_0_05_HZ		UINT8_C(0x0C)
#define BMP3_ODR_0_02_HZ		UINT8_C(0x0D)
#define BMP3_ODR_0_01_HZ		UINT8_C(0x0E)
#define BMP3_ODR_0_006_HZ		UINT8_C(0x0F)
#define BMP3_ODR_0_003_HZ		UINT8_C(0x10)
#define BMP3_ODR_0_001_HZ		UINT8_C(0x11)

/**\name API success code */
#define MS5849_OK							INT8_C(0)
/**\name API error codes */
#define MS5849_E_NULL_PTR					INT8_C(-1)
#define MS5849_E_DEV_NOT_FOUND				INT8_C(-2)
#define MS5849_E_INVALID_ODR_OSR_SETTINGS	INT8_C(-3)
#define MS5849_E_CMD_EXEC_FAILED			INT8_C(-4)
#define MS5849_E_CONFIGURATION_ERR			INT8_C(-5)
#define MS5849_E_INVALID_LEN				INT8_C(-6)
#define MS5849_E_COMM_FAIL					INT8_C(-7)
#define MS5849_E_FIFO_WATERMARK_NOT_REACHED	INT8_C(-8)

/**\name API warning codes */
#define BMP3_W_SENSOR_NOT_ENABLED		UINT8_C(1)
#define BMP3_W_INVALID_FIFO_REQ_FRAME_CNT	UINT8_C(2)

/**\name Macros to select the which sensor settings are to be set by the user.
   These values are internal for API implementation. Don't relate this to
   data sheet. */
#define	BMP3_PRESS_EN_SEL				UINT16_C(1 << 1)
#define BMP3_TEMP_EN_SEL				UINT16_C(1 << 2)
#define BMP3_DRDY_EN_SEL				UINT16_C(1 << 3)
#define BMP3_PRESS_OS_SEL				UINT16_C(1 << 4)
#define BMP3_TEMP_OS_SEL				UINT16_C(1 << 5)
#define BMP3_IIR_FILTER_SEL				UINT16_C(1 << 6)
#define BMP3_ODR_SEL					UINT16_C(1 << 7)
#define BMP3_OUTPUT_MODE_SEL				UINT16_C(1 << 8)
#define BMP3_LEVEL_SEL				UINT16_C(1 << 9)
#define BMP3_LATCH_SEL				UINT16_C(1 << 10)
#define BMP3_I2C_WDT_EN_SEL				UINT16_C(1 << 11)
#define BMP3_I2C_WDT_SEL_SEL				UINT16_C(1 << 12)
#define BMP3_ALL_SETTINGS				UINT16_C(0x7FF)

/**\name Macros to select the which FIFO settings are to be set by the user
   These values are internal for API implementation. Don't relate this to
   data sheet.*/
#define BMP3_FIFO_MODE_SEL			UINT16_C(1 << 1)
#define BMP3_FIFO_STOP_ON_FULL_EN_SEL		UINT16_C(1 << 2)
#define BMP3_FIFO_TIME_EN_SEL			UINT16_C(1 << 3)
#define BMP3_FIFO_PRESS_EN_SEL		UINT16_C(1 << 4)
#define BMP3_FIFO_TEMP_EN_SEL			UINT16_C(1 << 5)
#define BMP3_FIFO_DOWN_SAMPLING_SEL		UINT16_C(1 << 6)
#define BMP3_FIFO_FILTER_EN_SEL		UINT16_C(1 << 7)
#define BMP3_FIFO_FWTM_EN_SEL			UINT16_C(1 << 8)
#define BMP3_FIFO_FULL_EN_SEL			UINT16_C(1 << 9)
#define BMP3_FIFO_ALL_SETTINGS		UINT16_C(0x3FF)

/**\name Sensor component selection macros
   These values are internal for API implementation. Don't relate this to
   data sheet.*/
#define BMP3_PRESS        UINT8_C(1)
#define BMP3_TEMP         UINT8_C(1 << 1)
#define BMP3_ALL          UINT8_C(0x03)

/**\name Macros for bit masking */
#define BMP3_ERR_FATAL_MSK		UINT8_C(0x01)

#define BMP3_ERR_CMD_MSK		UINT8_C(0x02)
#define BMP3_ERR_CMD_POS		UINT8_C(0x01)

#define BMP3_ERR_CONF_MSK		UINT8_C(0x04)
#define BMP3_ERR_CONF_POS		UINT8_C(0x02)

#define BMP3_STATUS_CMD_RDY_MSK	UINT8_C(0x10)
#define BMP3_STATUS_CMD_RDY_POS	UINT8_C(0x04)

#define BMP3_STATUS_DRDY_PRESS_MSK	UINT8_C(0x20)
#define BMP3_STATUS_DRDY_PRESS_POS	UINT8_C(0x05)

#define BMP3_STATUS_DRDY_TEMP_MSK	UINT8_C(0x40)
#define BMP3_STATUS_DRDY_TEMP_POS	UINT8_C(0x06)

#define BMP3_OP_MODE_MSK		UINT8_C(0x30)
#define BMP3_OP_MODE_POS		UINT8_C(0x04)

#define BMP3_PRESS_EN_MSK		UINT8_C(0x01)

#define BMP3_TEMP_EN_MSK		UINT8_C(0x02)
#define BMP3_TEMP_EN_POS		UINT8_C(0x01)

#define BMP3_IIR_FILTER_MSK		UINT8_C(0x0E)
#define BMP3_IIR_FILTER_POS		UINT8_C(0x01)

#define BMP3_ODR_MSK			UINT8_C(0x1F)

#define BMP3_PRESS_OS_MSK		UINT8_C(0x07)

#define BMP3_TEMP_OS_MSK		UINT8_C(0x38)
#define BMP3_TEMP_OS_POS		UINT8_C(0x03)

#define BMP3_FIFO_MODE_MSK		UINT8_C(0x01)

#define BMP3_FIFO_STOP_ON_FULL_MSK	UINT8_C(0x02)
#define BMP3_FIFO_STOP_ON_FULL_POS	UINT8_C(0x01)

#define BMP3_FIFO_TIME_EN_MSK		UINT8_C(0x04)
#define BMP3_FIFO_TIME_EN_POS		UINT8_C(0x02)

#define BMP3_FIFO_PRESS_EN_MSK	UINT8_C(0x08)
#define BMP3_FIFO_PRESS_EN_POS	UINT8_C(0x03)

#define BMP3_FIFO_TEMP_EN_MSK		UINT8_C(0x10)
#define BMP3_FIFO_TEMP_EN_POS		UINT8_C(0x04)

#define BMP3_FIFO_FILTER_EN_MSK	UINT8_C(0x18)
#define BMP3_FIFO_FILTER_EN_POS	UINT8_C(0x03)

#define BMP3_FIFO_DOWN_SAMPLING_MSK	UINT8_C(0x07)

#define BMP3_FIFO_FWTM_EN_MSK		UINT8_C(0x08)
#define BMP3_FIFO_FWTM_EN_POS		UINT8_C(0x03)

#define BMP3_FIFO_FULL_EN_MSK		UINT8_C(0x10)
#define BMP3_FIFO_FULL_EN_POS		UINT8_C(0x04)

#define BMP3_INT_OUTPUT_MODE_MSK	UINT8_C(0x01)

#define BMP3_INT_LEVEL_MSK		UINT8_C(0x02)
#define BMP3_INT_LEVEL_POS		UINT8_C(0x01)

#define BMP3_INT_LATCH_MSK		UINT8_C(0x04)
#define BMP3_INT_LATCH_POS		UINT8_C(0x02)

#define BMP3_INT_DRDY_EN_MSK		UINT8_C(0x40)
#define BMP3_INT_DRDY_EN_POS		UINT8_C(0x06)

#define BMP3_I2C_WDT_EN_MSK		UINT8_C(0x02)
#define BMP3_I2C_WDT_EN_POS		UINT8_C(0x01)

#define BMP3_I2C_WDT_SEL_MSK		UINT8_C(0x04)
#define BMP3_I2C_WDT_SEL_POS		UINT8_C(0x02)

#define BMP3_INT_STATUS_FWTM_MSK	UINT8_C(0x01)

#define BMP3_INT_STATUS_FFULL_MSK	UINT8_C(0x02)
#define BMP3_INT_STATUS_FFULL_POS	UINT8_C(0x01)

#define BMP3_INT_STATUS_DRDY_MSK	UINT8_C(0x08)
#define BMP3_INT_STATUS_DRDY_POS	UINT8_C(0x03)


/**\name	UTILITY MACROS	*/
#define	BMP3_SET_LOW_BYTE			UINT16_C(0x00FF)
#define	BMP3_SET_HIGH_BYTE			UINT16_C(0xFF00)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)


#define BMP3_SET_BITS(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))
/* Macro variant to handle the bitname position if it is zero */
#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

#define BMP3_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
							(bitname##_POS))
/* Macro variant to handle the bitname position if it is zero */
#define BMP3_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

#define BMP3_GET_LSB(var)	(uint8_t)(var & BMP3_SET_LOW_BYTE)
#define BMP3_GET_MSB(var)	(uint8_t)((var & BMP3_SET_HIGH_BYTE) >> 8)

/**\name Macros related to size */
#define BMP3_CALIB_DATA_LEN		UINT8_C(21)
#define BMP3_P_AND_T_HEADER_DATA_LEN	UINT8_C(7)
#define BMP3_P_OR_T_HEADER_DATA_LEN	UINT8_C(4)
#define BMP3_P_T_DATA_LEN		UINT8_C(6)
#define BMP3_GEN_SETT_LEN		UINT8_C(7)
#define BMP3_P_DATA_LEN		UINT8_C(3)
#define BMP3_T_DATA_LEN		UINT8_C(3)
#define BMP3_SENSOR_TIME_LEN		UINT8_C(3)
#define BMP3_FIFO_MAX_FRAMES		UINT8_C(73)

// Add MS5849 specific commands and constants
#define MS5849_CMD_RESET            0x10
#define MS5849_CMD_READ_MEMORY      0xE0
#define MS5849_CMD_CONVERT_TEMP     0x40 // OSR 1024
#define MS5849_CMD_CONVERT_PRESS    0x44 // OSR 1024
#define MS5849_CMD_READ_ADC         0x00
#define MS5849_SERIAL_MSB_ADDR 		0x02
#define MS5849_SERIAL_LSB_ADDR 		0x03
#define MS5849_REG_COEF_C1          0x04
#define MS5849_REG_COEF_C2          0x05
#define MS5849_REG_COEF_C3          0x06
#define MS5849_REG_COEF_C4          0x07
#define MS5849_REG_COEF_C5          0x08
#define MS5849_REG_COEF_C6          0x09
#define MS5849_REG_COEF_C7          0x0A
#define MS5849_REG_COEF_C8          0x0B
#define MS5849_REG_COEF_C9          0x0C
#define MS5849_REG_COEF_C10         0x0D
#define MS5849_REG_PROM_ID_CRC      0x0F

// MS5849 Math Constants
#define MS5849_DT_C5_MULT           256.0f
#define MS5849_TEMP_OFFSET          2000.0f
#define MS5849_TEMP_C6_DIV          2048.0f
#define MS5849_OFF_C2_MULT          65536.0f
#define MS5849_OFF_C4_DIV           64.0f
#define MS5849_SENS_C1_MULT         32768.0f
#define MS5849_SENS_C3_DIV          128.0f
#define MS5849_P_SENS_DIV           2097152.0f
#define MS5849_P_OFF_DIV            32768.0f

#define MS5849_SNSR_CFG_SEL_PRESS           0x00
#define MS5849_SNSR_CFG_SEL_TEMP            0x01
#define MS5849_SNSR_CFG_SEL_BIT_MASK        0x01
#define MS5849_SNSR_CFG_RATIO_OFF           0x00
#define MS5849_SNSR_CFG_RATIO_1             0x01
#define MS5849_SNSR_CFG_RATIO_2             0x02
#define MS5849_SNSR_CFG_RATIO_4             0x03
#define MS5849_SNSR_CFG_RATIO_8             0x04
#define MS5849_SNSR_CFG_RATIO_16            0x05
#define MS5849_SNSR_CFG_RATIO_32            0x06
#define MS5849_SNSR_CFG_RATIO_BIT_MASK      0x07
#define MS5849_SNSR_CFG_FILTER_OFF          0x00
#define MS5849_SNSR_CFG_FILTER_2            0x01
#define MS5849_SNSR_CFG_FILTER_4            0x02
#define MS5849_SNSR_CFG_FILTER_8            0x03
#define MS5849_SNSR_CFG_FILTER_16           0x04
#define MS5849_SNSR_CFG_FILTER_32           0x05
#define MS5849_SNSR_CFG_FILTER_BIT_MASK     0x07
#define MS5849_SNSR_CFG_RES_24_BIT          0x00
#define MS5849_SNSR_CFG_RES_16_BIT          0x01
#define MS5849_SNSR_CFG_RES_8_BIT           0x02
#define MS5849_SNSR_CFG_RES_BIT_MASK        0x03
#define MS5849_SNSR_CFG_F_0              	0x00
#define MS5849_SNSR_CFG_OSR_1               0x01
#define MS5849_SNSR_CFG_OSR_2               0x02
#define MS5849_SNSR_CFG_OSR_3               0x03
#define MS5849_SNSR_CFG_OSR_4               0x04
#define MS5849_SNSR_CFG_OSR_5               0x05
#define MS5849_SNSR_CFG_OSR_6               0x06
#define MS5849_SNSR_CFG_OSR_BIT_MASK        0x07


#define MS5849_CMD_WRITE_CONFIG_PRESS       0x20
#define MS5849_CMD_WRITE_CONFIG_TEMP        0x22
#define MS5849_CMD_READ_CONFIG_PRESS        0x28
#define MS5849_CMD_READ_CONFIG_TEMP         0x2A
#define MS5849_CMD_START_CONVERSION         0x40
#define MS5849_CMD_START_CONVERSION_PRESS   0x44
#define MS5849_CMD_START_CONVERSION_TEMP    0x48
#define MS5849_CMD_READ_ADC_REG             0x50
#define MS5849_CMD_READ_ADC_REG_PRESS       0x54
#define MS5849_CMD_READ_ADC_REG_TEMP        0x58
#define MS5849_CMD_WRITE_OPERATION_REG      0x14
#define MS5849_CMD_READ_OPERATION_REG       0x16

#define MS5849_OP_REG_FIFO_INT_TH_OFF       0x00
#define MS5849_OP_REG_FIFO_INT_TH_BIT_MASK  0x1F
#define MS5849_OP_REG_FIFO_MODE_OFF         0x00
#define MS5849_OP_REG_FIFO_MODE_BIT_MASK    0x03
#define MS5849_OP_REG_DELAY_BIT_MASK        0x0F

/**
 * @brief Pressure 23 30BA calculation coefficients.
 * @details Specified calculation coefficients of Pressure 23 30BA Click driver.
 */
#define MS5849_COEF_TEMP_D2_DIV             536870912.0f
#define MS5849_COEF_TEMP_D1_DIV             34359738368.0f
#define MS5849_COEF_TEMP_C2_DIV             64.0f
#define MS5849_COEF_OFF_SENS_DIV            512.0f
#define MS5849_COEF_PRESS_DIV               4194304.0f

/**
 * @brief Pressure 23 30BA description of conversion selection and ADC selection data.
 * @details Specified conversion selection and ADC selection data of Pressure 23 30BA Click driver.
 */
#define MS5849_CNV_ADC_SEL_PRESS            0x00
#define MS5849_CNV_ADC_SEL_TEMP             0x01
#define MS5849_CNV_ADC_BIT_MASK             0x01
#define MS5849_CNV_ADC_REG_BIT_MASK         0x04



/********************************************************/

/*!
 * @brief Interface selection Enums
 */
enum ms5849_intf {
	/*! SPI interface */
	MS5849_SPI_INTF,
	/*! I2C interface */
	MS5849_I2C_INTF
};

/********************************************************/
/*!
 * @brief Type definitions
 */
typedef int8_t (*ms5849_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr,
		uint8_t *data, uint16_t len);

typedef void (*ms5849_delay_fptr_t)(uint32_t period);

/********************************************************/
/*!
 * @brief Register Trim Variables
 */
struct ms5849_calib_data {
 /**
 * @ Trim Variables
 */
/**@{*/
    uint32_t   prom_serial_num;      /**< Serial number. */
    uint8_t    prom_product_id;      /**< Product ID. */
    uint8_t    prom_crc;             /**< 8-bit CRC. */

    uint16_t   prom_c1;              /**< First order of temperature sensitivity to D2. */
    uint16_t   prom_c2;              /**< Zero order of temperature sensitivity. */
    uint16_t   prom_c3;              /**< First order of temperature sensitivity to D1. */
    uint16_t   prom_c4;              /**< Second order of temperature sensitivity at low temperature. */
    uint16_t   prom_c5;              /**< Second order of temperature sensitivity at high temperature. */
    uint16_t   prom_c6;              /**< Zero order of pressure sensitivity to D1 and zero order to temp. */
    uint16_t   prom_c7;              /**< Zero order of pressure sensitivity to D1 and first order to temp. */
    uint16_t   prom_c8;              /**< First order of pressure sensitivity to D1 and zero order to temp. */
    uint16_t   prom_c9;              /**< First order of pressure sensitivity to D1 and first order to temp. */
    uint16_t   prom_c10;             /**< Second order of pressure sensitivity. */
//    uint16_t   prom_c[11];

/**@}*/
};

/*!
 * @brief bmp3 device settings
 */
struct ms5849_settings {
    uint8_t    sel;           /**< Configuration selection. */
    uint8_t    ratio;         /**< Ratio configuration. */
    uint8_t    filter;        /**< Filter configuration. */
    uint8_t    resolution;    /**< Resolution configuration. */
    uint8_t    osr;           /**< OSR configuration. */
};


/*!
 * @brief bmp3 sensor structure which comprises of temperature and pressure
 * data.
 */
struct ms5849_data {
	/*! Compensated temperature */
	double temperature;
	/*! Compensated pressure */
	double pressure;
};

/*!
 * @brief bmp3 device structure
 */
struct ms5849_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! SPI/I2C interface */
	enum ms5849_intf intf;
	/*! Decide SPI or I2C read mechanism */
	// uint8_t dummy_byte;
	/*! Read function pointer */
	ms5849_com_fptr_t read;
	/*! Write function pointer */
	ms5849_com_fptr_t write;
	/*! Delay function pointer */
	ms5849_delay_fptr_t delay_ms;
	ms5849_delay_fptr_t delay_us;
	/*! Trim data */
	struct ms5849_calib_data calib_data;
	/*! Sensor Settings */
	struct ms5849_settings settings;

    uint8_t    osr_press;            
	/*! Pressure conversion time */
    uint8_t    osr_temp;             
	/*! Temperature conversion time */
};

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMP3_DEFS_H_ */
/** @}*/
/** @}*/


