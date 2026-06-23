/**\mainpage
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
 * File		ms5849.c (edited from bmp3.c for the ms5849 pressure sensor)
 * Date		05 Apr 2018
 * Version	1.1.0
 *
 */

/*! @file ms5849.c
    @brief Sensor driver for MS5849 sensor */
#include "ms5849.h"


/***************** Internal macros ******************************/

/*! Power control settings */
#define POWER_CNTL      UINT16_C(0x0006)
/*! Odr and filter settings */
#define ODR_FILTER      UINT16_C(0x00F0)
/*! Interrupt control settings */
#define INT_CTRL	UINT16_C(0x0708)
/*! Advance settings */
#define ADV_SETT	UINT16_C(0x1800)


#define DUMMY             0x00

/***************** Static function declarations ******************************/
/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it then compensates it and store in the device structure.
 *
 * @param[in] dev : Structure instance of ms5849_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_calib_data(struct ms5849_dev *dev);

/*!
 * @brief This internal API is used to read prom data, turning the uint8_t reg_data
 *  to a uint16_t prom
 *
 * @param[in] dev : Structure instance of ms5849_dev.
 * @param[out] dest : Contains prom word data to be parsed.
 *
 */
static int8_t read_prom_word(uint8_t addr, uint16_t *dest, struct ms5849_dev *dev);

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of ms5849_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct ms5849_dev *dev);

int8_t ms5849_get_adc_data_press ( struct ms5849_dev *dev, uint32_t *pressure );
int8_t ms5849_get_adc_data_temp ( struct ms5849_dev *dev, uint32_t *temperature );
int8_t ms5849_start_conversion ( struct ms5849_dev *dev, uint8_t sel_cnv );
int8_t ms5849_read_adc ( struct ms5849_dev *dev, uint8_t sel_data, uint32_t *adc_data );
int8_t ms5849_write_op_reg ( struct ms5849_dev *dev, uint8_t fifo_int_th, uint8_t fifo_mode, uint8_t delay );
int8_t ms5849_read_op_reg ( struct ms5849_dev *dev, uint8_t *fifo_int_th, uint8_t *fifo_mode, uint8_t *delay );

/****************** Global Function Definitions *******************************/
/*!
 *  @brief This API is the entry point.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id and calibration data of the sensor.
 */
static const uint8_t osr_delays[] = { 1, 1, 2, 3, 5, 9, 17 };


int8_t ms5849_init(struct ms5849_dev *dev)
{
	int8_t rslt = 0;
	uint8_t id_buffer[2];

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt ==  MS5849_OK) {
		// /* Read mechanism according to selected interface */
		// if (dev->intf != MS5849_I2C_INTF) {
		// 	/* If SPI interface is selected, read extra byte */
		// 	dev->dummy_byte = 0;
		// } else {
		// 	/* If I2C interface is selected, no need to read
		// 	extra byte */
		// 	dev->dummy_byte = 0;
		// }
		/* Read the chip-id of ms58493 sensor */
		rslt = ms5849_get_regs(MS5849_SERIAL_MSB_ADDR, id_buffer, 2, dev);
		/* Proceed if everything is fine until now */
		if (rslt == MS5849_OK) {
			uint16_t serial_no = ((uint16_t)id_buffer[0] << 8) | id_buffer[1];
			/* Check for chip id validity */
			if (serial_no != 0) {
				dev->chip_id = id_buffer[0];
				/* Reset the sensor */
				rslt = ms5849_soft_reset(dev);
				if (rslt == MS5849_OK) {
					/* Read the calibration data */
					dev->delay_ms(5);
//					HAL_Delay(50);
					rslt = get_calib_data(dev);
					if (rslt == MS5849_OK) {
						/* Validate the calibration coefficients.
						 *
						 * The exact PROM CRC algorithm differs across the
						 * MS58xx family, so rather than risk rejecting good
						 * parts with a wrong polynomial, sanity-check the
						 * coefficients directly: a correctly-read MS5849
						 * coefficient word is a non-zero, non-all-ones
						 * 16-bit value. An all-zero or all-ones word means
						 * the SPI read failed or the part is not
						 * responding -- and since ms5849_compensate() is
						 * quadratic in d1 and scaled by these coefficients,
						 * a bad coefficient silently corrupts every
						 * pressure sample from that channel.
						 */
						rslt = ms5849_check_calib(dev);
					}
					if (rslt == MS5849_OK) {
//						HAL_Delay(50);
						dev->delay_ms(5);
						rslt = ms5849_write_op_reg(dev,0x01,0x02,0x00);
					}
				}
			} else {
				rslt = MS5849_E_DEV_NOT_FOUND;
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t ms5849_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct ms5849_dev *dev)
{
	int8_t rslt;
	uint8_t command;
	uint16_t i;
    uint8_t data_buf[ 2 ] = { DUMMY };

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == MS5849_OK) {
		/* * MS5849 memory read protocol:
		 * Command = 0xE0 | (Address << 1)
		 */
		command = MS5849_CMD_READ_MEMORY | (reg_addr << 1);
		rslt = dev->read(dev->dev_id, command, data_buf, len);
		 for (i = 0; i < len; i++)
		 	reg_data[i] = data_buf[i];
//		reg_data = data_buf;
		/* Check for communication error */
		if (rslt != MS5849_OK) {
			rslt = MS5849_E_COMM_FAIL;
		}
//        *reg_data = data_buf[ 0 ];
//        *reg_data <<= 8;
//        *reg_data |= data_buf[ 1 ];
	}

	return rslt;

}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t ms5849_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct ms5849_dev *dev)
{
	int8_t rslt;
	uint8_t command;
	uint8_t reg_addr_cnt;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	if ((rslt == MS5849_OK) && (reg_addr != NULL) && (reg_data != NULL)) {
		if (len != 0) {
			for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++) {
				command = MS5849_CMD_READ_MEMORY | (reg_addr[reg_addr_cnt] << 1);
				rslt = dev->write(dev->dev_id, command, (uint8_t *)&reg_data[reg_addr_cnt], 1);

				if (rslt != MS5849_OK) {
					rslt = MS5849_E_COMM_FAIL;
					break;
				}
			}
		} else {
			rslt = MS5849_E_INVALID_LEN;
		}
	} else {
		rslt = MS5849_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t ms5849_soft_reset(const struct ms5849_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == MS5849_OK) {
		uint8_t cmd = MS5849_CMD_RESET;
		rslt = dev->write(dev->dev_id, cmd,NULL,0);

		/* Check for communication error */
		if (rslt != MS5849_OK) {
			rslt = MS5849_E_COMM_FAIL;
		}
	}

	return rslt;

}

/*!
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the ms5849_data structure
 * instance passed by the user.
 */
// In ms5849.c

// Kick off a pressure conversion, return immediately. NO DELAY.
int8_t ms5849_start_press_conversion(struct ms5849_dev *dev) {
    return ms5849_start_conversion(dev, MS5849_CNV_ADC_SEL_PRESS);
}

// Kick off a temperature conversion, return immediately. NO DELAY.
int8_t ms5849_start_temp_conversion(struct ms5849_dev *dev) {
    return ms5849_start_conversion(dev, MS5849_CNV_ADC_SEL_TEMP);
}

// Read the ADC result for whichever conversion was last started.
// Caller is responsible for ensuring enough time has elapsed since start.
int8_t ms5849_read_press_raw(struct ms5849_dev *dev, uint32_t *pressure) {
    return ms5849_read_adc(dev, MS5849_CNV_ADC_SEL_PRESS, pressure);
}

int8_t ms5849_read_temp_raw(struct ms5849_dev *dev, uint32_t *temperature) {
    return ms5849_read_adc(dev, MS5849_CNV_ADC_SEL_TEMP, temperature);
}

// Compensate using a cached d2 (temperature raw). Pure math, no I/O.
void ms5849_compensate(struct ms5849_dev *dev, uint32_t d1, uint32_t d2,
                       struct ms5849_data *comp_data) {
    float off, sens, temp, press;

    temp  = ((float)(dev->calib_data.prom_c1)) * ((float)d2) / MS5849_COEF_TEMP_D2_DIV;
    temp -= ((float)(dev->calib_data.prom_c3)) * ((float)d1) / MS5849_COEF_TEMP_D1_DIV;
    temp -= ((float)(dev->calib_data.prom_c2)) / MS5849_COEF_TEMP_C2_DIV;
    comp_data->temperature = temp;

    off  = (float)(dev->calib_data.prom_c6);
    off += ((float)(dev->calib_data.prom_c7)) * temp / MS5849_COEF_OFF_SENS_DIV;

    sens  = (float)(dev->calib_data.prom_c8);
    sens += ((float)(dev->calib_data.prom_c9)) * temp / MS5849_COEF_OFF_SENS_DIV;

    press  = (float)d1 * sens / MS5849_COEF_PRESS_DIV;
    press -= off;
    press *= 100.0f;
    comp_data->pressure = press;
}



int8_t ms5849_get_sensor_data(uint8_t sensor_comp, struct ms5849_data *comp_data, struct ms5849_dev *dev)
{
    uint32_t d1 = DUMMY, d2 = DUMMY;
    float off = DUMMY, sens = DUMMY, temp = DUMMY, press = DUMMY;

    int8_t err_flag = ms5849_get_adc_data_press( dev, &d1 );
//    dev->delay_ms(10);
    err_flag |= ms5849_get_adc_data_temp( dev, &d2 );

    // Compensated temperature in degree Celsius
    temp = ( ( ( float ) ( dev->calib_data.prom_c1 ) ) * ( ( float ) d2 ) ) / MS5849_COEF_TEMP_D2_DIV;
    temp -= ( ( ( float ) ( dev->calib_data.prom_c3 ) ) * ( ( float ) d1 ) ) / MS5849_COEF_TEMP_D1_DIV;
    temp -= ( ( float ) ( dev->calib_data.prom_c2 ) ) / MS5849_COEF_TEMP_C2_DIV;
    comp_data->temperature = temp;

    // Offset at actual temperature
    off = ( ( float ) ( dev->calib_data.prom_c6 ) );
    off += ( ( ( float ) ( dev->calib_data.prom_c7 ) ) * temp ) / MS5849_COEF_OFF_SENS_DIV;

     // Sensitivity at actual temperature
    sens = ( ( float ) ( dev->calib_data.prom_c8 ) );
    sens += ( ( ( float ) ( dev->calib_data.prom_c9 ) ) * temp ) / MS5849_COEF_OFF_SENS_DIV;

    // Temperature compensated pressure
    press = ( float ) d1;
    press *= ( float ) sens;
    press /= MS5849_COEF_PRESS_DIV;
    press -= off;		// pressure is in millibars
    press *= 100.0f;	// convert pressure to Pa
//    press = 123.45f; //debugging
    comp_data->pressure = press;

    return err_flag;
}

int8_t ms5849_get_adc_data_press ( struct ms5849_dev *dev, uint32_t *pressure )
{
    // Start pressure conversion
	int8_t err_flag = ms5849_start_conversion( dev, MS5849_CNV_ADC_SEL_PRESS );

    // Conversion time delay
	dev->delay_ms(osr_delays[dev->osr_press]);

    // Read pressure raw data
    err_flag |= ms5849_read_adc( dev, MS5849_CNV_ADC_SEL_PRESS, pressure );
    return err_flag;
}

int8_t ms5849_get_adc_data_temp ( struct ms5849_dev *dev, uint32_t *temperature )
{
    // Start temperature conversion
	int8_t err_flag = ms5849_start_conversion( dev, MS5849_CNV_ADC_SEL_TEMP );

    // Conversion time delay
	dev->delay_ms(osr_delays[dev->osr_temp]);

    // Read temperature raw data
    err_flag |= ms5849_read_adc( dev, MS5849_CNV_ADC_SEL_TEMP, temperature );
    return err_flag;
}

int8_t ms5849_start_conversion ( struct ms5849_dev *dev, uint8_t sel_cnv )
{
	int8_t rslt;
    sel_cnv &= MS5849_CNV_ADC_BIT_MASK;
    uint8_t command = MS5849_CMD_START_CONVERSION |  ( MS5849_CNV_ADC_REG_BIT_MASK << sel_cnv);
    rslt = dev->write(dev->dev_id, command, NULL, 0);
    return rslt;
}

int8_t ms5849_read_adc ( struct ms5849_dev *dev, uint8_t sel_data, uint32_t *adc_data )
{
    uint8_t data_buf[ 3 ] = { DUMMY };
    sel_data &= MS5849_CNV_ADC_BIT_MASK;
    int8_t err_flag = dev->read( dev->dev_id, MS5849_CMD_READ_ADC_REG |
                                              ( MS5849_CNV_ADC_REG_BIT_MASK << sel_data ), data_buf, 3 );
    *adc_data = data_buf[ 0 ];
    *adc_data <<= 8;
    *adc_data |= data_buf[ 1 ];
    *adc_data <<= 8;
    *adc_data |= data_buf[ 2 ];
    return err_flag;
}




/****************** Static Function Definitions *******************************/
/*!
 * @brief This internal API converts the no. of frames required by the user to
 * bytes so as to write in the watermark length register.
 */

int8_t ms5849_write_op_reg ( struct ms5849_dev *dev, uint8_t fifo_int_th, uint8_t fifo_mode, uint8_t delay )
{
    uint8_t data_buf[ 2 ] = { 0 };
    data_buf[ 0 ]  =   fifo_int_th & MS5849_OP_REG_FIFO_INT_TH_BIT_MASK;
    data_buf[ 1 ]  = ( fifo_mode   & MS5849_OP_REG_FIFO_MODE_BIT_MASK ) << 6;
    data_buf[ 1 ] |=   delay       & MS5849_OP_REG_DELAY_BIT_MASK;
    return dev->write( dev->dev_id, MS5849_CMD_WRITE_OPERATION_REG, data_buf, 2 );
}

int8_t ms5849_read_op_reg ( struct ms5849_dev *dev, uint8_t *fifo_int_th, uint8_t *fifo_mode, uint8_t *delay )
{
    uint8_t data_buf[ 2 ] = { 0 };
    int8_t err_flag = dev->read( dev->dev_id, MS5849_CMD_READ_OPERATION_REG, data_buf, 2 );
    *fifo_int_th =    data_buf[ 0 ] & MS5849_OP_REG_FIFO_INT_TH_BIT_MASK;
    *fifo_mode   =  ( data_buf[ 1 ] >> 6 ) & MS5849_OP_REG_FIFO_MODE_BIT_MASK;
    *delay       =    data_buf[ 1 ] & MS5849_OP_REG_DELAY_BIT_MASK;
    return err_flag;
}

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it then compensates it and store in the device structure.
 */
static int8_t get_calib_data(struct ms5849_dev *dev)
{
	int8_t rslt = 0;
    uint16_t val16;
    read_prom_word(MS5849_SERIAL_MSB_ADDR, &val16, dev );
	dev->calib_data.prom_serial_num = (uint32_t)val16 << 16;
	read_prom_word(MS5849_SERIAL_LSB_ADDR, &val16, dev );
	dev->calib_data.prom_serial_num |= val16;

    rslt |= read_prom_word( MS5849_REG_COEF_C1,  &dev->calib_data.prom_c1,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C2,  &dev->calib_data.prom_c2,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C3,  &dev->calib_data.prom_c3,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C4,  &dev->calib_data.prom_c4,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C5,  &dev->calib_data.prom_c5,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C6,  &dev->calib_data.prom_c6,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C7,  &dev->calib_data.prom_c7,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C8,  &dev->calib_data.prom_c8,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C9,  &dev->calib_data.prom_c9,  dev );
    rslt |= read_prom_word( MS5849_REG_COEF_C10, &dev->calib_data.prom_c10, dev );

    rslt |= read_prom_word( MS5849_REG_PROM_ID_CRC, &val16, dev );
    dev->calib_data.prom_product_id = ( uint8_t ) ( val16 >> 8 );
    dev->calib_data.prom_crc = ( uint8_t )  val16;
    return rslt;
}

static int8_t read_prom_word(uint8_t addr, uint16_t *dest, struct ms5849_dev *dev) {
    uint8_t temp_buf[2] = {0, 0};

    // Call your existing HAL-based read function
    int8_t rslt = ms5849_get_regs(addr, temp_buf, 2, dev);

    if (rslt == 0) {
        // Sensor is Big-Endian: [0] is MSB, [1] is LSB
        *dest = ((uint16_t)temp_buf[0] << 8) | temp_buf[1];
    }

    return rslt;
}

/*!
 * @brief Sanity-check the calibration coefficients read from PROM.
 *
 * A correctly-read MS5849 coefficient word is a non-zero, non-all-ones
 * 16-bit value. If a SPI read of the PROM fails (bad wiring, contention,
 * a part that did not power up) the word typically comes back as all
 * zeros (0x0000) or all ones (0xFFFF). Either value, fed into
 * ms5849_compensate(), silently corrupts EVERY pressure sample from that
 * channel -- the bug is invisible because compensate() still returns a
 * finite number, just a wrong one.
 *
 * This is intentionally a coefficient sanity check rather than a PROM
 * CRC: the CRC polynomial / word coverage differs across the MS58xx
 * family, and a wrong CRC implementation would reject good parts. The
 * sanity check has no false positives on good parts and reliably catches
 * the failure mode that matters.
 *
 * @return MS5849_OK if all coefficients look valid,
 *         MS5849_E_CONFIGURATION_ERR otherwise.
 */
int8_t ms5849_check_calib(const struct ms5849_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    if (rslt != MS5849_OK) {
        return rslt;
    }

    const uint16_t coeffs[10] = {
        dev->calib_data.prom_c1, dev->calib_data.prom_c2,
        dev->calib_data.prom_c3, dev->calib_data.prom_c4,
        dev->calib_data.prom_c5, dev->calib_data.prom_c6,
        dev->calib_data.prom_c7, dev->calib_data.prom_c8,
        dev->calib_data.prom_c9, dev->calib_data.prom_c10
    };

    for (int i = 0; i < 10; i++) {
        if (coeffs[i] == 0x0000 || coeffs[i] == 0xFFFF) {
            return MS5849_E_CONFIGURATION_ERR;
        }
    }

    /* The compensation math directly uses c1, c2, c3 (temperature) and
     * c6..c9 (offset / sensitivity). c1 and c8 in particular scale the
     * result; if either is implausibly small the channel will read a
     * near-constant pressure regardless of force. Guard against a word
     * that is technically non-zero but far too small to be a real
     * factory coefficient (real MS5849 coefficients are tens of
     * thousands). The threshold is deliberately loose to avoid false
     * positives; tighten it against your measured PROM dumps. */
    if (dev->calib_data.prom_c1 < 256 ||
        dev->calib_data.prom_c8 < 256) {
        return MS5849_E_CONFIGURATION_ERR;
    }

    return MS5849_OK;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct ms5849_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = MS5849_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = MS5849_OK;
	}

	return rslt;
}


int8_t ms5849_write_config ( struct ms5849_dev *dev )
{
    uint8_t data_buf[ 2 ] = { 0 };
    uint8_t cfg_sel = ( dev -> settings.sel & MS5849_SNSR_CFG_SEL_BIT_MASK ) << 1;
    if (MS5849_SNSR_CFG_SEL_PRESS == cfg_sel )
    {
        dev->osr_press = dev -> settings.osr;
    }
    else
    {
        dev->osr_temp = dev -> settings.osr;
    }
    data_buf[ 0 ]  =   dev -> settings.ratio      & MS5849_SNSR_CFG_RATIO_BIT_MASK;
    data_buf[ 1 ]  = ( dev -> settings.filter     & MS5849_SNSR_CFG_FILTER_BIT_MASK ) << 5;
    data_buf[ 1 ] |= ( dev -> settings.resolution & MS5849_SNSR_CFG_RES_BIT_MASK ) << 3;
    data_buf[ 1 ] |=   dev -> settings.osr        & MS5849_SNSR_CFG_OSR_BIT_MASK;
    return dev -> write( dev->dev_id, MS5849_CMD_WRITE_CONFIG_PRESS | cfg_sel, data_buf, 2 );
}

int8_t ms5849_read_config (  struct ms5849_dev *dev , uint8_t sel_cfg)
{
    uint8_t data_buf[ 2 ] = { 0 };
    sel_cfg &= MS5849_SNSR_CFG_SEL_BIT_MASK;
    sel_cfg <<= 1;
    int8_t err_flag = dev -> read( dev->dev_id, MS5849_CMD_READ_CONFIG_PRESS | sel_cfg, data_buf, 2 );
    dev -> settings.ratio      =   data_buf[ 0 ]        & MS5849_SNSR_CFG_RATIO_BIT_MASK;
    dev -> settings.filter     = ( data_buf[ 1 ] >> 5 ) & MS5849_SNSR_CFG_FILTER_BIT_MASK;
    dev -> settings.resolution = ( data_buf[ 1 ] >> 3 ) & MS5849_SNSR_CFG_RES_BIT_MASK;
    dev -> settings.osr        =   data_buf[ 1 ]        & MS5849_SNSR_CFG_OSR_BIT_MASK;
    return err_flag;
}

