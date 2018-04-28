/**\mainpage
 * Copyright (C) 2016 - 2017 Bosch Sensortec GmbH
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
 * File		BME280.c
 * Date		14 Feb 2018
 * Version	3.3.4
 *
 */

/*! @file BME280.c
    @brief Sensor driver for BME280 sensor */

#include <Wire.h>
#include "BoschBME280.h"

/********************************************************/
/*! @name		Common macros		        */
/********************************************************/

#include <stdint.h>
#include <stddef.h>

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)

#define SET_BITS(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))
#define SET_BITS_POS_0(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

#define GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
							(bitname##_POS))
#define GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))


/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 *
 *  @param[in] dev_id : Device Id (i.e. SS pin/i2c Address)
 *  @param[in] intf : SPI/I2C interface
 *  @param[in] settings : Sensor settings
 */
int8_t BoschBME280::begin(uint8_t _dev_id, IntfType _intf, const Settings& _settings)
{
	int8_t rslt;
	/* chip id read try count */
	uint8_t try_count = 5;
	uint8_t chip_id = 0;

	dev_id =_dev_id;
	intf = _intf;
	sens_settings = _settings;

	Wire.begin ();

    while (try_count) {
        /* Read the chip-id of bme280 sensor */
        rslt = get_regs(CHIP_ID_ADDR, &chip_id, 1);
        /* Check for chip id validity */
        if ((rslt == BME280_OK) && (chip_id == CHIP_ID)) {
//				dev->chip_id = chip_id;
            /* Reset the sensor */
            rslt = soft_reset();
            if (rslt == BME280_OK) {
                /* Read the calibration data */
                rslt = get_calib_data();
            }
            break;
        }
        /* Wait for 1 ms */
        delay(1);
        --try_count;
    }
    /* Chip id check failed */
    if (!try_count)
        rslt = E_DEV_NOT_FOUND;

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t BoschBME280::get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt;

    /* If interface selected is SPI */
    if (intf != INTF_I2C)
        reg_addr = reg_addr | 0x80;
    /* Read the data  */
    rslt = bus_read(dev_id, reg_addr, reg_data, len);
    /* Check for communication error */
    if (rslt != BME280_OK)
        rslt = E_COMM_FAIL;

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t BoschBME280::set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len)
{
	int8_t rslt;
	uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */

	if (len > 10)
		len = 10;

	uint16_t temp_len;
	uint8_t reg_addr_cnt;

	/* Check for arguments validity */
	if ((reg_addr != NULL) && (reg_data != NULL)) {
		if (len != 0) {
			temp_buff[0] = reg_data[0];
			/* If interface selected is SPI */
			if (intf != INTF_I2C) {
				for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
					reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
			}
			/* Burst write mode */
			if (len > 1) {
				/* Interleave register address w.r.t data for
				burst write*/
				interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
				temp_len = ((len * 2) - 1);
			} else {
				temp_len = len;
			}
			rslt = bus_write(dev_id, reg_addr[0], temp_buff, temp_len);
			/* Check for communication error */
			if (rslt != BME280_OK)
				rslt = E_COMM_FAIL;
		} else {
			rslt = E_INVALID_LEN;
		}
	} else {
		rslt = E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
int8_t BoschBME280::get_calib_data()
{
	int8_t rslt;
	uint8_t reg_addr = TEMP_PRESS_CALIB_DATA_ADDR;
	/* Array to store calibration data */
	uint8_t cdata[TEMP_PRESS_CALIB_DATA_LEN] = {0};

	/* Read the calibration data from the sensor */
	rslt = get_regs(reg_addr, cdata, TEMP_PRESS_CALIB_DATA_LEN);

	if (rslt == BME280_OK) {
		/* Parse temperature and pressure calibration data and store
		   it in device structure */
		parse_temp_press_calib_data(cdata);

		reg_addr = HUMIDITY_CALIB_DATA_ADDR;
		/* Read the humidity calibration data from the sensor */
		rslt = get_regs(reg_addr, cdata, HUMIDITY_CALIB_DATA_LEN);
		if (rslt == BME280_OK) {
			/* Parse humidity calibration data and store it in
			   device structure */
			parse_humidity_calib_data(cdata);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 */
void BoschBME280::interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
	uint8_t index;

	for (index = 1; index < len; index++) {
		temp_buff[(index * 2) - 1] = reg_addr[index];
		temp_buff[index * 2] = reg_data[index];
	}
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t BoschBME280::soft_reset()
{
	int8_t rslt;
	uint8_t reg_addr = RESET_ADDR;
	/* 0xB6 is the soft reset command */
	uint8_t soft_rst_cmd = 0xB6;

    /* Write the soft reset command in the sensor */
    rslt = set_regs(&reg_addr, &soft_rst_cmd, 1);
    /* As per data sheet, startup time is 2 ms. */
    delay(2);

	return rslt;
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
void BoschBME280::parse_temp_press_calib_data(const uint8_t *reg_data)
{
	calib_data.dig_T1 = CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data.dig_T2 = (int16_t)CONCAT_BYTES(reg_data[3], reg_data[2]);
	calib_data.dig_T3 = (int16_t)CONCAT_BYTES(reg_data[5], reg_data[4]);
	calib_data.dig_P1 = CONCAT_BYTES(reg_data[7], reg_data[6]);
	calib_data.dig_P2 = (int16_t)CONCAT_BYTES(reg_data[9], reg_data[8]);
	calib_data.dig_P3 = (int16_t)CONCAT_BYTES(reg_data[11], reg_data[10]);
	calib_data.dig_P4 = (int16_t)CONCAT_BYTES(reg_data[13], reg_data[12]);
	calib_data.dig_P5 = (int16_t)CONCAT_BYTES(reg_data[15], reg_data[14]);
	calib_data.dig_P6 = (int16_t)CONCAT_BYTES(reg_data[17], reg_data[16]);
	calib_data.dig_P7 = (int16_t)CONCAT_BYTES(reg_data[19], reg_data[18]);
	calib_data.dig_P8 = (int16_t)CONCAT_BYTES(reg_data[21], reg_data[20]);
	calib_data.dig_P9 = (int16_t)CONCAT_BYTES(reg_data[23], reg_data[22]);
	calib_data.dig_H1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
void BoschBME280::parse_humidity_calib_data(const uint8_t *reg_data)
{
	int16_t dig_H4_lsb;
	int16_t dig_H4_msb;
	int16_t dig_H5_lsb;
	int16_t dig_H5_msb;

	calib_data.dig_H2 = (int16_t)CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data.dig_H3 = reg_data[2];

	dig_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
	calib_data.dig_H4 = dig_H4_msb | dig_H4_lsb;

	dig_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
	calib_data.dig_H5 = dig_H5_msb | dig_H5_lsb;
	calib_data.dig_H6 = (int8_t)reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
bool BoschBME280::are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
{
	bool settings_changed = false;

	if (sub_settings & desired_settings) {
		/* User wants to modify this particular settings */
		settings_changed = true;
	} else {
		/* User don't want to modify this particular settings */
		settings_changed = false;
	}

	return settings_changed;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
int8_t BoschBME280::set_sensor_settings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t sensor_mode;

    rslt = get_sensor_mode(sensor_mode);
    if ((rslt == BME280_OK) && (sensor_mode != MODE_SLEEP))
        rslt = put_device_to_sleep();
    if (rslt == BME280_OK) {
        /* Check if user wants to change oversampling
           settings */
        if (are_settings_changed(SETTINGS_OVERSAMPLING, desired_settings))
            rslt = set_osr_settings(desired_settings, sens_settings);
        /* Check if user wants to change filter and/or
           standby settings */
        if ((rslt == BME280_OK) && are_settings_changed(SETTINGS_FILTER_STANDBY, desired_settings))
            rslt = set_filter_standby_settings(desired_settings, sens_settings);
    }

	return rslt;
}

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 */
int8_t BoschBME280::get_sensor_settings()
{
	int8_t rslt;
	uint8_t reg_data[4];

    rslt = get_regs(CTRL_HUM_ADDR, reg_data, 4);
    if (rslt == BME280_OK)
        parse_device_settings(reg_data, sens_settings);

	return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t BoschBME280::set_sensor_mode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t last_set_mode;

    rslt = get_sensor_mode(last_set_mode);
    /* If the sensor is not in sleep mode put the device to sleep
       mode */
    if ((rslt == BME280_OK) && (last_set_mode != MODE_SLEEP))
        rslt = put_device_to_sleep();
    /* Set the power mode */
    if (rslt == BME280_OK)
        rslt = write_power_mode(sensor_mode);

	return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
int8_t BoschBME280::get_sensor_mode(uint8_t& sensor_mode)
{
	int8_t rslt;

    /* Read the power mode register */
    rslt = get_regs(PWR_CTRL_ADDR, &sensor_mode, 1);
    /* Assign the power mode in the device structure */
    sensor_mode = GET_BITS_POS_0(sensor_mode, SENSOR_MODE);

	return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
int8_t BoschBME280::put_device_to_sleep()
{
	int8_t rslt;
	uint8_t reg_data[4];
	struct Settings settings;

	rslt = get_regs(CTRL_HUM_ADDR, reg_data, 4);
	if (rslt == BME280_OK) {
		parse_device_settings(reg_data, settings);
		rslt = soft_reset();
		if (rslt == BME280_OK)
			rslt = reload_device_settings(settings);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
int8_t BoschBME280::set_osr_settings(uint8_t desired_settings, const Settings& settings)
{
	int8_t rslt = W_INVALID_OSR_MACRO;

	if (desired_settings & SEL_OSR_HUM)
		rslt = set_osr_humidity_settings(settings);
	if (desired_settings & (SEL_OSR_PRESS | SEL_OSR_TEMP))
		rslt = set_osr_press_temp_settings(desired_settings, settings);

	return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
int8_t BoschBME280::set_osr_humidity_settings(const Settings& settings)
{
	int8_t rslt;
	uint8_t ctrl_hum;
	uint8_t ctrl_meas;
	uint8_t reg_addr = CTRL_HUM_ADDR;

	ctrl_hum = settings.osr_h & CTRL_HUM_MSK;
	/* Write the humidity control value in the register */
	rslt = set_regs(&reg_addr, &ctrl_hum, 1);
	/* Humidity related changes will be only effective after a
	   write operation to ctrl_meas register */
	if (rslt == BME280_OK) {
		reg_addr = CTRL_MEAS_ADDR;
		rslt = get_regs(reg_addr, &ctrl_meas, 1);
		if (rslt == BME280_OK)
			rslt = set_regs(&reg_addr, &ctrl_meas, 1);
	}

	return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 */
int8_t BoschBME280::set_osr_press_temp_settings(uint8_t desired_settings, const Settings& settings)
{
	int8_t rslt;
	uint8_t reg_addr = CTRL_MEAS_ADDR;
	uint8_t reg_data;

	rslt = get_regs(reg_addr, &reg_data, 1);

	if (rslt == BME280_OK) {
		if (desired_settings & SEL_OSR_PRESS)
			fill_osr_press_settings(&reg_data, settings);
		if (desired_settings & SEL_OSR_TEMP)
			fill_osr_temp_settings(&reg_data, settings);
		/* Write the oversampling settings in the register */
		rslt = set_regs(&reg_addr, &reg_data, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
int8_t BoschBME280::set_filter_standby_settings(uint8_t desired_settings, const Settings& settings)
{
	int8_t rslt;
	uint8_t reg_addr = CONFIG_ADDR;
	uint8_t reg_data;

	rslt = get_regs(reg_addr, &reg_data, 1);

	if (rslt == BME280_OK) {
		if (desired_settings & SEL_FILTER)
			fill_filter_settings(&reg_data, settings);
		if (desired_settings & SEL_STANDBY)
			fill_standby_settings(&reg_data, settings);
		/* Write the oversampling settings in the register */
		rslt = set_regs(&reg_addr, &reg_data, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
void BoschBME280::fill_filter_settings(uint8_t *reg_data, const Settings& settings)
{
	*reg_data = SET_BITS(*reg_data, FILTER, settings.filter);
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void BoschBME280::fill_standby_settings(uint8_t *reg_data, const Settings& settings)
{
	*reg_data = SET_BITS(*reg_data, STANDBY, settings.standby_time);
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void BoschBME280::fill_osr_press_settings(uint8_t *reg_data, const Settings& settings)
{
	*reg_data = SET_BITS(*reg_data, CTRL_PRESS, settings.osr_p);
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
void BoschBME280::fill_osr_temp_settings(uint8_t *reg_data, const Settings& settings)
{
	*reg_data = SET_BITS(*reg_data, CTRL_TEMP, settings.osr_t);
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 */
void BoschBME280::parse_device_settings(const uint8_t *reg_data, Settings& settings)
{
	settings.osr_h = GET_BITS_POS_0(reg_data[0], CTRL_HUM);
	settings.osr_p = GET_BITS(reg_data[2], CTRL_PRESS);
	settings.osr_t = GET_BITS(reg_data[2], CTRL_TEMP);
	settings.filter = GET_BITS(reg_data[3], FILTER);
	settings.standby_time = GET_BITS(reg_data[3], STANDBY);
}
/*!
 * @brief This internal API writes the power mode in the sensor.
 */
int8_t BoschBME280::write_power_mode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t reg_addr = PWR_CTRL_ADDR;
	/* Variable to store the value read from power mode register */
	uint8_t sensor_mode_reg_val;

	/* Read the power mode register */
	rslt = get_regs(reg_addr, &sensor_mode_reg_val, 1);
	/* Set the power mode */
	if (rslt == BME280_OK) {
		sensor_mode_reg_val = SET_BITS_POS_0(sensor_mode_reg_val, SENSOR_MODE, sensor_mode);
		/* Write the power mode in the register */
		rslt = set_regs(&reg_addr, &sensor_mode_reg_val, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API reloads the already existing device settings in
 * the sensor after soft reset.
 */
int8_t BoschBME280::reload_device_settings(const Settings& settings)
{
	int8_t rslt;

	rslt = set_osr_settings(SEL_ALL_SETTINGS, settings);
	if (rslt == BME280_OK)
		rslt = set_filter_standby_settings(SEL_ALL_SETTINGS, settings);

	return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t BoschBME280::get_sensor_data(uint8_t sensor_comp, data& comp_data)
{
	int8_t rslt;
	/* Array to store the pressure, temperature and humidity data read from
	the sensor */
	uint8_t reg_data[P_T_H_DATA_LEN] = {0};
	struct uncomp_data uncomp_data = {0, 0, 0};

    /* Read the pressure and temperature data from the sensor */
    rslt = get_regs(DATA_ADDR, reg_data, P_T_H_DATA_LEN);

    if (rslt == BME280_OK) {
        /* Parse the read data from the sensor */
        parse_sensor_data(reg_data, uncomp_data);
        /* Compensate the pressure and/or temperature and/or
           humidity data from the sensor */
			rslt = compensate_data(sensor_comp, uncomp_data, comp_data);
    }

	return rslt;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void BoschBME280::parse_sensor_data(const uint8_t *reg_data, uncomp_data& uncomp_data)
{
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)reg_data[0] << 12;
	data_lsb = (uint32_t)reg_data[1] << 4;
	data_xlsb = (uint32_t)reg_data[2] >> 4;
	uncomp_data.pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)reg_data[3] << 12;
	data_lsb = (uint32_t)reg_data[4] << 4;
	data_xlsb = (uint32_t)reg_data[5] >> 4;
	uncomp_data.temperature = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_lsb = (uint32_t)reg_data[6] << 8;
	data_msb = (uint32_t)reg_data[7];
	uncomp_data.humidity = data_msb | data_lsb;
}

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
int8_t BoschBME280::compensate_data(uint8_t sensor_comp, const uncomp_data& uncomp_data,
				     data& comp_data)
{
	int8_t rslt = BME280_OK;

    /* Initialize to zero */
    comp_data.temperature = 0;
    comp_data.pressure = 0;
    comp_data.humidity = 0;
    /* If pressure or temperature component is selected */
    if (sensor_comp & (DATA_PRESS | DATA_TEMP | DATA_HUM)) {
        /* Compensate the temperature data */
        comp_data.temperature = compensate_temperature(uncomp_data);
    }
    if (sensor_comp & DATA_PRESS) {
        /* Compensate the pressure data */
        comp_data.pressure = compensate_pressure(uncomp_data);
    }
    if (sensor_comp & DATA_HUM) {
        /* Compensate the humidity data */
        comp_data.humidity = compensate_humidity(uncomp_data);
    }

	return rslt;
}


#ifdef FLOAT_ENABLE
/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
double BoschBME280::compensate_temperature(const uncomp_data& uncomp_data)
{
	double var1;
	double var2;
	double temperature;
	double temperature_min = -40;
	double temperature_max = 85;

	var1 = ((double)uncomp_data.temperature) / 16384.0 - ((double)calib_data.dig_T1) / 1024.0;
	var1 = var1 * ((double)calib_data.dig_T2);
	var2 = (((double)uncomp_data.temperature) / 131072.0 - ((double)calib_data.dig_T1) / 8192.0);
	var2 = (var2 * var2) * ((double)calib_data.dig_T3);
	calib_data.t_fine = (int32_t)(var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
double BoschBME280::compensate_pressure(const struct bme280_uncomp_data& uncomp_data)
{
	double var1;
	double var2;
	double var3;
	double pressure;
	double pressure_min = 30000.0;
	double pressure_max = 110000.0;

	var1 = ((double)calib_data.t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib_data.dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)calib_data.dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib_data.dig_P4) * 65536.0);
	var3 = ((double)calib_data.dig_P3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)calib_data.dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)calib_data.dig_P1);
	/* avoid exception caused by division by zero */
	if (var1) {
		pressure = 1048576.0 - (double) uncomp_data.pressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)calib_data.dig_P9) * pressure * pressure / 2147483648.0;
		var2 = pressure * ((double)calib_data.dig_P8) / 32768.0;
		pressure = pressure + (var1 + var2 + ((double)calib_data.dig_P7)) / 16.0;

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else { /* Invalid case */
		pressure = pressure_min;
	}

	return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
double BoschBME280::compensate_humidity(const uncomp_data& uncomp_data)
{
	double humidity;
	double humidity_min = 0.0;
	double humidity_max = 100.0;
	double var1;
	double var2;
	double var3;
	double var4;
	double var5;
	double var6;

	var1 = ((double)calib_data.t_fine) - 76800.0;
	var2 = (((double)calib_data.dig_H4) * 64.0 + (((double)calib_data.dig_H5) / 16384.0) * var1);
	var3 = uncomp_data.humidity - var2;
	var4 = ((double)calib_data.dig_H2) / 65536.0;
	var5 = (1.0 + (((double)calib_data.dig_H3) / 67108864.0) * var1);
	var6 = 1.0 + (((double)calib_data.dig_H6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6 * (1.0 - ((double)calib_data.dig_H1) * var6 / 524288.0);

	if (humidity > humidity_max)
		humidity = humidity_max;
	else if (humidity < humidity_min)
		humidity = humidity_min;

	return humidity;
}

#else
/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
int32_t BoschBME280::compensate_temperature(const uncomp_data& uncomp_data)
{
	int32_t var1;
	int32_t var2;
	int32_t temperature;
	int32_t temperature_min = -4000;
	int32_t temperature_max = 8500;

	var1 = (int32_t)((uncomp_data.temperature / 8) - ((int32_t)calib_data.dig_T1 * 2));
	var1 = (var1 * ((int32_t)calib_data.dig_T2)) / 2048;
	var2 = (int32_t)((uncomp_data.temperature / 16) - ((int32_t)calib_data.dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data.dig_T3)) / 16384;
	calib_data.t_fine = var1 + var2;
	temperature = (calib_data.t_fine * 5 + 128) / 256;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}

#ifdef BME280_64BIT_ENABLE
/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type with higher
 * accuracy.
 */
uint32_t BoschBME280::compensate_pressure(const uncomp_data& uncomp_data)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int64_t var4;
	uint32_t pressure;
	uint32_t pressure_min = 3000000;
	uint32_t pressure_max = 11000000;

	var1 = ((int64_t)calib_data.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) * 131072);
	var2 = var2 + (((int64_t)calib_data.dig_P4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) / 256) + ((var1 * ((int64_t)calib_data.dig_P2) * 4096));
	var3 = ((int64_t)1) * 140737488355328;
	var1 = (var3 + var1) * ((int64_t)calib_data.dig_P1) / 8589934592;

	/* To avoid divide by zero exception */
	if (var1 != 0) {
		var4 = 1048576 - uncomp_data.pressure;
		var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
		var1 = (((int64_t)calib_data.dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
		var2 = (((int64_t)calib_data.dig_P8) * var4) / 524288;
		var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calib_data.dig_P7) * 16);
		pressure = (uint32_t)(((var4 / 2) * 100) / 128);

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#else
/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
uint32_t BoschBME280::compensate_pressure(const uncomp_data& uncomp_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = (((int32_t)calib_data.t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data.dig_P6);
	var2 = var2 + ((var1 * ((int32_t)calib_data.dig_P5)) * 2);
	var2 = (var2 / 4) + (((int32_t)calib_data.dig_P4) * 65536);
	var3 = (calib_data.dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)calib_data.dig_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)calib_data.dig_P1)) / 32768;
	 /* avoid exception caused by division by zero */
	if (var1) {
		var5 = (uint32_t)((uint32_t)1048576) - uncomp_data.pressure;
		pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
		if (pressure < 0x80000000)
			pressure = (pressure << 1) / ((uint32_t)var1);
		else
			pressure = (pressure / (uint32_t)var1) * 2;

		var1 = (((int32_t)calib_data.dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data.dig_P8)) / 8192;
		pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data.dig_P7) / 16));

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#endif

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
uint32_t BoschBME280::compensate_humidity(const uncomp_data& uncomp_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;
	uint32_t humidity_max = 102400;

	var1 = calib_data.t_fine - ((int32_t)76800);
	var2 = (int32_t)(uncomp_data.humidity * 16384);
	var3 = (int32_t)(((int32_t)calib_data.dig_H4) * 1048576);
	var4 = ((int32_t)calib_data.dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)calib_data.dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)calib_data.dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)calib_data.dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)calib_data.dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (uint32_t)(var5 / 4096);

	if (humidity > humidity_max)
		humidity = humidity_max;

	return humidity;
}
#endif


int8_t BoschBME280::bus_read (uint8_t dev_id, uint8_t reg_addr,
                                uint8_t *reg_data,
                                uint16_t len) {

	/*
	 * Data on the bus should be like
	 * |------------+---------------------|
	 * | I2C action | Data                |
	 * |------------+---------------------|
	 * | Start      | -                   |
	 * | Write      | (reg_addr)          |
	 * | Stop       | -                   |
	 * | Start      | -                   |
	 * | Read       | (reg_data[0])       |
	 * | Read       | (....)              |
	 * | Read       | (reg_data[len - 1]) |
	 * | Stop       | -                   |
	 * |------------+---------------------|
	 */

	/* Return 0 for Success, non-zero for failure */
	uint16_t recv = 0;

	// Send address
	Wire.beginTransmission (dev_id);
	Wire.write (reg_addr);

	if (Wire.endTransmission() == 0) {
		Wire.requestFrom (dev_id, (byte) len);

		unsigned long start = millis();

		while (recv < len && millis() - start < TIMEOUT) {
			if (Wire.available() > 0) {
				reg_data[recv++] = Wire.read();
			}
		}
	}

	return len - recv;
}

int8_t BoschBME280::bus_write (uint8_t dev_id, uint8_t reg_addr,
                                 uint8_t *reg_data,
                                 uint16_t len) {

	/*
	 * Data on the bus should be like
	 * |------------+---------------------|
	 * | I2C action | Data                |
	 * |------------+---------------------|
	 * | Start      | -                   |
	 * | Write      | (reg_addr)          |
	 * | Write      | (reg_data[0])       |
	 * | Write      | (....)              |
	 * | Write      | (reg_data[len - 1]) |
	 * | Stop       | -                   |
	 * |------------+---------------------|
	 */

	int8_t rslt = 1; /* Return 0 for Success, non-zero for failure */

	Wire.beginTransmission (dev_id);
	Wire.write (reg_addr);

	for (uint16_t i = 0; i < len; ++i) {
		Wire.write (reg_data[i]);
	}

	if (Wire.endTransmission() == 0) {
		rslt = 0;
	}

	return rslt;
}
