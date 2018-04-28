/**
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
 * @file	bme280.h
 * @date	14 Feb 2018
 * @version	3.3.4
 * @brief
 *
 */

/*! @file BME280.h
    @brief Sensor driver for BME280 sensor */

#ifndef BOSCH_BME280_H
#define BOSCH_BME280_H

#include <Arduino.h>

#ifndef BME280_FLOAT_ENABLE
/* #define BME280_FLOAT_ENABLE */
#endif

#ifndef BME280_FLOAT_ENABLE
#ifndef BME280_64BIT_ENABLE
//#define BME280_64BIT_ENABLE
#endif
#endif

class BoschBME280 {
public:

    /**\name I2C addresses */
    static const uint8_t I2C_ADDR_PRIM = 0x76;
    static const uint8_t I2C_ADDR_SEC = 0x77;

    /**\name Settings selection macros */
    static const uint8_t SEL_OSR_PRESS = 1 << 0;
    static const uint8_t SEL_OSR_TEMP = 1 << 1;
    static const uint8_t SEL_OSR_HUM = 1 << 2;
    static const uint8_t SEL_FILTER = 1 << 3;
    static const uint8_t SEL_STANDBY = 1 << 4;
    static const uint8_t SEL_ALL_SETTINGS = 0x1F;

    /**\name Oversampling macros */
    static const uint8_t OVERSAMPLING_OFF = 0x00;
    static const uint8_t OVERSAMPLING_1X = 0x01;
    static const uint8_t OVERSAMPLING_2X = 0x02;
    static const uint8_t OVERSAMPLING_4X = 0x03;
    static const uint8_t OVERSAMPLING_8X = 0x04;
    static const uint8_t OVERSAMPLING_16X = 0x05;

    /**\name Standby duration selection macros */
    static const uint8_t STANDBY_TIME_1_MS = 0x00;
    static const uint8_t STANDBY_TIME_62_5_MS = 0x01;
    static const uint8_t STANDBY_TIME_125_MS = 0x02;
    static const uint8_t STANDBY_TIME_250_MS = 0x03;
    static const uint8_t STANDBY_TIME_500_MS = 0x04;
    static const uint8_t STANDBY_TIME_1000_MS = 0x05;
    static const uint8_t STANDBY_TIME_10_MS = 0x06;
    static const uint8_t STANDBY_TIME_20_MS = 0x07;

    /**\name Filter coefficient selection macros */
    static const uint8_t FILTER_COEFF_OFF = 0x00;
    static const uint8_t FILTER_COEFF_2 = 0x01;
    static const uint8_t FILTER_COEFF_4 = 0x02;
    static const uint8_t FILTER_COEFF_8 = 0x03;
    static const uint8_t FILTER_COEFF_16 = 0x04;

    /**\name API success code */
    static const uint8_t BME280_OK = 0;

    /**\name Sensor power modes */
    static const uint8_t MODE_SLEEP = 0x00;
    static const uint8_t MODE_FORCED = 0x01;
    static const uint8_t MODE_NORMAL = 0x03;

    /*!
     * @brief Interface selection Enums
     */
    enum IntfType {
        /*! SPI interface */
        INTF_SPI,
        /*! I2C interface */
        INTF_I2C
    };

    /*!
     * @brief bme280 sensor settings structure which comprises of mode,
     * oversampling and filter settings.
     */
    struct Settings {
        /*! pressure oversampling */
        uint8_t osr_p;
        /*! temperature oversampling */
        uint8_t osr_t;
        /*! humidity oversampling */
        uint8_t osr_h;
        /*! filter coefficient */
        uint8_t filter;
        /*! standby time */
        uint8_t standby_time;
    };

    /*!
     * @brief bme280 sensor structure which comprises of temperature, pressure and
     * humidity data
     */
#ifdef BME280_FLOAT_ENABLE
    struct data {
        /*! Compensated pressure */
        double pressure;
        /*! Compensated temperature */
        double temperature;
        /*! Compensated humidity */
        double humidity;
    };
#else
    struct data {
        /*! Compensated pressure */
        uint32_t pressure;
        /*! Compensated temperature */
        int32_t temperature;
        /*! Compensated humidity */
        uint32_t humidity;
    };
#endif /* BME280_FLOAT_ENABLE */

    /*!
     * @brief bme280 sensor structure which comprises of uncompensated temperature,
     * pressure and humidity data
     */
    struct uncomp_data {
        /*! un-compensated pressure */
        uint32_t pressure;
        /*! un-compensated temperature */
        uint32_t temperature;
        /*! un-compensated humidity */
        uint32_t humidity;
    };

    /**\name Sensor component selection macros
       These values are internal for API implementation. Don't relate this to
       data sheet.*/
    static const uint8_t DATA_PRESS  = 1 << 0;
    static const uint8_t DATA_TEMP   = 1 << 1;
    static const uint8_t DATA_HUM    = 1 << 2;
    static const uint8_t DATA_ALL    = 0x07;

    /*!
     *  @brief This API is the entry point.
     *  It reads the chip-id and calibration data from the sensor.
     *
     *  @param[in] dev_id : Device Id (i.e. SS pin/i2c Address)
     *  @param[in] intf : SPI/I2C interface
     *  @param[in] settings : Sensor settings
     *
     *  @return Result of API execution status
     *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t begin(uint8_t dev_id, IntfType intf, const Settings& settings);

    /*!
     * @brief This API sets the oversampling, filter and standby duration
     * (normal mode) settings in the sensor.
     *
     * @param[in] desired_settings : Variable used to select the settings which
     * are to be set in the sensor.
     *
     * @note : Below are the macros to be used by the user for selecting the
     * desired settings. User can do OR operation of these macros for configuring
     * multiple settings.
     *
     * Macros		  |   Functionality
     * -----------------------|----------------------------------------------
     * BME280_OSR_PRESS_SEL    |   To set pressure oversampling.
     * BME280_OSR_TEMP_SEL     |   To set temperature oversampling.
     * BME280_OSR_HUM_SEL    |   To set humidity oversampling.
     * BME280_FILTER_SEL     |   To set filter setting.
     * BME280_STANDBY_SEL  |   To set standby duration setting.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
     */
    int8_t set_sensor_settings(uint8_t desired_settings);

    /*!
     * @brief This API gets the oversampling, filter and standby duration
     * (normal mode) settings from the sensor.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
     */
    int8_t get_sensor_settings();

    /*!
     * @brief This API sets the power mode of the sensor.
     *
     * @param[in] sensor_mode : Variable which contains the power mode to be set.
     *
     *    sensor_mode           |   Macros
     * ---------------------|-------------------
     *     0                | BME280_SLEEP_MODE
     *     1                | BME280_FORCED_MODE
     *     3                | BME280_NORMAL_MODE
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t set_sensor_mode(uint8_t sensor_mode);

    /*!
     * @brief This API gets the power mode of the sensor.
     *
     * @param[out] sensor_mode : Pointer variable to store the power mode.
     *
     *   sensor_mode            |   Macros
     * ---------------------|-------------------
     *     0                | BME280_SLEEP_MODE
     *     1                | BME280_FORCED_MODE
     *     3                | BME280_NORMAL_MODE
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t get_sensor_mode(uint8_t& sensor_mode);

    /*!
     * @brief This API performs the soft reset of the sensor.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
     */
    int8_t soft_reset();

    /*!
     * @brief This API reads the pressure, temperature and humidity data from the
     * sensor, compensates the data and store it in the bme280_data structure
     * instance passed by the user.
     *
     * @param[in] sensor_comp : Variable which selects which data to be read from
     * the sensor.
     *
     * sensor_comp |   Macros
     * ------------|-------------------
     *     1       | BME280_PRESS
     *     2       | BME280_TEMP
     *     4       | BME280_HUM
     *     7       | BME280_ALL
     *
     * @param[out] comp_data : Structure instance of bme280_data.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t get_sensor_data(uint8_t sensor_comp, data& comp_data);

private:
    /**\name API error codes */
    static const int8_t E_NULL_PTR          = -1;
    static const int8_t E_DEV_NOT_FOUND     = -2;
    static const int8_t E_INVALID_LEN       = -3;
    static const int8_t E_COMM_FAIL         = -4;
    static const int8_t E_SLEEP_MODE_FAIL   = -5;

    /**\name BME280 chip identifier */
    static const uint8_t CHIP_ID = 0x60;

    /**\name Register Address */
    static const uint8_t CHIP_ID_ADDR = 0xD0;
    static const uint8_t RESET_ADDR = 0xE0;
    static const uint8_t TEMP_PRESS_CALIB_DATA_ADDR = 0x88;
    static const uint8_t HUMIDITY_CALIB_DATA_ADDR = 0xE1;
    static const uint8_t PWR_CTRL_ADDR = 0xF4;
    static const uint8_t CTRL_HUM_ADDR = 0xF2;
    static const uint8_t CTRL_MEAS_ADDR = 0xF4;
    static const uint8_t CONFIG_ADDR = 0xF5;
    static const uint8_t DATA_ADDR = 0xF7;

    /**\name Macros related to size */
    static const uint8_t TEMP_PRESS_CALIB_DATA_LEN = 26;
    static const uint8_t HUMIDITY_CALIB_DATA_LEN = 7;
    static const uint8_t P_T_H_DATA_LEN = 8;

    /**\name Macros for bit masking */
    static const uint8_t SENSOR_MODE_MSK = 0x03;
    static const uint8_t SENSOR_MODE_POS = 0x00;

    static const uint8_t CTRL_HUM_MSK = 0x07;
    static const uint8_t CTRL_HUM_POS = 0x00;

    static const uint8_t CTRL_PRESS_MSK = 0x1C;
    static const uint8_t CTRL_PRESS_POS = 0x02;

    static const uint8_t CTRL_TEMP_MSK = 0xE0;
    static const uint8_t CTRL_TEMP_POS = 0x05;

    static const uint8_t FILTER_MSK = 0x1C;
    static const uint8_t FILTER_POS = 0x02;

    static const uint8_t STANDBY_MSK = 0xE0;
    static const uint8_t STANDBY_POS = 0x05;

    /**\name API warning codes */
    static const int8_t W_INVALID_OSR_MACRO = 1;

    // i2c timeout (ms)
	static const int TIMEOUT = 3000;

    /*!
     * @brief Calibration data
     */
    struct CalibData {
         /**
         * @ Trim Variables
         */
        /**@{*/
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
        uint8_t  dig_H1;
        int16_t dig_H2;
        uint8_t  dig_H3;
        int16_t dig_H4;
        int16_t dig_H5;
        int8_t  dig_H6;
        int32_t t_fine;
        /**@}*/
    };

	/*! Device Id (i.e.: SS pin/i2c Address) */
	uint8_t dev_id;

    /*! Trim data */
	CalibData calib_data;

	IntfType intf;
	Settings sens_settings;

    /*!
     * @brief This API writes the given data to the register address
     * of the sensor.
     *
     * @param[in] reg_addr : Register address from where the data to be written.
     * @param[in] reg_data : Pointer to data buffer which is to be written
     * in the sensor.
     * @param[in] len : No of bytes of data to write..
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len);

    /*!
     * @brief This API reads the data from the given register address of the sensor.
     *
     * @param[in] reg_addr : Register address from where the data to be read
     * @param[out] reg_data : Pointer to data buffer to store the read data.
     * @param[in] len : No of bytes of data to be read.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

    /*!
     * @brief This internal API interleaves the register address between the
     * register data buffer for burst write operation.
     *
     * @param[in] reg_addr : Contains the register address array.
     * @param[out] temp_buff : Contains the temporary buffer to store the
     * register data and register address.
     * @param[in] reg_data : Contains the register data to be written in the
     * temporary buffer.
     * @param[in] len : No of bytes of data to be written for burst write.
     */
    void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);

    /*!
     * @brief This internal API reads the calibration data from the sensor, parse
     * it and store in the device structure.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t get_calib_data();

    /*!
     *  @brief This internal API is used to parse the temperature and
     *  pressure calibration data and store it in the device structure.
     *
     *  @param[in] reg_data : Contains the calibration data to be parsed.
     */
    void parse_temp_press_calib_data(const uint8_t *reg_data);

    /*!
     *  @brief This internal API is used to parse the humidity calibration data
     *  and store it in device structure.
     *
     *  @param[in] reg_data : Contains calibration data to be parsed.
     */
    void parse_humidity_calib_data(const uint8_t *reg_data);

    /**\name Internal macros */
    /* To identify osr settings selected by user */
    static const uint8_t SETTINGS_OVERSAMPLING = 0x07;
    /* To identify filter and standby settings selected by user */
    static const uint8_t SETTINGS_FILTER_STANDBY = 0x18;

    /*!
     * @brief This internal API is used to identify the settings which the user
     * wants to modify in the sensor.
     *
     * @param[in] sub_settings : Contains the settings subset to identify particular
     * group of settings which the user is interested to change.
     * @param[in] desired_settings : Contains the user specified settings.
     *
     * @return Indicates whether user is interested to modify the settings which
     * are related to sub_settings.
     * @retval True -> User wants to modify this group of settings
     * @retval False -> User does not want to modify this group of settings
     */
    bool are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);

    /*!
     * @brief This internal API puts the device to sleep mode.
     *
     * @return Result of API execution status.
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t put_device_to_sleep();

    /*!
     * @brief This internal API writes the power mode in the sensor.
     *
     * @param[in] sensor_mode : Variable which contains the power mode to be set.
     *
     * @return Result of API execution status.
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t write_power_mode(uint8_t sensor_mode);

    /*!
     * @brief This internal API reloads the already existing device settings in the
     * sensor after soft reset.
     *
     * @param[in] settings : Pointer variable which contains the settings to
     * be set in the sensor.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t reload_device_settings(const Settings& settings);

    /*!
     * @brief This internal API sets the oversampling settings for pressure,
     * temperature and humidity in the sensor.
     *
     * @param[in] desired_settings : Variable used to select the settings which
     * are to be set.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t set_osr_settings(uint8_t desired_settings, const Settings& settings);

    /*!
     * @brief This API sets the humidity oversampling settings of the sensor.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t set_osr_humidity_settings(const Settings& settings);

    /*!
     * @brief This API sets the pressure and/or temperature oversampling settings
     * in the sensor according to the settings selected by the user.
     *
     * @param[in] desired_settings: variable to select the pressure and/or
     * temperature oversampling settings.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t set_osr_press_temp_settings(uint8_t desired_settings, const Settings& settings);

    /*!
     * @brief This internal API sets the filter and/or standby duration settings
     * in the sensor according to the settings selected by the user.
     *
     * @param[in] desired_settings : variable to select the filter and/or
     * standby duration settings.
     *
     * @return Result of API execution status
     * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
     */
    int8_t set_filter_standby_settings(uint8_t desired_settings, const Settings& settings);

    /*!
     * @brief This internal API fills the filter settings provided by the user
     * in the data buffer so as to write in the sensor.
     *
     * @param[out] reg_data : Variable which is filled according to the filter
     * settings data provided by the user.
     */
    void fill_filter_settings(uint8_t *reg_data, const Settings& settings);

    /*!
     * @brief This internal API fills the standby duration settings provided by the
     * user in the data buffer so as to write in the sensor.
     *
     * @param[out] reg_data : Variable which is filled according to the standby
     * settings data provided by the user.
     */
    void fill_standby_settings(uint8_t *reg_data, const Settings& settings);

    /*!
     * @brief This internal API fills the pressure oversampling settings provided by
     * the user in the data buffer so as to write in the sensor.
     *
     * @param[out] reg_data : Variable which is filled according to the pressure
     * oversampling data provided by the user.
     */
    void fill_osr_press_settings(uint8_t *reg_data, const Settings& settings);

    /*!
     * @brief This internal API fills the temperature oversampling settings provided
     * by the user in the data buffer so as to write in the sensor.
     *
     * @param[in] dev : Structure instance of bme280_dev.
     * @param[out] reg_data : Variable which is filled according to the temperature
     * oversampling data provided by the user.
     */
    void fill_osr_temp_settings(uint8_t *reg_data, const Settings& settings);

    /*!
     * @brief This internal API parse the oversampling(pressure, temperature
     * and humidity), filter and standby duration settings and store in the
     * device structure.
     *
     * @param[in] reg_data : Register data to be parsed.
     */
    void parse_device_settings(const uint8_t *reg_data, Settings& settings);

    /*!
     *  @brief This API is used to parse the pressure, temperature and
     *  humidity data and store it in the bme280_uncomp_data structure instance.
     *
     *  @param[in] reg_data     : Contains register data which needs to be parsed
     *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
     *  and humidity data.
     */
    void parse_sensor_data(const uint8_t *reg_data, uncomp_data& uncomp_data);

    /*!
     * @brief This API is used to compensate the pressure and/or
     * temperature and/or humidity data according to the component selected by the
     * user.
     *
     * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
     * humidity.
     * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
     * humidity data.
     * @param[out] comp_data : Contains the compensated pressure and/or temperature
     * and/or humidity data.
     * @param[in] calib_data : Pointer to the calibration data structure.
     *
     * @return Result of API execution status.
     * @retval zero -> Success / -ve value -> Error
     */
    int8_t compensate_data(uint8_t sensor_comp, const uncomp_data& uncomp_data,
                        data& comp_data);

    /*!
     * @brief This internal API is used to compensate the raw temperature data and
     * return the compensated temperature data in double data type.
     *
     * @param[in] uncomp_data : Contains the uncompensated temperature data.
     *
     * @return Compensated temperature data.
     * @retval Compensated temperature data in double.
     */
    int32_t compensate_temperature(const uncomp_data& uncomp_data);

    /*!
     * @brief This internal API is used to compensate the raw pressure data and
     * return the compensated pressure data.
     *
     * @param[in] uncomp_data : Contains the uncompensated pressure data.
     *
     * @return Compensated pressure data.
     * @retval Compensated pressure data in double.
     */
    uint32_t compensate_pressure(const uncomp_data& uncomp_data);

    /*!
     * @brief This internal API is used to compensate the raw humidity data and
     * return the compensated humidity data.
     *
     * @param[in] uncomp_data : Contains the uncompensated humidity data.
     *
     * @return Compensated humidity data.
     * @retval Compensated humidity data in double.
     */
    uint32_t compensate_humidity(const uncomp_data& uncomp_data);

    int8_t bus_read (uint8_t dev_id, uint8_t reg_addr,
                                uint8_t *reg_data,
                                uint16_t len);

    int8_t bus_write (uint8_t dev_id, uint8_t reg_addr,
                                 uint8_t *reg_data,
                                 uint16_t len);
};

#endif // BOSCH_BME280_H
