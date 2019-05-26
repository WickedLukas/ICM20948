/***************************************************************************//**
 * @file ICM20948.cpp
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "ICM20948.h"

/** 
 * ICM20948 constructor
 */
ICM20948::ICM20948(void) {
}

/**
 * ICM20948 destructor
 */
ICM20948::~ICM20948(void) {
}

/** Probe for ICM20948 and try to initialize sensor
*
* @return
*   'true' if successful,
*   'false' on error.
*/
bool ICM20948::init() {
    uint8_t data;

    //reset();

    /* Reset I2C Slave module and use SPI */
    /* Enable I2C Master I/F module */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_I2C_IF_DIS | ICM20948_BIT_I2C_MST_EN);

    /* Read "Who am I" register */
    read_register(ICM20948_REG_WHO_AM_I, 1, &data);

    /* Check if "Who am I" register was succesfully read */
    if (data != ICM20948_DEVICE_ID) {
        return ERROR;
    }
    
    /* Read AK09916 "Who am I" register */
    read_mag_register(AK09916_REG_WHO_AM_I, 1, &data);
    
    /* Check if AK09916 "Who am I" register was succesfully read */
    if (data != AK09916_DEVICE_ID) {
        return ERROR;
    }
	
	// TODO: odr_align_en to sync sample rates seems not to be necessary
	// configure gyroscope
	this->set_gyro_bandwidth(ICM20948_GYRO_BW_12100HZ);
	this->set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_1000DPS);
	// the gyroscope sample rate is 9000 Hz for ICM20948_GYRO_BW_12100HZ
	//this->set_gyro_sample_rate(...);
		
	// configure accelerometer
	this->set_accel_bandwidth(ICM20948_ACCEL_BW_1210HZ);
	this->set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_8G);
	// the accelerometer sample rate is 4500 Hz for ICM20948_ACCEL_BW_1210HZ
	//this->set_accel_sample_rate(...);

    /* Auto select best available clock source PLL if ready, else use internal oscillator */
    write_register(ICM20948_REG_PWR_MGMT_1, ICM20948_BIT_CLK_PLL);

    /* PLL startup time - maybe it is too long, but better stay on the safe side - no spec in datasheet */
    delay(30);
	
	// TODO: make it configurable
	// Enable Raw Data Ready interrupt
	this->enable_irq(true, false);

    return OK;
}

/** Perform a measurement
 *
 * @return true if measurement was successful
 */
bool ICM20948::measure() {
	return OK;
}

/** Get gyroscope measurement
 *
 * @param[out] gyr_x Gyroscope measurement on X axis
 * @param[out] gyr_y Gyroscope measurement on Y axis
 * @param[out] gyr_z Gyroscope measurement on Z axis
 *
 * @return true if measurement was successful
 */
bool ICM20948::get_gyroscope(float *gyr_x, float *gyr_y, float *gyr_z) {
    float buf[3];
    if(read_gyro_data(buf)) {
        return false;
    }

    *gyr_x = buf[0];
    *gyr_y = buf[1];
    *gyr_z = buf[2];
}

/** Get accelerometer measurement
 *
 * @param[out] acc_x Accelerometer measurement on X axis
 * @param[out] acc_y Accelerometer measurement on Y axis
 * @param[out] acc_z Accelerometer measurement on Z axis
 *
 * @return true if measurement was successful
 */
bool ICM20948::get_accelerometer(float *acc_x, float *acc_y, float *acc_z) {
    float buf[3];
    if(read_accel_data(buf)) {
        return false;
    }

    *acc_x = buf[0];
    *acc_y = buf[1];
    *acc_z = buf[2];
}

/** Get temperature measurement
 *
 * @param [out] measured temperature
 *
 * @return true if measurement was successful
 */
bool ICM20948::get_temperature(float *temperature) {
    read_temperature(temperature);
    return true;
}

/***************************************************************************//**
 * @brief
 *    Read register in the ICM20948 device
 *
 * @param[in] addr
 *    Address to read from
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] numBytes
 *    Number of bytes to read
 *
 * @param[out] data
 *    Data read from register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20948::read_register(uint16_t addr, uint8_t numBytes, uint8_t *data) {
    uint8_t regAddr;
    uint8_t bank;

    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);

    /* Enable chip select */
    m_CS = 0;

    /* Set R/W bit to 1 - read */
    m_SPI.write(regAddr | 0x80);
    /* Transmit 0's to provide clock and read data */
    m_SPI.write(NULL, 0, (char*)data, numBytes);

    /* Disable chip select */
    m_CS = 1;

    return;
}

/***************************************************************************//**
 * @brief
 *    Writes a register in the ICM20948 device
 *
 * @param[in] addr
 *    Address to write to
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] data
 *    Data to write to register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20948::write_register(uint16_t addr, uint8_t data) {
    uint8_t regAddr;
    uint8_t bank;

    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);

    /* Enable chip select */
    m_CS = 0;

    /* Clear R/W bit - write, send address */
    m_SPI.write(regAddr & 0x7F);
    m_SPI.write(data);

    /* Disable chip select */
    m_CS = 1;

    return;
}

/***************************************************************************//**
 * @brief
 *    Sets desired magnetometer transfer mode
 *
 * @param[in] read
 *    If true sets transfer mode to read.
 *    If false sets transfer mode to write.
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20948::set_mag_transfer(bool read) {
    static const uint8_t MAG_BIT_READ   = AK09916_BIT_I2C_SLV_ADDR | ICM20948_BIT_I2C_SLV_READ;
    static const uint8_t MAG_BIT_WRITE  = AK09916_BIT_I2C_SLV_ADDR;
    
    static bool read_old = !read;
    
    /* Set transfer mode if it has changed */
    if (read != read_old) {   
        if (read) {
            write_register(ICM20948_REG_I2C_SLV0_ADDR, MAG_BIT_READ);
        }
        else {
            write_register(ICM20948_REG_I2C_SLV0_ADDR, MAG_BIT_WRITE);
        }
        read_old = read;
    }
}

/***************************************************************************//**
 * @brief
 *    Read register in the AK09916 magnetometer device
 *
 * @param[in] addr
 *    Register address to read from
 *    Bit[6:0] - register address
 *
 * @param[in] numBytes
 *    Number of bytes to read
 *
 * @param[out] data
 *    Data read from register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20948::read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data) {
    /* Set transfer mode to read */
    set_mag_transfer(true);
    
    /* Set SLV0_REG to magnetometer register address */
	write_register(ICM20948_REG_I2C_SLV0_REG, addr);
    
    /* Request bytes */
    write_register(ICM20948_REG_I2C_SLV0_CTRL, ICM20948_BIT_I2C_SLV_EN | numBytes);
    
    /* Wait some time for registers to fill */
    delay(1);
    
    /* Read bytes from the ICM-20948 EXT_SLV_SENS_DATA registers */
    read_register(ICM20948_REG_EXT_SLV_SENS_DATA_00, numBytes, data); 
    
    return;
}

/***************************************************************************//**
 * @brief
 *    Writes a register in the AK09916 magnetometer device
 *
 * @param[in] addr
 *    Register address to write to
 *    Bit[6:0] - register address
 *
 * @param[out] data
 *    Data to write to register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20948::write_mag_register(uint8_t addr, uint8_t data) {
    /* Set transfer mode to write */
    set_mag_transfer(false);
    
    /* Set SLV0_REG to magnetometer register address */
	write_register(ICM20948_REG_I2C_SLV0_REG, addr);
    
    /* Store data to write inside SLV0_DO */
	write_register(ICM20948_REG_I2C_SLV0_DO, data);
    
    /* Send one byte */
    write_register(ICM20948_REG_I2C_SLV0_CTRL, ICM20948_BIT_I2C_SLV_EN | 0x01);
    
    return;
}

/***************************************************************************//**
 * @brief
 *    Select desired register bank
 *
 * @param[in] bank
 *    Address of register bank (0..3)
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20948::select_bank(uint8_t bank) {
    static uint8_t bank_old = bank;
    
    /* Select bank if it has changed */
    if (bank != bank_old) {
        /* Enable chip select */
        m_CS = 0;
    
        /* Clear R/W bit - write, send address */
        m_SPI.write(ICM20948_REG_BANK_SEL);
        m_SPI.write((uint8_t)(bank << 4));

        /* Disable chip select */
        m_CS = 1;
        
        bank_old = bank;
    }
    return;
}

/***************************************************************************//**
 * @brief
 *    Perform ICM20948 soft reset
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::reset(void) {
    /* Set H_RESET bit to initiate soft reset */
    write_register(ICM20948_REG_PWR_MGMT_1, ICM20948_BIT_H_RESET);

    /* Wait 100ms to complete reset sequence */
    delay(100);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Perform AK09916 soft reset
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::reset_mag(void) {
    /* Set SRST bit to initiate soft reset */
    write_mag_register(AK09916_REG_CONTROL_3, AK09916_BIT_SRST);

    /* Wait 100ms to complete reset sequence */
    delay(100);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set accelerometer sample rate
 *
 * @param[in] sampleRate
 *    Desired sample rate in Hz
 *
 * @return
 *    Sample rate. May be different from desired value because
 *    of the finite and discrete number of divider settings
 ******************************************************************************/
float ICM20948::set_gyro_sample_rate(float sampleRate) {
    uint8_t gyroDiv;
    float gyroSampleRate;

    /* Calculate sample rate divider */
    gyroSampleRate = (1125.0f / sampleRate) - 1.0f;

    /* Check if it fits inside divider register */
    if ( gyroSampleRate > 255.0f ) {
        gyroSampleRate = 255.0f;
    }

    if ( gyroSampleRate < 0 ) {
        gyroSampleRate = 0.0f;
    }

    /* Write value to register */
    gyroDiv = (uint8_t) gyroSampleRate;
    write_register(ICM20948_REG_GYRO_SMPLRT_DIV, gyroDiv);

    /* Calculate sample rate from divider value */
    gyroSampleRate = 1125.0f / (gyroDiv + 1);

    return gyroSampleRate;
}

/***************************************************************************//**
 * @brief
 *    Set gyroscope sample rate
 *
 * @param[in] sampleRate
 *    Desired sample rate in Hz
 *
 * @return
 *    Sample rate. May be different from desired value because
 *    of the finite and discrete number of divider settings
 ******************************************************************************/
float ICM20948::set_accel_sample_rate(float sampleRate) {
    uint16_t accelDiv;
    float accelSampleRate;

    /* Calculate sample rate divider */
    accelSampleRate = (1125.0f / sampleRate) - 1.0f;

    /* Check if it fits inside divider registers */
    if ( accelSampleRate > 4095.0f ) {
        accelSampleRate = 4095.0f;
    }

    if ( accelSampleRate < 0 ) {
        accelSampleRate = 0.0f;
    }

    /* Write value to registers */
    accelDiv = (uint16_t) accelSampleRate;
    write_register(ICM20948_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accelDiv >> 8) );
    write_register(ICM20948_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF) );

    /* Calculate sample rate from divider value */
    accelSampleRate = 1125.0f / (accelDiv + 1);

    return accelSampleRate;
}

/***************************************************************************//**
 * @brief
 *    Set gyroscope bandwidth
 *
 * @param[in] gyroBw
 *    Desired bandwidth value. Use ICM20948_GYRO_BW macros.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::set_gyro_bandwidth(uint8_t gyroBw) {
    uint8_t reg;

    /* Read GYRO_CONFIG_1 register */
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM20948_MASK_GYRO_BW);

    /* Write new bandwidth value to gyro config register */
    reg |= (gyroBw & ICM20948_MASK_GYRO_BW);
    write_register(ICM20948_REG_GYRO_CONFIG_1, reg);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set accelerometer bandwidth
 *
 * @param[in] accelBw
 *    Desired bandwidth value. Use ICM20948_ACCEL_BW macros.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::set_accel_bandwidth(uint8_t accelBw) {
    uint8_t reg;

    /* Read GYRO_CONFIG_1 register */
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM20948_MASK_ACCEL_BW);

    /* Write new bandwidth value to gyro config register */
    reg |= (accelBw & ICM20948_MASK_ACCEL_BW);
    write_register(ICM20948_REG_ACCEL_CONFIG, reg);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Read raw acceleration values and convert them to G values
 *
 * @param[out] accel
 *    A 3-element array of float numbers containing acceleration values
 *    for x, y and z axes in g units.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::read_accel_data(float *accel) {
    uint8_t rawData[6];
    float accelRes;
    int16_t temp;

    /* Retrieve current resolution */
    get_accel_resolution(&accelRes);

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_ACCEL_XOUT_H_SH, 6, &rawData[0]);

    /* Convert the MSB and LSB into a signed 16-bit value and multiply it with the resolution to get the G value */
    temp = ( (int16_t) rawData[0] << 8) | rawData[1];
    accel[0] = (float) temp * accelRes;
    temp = ( (int16_t) rawData[2] << 8) | rawData[3];
    accel[1] = (float) temp * accelRes;
    temp = ( (int16_t) rawData[4] << 8) | rawData[5];
    accel[2] = (float) temp * accelRes;

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Read raw gyroscope values and convert them to deg/sec
 *
 * @param[out] gyro
 *    A 3-element array of float numbers containing gyroscope values
 *    for x, y and z axes in deg/sec units.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::read_gyro_data(float *gyro) {
    uint8_t rawData[6];
    float gyroRes;
    int16_t temp;

    /* Retrieve current resolution */
    get_gyro_resolution(&gyroRes);

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_GYRO_XOUT_H_SH, 6, &rawData[0]);

    /* Convert the MSB and LSB into a signed 16-bit value and multiply it with the resolution to get the dps value */
    temp = ( (int16_t) rawData[0] << 8) | rawData[1];
    gyro[0] = (float) temp * gyroRes;
    temp = ( (int16_t) rawData[2] << 8) | rawData[3];
    gyro[1] = (float) temp * gyroRes;
    temp = ( (int16_t) rawData[4] << 8) | rawData[5];
    gyro[2] = (float) temp * gyroRes;

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Read raw magnetometer values and convert them to uT
 *
 * @param[out] mag
 *    A 3-element array of float numbers containing magnetometer values
 *    for x, y and z axes in uT units.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::read_mag_data(float *mag) {
    uint8_t rawData[6];
    
    /* Calculate resolution */
    /* Measurement range of each axis +-4912 uT is saved in 16 bit output +-32752 */
    static const float magRes = 4912.0f / 32752.0f;
    
    int16_t temp;

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_EXT_SLV_SENS_DATA_00, 6, &rawData[0]);

    /* Convert the MSB and LSB into a signed 16-bit value and multiply it with the resolution to get the uT value */
    temp = ( (int16_t) rawData[0] << 8) | rawData[1];
    mag[0] = (float) temp * magRes;
    temp = ( (int16_t) rawData[2] << 8) | rawData[3];
    mag[1] = (float) temp * magRes;
    temp = ( (int16_t) rawData[4] << 8) | rawData[5];
    mag[2] = (float) temp * magRes;

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Get accelerometer resolution
 *
 * @param[out] accelRes
 *    Resolution in g/bit units
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::get_accel_resolution(float *accelRes) {
    uint8_t reg;

    /* Read acceleration full scale setting */
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ICM20948_MASK_ACCEL_FULLSCALE;

    /* Calculate resolution */
    switch ( reg ) {
        case ICM20948_ACCEL_FULLSCALE_2G:
            *accelRes = 2.0f / 32768.0f;
            break;

        case ICM20948_ACCEL_FULLSCALE_4G:
            *accelRes = 4.0f / 32768.0f;
            break;

        case ICM20948_ACCEL_FULLSCALE_8G:
            *accelRes = 8.0f / 32768.0f;
            break;

        case ICM20948_ACCEL_FULLSCALE_16G:
            *accelRes = 16.0f / 32768.0f;
            break;
    }

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Get gyroscope resolution
 *
 * @param[out] gyroRes
 *    Resolution in (deg/sec)/bit units
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::get_gyro_resolution(float *gyroRes) {
    uint8_t reg;

    /* Read gyroscope full scale setting */
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ICM20948_MASK_GYRO_FULLSCALE;

    /* Calculate resolution */
    switch ( reg ) {
        case ICM20948_GYRO_FULLSCALE_250DPS:
            *gyroRes = 250.0f / 32768.0f;
            break;

        case ICM20948_GYRO_FULLSCALE_500DPS:
            *gyroRes = 500.0f / 32768.0f;
            break;

        case ICM20948_GYRO_FULLSCALE_1000DPS:
            *gyroRes = 1000.0f / 32768.0f;
            break;

        case ICM20948_GYRO_FULLSCALE_2000DPS:
            *gyroRes = 2000.0f / 32768.0f;
            break;
    }

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set accelerometer full scale
 *
 * @param[in] accelFs
 *    Desired full scale value. Use ICM20948_ACCEL_FULLSCALE macros.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::set_accel_fullscale(uint8_t accelFs) {
    uint8_t reg;

    accelFs &= ICM20948_MASK_ACCEL_FULLSCALE;
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM20948_MASK_ACCEL_FULLSCALE);
    reg |= accelFs;
    write_register(ICM20948_REG_ACCEL_CONFIG, reg);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set gyroscope full scale
 *
 * @param[in] gyroFs
 *    Desired full scale value. Use ICM20948_GYRO_FULLSCALE macros.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::set_gyro_fullscale(uint8_t gyroFs) {
    uint8_t reg;

    gyroFs &= ICM20948_MASK_GYRO_FULLSCALE;
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM20948_MASK_GYRO_FULLSCALE);
    reg |= gyroFs;
    write_register(ICM20948_REG_GYRO_CONFIG_1, reg);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables sleep mode
 *
 * @param[in] enable
 *    If true, sleep mode is enabled. Set to false to disable sleep mode.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::enable_sleepmode(bool enable) {
    uint8_t reg;

    read_register(ICM20948_REG_PWR_MGMT_1, 1, &reg);

    if ( enable ) {
        /* Sleep: set SLEEP bit */
        reg |= ICM20948_BIT_SLEEP;
    } else {
        /* Wake up: clear SLEEP bit */
        reg &= ~(ICM20948_BIT_SLEEP);
    }

    write_register(ICM20948_REG_PWR_MGMT_1, reg);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables cycle mode operation for accel and gyro
 *
 * @param[in] enable
 *    If true accel and gyro will operate in cycle mode. If
 *    false they will operate in continuous mode.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::enable_cyclemode(bool enable) {
    uint8_t reg;

    reg = 0x00;

    if ( enable ) {
        reg = ICM20948_BIT_ACCEL_CYCLE | ICM20948_BIT_GYRO_CYCLE;
    }

    write_register(ICM20948_REG_LP_CONFIG, reg);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables sensors
 *
 * @param[in] accel
 *    If true enables acceleration sensor
 *
 * @param[in] gyro
 *    If true enables gyroscope sensor
 *
 * @param[in] temp
 *    If true enables temperature sensor
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::enable_sensor(bool accel, bool gyro, bool temp) {
    uint8_t pwrManagement1;
    uint8_t pwrManagement2;

    read_register(ICM20948_REG_PWR_MGMT_1, 1, &pwrManagement1);
    pwrManagement2 = 0;

    /* To enable accelerometer clear DISABLE_ACCEL bits in PWR_MGMT_2 */
    if ( accel ) {
        pwrManagement2 &= ~(ICM20948_BIT_PWR_ACCEL_STBY);
    } else {
        pwrManagement2 |= ICM20948_BIT_PWR_ACCEL_STBY;
    }

    /* To enable gyro clear DISABLE_GYRO bits in PWR_MGMT_2 */
    if ( gyro ) {
        pwrManagement2 &= ~(ICM20948_BIT_PWR_GYRO_STBY);
    } else {
        pwrManagement2 |= ICM20948_BIT_PWR_GYRO_STBY;
    }

    /* To enable temperature sensor clear TEMP_DIS bit in PWR_MGMT_1 */
    if ( temp ) {
        pwrManagement1 &= ~(ICM20948_BIT_TEMP_DIS);
    } else {
        pwrManagement1 |= ICM20948_BIT_TEMP_DIS;
    }

    /* Write back modified values */
    write_register(ICM20948_REG_PWR_MGMT_1, pwrManagement1);
    write_register(ICM20948_REG_PWR_MGMT_2, pwrManagement2);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables sensors in low power mode
 *
 * @param[in] enAccel
 *    If true enables acceleration sensor in low power mode
 *
 * @param[in] enGyro
 *    If true enables gyroscope sensor in low power mode
 *
 * @param[in] enTemp
 *    If true enables temperature sensor in low power mode
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::enter_lowpowermode(bool enAccel, bool enGyro, bool enTemp) {
    uint8_t data;

    read_register(ICM20948_REG_PWR_MGMT_1, 1, &data);

    if ( enAccel || enGyro || enTemp ) {
        /* Make sure that chip is not in sleep */
        enable_sleepmode(false);

        /* And in continuous mode */
        enable_cyclemode(false);

        /* Enable accelerometer and gyroscope*/
        enable_sensor(enAccel, enGyro, enTemp);
        delay(50);

        /* Enable cycle mode */
        enable_cyclemode(true);

        /* Set LP_EN bit to enable low power mode */
        data |= ICM20948_BIT_LP_EN;
    } else {
        /* Enable continuous mode */
        enable_cyclemode(false);

        /* Clear LP_EN bit to disable low power mode */
        data &= ~ICM20948_BIT_LP_EN;
    }

    /* Write updated value to PWR_MGNT_1 register */
    write_register(ICM20948_REG_PWR_MGMT_1, data);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables interrupts
 *
 * @param[in] dataReadyEnable
 *    If true enables Raw Data Ready interrupt, otherwise disables.
 *
 * @param[in] womEnable
 *    If true enables Wake-up On Motion interrupt, otherwise disables.
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::enable_irq(bool dataReadyEnable, bool womEnable) {
    /* Enable one or both of interrupt sources if required */
	if ( dataReadyEnable ) {
		write_register(ICM20948_REG_INT_ENABLE_1, ICM20948_BIT_RAW_DATA_0_RDY_EN);
	}
	else {
		write_register(ICM20948_REG_INT_ENABLE_1, 0);
	}
	
	if ( womEnable ) {
        write_register(ICM20948_REG_INT_ENABLE, ICM20948_BIT_WOM_INT_EN);
    }
	else {
		write_register(ICM20948_REG_INT_ENABLE, 0);
	}
	
	/* INT pin: active low, open drain, IT status read clears. It seems that latched mode does not work, the INT pin cannot be cleared if set */
	write_register(ICM20948_REG_INT_PIN_CFG, ICM20948_BIT_INT_ACTL | ICM20948_BIT_INT_OPEN);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Read interrupt status registers
 *
 * @param[out] intStatus
 *    Content of the four interrupt registers. LSByte is INT_STATUS, MSByte is
 *    INT_STATUS_3
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::read_irqstatus(uint32_t *int_status) {
    uint8_t reg[4];

    read_register(ICM20948_REG_INT_STATUS, 4, reg);
    *int_status = (uint32_t) reg[0];
    *int_status |= ( ( (uint32_t) reg[1]) << 8);
    *int_status |= ( ( (uint32_t) reg[2]) << 16);
    *int_status |= ( ( (uint32_t) reg[3]) << 24);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Checks if new data is available for read
 *
 * @return
 *    Return true if Raw Data Ready interrupt bit set, false otherwise
 ******************************************************************************/
bool ICM20948::is_data_ready(void) {
    uint8_t status;
    bool ret;

    ret = false;
    read_register(ICM20948_REG_INT_STATUS_1, 1, &status);

    if ( status & ICM20948_BIT_RAW_DATA_0_RDY_INT ) {
        ret = true;
    }

    return ret;
}

/***************************************************************************//**
 * @brief
 *    Set up and enables Wake-up On Motion feature
 *
 * @param[in] enable
 *    If true enables WOM feature, disables otherwise
 *
 * @param[in] womThreshold
 *    Threshold value for the Wake on Motion Interrupt for ACCEL x/y/z axes.
 *    LSB = 4mg. Range is 0mg to 1020mg
 *
 * @param[in] sampleRate
 *    Desired accel sensor sample rate in Hz
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::enable_wake_on_motion(bool enable, uint8_t womThreshold, float sampleRate) {
    if ( enable ) {
        /* Make sure that chip is not in sleep */
        enable_sleepmode(false);

        /* And in continuous mode */
        enable_cyclemode(false);

        /* Enable accelerometer only */
        enable_sensor(true, false, false);

        /* Set sample rate */
        set_sample_rate(sampleRate);

        /* Set bandwidth to 1210Hz */
        set_accel_bandwidth(ICM20948_ACCEL_BW_1210HZ);

        /* Accel: 2G full scale */
        set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_2G);

        /* Enable Wake On Motion interrupt */
        enable_irq(false, true);
        delay(50);

        /* Enable Wake On Motion feature */
        write_register(ICM20948_REG_ACCEL_INTEL_CTRL, ICM20948_BIT_ACCEL_INTEL_EN | ICM20948_BIT_ACCEL_INTEL_MODE);

        /* Set wake on motion threshold value */
        write_register(ICM20948_REG_ACCEL_WOM_THR, womThreshold);

        /* Enable low power mode */
        enter_lowpowermode(true, false, false);
	} else {
        /* Disable Wake On Motion feature */
        write_register(ICM20948_REG_ACCEL_INTEL_CTRL, 0x00);

        /* Disable Wake On Motion interrupt */
        enable_irq(false, false);

        /* Disable cycle mode */
        enable_cyclemode(false);
    }

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Accelerometer and gyroscope calibration function. Read gyroscope
 *    and accelerometer values while device is at rest and in level. The
 *    resulting values are loaded to accel and gyro bias registers to remove
 *    the static offset error.
 *
 * @param[out] accelBiasScaled
 *    Measured acceleration sensor bias in mg
 *
 * @param[out] gyroBiasScaled
 *    Measured gyro sensor bias in deg/sec
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::calibrate(float *accelBiasScaled, float *gyroBiasScaled) {
    uint8_t data[12];
    uint16_t i, packetCount, fifoCount;
    int32_t gyroBias[3] = { 0, 0, 0 };
    int32_t accelBias[3] = { 0, 0, 0 };
    int32_t accelTemp[3];
    int32_t gyroTemp[3];
    int32_t accelBiasFactory[3];
    int32_t gyroBiasStored[3];
    float gyroRes, accelRes;

    /* Enable accelerometer and gyro */
    enable_sensor(true, true, false);

    /* Set 1kHz sample rate */
    set_sample_rate(1100.0f);

    /* 246Hz BW for accelerometer and 200Hz for gyroscope */
    set_accel_bandwidth(ICM20948_ACCEL_BW_246HZ);
    set_gyro_bandwidth(ICM20948_GYRO_BW_12HZ);

    /* Set most sensitive range: 2G full scale and 250dps full scale */
    set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_2G);
    set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_250DPS);

    /* Retrieve resolution per bit */
    get_accel_resolution(&accelRes);
    get_gyro_resolution(&gyroRes);

    /* Accel sensor needs max 30ms, gyro max 35ms to fully start */
    /* Experiments show that gyro needs more time to get reliable results */
    delay(50);

    /* Disable FIFO */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_FIFO_EN);
    write_register(ICM20948_REG_FIFO_MODE, 0x0F);

    /* Enable accelerometer and gyro to store data in FIFO */
    write_register(ICM20948_REG_FIFO_EN_2, ICM20948_BIT_ACCEL_FIFO_EN | ICM20948_BITS_GYRO_FIFO_EN);

    /* Reset FIFO */
    write_register(ICM20948_REG_FIFO_RST, 0x0F);
    write_register(ICM20948_REG_FIFO_RST, 0x00);

    /* Enable FIFO */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_FIFO_EN);

    /* Max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
    /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
    /* Loop until at least 4080 samples gathered */
    fifoCount = 0;
    while ( fifoCount < 4080 ) {
        delay(5);
        /* Read FIFO sample count */
        read_register(ICM20948_REG_FIFO_COUNT_H, 2, &data[0]);
        /* Convert to a 16 bit value */
        fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);
    }

    /* Disable accelerometer and gyro to store data in FIFO */
    write_register(ICM20948_REG_FIFO_EN_2, 0x00);

    /* Read FIFO sample count */
    read_register(ICM20948_REG_FIFO_COUNT_H, 2, &data[0]);

    /* Convert to a 16 bit value */
    fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);

    /* Calculate number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
    packetCount = fifoCount / 12;

    /* Retrieve data from FIFO */
    for ( i = 0; i < packetCount; i++ ) {
        read_register(ICM20948_REG_FIFO_R_W, 12, &data[0]);
        /* Convert to 16 bit signed accel and gyro x,y and z values */
        accelTemp[0] = ( (int16_t) (data[0] << 8) | data[1]);
        accelTemp[1] = ( (int16_t) (data[2] << 8) | data[3]);
        accelTemp[2] = ( (int16_t) (data[4] << 8) | data[5]);
        gyroTemp[0] = ( (int16_t) (data[6] << 8) | data[7]);
        gyroTemp[1] = ( (int16_t) (data[8] << 8) | data[9]);
        gyroTemp[2] = ( (int16_t) (data[10] << 8) | data[11]);

        /* Sum up values */
        accelBias[0] += accelTemp[0];
        accelBias[1] += accelTemp[1];
        accelBias[2] += accelTemp[2];
        gyroBias[0] += gyroTemp[0];
        gyroBias[1] += gyroTemp[1];
        gyroBias[2] += gyroTemp[2];
    }

    /* Divide by packet count to get average */
    accelBias[0] /= packetCount;
    accelBias[1] /= packetCount;
    accelBias[2] /= packetCount;
    gyroBias[0] /= packetCount;
    gyroBias[1] /= packetCount;
    gyroBias[2] /= packetCount;

    /* Acceleormeter: add or remove (depends on chip orientation) 1G (gravity) from Z axis value */
    if ( accelBias[2] > 0L ) {
        accelBias[2] -= (int32_t) (1.0f / accelRes);
    } else {
        accelBias[2] += (int32_t) (1.0f / accelRes);
    }

    /* Convert values to degrees per sec for displaying */
    gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
    gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
    gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

    /* Read stored gyro trim values. After reset these values are all 0 */
    read_register(ICM20948_REG_XG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[0] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20948_REG_YG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[1] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20948_REG_ZG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[2] = ( (int16_t) (data[0] << 8) | data[1]);

    /* Gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
    /* best sensitivity, so need to divide by 4 */
    /* Substract from stored calibration value */
    gyroBiasStored[0] -= gyroBias[0] / 4;
    gyroBiasStored[1] -= gyroBias[1] / 4;
    gyroBiasStored[2] -= gyroBias[2] / 4;

    /* Split values into two bytes */
    data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
    data[1] = (gyroBiasStored[0]) & 0xFF;
    data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
    data[3] = (gyroBiasStored[1]) & 0xFF;
    data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
    data[5] = (gyroBiasStored[2]) & 0xFF;

    /* Write gyro bias values to chip */
    write_register(ICM20948_REG_XG_OFFS_USRH, data[0]);
    write_register(ICM20948_REG_XG_OFFS_USRL, data[1]);
    write_register(ICM20948_REG_YG_OFFS_USRH, data[2]);
    write_register(ICM20948_REG_YG_OFFS_USRL, data[3]);
    write_register(ICM20948_REG_ZG_OFFS_USRH, data[4]);
    write_register(ICM20948_REG_ZG_OFFS_USRL, data[5]);

    /* Calculate accelerometer bias values to store in hardware accelerometer bias registers. These registers contain */
    /* factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold */
    /* non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature */
    /* compensation calculations(? the datasheet is not clear). Accelerometer bias registers expect bias input */
    /* as 2048 LSB per g, so that accelerometer biases calculated above must be divided by 8. */

    /* Read factory accelerometer trim values */
    read_register(ICM20948_REG_XA_OFFSET_H, 2, &data[0]);
    accelBiasFactory[0] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20948_REG_YA_OFFSET_H, 2, &data[0]);
    accelBiasFactory[1] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20948_REG_ZA_OFFSET_H, 2, &data[0]);
    accelBiasFactory[2] = ( (int16_t) (data[0] << 8) | data[1]);

    /* Construct total accelerometer bias, including calculated average accelerometer bias from above */
    /* Scale 2g full scale (most sensitive range) results to 16g full scale - divide by 8 */
    /* Clear last bit (temperature compensation? - the datasheet is not clear) */
    /* Substract from factory calibration value */

    accelBiasFactory[0] -= ( (accelBias[0] / 8) & ~1);
    accelBiasFactory[1] -= ( (accelBias[1] / 8) & ~1);
    accelBiasFactory[2] -= ( (accelBias[2] / 8) & ~1);

    /* Split values into two bytes */
    data[0] = (accelBiasFactory[0] >> 8) & 0xFF;
    data[1] = (accelBiasFactory[0]) & 0xFF;
    data[2] = (accelBiasFactory[1] >> 8) & 0xFF;
    data[3] = (accelBiasFactory[1]) & 0xFF;
    data[4] = (accelBiasFactory[2] >> 8) & 0xFF;
    data[5] = (accelBiasFactory[2]) & 0xFF;

    /* Store them in accelerometer offset registers */
    write_register(ICM20948_REG_XA_OFFSET_H, data[0]);
    write_register(ICM20948_REG_XA_OFFSET_L, data[1]);
    write_register(ICM20948_REG_YA_OFFSET_H, data[2]);
    write_register(ICM20948_REG_YA_OFFSET_L, data[3]);
    write_register(ICM20948_REG_ZA_OFFSET_H, data[4]);
    write_register(ICM20948_REG_ZA_OFFSET_L, data[5]);

    /* Convert values to G for displaying */
    accelBiasScaled[0] = (float) accelBias[0] * accelRes;
    accelBiasScaled[1] = (float) accelBias[1] * accelRes;
    accelBiasScaled[2] = (float) accelBias[2] * accelRes;

    /* Turn off FIFO */
    write_register(ICM20948_REG_USER_CTRL, 0x00);

    /* Disable all sensors */
    enable_sensor(false, false, false);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Gyroscope calibration function. Read gyroscope
 *    values while device is at rest and in level. The
 *    resulting values are loaded to the gyro bias registers to remove
 *    the static offset error.
 *
 * @param[out] gyroBiasScaled
 *    Measured gyro sensor bias in deg/sec
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::calibrate_gyro(float *gyroBiasScaled) {
    uint8_t data[12];
    uint16_t i, packetCount, fifoCount;
    int32_t gyroBias[3] = { 0, 0, 0 };
    int32_t gyroTemp[3];
    int32_t gyroBiasStored[3];
    float gyroRes;

    /* Enable accelerometer and gyro */
    enable_sensor(true, true, false);

    /* Set 1kHz sample rate */
    set_sample_rate(1100.0f);

    /* Configure bandwidth for gyroscope to 12Hz */
    set_gyro_bandwidth(ICM20948_GYRO_BW_12HZ);

    /* Configure sensitivity to 250dps full scale */
    set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_250DPS);

    /* Retrieve resolution per bit */
    get_gyro_resolution(&gyroRes);

    /* Accel sensor needs max 30ms, gyro max 35ms to fully start */
    /* Experiments show that gyro needs more time to get reliable results */
    delay(50);

    /* Disable FIFO */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_FIFO_EN);
    write_register(ICM20948_REG_FIFO_MODE, 0x0F);

    /* Enable accelerometer and gyro to store data in FIFO */
    write_register(ICM20948_REG_FIFO_EN_2, ICM20948_BITS_GYRO_FIFO_EN);

    /* Reset FIFO */
    write_register(ICM20948_REG_FIFO_RST, 0x0F);
    write_register(ICM20948_REG_FIFO_RST, 0x00);

    /* Enable FIFO */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_FIFO_EN);

    /* Max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
    /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
    /* Loop until at least 4080 samples gathered */
    fifoCount = 0;
    while ( fifoCount < 4080 ) {
        delay(5);

        /* Read FIFO sample count */
        read_register(ICM20948_REG_FIFO_COUNT_H, 2, &data[0]);

        /* Convert to a 16 bit value */
        fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);
    }

    /* Disable accelerometer and gyro to store data in FIFO */
    write_register(ICM20948_REG_FIFO_EN_2, 0x00);

    /* Read FIFO sample count */
    read_register(ICM20948_REG_FIFO_COUNT_H, 2, &data[0]);

    /* Convert to a 16 bit value */
    fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);

    /* Calculate number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
    packetCount = fifoCount / 12;

    /* Retrieve data from FIFO */
    for ( i = 0; i < packetCount; i++ ) {
        read_register(ICM20948_REG_FIFO_R_W, 12, &data[0]);
        /* Convert to 16 bit signed accel and gyro x,y and z values */
        gyroTemp[0] = ( (int16_t) (data[6] << 8) | data[7]);
        gyroTemp[1] = ( (int16_t) (data[8] << 8) | data[9]);
        gyroTemp[2] = ( (int16_t) (data[10] << 8) | data[11]);

        /* Sum up values */
        gyroBias[0] += gyroTemp[0];
        gyroBias[1] += gyroTemp[1];
        gyroBias[2] += gyroTemp[2];
    }

    /* Divide by packet count to get average */
    gyroBias[0] /= packetCount;
    gyroBias[1] /= packetCount;
    gyroBias[2] /= packetCount;

    /* Convert values to degrees per sec for displaying */
    gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
    gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
    gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

    /* Read stored gyro trim values. After reset these values are all 0 */
    read_register(ICM20948_REG_XG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[0] = ( (int16_t) (data[0] << 8) | data[1]);

    read_register(ICM20948_REG_YG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[1] = ( (int16_t) (data[0] << 8) | data[1]);

    read_register(ICM20948_REG_ZG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[2] = ( (int16_t) (data[0] << 8) | data[1]);

    /* Gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
    /* best sensitivity, so need to divide by 4 */
    /* Substract from stored calibration value */
    gyroBiasStored[0] -= gyroBias[0] / 4;
    gyroBiasStored[1] -= gyroBias[1] / 4;
    gyroBiasStored[2] -= gyroBias[2] / 4;

    /* Split values into two bytes */
    data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
    data[1] = (gyroBiasStored[0]) & 0xFF;
    data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
    data[3] = (gyroBiasStored[1]) & 0xFF;
    data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
    data[5] = (gyroBiasStored[2]) & 0xFF;

    /* Write gyro bias values to chip */
    write_register(ICM20948_REG_XG_OFFS_USRH, data[0]);
    write_register(ICM20948_REG_XG_OFFS_USRL, data[1]);
    write_register(ICM20948_REG_YG_OFFS_USRH, data[2]);
    write_register(ICM20948_REG_YG_OFFS_USRL, data[3]);
    write_register(ICM20948_REG_ZG_OFFS_USRH, data[4]);
    write_register(ICM20948_REG_ZG_OFFS_USRL, data[5]);

    /* Turn off FIFO */
    write_register(ICM20948_REG_USER_CTRL, 0x00);

    /* Disable all sensors */
    enable_sensor(false, false, false);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Read temperature sensor raw value and convert it to Celsius.
 *
 * @param[out] temperature
 *    Measured temperature in Celsius
 *
 * @return
 *    Return zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20948::read_temperature(float *temperature) {
    uint8_t data[2];
    int16_t raw_temp;

    /* Read temperature registers */
    read_register(ICM20948_REG_TEMPERATURE_H, 2, data);

    /* Convert to int16 */
    raw_temp = (int16_t) ( (data[0] << 8) + data[1]);

    /* Calculate Celsius value from raw reading */
    *temperature = ( (float) raw_temp / 333.87f) + 21.0f;

    return OK;
}

/** Create an ICM20948_SPI object connected to specified SPI pins
 *
 * @param[in] csPin		SPI Chip Select pin.
 * @param[in] spiPort	SPI port.
 * 
 */
ICM20948_SPI::ICM20948_SPI(uint8_t csPin, SPIClass &spiPort = SPI) {
}

/**
 * ICM20948_SPI destructor
 */
ICM20948_SPI::~ICM20948_SPI(void) {
}