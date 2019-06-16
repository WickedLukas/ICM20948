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
    uint8_t data[7];

    //reset();
    
    /* Reset I2C Slave module and use SPI */
    /* Enable I2C Master I/F module */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_I2C_IF_DIS | ICM20948_BIT_I2C_MST_EN);
    
    /* Read "Who am I" register */
    read_register(ICM20948_REG_WHO_AM_I, 1, &data[0]);
    
    /* Check if "Who am I" register was successfully read */
    if (data[0] != ICM20948_DEVICE_ID) {
        delay(1000);
        Serial.println("ERROR_ICM20948");
        delay(1000);
        return ERROR;
    }
    delay(1000);
    Serial.println("ICM20948");
    Serial.println(data[0]);
    delay(1000);
    
    /* Read AK09916 "Who am I" register */
    read_mag_register(AK09916_REG_WHO_AM_I, 1, &data[0]);
    
    /* Check if AK09916 "Who am I" register was successfully read */
    if (data[0] != AK09916_DEVICE_ID) {
        delay(1000);
        Serial.println("ERROR_AK09916");
        Serial.println(data[0]);
        delay(1000);
        return ERROR;
    }
    
    // TODO: odr_align_en to sync sample rates seems not to be necessary
    /* configure gyroscope */
    this->set_gyro_bandwidth(ICM20948_GYRO_BW_12100HZ);
    this->set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_1000DPS);
    //this->set_gyro_sample_rate_div(...);    /* the gyroscope sample rate is 9000 Hz for ICM20948_GYRO_BW_12100HZ */
    
    /* configure accelerometer */
    this->set_accel_bandwidth(ICM20948_ACCEL_BW_1210HZ);
    this->set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_8G);
    //this->set_accel_sample_rate_div(...);    /* the accelerometer sample rate is 4500 Hz for ICM20948_ACCEL_BW_1210HZ */
    
    /* configure magnetometer */
    this->set_mag_mode(AK09916_MODE_100HZ);
    
    /* Instruct the ICM20948 to get data from the magnetometer */ 
    read_mag_register(AK09916_REG_HXL, 7, &data[0]);
    
    /* Auto select best available clock source PLL if ready, else use internal oscillator */
    write_register(ICM20948_REG_PWR_MGMT_1, ICM20948_BIT_CLK_PLL);
    
    /* PLL startup time - maybe it is too long, but better stay on the safe side - no spec in data sheet */
    delay(30);
    
    // TODO: make it configurable
    /* Enable Raw Data Ready interrupt */
    this->enable_irq(true, false);
    
    return true;
}

/** Read sensors raw data
 *
 * @param[out] accel_x Accelerometer raw measurement on X axis
 * @param[out] accel_y Accelerometer raw measurement on Y axis
 * @param[out] accel_z Accelerometer raw measurement on Z axis
 * @param[out] gyro_x Gyroscope raw measurement on X axis
 * @param[out] gyro_y Gyroscope raw measurement on Y axis
 * @param[out] gyro_z Gyroscope raw measurement on Z axis
 * @param[out] temperature Temperature raw measurement
 * @param[out] mag_x Magnetometer raw measurement on X axis
 * @param[out] mag_y Magnetometer raw measurement on Y axis
 * @param[out] mag_z Magnetometer raw measurement on Z axis
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_sensors(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z, int16_t &gyro_x, int16_t &gyro_y, int16_t &gyro_z, int16_t &temperature, int16_t &mag_x, int16_t &mag_y, int16_t &mag_z) {
    static uint8_t data[20];

    /* Read twenty raw data registers into a data array */
    read_register(ICM20948_REG_ACCEL_XOUT_H_SH, 20, &data[0]);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    accel_x = ((int16_t) data[0] << 8) | data[1];
    accel_y = ((int16_t) data[2] << 8) | data[3];
    accel_z = ((int16_t) data[4] << 8) | data[5];
    gyro_x = ((int16_t) data[6] << 8) | data[7];
    gyro_y = ((int16_t) data[8] << 8) | data[9];
    gyro_z = ((int16_t) data[10] << 8) | data[11];
    temperature = ((int16_t) data[12] << 8) | data[13];
    mag_x = ((int16_t) data[14] << 8) | data[15];
    mag_y = ((int16_t) data[16] << 8) | data[17];
    mag_z = ((int16_t) data[18] << 8) | data[19];

    return true;
}

/** Read gyroscope raw data
 *
 * @param[out] gyro_x Gyroscope raw measurement on X axis
 * @param[out] gyro_y Gyroscope raw measurement on Y axis
 * @param[out] gyro_z Gyroscope raw measurement on Z axis
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_gyro(int16_t &gyro_x, int16_t &gyro_y, int16_t &gyro_z) {
    static uint8_t data[6];

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_GYRO_XOUT_H_SH, 6, &data[0]);

    /* Convert the MSB and LSB into a signed 16-bit value */
    gyro_x = ((int16_t) data[0] << 8) | data[1];
    gyro_y = ((int16_t) data[2] << 8) | data[3];
    gyro_z = ((int16_t) data[4] << 8) | data[5];

    return true;
}

/** Read accelerometer raw data
 *
 * @param[out] accel_x Accelerometer raw measurement on X axis
 * @param[out] accel_y Accelerometer raw measurement on Y axis
 * @param[out] accel_z Accelerometer raw measurement on Z axis
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_accel(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z) {
    static uint8_t data[6];

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_ACCEL_XOUT_H_SH, 6, &data[0]);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    accel_x = ((int16_t) data[0] << 8) | data[1];
    accel_y = ((int16_t) data[2] << 8) | data[3];
    accel_z = ((int16_t) data[4] << 8) | data[5];

    return true;
}

/** Read magnetometer raw data
 *
 * @param[out] mag_x Magnetometer raw measurement on X axis
 * @param[out] mag_y Magnetometer raw measurement on Y axis
 * @param[out] mag_z Magnetometer raw measurement on Z axis
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_mag(int16_t &mag_x, int16_t &mag_y, int16_t &mag_z) {
    static uint8_t data[6];

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_EXT_SLV_SENS_DATA_00, 6, &data[0]);

    /* Convert the MSB and LSB into a signed 16-bit value */
    mag_x = ((int16_t) data[0] << 8) | data[1];
    mag_y = ((int16_t) data[2] << 8) | data[3];
    mag_z = ((int16_t) data[4] << 8) | data[5];

    return true;
}

/** Read temperature raw data
 *
 * @param[out]
 *    Temperature raw measurement
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_temperature(int16_t &temperature) {
    static uint8_t data[2];

    /* Read two raw data registers into a data array */
    read_register(ICM20948_REG_TEMPERATURE_H, 2, &data[0]);

    /* Convert the MSB and LSB into a signed 16-bit value */
    temperature = ((int16_t) data[0] << 8) | data[1];

    return true;
}

/** Read gyroscope in deg/s
 *
 * @param[out] gyr_x Gyroscope measurement on X axis in deg/s
 * @param[out] gyr_y Gyroscope measurement on Y axis in deg/s
 * @param[out] gyr_z Gyroscope measurement on Z axis in deg/s
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_gyro_dps(float &gyro_x_dps, float &gyro_y_dps, float &gyro_z_dps) {
    static int16_t gyro_x, gyro_y, gyro_z;
    
    read_gyro(gyro_x, gyro_y, gyro_z);

    /* Multiply the raw measurements with the resolution to transform them into dps */
    gyro_x_dps = (float) gyro_x * m_gyroRes;
    gyro_y_dps = (float) gyro_y * m_gyroRes;
    gyro_z_dps = (float) gyro_z * m_gyroRes;
    
    return true;
}

/** Read accelerometer in g
 *
 * @param[out] accel_x Accelerometer measurement on X axis in g
 * @param[out] accel_y Accelerometer measurement on Y axis in g
 * @param[out] accel_z Accelerometer measurement on Z axis in g
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_accel_g(float &accel_x_g, float &accel_y_g, float &accel_z_g) {
    static int16_t accel_x, accel_y, accel_z;
    
    read_accel(accel_x, accel_y, accel_z);
    
    /* Multiply the raw measurements with the resolution to transform them into g */
    accel_x_g = (float) accel_x * m_accelRes;
    accel_y_g = (float) accel_y * m_accelRes;
    accel_z_g = (float) accel_z * m_accelRes;
    
    return true;
}

/** Read temperature in Celsius
 *
 * @param [out] temperature_c
 *      Temperature measurement in Celsius
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_temperature_c(float &temperature_c) {
    static int16_t temperature;

    read_temperature(temperature);
    
    /* Transform the raw measurement into Celsius */
    temperature_c = ( (float) temperature / 333.87f) + 21.0f;

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
void ICM20948_SPI::read_register(uint16_t addr, uint8_t numBytes, uint8_t *data) {
    static uint8_t regAddr;
    static uint8_t bank;

    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);
    
    m_port.beginTransaction(SPISettings(M_CLOCK, M_BIT_ORDER, M_DATA_MODE));
    
    /* Enable chip select */
    digitalWrite(M_CS_PIN, LOW);
    
    /* Transfer address with read-bit set */
    m_port.transfer(regAddr | 0x80);
    /* Receive data array */
    m_port.transfer(data, numBytes);

    /* Disable chip select */
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();

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
void ICM20948_SPI::write_register(uint16_t addr, uint8_t data) {
    static uint8_t regAddr;
    static uint8_t bank;
    
    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);
    
    m_port.beginTransaction(SPISettings(M_CLOCK, M_BIT_ORDER, M_DATA_MODE));
    
    /* Enable chip select */
    digitalWrite(M_CS_PIN, LOW);
    
    /* Transfer address without read-bit set */
    m_port.transfer(regAddr);
    /* Send data byte */
    m_port.transfer(data);
    
    /* Disable chip select */
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();

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
void ICM20948_SPI::select_bank(uint8_t bank) {
    static uint8_t bank_old = 255;
    
    /* Select bank if it has changed */
    if (bank != bank_old) {
        m_port.beginTransaction(SPISettings(M_CLOCK, M_BIT_ORDER, M_DATA_MODE));
        
        /* Enable chip select */
        digitalWrite(M_CS_PIN, LOW);
        
        /* Transfer address without read-bit set */
        m_port.transfer(ICM20948_REG_BANK_SEL);
        /* Send data byte */
        m_port.transfer(bank << 4);
        
        /* Disable chip select */
        digitalWrite(M_CS_PIN, HIGH);
        
        m_port.endTransaction();
        
        bank_old = bank;
    }
    return;
}

/***************************************************************************//**
 * @brief
 *    Sets desired magnetometer transfer mode
 *
 * @param[in] read
 *    'true' sets transfer mode to read,
 *    'false' sets transfer mode to write.
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
    delay(1000);
    
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
 *    Perform ICM20948 soft reset
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *    Set gyroscope sample rate divider
 *
 *    gyroSampleRate = 1125Hz / (1 + gyroDiv)
 *    gyroSampleRate = 9000 Hz for ICM20948_GYRO_BW_12100HZ
 *
 * @param[in] gyroDiv
 *    Gyroscope sample rate divider
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::set_gyro_sample_rate_div(uint8_t gyroDiv) {
    /* Write value to register */
    write_register(ICM20948_REG_GYRO_SMPLRT_DIV, gyroDiv);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set accelerometer sample rate divider
 *
 *    accelSampleRate = 1125Hz / (1 + accelDiv)
 *    accelSampleRate = 4500 Hz for ICM20948_ACCEL_BW_1210HZ
 *
 * @param[in] accelDiv
 *    Accelerometer sample rate divider
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::set_accel_sample_rate_div(uint16_t accelDiv) {
    /* Check if it fits inside divider registers */
    if ( accelDiv > 4095 ) {
        accelDiv = 4095;
    }

    /* Write value to registers */
    write_register(ICM20948_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accelDiv >> 8));
    write_register(ICM20948_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF));

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set gyroscope bandwidth
 *
 * @param[in] gyroBw
 *    Desired bandwidth value. Use ICM20948_GYRO_BW macros.
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *    Set gyroscope full scale
 *
 * @param[in] gyroFs
 *    Desired full scale value. Use ICM20948_GYRO_FULLSCALE macros.
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::set_gyro_fullscale(uint8_t gyroFs) {
    uint8_t reg;
    
    /* Calculate and save gyro resolution */
    switch ( gyroFs ) {
        case ICM20948_GYRO_FULLSCALE_250DPS:
            m_gyroRes = 250.0f / 32768.0f;
            break;
        case ICM20948_GYRO_FULLSCALE_500DPS:
            m_gyroRes = 500.0f / 32768.0f;
            break;
        case ICM20948_GYRO_FULLSCALE_1000DPS:
            m_gyroRes = 1000.0f / 32768.0f;
            break;
        case ICM20948_GYRO_FULLSCALE_2000DPS:
            m_gyroRes = 2000.0f / 32768.0f;
            break;
        default:
            return ERROR;
    }
        
    gyroFs &= ICM20948_MASK_GYRO_FULLSCALE;
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM20948_MASK_GYRO_FULLSCALE);
    reg |= gyroFs;
    write_register(ICM20948_REG_GYRO_CONFIG_1, reg);

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
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::set_accel_fullscale(uint8_t accelFs) {
    uint8_t reg;

    /* Calculate and save accel resolution */
    switch ( accelFs ) {
        case ICM20948_ACCEL_FULLSCALE_2G:
            m_accelRes = 2.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_4G:
            m_accelRes = 4.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_8G:
            m_accelRes = 8.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_16G:
            m_accelRes = 16.0f / 32768.0f;
            break;
        default:
            return ERROR;
    }
        
    accelFs &= ICM20948_MASK_ACCEL_FULLSCALE;
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM20948_MASK_ACCEL_FULLSCALE);
    reg |= accelFs;
    write_register(ICM20948_REG_ACCEL_CONFIG, reg);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set magnetometer mode
 *
 * @param[in] accelFs
 *    Desired magnetometer mode. Use AK09916_MODE macros.
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::set_mag_mode(uint8_t magMode){
    switch ( magMode ) {
        case AK09916_BIT_MODE_POWER_DOWN:
            break;
        case AK09916_MODE_SINGLE:
            break;
        case AK09916_MODE_10HZ:
            break;
        case AK09916_MODE_20HZ:
            break;
        case AK09916_MODE_50HZ:
            break;
        case AK09916_MODE_100HZ:
            break;
        case AK09916_MODE_ST:
            break;
        default:
            return ERROR;
    }
    
    write_register(AK09916_REG_CONTROL_2, magMode);
    
    return OK;
}

/***************************************************************************//**
 * @brief
 *    Get gyroscope resolution
 *
 * @param[out] gyroRes
 *    Resolution in (deg/s)/bit
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::get_gyro_resolution(float *gyroRes) {
    uint8_t reg;

    /* Read gyroscope full scale setting */
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ICM20948_MASK_GYRO_FULLSCALE;

    /* Calculate gyro resolution */
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
        default:
            return ERROR;
    }

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Get accelerometer resolution
 *
 * @param[out] accelRes
 *    Resolution in g/bit
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::get_accel_resolution(float *accelRes) {
    uint8_t reg;

    /* Read acceleration full scale setting */
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ICM20948_MASK_ACCEL_FULLSCALE;

    /* Calculate accel resolution */
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
        default:
            return ERROR;
    }

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
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::enable_lowpowermode(bool enAccel, bool enGyro, bool enTemp) {
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
    } 
    else {
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
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::enable_wake_on_motion(bool enable, uint8_t womThreshold, uint16_t accelDiv) {
    if ( enable ) {
        /* Make sure that chip is not in sleep */
        enable_sleepmode(false);

        /* And in continuous mode */
        enable_cyclemode(false);

        /* Enable accelerometer only */
        enable_sensor(true, false, false);

        /* Set sample rate */
        set_accel_sample_rate_div(accelDiv);

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
        enable_lowpowermode(true, false, false);
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
 *    Measured gyro sensor bias in deg/s
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
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

    /* Set sample rates */
    set_gyro_sample_rate_div(0);
    set_accel_sample_rate_div(0);

    /* 246Hz BW for accelerometer and 200Hz for gyroscope */
    set_gyro_bandwidth(ICM20948_GYRO_BW_12HZ);
    set_accel_bandwidth(ICM20948_ACCEL_BW_246HZ);

    /* Set most sensitive range: 2G full scale and 250dps full scale */
    set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_250DPS);
    set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_2G);

    /* Retrieve resolution per bit */
    get_gyro_resolution(&gyroRes);
    get_accel_resolution(&accelRes);

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

    /* Convert values to deg/s for displaying */
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
    /* Subtract from stored calibration value */
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
    /* compensation calculations(? the data sheet is not clear). Accelerometer bias registers expect bias input */
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
    /* Clear last bit (temperature compensation? - the data sheet is not clear) */
    /* Subtract from factory calibration value */

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
 *    Measured gyro sensor bias in deg/s
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
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

    /* Set gyro sample rate */
    set_gyro_sample_rate_div(0);

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

    /* Convert values to deg/s for displaying */
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
    /* Subtract from stored calibration value */
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
 *    Enables or disables interrupts
 *
 * @param[in] dataReadyEnable
 *    If true enables Raw Data Ready interrupt, otherwise disables.
 *
 * @param[in] womEnable
 *    If true enables Wake-up On Motion interrupt, otherwise disables.
 *
 * @return
 *   'OK' if successful,
 *   'ERROR' on error.
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
 *   'OK' if successful,
 *   'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::read_irqstatus(uint32_t *int_status) {
    static uint8_t reg[4];

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
 *   'true' if Raw Data Ready interrupt bit set,
 *   'false' otherwise.
 ******************************************************************************/
bool ICM20948::is_data_ready(void) {
    static uint8_t status;
    static bool ready;

    ready = false;
    read_register(ICM20948_REG_INT_STATUS_1, 1, &status);

    if ( status & ICM20948_BIT_RAW_DATA_0_RDY_INT ) {
        ready = true;
    }

    return ready;
}

/** Create an ICM20948_SPI object connected to specified SPI pins and with specified SPI settings for clock, bit order and data mode
 *
 * @param[in] cs_pin    SPI chip select pin.
 * @param[in] spi_port  SPI port.
 * @param[in] clock     SPI clock.
 * @param[in] bit_order SPI bit order.
 * @param[in] data_mode SPI data mode.
 * 
 */
ICM20948_SPI::ICM20948_SPI(uint8_t cs_pin, SPIClass &port, uint32_t clock, uint8_t bit_order, uint8_t data_mode) : M_CS_PIN(cs_pin), m_port(port), M_CLOCK(clock), M_BIT_ORDER(bit_order), M_DATA_MODE(data_mode) {
    /* setup chip select pin */
    pinMode(M_CS_PIN, OUTPUT);
    digitalWrite(M_CS_PIN, HIGH);
}

/**
 * ICM20948_SPI destructor
 */
ICM20948_SPI::~ICM20948_SPI(void) {
}