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

/* factor for converting a radian number to an equivalent number in degrees */
const float RAD2DEG = (float) 4068 / 71;

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT2(x,y) Serial.print(x,y)
#define DEBUG_PRINTLN2(x,y) Serial.println(x,y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT2(x,y)
#define DEBUG_PRINTLN2(x,y)
#endif

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
 * @param[in] gyroOffset_1000dps_xyz Gyroscope XYZ axis offsets in current full scale format.
 * @param[in] accelOffset_32g_xyz Accelerometer XYZ axis offsets in current full scale format.
 * @param[in] magOffset_xyz Magnetometer XYZ axis hard iron distortion correction.
 * @param[in] magScale_xyz Magnetometer XYZ axis soft iron distortion correction.
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::init(int16_t *gyroOffset_1000dps_xyz, int16_t *accelOffset_32g_xyz, float *magOffset_xyz, float *magScale_xyz) {
    uint8_t data[1];
    
    /* Reset ICM20948 */
    //reset();
    
    /* Auto select best available clock source PLL if ready, else use internal oscillator */
    write_register(ICM20948_REG_PWR_MGMT_1, ICM20948_BIT_CLK_PLL);
     
    /* PLL startup time - no spec in data sheet */
    delay(10);
    
    /* Reset I2C Slave module and use SPI */
    /* Enable I2C Master I/F module */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_I2C_IF_DIS | ICM20948_BIT_I2C_MST_EN);
    
    /* Set I2C Master clock frequency */
    write_register(ICM20948_REG_I2C_MST_CTRL, ICM20948_I2C_MST_CTRL_CLK_400KHZ);
    
    /* Read ICM20948 "Who am I" register */
    read_register(ICM20948_REG_WHO_AM_I, 1, data);
    
    /* Check if "Who am I" register was successfully read */
    if (data[0] != ICM20948_DEVICE_ID) {
        return false;
    }
    
    /* Disable bypass for I2C Master interface pins */
    enable_irq(false, false);
    
    /* Read AK09916 "Who am I" register */
    read_mag_register(AK09916_REG_WHO_AM_I, 1, data);
    
    /* Check if AK09916 "Who am I" register was successfully read */
    if (data[0] != AK09916_DEVICE_ID) {
        return false;
    }
     
    // TODO: odr_align_en to sync sample rates seems not to be necessary, at least for maximum sample rates.
    
    /* Configure accelerometer */
    set_accel_bandwidth(ICM20948_ACCEL_BW_6HZ);
    set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_8G);
    //set_accel_sample_rate_div(...);    /* the accelerometer sample rate is 4500 Hz for ICM20948_ACCEL_BW_1210HZ */
    
    /* Configure gyroscope */
    set_gyro_bandwidth(ICM20948_GYRO_BW_12100HZ);
    set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_1000DPS);
    //set_gyro_sample_rate_div(...);    /* the gyroscope sample rate is 9000 Hz for ICM20948_GYRO_BW_12100HZ */
    
    /* Configure magnetometer */
    set_mag_mode(AK09916_MODE_100HZ);
    
    /* Apply calibration data */
    set_gyro_offsets(gyroOffset_1000dps_xyz);
    set_accel_offsets(accelOffset_32g_xyz);
    m_magOffset_xyz[0] = magOffset_xyz[0]; m_magOffset_xyz[1] = magOffset_xyz[1]; m_magOffset_xyz[2] = magOffset_xyz[2];
    m_magScale_xyz[0] = magScale_xyz[0]; m_magScale_xyz[1] = magScale_xyz[1]; m_magScale_xyz[2] = magScale_xyz[2];

    DEBUG_PRINTLN(F("Gyroscope offsets:"));
    DEBUG_PRINT("g:\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[0]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[1]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[2]);
    DEBUG_PRINTLN();

    DEBUG_PRINTLN(F("Accelerometer offsets:"));
    DEBUG_PRINT("a:\t");
    DEBUG_PRINT(accelOffset_32g_xyz[0]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(accelOffset_32g_xyz[1]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(accelOffset_32g_xyz[2]);
    DEBUG_PRINTLN();

    DEBUG_PRINTLN(F("Magnetometer offsets:"));
    DEBUG_PRINT2(magOffset_xyz[0], 1);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magOffset_xyz[1], 1);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magOffset_xyz[2], 1);
    DEBUG_PRINT("\t");
    DEBUG_PRINTLN();

    DEBUG_PRINTLN(F("Magnetometer scalings:"));
    DEBUG_PRINT2(magScale_xyz[0], 4);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magScale_xyz[1], 4);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magScale_xyz[2], 4);
    DEBUG_PRINTLN();
    
    /* Read the magnetometer ST2 register, because else the data is not updated */
    read_mag_register(AK09916_REG_STATUS_2, 1, data);
    
    /* Enable Raw Data Ready interrupt */
    enable_irq(true, false);
    
    return true;
}

/** Read accelerometer resolution
 *
 * @param[out] accelRes Accelerometer resolution in g/bit
 *
 */
void ICM20948::read_accelRes(float &accelRes){
    accelRes = m_accelRes;
}

/** Read accelerometer and gyroscope values
 *
 * @param[out] ax Accelerometer X axis value
 * @param[out] ay Accelerometer Y axis value
 * @param[out] az Accelerometer Z axis value
 * @param[out] gx Gyroscope X axis value
 * @param[out] gy Gyroscope Y axis value
 * @param[out] gz Gyroscope Z axis value
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    static uint8_t data[12];
    
    /* Read accelerometer and gyroscope raw data into a data array */
    read_register(ICM20948_REG_ACCEL_XOUT_H_SH, 12, data);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    ax = ((int16_t) data[0] << 8) | data[1];
    ay = ((int16_t) data[2] << 8) | data[3];
    az = ((int16_t) data[4] << 8) | data[5];
    gx = ((int16_t) data[6] << 8) | data[7];
    gy = ((int16_t) data[8] << 8) | data[9];
    gz = ((int16_t) data[10] << 8) | data[11];
    
    return true;
}

/** Read accelerometer values
 *
 * @param[out] ax Accelerometer X axis value
 * @param[out] ay Accelerometer Y axis value
 * @param[out] az Accelerometer Z axis value
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_accel(int16_t &ax, int16_t &ay, int16_t &az) {
    static uint8_t data[6];

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_ACCEL_XOUT_H_SH, 6, data);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    ax = ((int16_t) data[0] << 8) | data[1];
    ay = ((int16_t) data[2] << 8) | data[3];
    az = ((int16_t) data[4] << 8) | data[5];

    return true;
}

/** Read gyroscope values
 *
 * @param[out] gx Gyroscope X axis value
 * @param[out] gy Gyroscope Y axis value
 * @param[out] gz Gyroscope Z axis value
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_gyro(int16_t &gx, int16_t &gy, int16_t &gz) {
    static uint8_t data[6];

    /* Read six raw data registers into a data array */
    read_register(ICM20948_REG_GYRO_XOUT_H_SH, 6, data);

    /* Convert the MSB and LSB into a signed 16-bit value */
    gx = ((int16_t) data[0] << 8) | data[1];
    gy = ((int16_t) data[2] << 8) | data[3];
    gz = ((int16_t) data[4] << 8) | data[5];

    return true;
}

/** Read magnetometer values
 *
 * @param[out] mx Magnetometer X axis value
 * @param[out] my Magnetometer Y axis value
 * @param[out] mz Magnetometer Z axis value
 *
 * @return
 *   'true' if new data,
 *   'false' else.
 */
bool ICM20948::read_mag(int16_t &mx, int16_t &my, int16_t &mz) {
    static uint8_t status = 0;
    static uint32_t t_start;
    static uint8_t data[6];
    
    switch (status) {
        case 0:     /* Request AK09916C status_1 */
            /* Set ICM20948 SLV0_REG to AK09916C status_1 address */
            write_register(ICM20948_REG_I2C_SLV0_REG, AK09916_REG_STATUS_1);
            /* Request AK09916C status_1 */
            write_register(ICM20948_REG_I2C_SLV0_CTRL, ICM20948_BIT_I2C_SLV_EN | 1);
        
            t_start = micros();
            status = 1;
            break;
        case 1:     /* Read AK09916C status_1 from ICM20948 to check if data is ready */
            /* Wait for ICM20948 registers to fill with AK09916C status_1 data */
            if ((micros() - t_start) > 1000) {
                /* Read AK09916C status_1 from ICM20948 EXT_SLV_SENS_DATA registers */
                read_register(ICM20948_REG_EXT_SLV_SENS_DATA_00, 1, data);
                
                /* Check AK09916C status_1 for data ready */
                if ((data[0] & AK09916_BIT_DRDY) == AK09916_BIT_DRDY) {
                    t_start = micros();
                    status = 2;
                }
            }
            break;
        case 2:     /* Request AK09916C measurement data */
            /* Set ICM20948 SLV0_REG to AK09916C measurement data address */
            write_register(ICM20948_REG_I2C_SLV0_REG, AK09916_REG_HXL);
            /* Request AK09916C measurement data */
            write_register(ICM20948_REG_I2C_SLV0_CTRL, ICM20948_BIT_I2C_SLV_EN | 6);
            
            t_start = micros();
            status = 3;
            break;
        case 3:     /* Read AK09916C measurement data from ICM20948 */
            /* Wait for ICM20948 registers to fill with AK09916C measurement data */
            if ((micros() - t_start) > 1000) {
                /* Read AK09916C measurement data from ICM20948 EXT_SLV_SENS_DATA registers */
                read_register(ICM20948_REG_EXT_SLV_SENS_DATA_00, 6, data);
                
                /* Convert the LSB and MSB into a signed 16-bit value */
                mx = ((int16_t) data[1] << 8) | data[0];
                my = ((int16_t) data[3] << 8) | data[2];
                mz = ((int16_t) data[5] << 8) | data[4];
                
                /* Transform magnetometer values to match the coordinate system of accelerometer and gyroscope */
                my = -my;
                mz = -mz;                
                
                /* Apply hard and soft iron distortion correction */
                mx = ((mx + m_magOffset_xyz[0]) * m_magScale_xyz[0]) + 0.5;
                my = ((my + m_magOffset_xyz[1]) * m_magScale_xyz[1]) + 0.5;
                mz = ((mz + m_magOffset_xyz[2]) * m_magScale_xyz[2]) + 0.5;
                
                status = 4;
                return true;
            }
            break;
        case 4:    /* Request AK09916C status_2 to indicate that data is read and allow AK09916C to update the measurement data */
            /* Set ICM20948 SLV0_REG to AK09916C status_2 address */
            write_register(ICM20948_REG_I2C_SLV0_REG, AK09916_REG_STATUS_2);
            /* Request AK09916C status_1 */
            write_register(ICM20948_REG_I2C_SLV0_CTRL, ICM20948_BIT_I2C_SLV_EN | 1);
            
            t_start = micros();
            status = 5;
            break;
        case 5:     /* Wait for AK09916C status_2 request from ICM20948 */
            if ((micros() - t_start) > 1000) {
                status = 0;
            }
            break;           
        default:
            status = 0;
            break;
    }
    
    return false;
}

/** Read temperature value
 *
 * @param[out] temperature Temperature value
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_temperature(int16_t &temperature) {
    static uint8_t data[2];

    /* Read two raw data registers into a data array */
    read_register(ICM20948_REG_TEMPERATURE_H, 2, data);

    /* Convert the MSB and LSB into a signed 16-bit value */
    temperature = ((int16_t) data[0] << 8) | data[1];

    return true;
}

/** Read accelerometer in g and gyroscope in deg/s
 *
 * @param[out] ax_g Accelerometer X axis value in g
 * @param[out] ay_g Accelerometer Y axis value in g
 * @param[out] az_g Accelerometer Z axis value in g
 * @param[out] gx_dps Gyroscope X axis value in deg/s
 * @param[out] gy_dps Gyroscope Y axis value in deg/s
 * @param[out] gz_dps Gyroscope Z axis value in deg/s
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_accel_gyro_g_dps(float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps, float &gz_dps) {
    static int16_t ax, ay, az, gx, gy, gz;
    
    read_accel_gyro(ax, ay, az, gx, gy, gz);

    /* Multiply the accelerometer values with their resolution to transform them into g */
    ax_g = (float) ax * m_accelRes;
    ay_g = (float) ay * m_accelRes;
    az_g = (float) az * m_accelRes;
    
    /* Multiply the gyroscope values with their resolution to transform them into deg/s */
    gx_dps = (float) gx * m_gyroRes;
    gy_dps = (float) gy * m_gyroRes;
    gz_dps = (float) gz * m_gyroRes;
    
    return true;
}

/** Read accelerometer in g
 *
 * @param[out] ax_g Accelerometer X axis value in g
 * @param[out] ay_g Accelerometer Y axis value in g
 * @param[out] az_g Accelerometer Z axis value in g
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_accel_g(float &ax_g, float &ay_g, float &az_g) {
    static int16_t ax, ay, az;
    
    read_accel(ax, ay, az);
    
    /* Multiply the values with the resolution to transform them into g */
    ax_g = (float) ax * m_accelRes;
    ay_g = (float) ay * m_accelRes;
    az_g = (float) az * m_accelRes;
    
    return true;
}

/** Read gyroscope in deg/s
 *
 * @param[out] gx_dps Gyroscope X axis value in deg/s
 * @param[out] gy_dps Gyroscope Y axis value in deg/s
 * @param[out] gz_dps Gyroscope Z axis value in deg/s
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_gyro_dps(float &gx_dps, float &gy_dps, float &gz_dps) {
    static int16_t gx, gy, gz;
    
    read_gyro(gx, gy, gz);

    /* Multiply the values with the resolution to transform them into deg/s */
    gx_dps = (float) gx * m_gyroRes;
    gy_dps = (float) gy * m_gyroRes;
    gz_dps = (float) gz * m_gyroRes;
    
    return true;
}

/** Read magnetometer in uT
 *
 * @param[out] mag_x_ut Magnetometer X axis value in uT
 * @param[out] mag_y_ut Magnetometer Y axis value in uT
 * @param[out] mag_z_ut Magnetometer Z axis value in uT
 *
 * @return
 *   'true' if new data,
 *   'false' else.
 */
bool ICM20948::read_mag_ut(float &mag_x_ut, float &mag_y_ut, float &mag_z_ut) {
    static int16_t mx, my, mz;
    static bool new_mag;
    
    new_mag = read_mag(mx, my, mz);
    
    /* Multiply the values with the resolution to transform them into uT */
    mag_x_ut = (float) mx * m_magRes;
    mag_y_ut = (float) my * m_magRes;
    mag_z_ut = (float) mz * m_magRes;
    
    return new_mag;
}

/** Read temperature in Celsius
 *
 * @param [out] temperature_c Temperature value in Celsius
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_temperature_c(float &temperature_c) {
    static int16_t temperature;

    read_temperature(temperature);
    
    /* Transform the value into Celsius */
    temperature_c = ( (float) temperature / 333.87f) + 21.0f;

    return true;
}

/** Read accelerometer values and gyroscope in rad/s
 *
 * @param[out] ax Accelerometer X axis value
 * @param[out] ay Accelerometer Y axis value
 * @param[out] az Accelerometer Z axis value
 * @param[out] gx_rps Gyroscope X axis value in rad/s
 * @param[out] gy_rps Gyroscope Y axis value in rad/s
 * @param[out] gz_rps Gyroscope Z axis value in rad/s
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_accel_gyro_rps(int16_t &ax, int16_t &ay, int16_t &az, float &gx_rps, float &gy_rps, float &gz_rps) {
    static int16_t gx, gy, gz;
    
    read_accel_gyro(ax, ay, az, gx, gy, gz);
    
    /* Multiply the gyroscope values with their resolution to transform them into rad/s */
    gx_rps = (float) gx * m_gyroRes_rad;
    gy_rps = (float) gy * m_gyroRes_rad;
    gz_rps = (float) gz * m_gyroRes_rad;
    
    return true;
}

/** read gyroscope in rad/s
 *
 * @param[out] gx_rps Gyroscope X axis value in rad/s
 * @param[out] gy_rps Gyroscope Y axis value in rad/s
 * @param[out] gz_rps Gyroscope Z axis value in rad/s
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::read_gyro_rps(float &gx_rps, float &gy_rps, float &gz_rps) {
    static int16_t gx, gy, gz;
    
    read_gyro(gx, gy, gz);

    /* Multiply the values with the radian resolution to transform them into rad/s */
    gx_rps = (float) gx * m_gyroRes_rad;
    gy_rps = (float) gy * m_gyroRes_rad;
    gz_rps = (float) gz * m_gyroRes_rad;
    
    return true;
}

/** Reset accelerometer and gyroscope offsets to factory defaults
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::reset_accel_gyro_offsets(){
    /* set factory default offsets */
    set_accel_offsets((const int16_t[]) {32513, 2081, 32679});
    set_gyro_offsets((const int16_t[]) {0, 0, 0});
    
    /*set_accel_offsets((const int16_t[]) {1625, 749, 32592});
    set_gyro_offsets((const int16_t[]) {0, 0, 0});*/
    
    return true;
}

/** Accelerometer and gyroscope calibration function. Get accelerometer and
 *  gyroscope mean values, while device is at rest and in level. Those
 *  are then loaded into ICM20948 bias registers to remove the static 
 *  offset error.
 *
 * @param[in] imuInterrupt imu interrupt flag
 * @param[in] time_s Time period in seconds for mean value calculation
 * @param[in] accel_tolerance_32g Maximum accelerometer mean value deviation from target value in 32g full scale format. The accelerometer
 * target values in x and y direction are zero and in z direction it is the acceleration due to gravity.
 * @param[in] gyro_tolerance_1000dps Maximum gyroscope mean value deviation from zero after calibration at 1000dps full scale
 * @param[out] accelOffset_32g_xyz Accelerometer XYZ axis offsets in current full scale format.
 * @param[out] gyroOffset_1000dps_xyz Gyroscope XYZ axis offsets in current full scale format.
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::calibrate_accel_gyro(volatile bool &imuInterrupt, float time_s, int32_t accel_tolerance_32g, int32_t gyro_tolerance_1000dps, int16_t *accelOffset_32g_xyz, int16_t *gyroOffset_1000dps_xyz) {
    /* Scale factor to convert accelerometer values into 32g full scale */
    float accel_offset_scale = (m_accelRes * 32768.0f) / 32;
    /* Scale factor to convert gyroscope values into 1000dps full scale */
    float gyro_offset_scale = (m_gyroRes * 32768.0f) / 1000;
    
    /* Accelerometer tolerance in current full scale format */
    int32_t accel_tolerance = accel_tolerance_32g * accel_offset_scale + 0.5;
    /* Gyroscope tolerance in current full scale format */
    int32_t gyro_tolerance = gyro_tolerance_1000dps * gyro_offset_scale + 0.5;
    
    int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int32_t offset_ax, offset_ay, offset_az, offset_gx, offset_gy, offset_gz;
    
    DEBUG_PRINTLN(F("Calibrating accelerometer and gyroscope. Keep device at rest and in level ..."));
    
    get_accel_offsets(accelOffset_32g_xyz);
    get_gyro_offsets(gyroOffset_1000dps_xyz);
    
    /* Convert offsets to the current accelerometer and gyroscope full scale settings */
    offset_ax = accelOffset_32g_xyz[0] / accel_offset_scale + 0.5;
    offset_ay = accelOffset_32g_xyz[1] / accel_offset_scale + 0.5;
    offset_az = accelOffset_32g_xyz[2] / accel_offset_scale + 0.5;
    offset_gx = gyroOffset_1000dps_xyz[0] / gyro_offset_scale + 0.5;
    offset_gy = gyroOffset_1000dps_xyz[1] / gyro_offset_scale + 0.5;
    offset_gz = gyroOffset_1000dps_xyz[2] / gyro_offset_scale + 0.5;
    
    static uint16_t step = 0;
    while (1) {
        mean_accel_gyro(imuInterrupt, time_s, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);
        
        if ((abs(mean_ax) < accel_tolerance) &&
        (abs(mean_ay) < accel_tolerance) &&
        (abs(mean_az - m_g) < accel_tolerance) &&
        (abs(mean_gx) < gyro_tolerance) &&
        (abs(mean_gy) < gyro_tolerance) &&
        (abs(mean_gz) < gyro_tolerance)) {
            break;
        }
        
        offset_ax -= mean_ax;
        offset_ay -= mean_ay;
        offset_az -= mean_az - m_g;
        offset_gx -= mean_gx;
        offset_gy -= mean_gy;
        offset_gz -= mean_gz;
        
        /* Before writing the offsets to the registers, they need need to be converted to 32g accelerometer full scale and 1000dps gyroscope full scale */
        accelOffset_32g_xyz[0] = offset_ax * accel_offset_scale + 0.5;
        accelOffset_32g_xyz[1] = offset_ay * accel_offset_scale + 0.5;
        accelOffset_32g_xyz[2] = offset_az * accel_offset_scale + 0.5;
        gyroOffset_1000dps_xyz[0] = offset_gx * gyro_offset_scale + 0.5;
        gyroOffset_1000dps_xyz[1] = offset_gy * gyro_offset_scale + 0.5;
        gyroOffset_1000dps_xyz[2] = offset_gz * gyro_offset_scale + 0.5;
        
        set_accel_offsets(accelOffset_32g_xyz);
        set_gyro_offsets(gyroOffset_1000dps_xyz);
        
        step++;
    }
    
    DEBUG_PRINTLN(F("Updated internal accelerometer and gyroscope offsets:"));
    DEBUG_PRINT("a/g:\t");
    DEBUG_PRINT(accelOffset_32g_xyz[0]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(accelOffset_32g_xyz[1]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(accelOffset_32g_xyz[2]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[0]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[1]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[2]);
    DEBUG_PRINTLN();
    
    DEBUG_PRINTLN(F("Mean measurement error:"));
    DEBUG_PRINT("a/g:\t");
    DEBUG_PRINT(mean_ax);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_ay);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_az - m_g);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_gx);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_gy);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_gz);
    DEBUG_PRINTLN();
    
    DEBUG_PRINT(F("Step count:\t"));
    DEBUG_PRINT(step);
    DEBUG_PRINTLN();
    
    return true;
}

/** Gyroscope calibration function. Get gyroscope mean values, while device is at rest. 
 *  Those are then loaded into ICM20948 bias registers to remove the static offset error.
 *
 * @param[in] imuInterrupt imu interrupt flag
 * @param[in] time_s Time period in seconds for mean value calculation
 * @param[in] gyro_tolerance_1000dps Maximum gyroscope mean value deviation from zero in 1000dps full scale format
 * @param[out] gyroOffset_1000dps_xyz Gyroscope XYZ axis offsets in current full scale format.
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::calibrate_gyro(volatile bool &imuInterrupt, float time_s, int32_t gyro_tolerance_1000dps, int16_t *gyroOffset_1000dps_xyz) {
    /* Scale factor to convert gyroscope values into 1000dps full scale */
    float gyro_offset_scale = (m_gyroRes * 32768.0f) / 1000;
    
    /* Gyroscope tolerance in current full scale format */
    int32_t gyro_tolerance = gyro_tolerance_1000dps * gyro_offset_scale + 0.5;
    
    int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int32_t offset_gx, offset_gy, offset_gz;
    
    DEBUG_PRINTLN(F("Calibrating gyroscope. Keep device at rest ..."));
    
    get_gyro_offsets(gyroOffset_1000dps_xyz);
    
    /* Convert offsets to the current gyroscope full scale setting */
    offset_gx = gyroOffset_1000dps_xyz[0] / gyro_offset_scale + 0.5;
    offset_gy = gyroOffset_1000dps_xyz[1] / gyro_offset_scale + 0.5;
    offset_gz = gyroOffset_1000dps_xyz[2] / gyro_offset_scale + 0.5;
    
    static uint16_t step = 0;
    while (1) {
        mean_accel_gyro(imuInterrupt, time_s, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);
        
        if ((abs(mean_gx) < gyro_tolerance) &&
        (abs(mean_gy) < gyro_tolerance) &&
        (abs(mean_gz) < gyro_tolerance)) {
            break;
        }
        
        offset_gx -= mean_gx;
        offset_gy -= mean_gy;
        offset_gz -= mean_gz;
        
        /* Before writing the offsets to the registers, they need need to be converted to 1000dps gyroscope full scale */
        gyroOffset_1000dps_xyz[0] = offset_gx * gyro_offset_scale + 0.5;
        gyroOffset_1000dps_xyz[1] = offset_gy * gyro_offset_scale + 0.5;
        gyroOffset_1000dps_xyz[2] = offset_gz * gyro_offset_scale + 0.5;
        
        set_gyro_offsets(gyroOffset_1000dps_xyz);
        
        step++;
    }
    
    DEBUG_PRINTLN(F("Updated internal gyroscope offsets:"));
    DEBUG_PRINT("g:\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[0]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[1]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(gyroOffset_1000dps_xyz[2]);
    DEBUG_PRINTLN();
    
    DEBUG_PRINTLN(F("Mean measurement error:"));
    DEBUG_PRINT("g:\t");
    DEBUG_PRINT(mean_gx);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_gy);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_gz);
    DEBUG_PRINTLN();
    
    DEBUG_PRINT(F("Step count:\t"));
    DEBUG_PRINT(step);
    DEBUG_PRINTLN();
    
    return true;
}

/** Accelerometer calibration function. Get accelerometer mean values, while device is at rest and in level.
 *  Those are then loaded into ICM20948 bias registers to remove the static offset error.
 *
 * @param[in] imuInterrupt imu interrupt flag
 * @param[in] time_s Time period in seconds for mean value calculation
 * @param[in] accel_tolerance_32g Maximum accelerometer mean value deviation from target value in 32g full scale format. The accelerometer
 * target values in x and y direction are zero and in z direction it is the acceleration due to gravity.
 * @param[out] accelOffset_32g_xyz Accelerometer XYZ axis offsets in current full scale format.
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::calibrate_accel(volatile bool &imuInterrupt, float time_s, int32_t accel_tolerance_32g, int16_t *accelOffset_32g_xyz) {
    /* Scale factor to convert accelerometer values into 32g full scale */
    float accel_offset_scale = (m_accelRes * 32768.0f) / 32;
    
    /* Accelerometer tolerance in current full scale format */
    int32_t accel_tolerance = accel_tolerance_32g * accel_offset_scale + 0.5;
    
    int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int32_t offset_ax, offset_ay, offset_az;
    
    DEBUG_PRINTLN(F("Calibrating accelerometer. Keep device at rest and in level ..."));
    
    get_accel_offsets(accelOffset_32g_xyz);
    
    /* Convert offsets to the current accelerometer full scale settings */
    offset_ax = accelOffset_32g_xyz[0] / accel_offset_scale + 0.5;
    offset_ay = accelOffset_32g_xyz[1] / accel_offset_scale + 0.5;
    offset_az = accelOffset_32g_xyz[2] / accel_offset_scale + 0.5;
      
    static uint16_t step = 0;
    while (1) {
        mean_accel_gyro(imuInterrupt, time_s, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);
        
        if ((abs(mean_ax) < accel_tolerance) &&
        (abs(mean_ay) < accel_tolerance) &&
        (abs(mean_az - m_g) < accel_tolerance)) {
            break;
        }
        
        offset_ax -= mean_ax;
        offset_ay -= mean_ay;
        offset_az -= mean_az - m_g;
        
        /* Before writing the offsets to the registers, they need need to be converted to 32g accelerometer full scale */
        accelOffset_32g_xyz[0] = offset_ax * accel_offset_scale + 0.5;
        accelOffset_32g_xyz[1] = offset_ay * accel_offset_scale + 0.5;
        accelOffset_32g_xyz[2] = offset_az * accel_offset_scale + 0.5;
        
        set_accel_offsets(accelOffset_32g_xyz);
        
        step++;
    }
    
    DEBUG_PRINTLN(F("Updated internal accelerometer offsets:"));
    DEBUG_PRINT("a:\t");
    DEBUG_PRINT(accelOffset_32g_xyz[0]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(accelOffset_32g_xyz[1]);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(accelOffset_32g_xyz[2]);
    DEBUG_PRINTLN();
    
    DEBUG_PRINTLN(F("Mean measurement error:"));
    DEBUG_PRINT("a:\t");
    DEBUG_PRINT(mean_ax);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_ay);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mean_az - m_g);
    DEBUG_PRINTLN();
    
    DEBUG_PRINT(F("Step count:\t"));
    DEBUG_PRINT(step);
    DEBUG_PRINTLN();
    return true;
}

/** Magnetometer calibration function. Get magnetometer minimum and maximum values, while moving 
 *  the device in a figure eight. Those values are then used to cancel out hard and soft iron distortions.
 *
 * @param[in]  imuInterrupt imu interrupt flag
 * @param[in]  time_s Time period in seconds for minimum and maximum value calculation
 * @param[in]  mag_minimumRange Minimum range (maximum - minimum value) for all directions. 
 *             if the range is smaller than the minimum range, the time period starts again.
 * @param[out] magOffset_xyz Magnetometer XYZ axis hard iron distortion corrections.
 * @param[out] magScale_xyz Magnetometer XYZ axis soft iron distortion corrections.
 *
 * @return
 *   'true' if successful,
 *   'false' on error.
 */
bool ICM20948::calibrate_mag(volatile bool &imuInterrupt, float time_s, int32_t mag_minimumRange, float *magOffset_xyz, float *magScale_xyz) {
    int16_t min_mx, max_mx, min_my, max_my, min_mz, max_mz;
    int32_t sum_mx, sum_my, sum_mz;
    int32_t dif_mx, dif_my, dif_mz, dif_m;
    
    /* Reset hard and soft iron correction before calibration. */
    m_magOffset_xyz[0] = 0; m_magOffset_xyz[1] = 0; m_magOffset_xyz[2] = 0;
    m_magScale_xyz[0] = 1; m_magScale_xyz[1] = 1; m_magScale_xyz[2] = 1;
    
    DEBUG_PRINTLN(F("Calibrating magnetometer. Move the device in a figure eight ..."));
    
    min_max_mag(imuInterrupt, time_s, mag_minimumRange, min_mx, max_mx, min_my, max_my, min_mz, max_mz);
    
    sum_mx = (int32_t) max_mx + min_mx;
    sum_my = (int32_t) max_my + min_my;
    sum_mz = (int32_t) max_mz + min_mz;
    
    dif_mx = (int32_t) max_mx - min_mx;
    dif_my = (int32_t) max_my - min_my;
    dif_mz = (int32_t) max_mz - min_mz;
    
    magOffset_xyz[0] = -0.5f * sum_mx;
    magOffset_xyz[1] = -0.5f * sum_my;
    magOffset_xyz[2] = -0.5f * sum_mz;
    
    dif_m = (dif_mx + dif_my + dif_mz) / 3;
    
    magScale_xyz[0] = (float) dif_m / dif_mx;
    magScale_xyz[1] = (float) dif_m / dif_my;
    magScale_xyz[2] = (float) dif_m / dif_mz;
    
    /* Apply calibration result. */
    m_magOffset_xyz[0] = magOffset_xyz[0]; m_magOffset_xyz[1] = magOffset_xyz[1]; m_magOffset_xyz[2] = magOffset_xyz[2];
    m_magScale_xyz[0] = magScale_xyz[0]; m_magScale_xyz[1] = magScale_xyz[1]; m_magScale_xyz[2] = magScale_xyz[2];
    
    DEBUG_PRINTLN(F("Updated magnetometer offsets:"));
    DEBUG_PRINT2(magOffset_xyz[0], 1);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magOffset_xyz[1], 1);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magOffset_xyz[2], 1);
    DEBUG_PRINT("\t");
    DEBUG_PRINTLN();

    DEBUG_PRINTLN(F("Updated magnetometer scalings:"));
    DEBUG_PRINT2(magScale_xyz[0], 4);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magScale_xyz[1], 4);
    DEBUG_PRINT("\t");
    DEBUG_PRINT2(magScale_xyz[2], 4);
    DEBUG_PRINTLN();
    
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
    return;
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
    delay(10);
    
    /* Read bytes from the ICM20948 EXT_SLV_SENS_DATA registers */
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
    
    /* Wait some time for registers to fill */
    delay(10);
    
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
    
    /*  Acceleration of gravity in LSB  */
    m_g = (int16_t) (1 / m_accelRes + 0.5);

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
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM20948_GYRO_FULLSCALE_500DPS:
            m_gyroRes = 500.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM20948_GYRO_FULLSCALE_1000DPS:
            m_gyroRes = 1000.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM20948_GYRO_FULLSCALE_2000DPS:
            m_gyroRes = 2000.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
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
    
    write_mag_register(AK09916_REG_CONTROL_2, magMode);
    
    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set accelerometer offsets
 *
 * @param[in] accelOffset_xyz Accelerometer XYZ axis offsets
 *
 * @return
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::set_accel_offsets(const int16_t * const accelOffset_xyz) {
    static uint8_t data[3];
    
    /* Bit 0 of the LSB offset register must be preserved, since it is used for temperature compensation calculations (? the data sheet is not clear). */
    
    /* Read x LSB offset register into a data array */
    read_register(ICM20948_REG_XA_OFFSET_L, 1, &data[0]);
    /* Read y LSB offset register into a data array */
    read_register(ICM20948_REG_YA_OFFSET_L, 1, &data[1]);
    /* Read z LSB offset register into a data array */
    read_register(ICM20948_REG_ZA_OFFSET_L, 1, &data[2]);
    
    /* Write x offset to registers */
    write_register(ICM20948_REG_XA_OFFSET_H, (uint8_t) ((accelOffset_xyz[0] >> 7) & 0xFF));
    write_register(ICM20948_REG_XA_OFFSET_L, (uint8_t) (((accelOffset_xyz[0] << 1) & 0xFF) | (data[0] & 0x01)));
    /* Write y offset to registers */
    write_register(ICM20948_REG_YA_OFFSET_H, (uint8_t) ((accelOffset_xyz[1] >> 7) & 0xFF));
    write_register(ICM20948_REG_YA_OFFSET_L, (uint8_t) (((accelOffset_xyz[1] << 1) & 0xFF) | (data[1] & 0x01)));
    /* Write z offset to registers */
    write_register(ICM20948_REG_ZA_OFFSET_H, (uint8_t) ((accelOffset_xyz[2] >> 7) & 0xFF));
    write_register(ICM20948_REG_ZA_OFFSET_L, (uint8_t) (((accelOffset_xyz[2] << 1) & 0xFF) | (data[2] & 0x01)));
    
    return OK;
}

/***************************************************************************//**
 * @brief
 *    Set gyroscope offsets
 *
 * @param[in] gyroOffset_xyz Gyroscope XYZ axis offsets
 *
 * @return
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::set_gyro_offsets(const int16_t * const gyroOffset_xyz) {
    /* Write x offset to registers */
    write_register(ICM20948_REG_XG_OFFS_USRH, (uint8_t) (gyroOffset_xyz[0] >> 8));
    write_register(ICM20948_REG_XG_OFFS_USRL, (uint8_t) (gyroOffset_xyz[0] & 0xFF));
    /* Write y offset to registers */
    write_register(ICM20948_REG_YG_OFFS_USRH, (uint8_t) (gyroOffset_xyz[1] >> 8));
    write_register(ICM20948_REG_YG_OFFS_USRL, (uint8_t) (gyroOffset_xyz[1] & 0xFF));
    /* Write z offset to registers */
    write_register(ICM20948_REG_ZG_OFFS_USRH, (uint8_t) (gyroOffset_xyz[2] >> 8));
    write_register(ICM20948_REG_ZG_OFFS_USRL, (uint8_t) (gyroOffset_xyz[2] & 0xFF));
    
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
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::get_accel_resolution(float &accelRes) {
    uint8_t reg;

    /* Read accelerometer full scale setting */
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ICM20948_MASK_ACCEL_FULLSCALE;

    /* Calculate accel resolution */
    switch ( reg ) {
        case ICM20948_ACCEL_FULLSCALE_2G:
            accelRes = 2.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_4G:
            accelRes = 4.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_8G:
            accelRes = 8.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_16G:
            accelRes = 16.0f / 32768.0f;
            break;
        default:
            return ERROR;
    }

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
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::get_gyro_resolution(float &gyroRes) {
    uint8_t reg;

    /* Read gyroscope full scale setting */
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ICM20948_MASK_GYRO_FULLSCALE;

    /* Calculate gyro resolution */
    switch ( reg ) {
        case ICM20948_GYRO_FULLSCALE_250DPS:
            gyroRes = 250.0f / 32768.0f;
            break;
        case ICM20948_GYRO_FULLSCALE_500DPS:
            gyroRes = 500.0f / 32768.0f;
            break;
        case ICM20948_GYRO_FULLSCALE_1000DPS:
            gyroRes = 1000.0f / 32768.0f;
            break;
        case ICM20948_GYRO_FULLSCALE_2000DPS:
            gyroRes = 2000.0f / 32768.0f;
            break;
        default:
            return ERROR;
    }

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Get accelerometer offsets
 *
 * @param[out] accelOffset_xyz Accelerometer XYZ axis offsets
 *
 * @return
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::get_accel_offsets(int16_t *accelOffset_xyz) {
    static uint8_t data[6];
    
    /* Read x offset registers into a data array */
    read_register(ICM20948_REG_XA_OFFSET_H, 2, &data[0]);
    /* Read y offset registers into a data array */
    read_register(ICM20948_REG_YA_OFFSET_H, 2, &data[2]);
    /* Read z offset registers into a data array */
    read_register(ICM20948_REG_ZA_OFFSET_H, 2, &data[4]);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    accelOffset_xyz[0] = ((uint16_t) data[0] << 7) | (data[1] >> 1);
    accelOffset_xyz[1] = ((uint16_t) data[2] << 7) | (data[3] >> 1);
    accelOffset_xyz[2] = ((uint16_t) data[4] << 7) | (data[5] >> 1);
    
    return OK;
}

/***************************************************************************//**
 * @brief
 *    Get gyroscope offsets
 *
 * @param[out] gyroOffset_xyz Gyroscope XYZ axis offsets
 *
 * @return
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::get_gyro_offsets(int16_t *gyroOffset_xyz) {
    static uint8_t data[6];
    
    /* Read x raw data registers into a data array */
    read_register(ICM20948_REG_XG_OFFS_USRH, 2, &data[0]);
    /* Read y raw data registers into a data array */
    read_register(ICM20948_REG_YG_OFFS_USRH, 2, &data[2]);
    /* Read z raw data registers into a data array */
    read_register(ICM20948_REG_ZG_OFFS_USRH, 2, &data[4]);
        
    /* Convert the MSB and LSB into a signed 16-bit value */
    gyroOffset_xyz[0] = ((uint16_t) data[0] << 8) | data[1];
    gyroOffset_xyz[1] = ((uint16_t) data[2] << 8) | data[3];
    gyroOffset_xyz[2] = ((uint16_t) data[4] << 8) | data[5];
    
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
 *    'OK' if successful,
 *    'ERROR' on error.
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
 *    'OK' if successful,
 *    'ERROR' on error.
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
 *    'OK' if successful,
 *    'ERROR' on error.
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
 *    'OK' if successful,
 *    'ERROR' on error.
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
 *    'OK' if successful,
 *    'ERROR' on error.
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
 *    Calculate mean accelerometer and gyroscope values, 
 *    which are used for calibration.
 *
 * @param[in] imuInterrupt imu interrupt flag
 * @param[in] time_s Time period in seconds for mean value calculation
 *
 * @param[out] mean_ax Mean accelerometer X axis value
 * @param[out] mean_ay Mean accelerometer Y axis value
 * @param[out] mean_az Mean accelerometer Z axis value
 * @param[out] mean_gx Mean gyroscope X axis value
 * @param[out] mean_gy Mean gyroscope Y axis value
 * @param[out] mean_gz Mean gyroscope Z axis value
 *
 * @return
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::mean_accel_gyro(volatile bool &imuInterrupt, float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz) {
    int16_t gx, gy, gz, ax, ay, az;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;
    
    int32_t num = 0;
    uint32_t t_start = micros();
    const uint32_t time = (time_s * 1e6) + 0.5;
    
    while ((micros() - t_start) < time) {
        while (!imuInterrupt) {
            /* wait for next imu interrupt */
        }
        /* reset imu interrupt flag */
        imuInterrupt = false;
        
        /* read imu measurements */
        read_accel_gyro(ax, ay, az, gx, gy, gz);
        
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        
        ++num;
    }

    mean_ax = sum_ax / num;
    mean_ay = sum_ay / num;
    mean_az = sum_az / num;
    mean_gx = sum_gx / num;
    mean_gy = sum_gy / num;
    mean_gz = sum_gz / num;
    
    return OK;
}

/***************************************************************************//**
 * @brief
 *    Calculate minimum and maximum magnetometer values, 
 *    which are used for calibration.
 *
 * @param[in] imuInterrupt imu interrupt flag
 * @param[in] time_s Time period in seconds for minimum and maximum value calculation
 * @param[in] mag_minimumRange Minimum range (maximum - minimum value) for all directions.
 *            If the range is smaller than the minimum range, the time period starts again.
 *
 * @param[out] min_mx Minimum magnetometer X axis value
 * @param[out] max_mx Maximum magnetometer X axis value
 * @param[out] min_my Minimum magnetometer Y axis value
 * @param[out] max_my Maximum magnetometer Y axis value
 * @param[out] min_mz Minimum magnetometer Z axis value
 * @param[out] max_mz Maximum magnetometer Z axis value
 *
 * @return
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::min_max_mag(volatile bool &imuInterrupt, float time_s, int32_t mag_minimumRange, int16_t &min_mx, int16_t &max_mx, int16_t &min_my, int16_t &max_my, int16_t &min_mz, int16_t &max_mz) {
    int16_t mx, my, mz;
    
    min_mx = 32767; min_my = 32767; min_mz = 32767;
    max_mx = -32768; max_my = -32768; max_mz = -32768;
    
    /* flag to indicate new magnetometer data */
    bool new_mag;
    
    const uint32_t time = (time_s * 1e6) + 0.5;
    
    /* Flags for if axis mag_minimumRange is fulfilled */
    bool miniumRange_mx = false;
    bool miniumRange_my = false;
    bool miniumRange_mz = false;
    
    while (!miniumRange_mx || !miniumRange_my || !miniumRange_mz) {
        uint32_t t_start = micros();
        while ((micros() - t_start) < time)  {
            while (!imuInterrupt) {
                /* wait for next imu interrupt */
            }
            /* reset imu interrupt flag */
            imuInterrupt = false;
            
            /* read magnetometer measurement */
            new_mag = read_mag(mx, my, mz);
            
            if (new_mag) {
                if (mx < min_mx) {
                    min_mx = mx;
                }                
                if (mx > max_mx) {
                    max_mx = mx;
                }
                
                if (my < min_my) {
                    min_my = my;
                }
                if (my > max_my) {
                    max_my = my;
                }
                
                if (mz < min_mz) {
                    min_mz = mz;
                }
                if (mz > max_mz) {
                    max_mz = mz;
                }
                
                if ((max_mx - min_mx) > mag_minimumRange) {
                    miniumRange_mx = true;
                }
                if ((max_my - min_my) > mag_minimumRange) {
                    miniumRange_my = true;
                }
                if ((max_mz - min_mz) > mag_minimumRange) {
                    miniumRange_mz = true;
                }
                
                // run serial print at a rate independent of the main loop
                static uint32_t t0_serial = 0;
                if (micros() - t0_serial > 16666) {
			        t0_serial = micros();
                    DEBUG_PRINT(min_mx); DEBUG_PRINT("\t"); DEBUG_PRINT(min_my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(min_mz);
                    DEBUG_PRINT(max_mx); DEBUG_PRINT("\t"); DEBUG_PRINT(max_my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(max_mz);
                    DEBUG_PRINTLN();
				}
            }
        }
    }
    
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
 *    'OK' if successful,
 *    'ERROR' on error.
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
 *    'OK' if successful,
 *    'ERROR' on error.
 ******************************************************************************/
uint32_t ICM20948::read_irqstatus(uint32_t &int_status) {
    static uint8_t reg[4];

    read_register(ICM20948_REG_INT_STATUS, 4, reg);
    int_status = (uint32_t) reg[0];
    int_status |= ( ( (uint32_t) reg[1]) << 8);
    int_status |= ( ( (uint32_t) reg[2]) << 16);
    int_status |= ( ( (uint32_t) reg[3]) << 24);

    return OK;
}

/***************************************************************************//**
 * @brief
 *    Checks if new data is available for read
 *
 * @return
 *    'true' if Raw Data Ready interrupt bit set,
 *    'false' otherwise.
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