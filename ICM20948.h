/***************************************************************************//**
 * @file ICM20948.h
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

#ifndef ICM20948_H
#define ICM20948_H

/** ICM20948 class.
 *  Used for taking accelerometer and gyroscope measurements.
 *
 * Example:
 * @code
 * //#include "mbed.h"
 * #include "ICM20948.h"
 *
 * //Create an ICM20948 object
 * ICM20948 sensor(PC4, PC5);
 *
 * int main()
 * {
 *     //Try to open ICM20948
 *     if (sensor.open()) {
 *         printf("Device detected!\n");
 *
 *         while (1) {
 *             float acc_x, acc_y, acc_z;
 *             float gyr_x, gyr_y, gyr_z;
 *             
 *             sensor.measure();
 *
 *             sensor.get_accelerometer(&acc_x, &acc_y, &acc_z);
 *             sensor.get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
 *
 *             //Print current accelerometer measurement
 *             printf("acc: %.3f  %.3f  %.3f\n", acc_x, acc_y, acc_z);
 *             //Print current gyroscope measurement
 *             printf("gyr: %.3f  %.3f  %.3f\n", gyr_x, gyr_y, gyr_z);
 *
 *             //Sleep for 0.5 seconds
 *             wait(0.5);
 *         }
 *     } else {
 *         error("Device not detected!\n");
 *     }
 * }
 * @endcode
 */
class ICM20948
{
public:

    /** Create an ICM20948 object connected to specified SPI pins
     *
     * @param[in] mosi  SPI MOSI pin.
     * @param[in] miso  SPI MISO pin.
     * @param[in] sclk  SPI clock pin.
     * @param[in] cs    SPI Chip Select pin.
     * @param[in] irq   ICM20948 irq pin.
     */
    ICM20948(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName irq = NC);

    /**
    * ICM20948 destructor
    */
    ~ICM20948(void);

    /** Probe for ICM20948 and try to initialize sensor
     *
     * @returns
     *   'true' if device exists on bus,
     *   'false' if device doesn't exist on bus.
     */
    bool open();

    /** Perform a measurement
     *
     * @returns true if measurement was successful
     */
    bool measure();

    /** Gets gyroscope measurement
    *
    * @param[out] gyr_x Gyroscope measurement on X axis
    * @param[out] gyr_y Gyroscope measurement on Y axis
    * @param[out] gyr_z Gyroscope measurement on Z axis
    *
    * @returns true if measurement was successful
    */
    bool get_gyroscope(float *gyr_x, float *gyr_y, float *gyr_z);

    /** Gets accelerometer measurement
    *
    * @param[out] acc_x Accelerometer measurement on X axis
    * @param[out] acc_y Accelerometer measurement on Y axis
    * @param[out] acc_z Accelerometer measurement on Z axis
    *
    * @returns true if measurement was successful
    */
    bool get_accelerometer(float *acc_x, float *acc_y, float *acc_z);

    /** Gets temperature measurement
     *
     * @param [out] measured temperature
     *
     * @returns true if measurement was successful
     */
    bool get_temperature(float *temperature);

private:
    /* Private functions */
    void        read_register(uint16_t addr, uint8_t numBytes, uint8_t *data);
    void        write_register(uint16_t addr, uint8_t data);
    void        read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data);
    void        write_mag_register(uint8_t addr, uint8_t data);
    void        select_bank(uint8_t bank);
    uint32_t    reset(void);
    uint32_t    set_sample_rate(float sampleRate);
    float       set_gyro_sample_rate(float sampleRate);
    float       set_accel_sample_rate(float sampleRate);
    uint32_t    set_gyro_bandwidth(uint8_t gyroBw);
    uint32_t    set_accel_bandwidth(uint8_t accelBw);
    uint32_t    read_accel_data(float *accel);
    uint32_t    read_gyro_data(float *gyro);
    uint32_t    read_mag_data(float *mag);
    uint32_t    get_accel_resolution(float *accelRes);
    uint32_t    get_gyro_resolution(float *gyroRes);
    uint32_t    set_accel_fullscale(uint8_t accelFs);
    uint32_t    set_gyro_fullscale(uint8_t gyroFs);
    uint32_t    enable_sleepmode(bool enable);
    uint32_t    enable_cyclemode(bool enable);
    uint32_t    enable_sensor(bool accel, bool gyro, bool temp);
    uint32_t    enter_lowpowermode(bool enAccel, bool enGyro, bool enTemp);
    uint32_t    enable_irq(bool dataReadyEnable, bool womEnable);
    uint32_t    read_irqstatus(uint32_t *int_status);
    bool        is_data_ready(void);
    uint32_t    enable_wake_on_motion(bool enable, uint8_t womThreshold, float sampleRate);
    uint32_t    calibrate(float *accelBiasScaled, float *gyroBiasScaled);
    uint32_t    calibrate_gyro(float *gyroBiasScaled);
    uint32_t    read_temperature(float *temperature);
    uint32_t    get_device_id(uint8_t *device_id);
    void        set_mag_transfer(bool read);

    void irq_handler(void);

    /* Member variables */
    SPI m_SPI;
    DigitalOut m_CS;
    InterruptIn m_IRQ;
};

#endif