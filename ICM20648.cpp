/***************************************************************************//**
 * @file ICM20648.cpp
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

#include "ICM20648.h"

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/**************************************************************************//**
* @name Error Codes
* @{
******************************************************************************/
#define ICM20648_OK                                 0x0000   /**< No errors         */
#define ICM20648_ERROR_INVALID_DEVICE_ID            0x0001   /**< Invalid device ID */
/**@}*/

/**************************************************************************//**
* @name ICM20648 register banks
* @{
******************************************************************************/
#define ICM20648_BANK_0                  (0 << 7)     /**< Register bank 0 */
#define ICM20648_BANK_1                  (1 << 7)     /**< Register bank 1 */
#define ICM20648_BANK_2                  (2 << 7)     /**< Register bank 2 */
#define ICM20648_BANK_3                  (3 << 7)     /**< Register bank 3 */
/**@}*/

/**************************************************************************//**
* @name Register and associated bit definitions
* @{
******************************************************************************/
/***********************/
/* Bank 0 register map */
/***********************/
#define ICM20648_REG_WHO_AM_I            (ICM20648_BANK_0 | 0x00)    /**< Device ID register                                     */

#define ICM20648_REG_USER_CTRL           (ICM20648_BANK_0 | 0x03)    /**< User control register                                  */
#define ICM20648_BIT_DMP_EN              0x80                        /**< DMP enable bit                                         */
#define ICM20648_BIT_FIFO_EN             0x40                        /**< FIFO enable bit                                        */
#define ICM20648_BIT_I2C_MST_EN          0x20                        /**< I2C master I/F enable bit                              */
#define ICM20648_BIT_I2C_IF_DIS          0x10                        /**< Disable I2C, enable SPI bit                            */
#define ICM20648_BIT_DMP_RST             0x08                        /**< DMP module reset bit                                   */
#define ICM20648_BIT_DIAMOND_DMP_RST     0x04                        /**< SRAM module reset bit                                  */

#define ICM20648_REG_LP_CONFIG           (ICM20648_BANK_0 | 0x05)    /**< Low Power mode config register                         */
#define ICM20648_BIT_I2C_MST_CYCLE       0x40                        /**< I2C master cycle mode enable                           */
#define ICM20648_BIT_ACCEL_CYCLE         0x20                        /**< Accelerometer cycle mode enable                        */
#define ICM20648_BIT_GYRO_CYCLE          0x10                        /**< Gyroscope cycle mode enable                            */

#define ICM20648_REG_PWR_MGMT_1          (ICM20648_BANK_0 | 0x06)    /**< Power Management 1 register                            */
#define ICM20648_BIT_H_RESET             0x80                        /**< Device reset bit                                       */
#define ICM20648_BIT_SLEEP               0x40                        /**< Sleep mode enable bit                                  */
#define ICM20648_BIT_LP_EN               0x20                        /**< Low Power feature enable bit                           */
#define ICM20648_BIT_TEMP_DIS            0x08                        /**< Temperature sensor disable bit                         */
#define ICM20648_BIT_CLK_PLL             0x01                        /**< Auto clock source selection setting                    */

#define ICM20648_REG_PWR_MGMT_2          (ICM20648_BANK_0 | 0x07)    /**< Power Management 2 register                            */
#define ICM20648_BIT_PWR_ACCEL_STBY      0x38                        /**< Disable accelerometer                                  */
#define ICM20648_BIT_PWR_GYRO_STBY       0x07                        /**< Disable gyroscope                                      */
#define ICM20648_BIT_PWR_ALL_OFF         0x7F                        /**< Disable both accel and gyro                            */

#define ICM20648_REG_INT_PIN_CFG         (ICM20648_BANK_0 | 0x0F)    /**< Interrupt Pin Configuration register                   */
#define ICM20648_BIT_INT_ACTL            0x80                        /**< Active low setting bit                                 */
#define ICM20648_BIT_INT_OPEN            0x40                        /**< Open collector onfiguration bit                        */
#define ICM20648_BIT_INT_LATCH_EN        0x20                        /**< Latch enable bit                                       */

#define ICM20648_REG_INT_ENABLE          (ICM20648_BANK_0 | 0x10)    /**< Interrupt Enable register                              */
#define ICM20648_BIT_WOM_INT_EN          0x08                        /**< Wake-up On Motion enable bit                           */

#define ICM20648_REG_INT_ENABLE_1        (ICM20648_BANK_0 | 0x11)    /**< Interrupt Enable 1 register                            */
#define ICM20648_BIT_RAW_DATA_0_RDY_EN   0x01                        /**< Raw data ready interrupt enable bit                    */

#define ICM20648_REG_INT_ENABLE_2        (ICM20648_BANK_0 | 0x12)    /**< Interrupt Enable 2 register                            */
#define ICM20648_BIT_FIFO_OVERFLOW_EN_0  0x01                        /**< FIFO overflow interrupt enable bit                     */

#define ICM20648_REG_INT_ENABLE_3        (ICM20648_BANK_0 | 0x13)    /**< Interrupt Enable 2 register                            */

#define ICM20648_REG_INT_STATUS          (ICM20648_BANK_0 | 0x19)    /**< Interrupt Status register                              */
#define ICM20648_BIT_WOM_INT             0x08                        /**< Wake-up on motion interrupt occured bit                */
#define ICM20648_BIT_PLL_RDY             0x04                        /**< PLL ready interrupt occured bit                        */

#define ICM20648_REG_INT_STATUS_1        (ICM20648_BANK_0 | 0x1A)    /**< Interrupt Status 1 register                            */
#define ICM20648_BIT_RAW_DATA_0_RDY_INT  0x01                        /**< Raw data ready interrupt occured bit                   */

#define ICM20648_REG_INT_STATUS_2        (ICM20648_BANK_0 | 0x1B)    /**< Interrupt Status 2 register                            */

#define ICM20648_REG_ACCEL_XOUT_H_SH     (ICM20648_BANK_0 | 0x2D)    /**< Accelerometer X-axis data high byte                    */
#define ICM20648_REG_ACCEL_XOUT_L_SH     (ICM20648_BANK_0 | 0x2E)    /**< Accelerometer X-axis data low byte                     */
#define ICM20648_REG_ACCEL_YOUT_H_SH     (ICM20648_BANK_0 | 0x2F)    /**< Accelerometer Y-axis data high byte                    */
#define ICM20648_REG_ACCEL_YOUT_L_SH     (ICM20648_BANK_0 | 0x30)    /**< Accelerometer Y-axis data low byte                     */
#define ICM20648_REG_ACCEL_ZOUT_H_SH     (ICM20648_BANK_0 | 0x31)    /**< Accelerometer Z-axis data high byte                    */
#define ICM20648_REG_ACCEL_ZOUT_L_SH     (ICM20648_BANK_0 | 0x32)    /**< Accelerometer Z-axis data low byte                     */

#define ICM20648_REG_GYRO_XOUT_H_SH      (ICM20648_BANK_0 | 0x33)    /**< Gyroscope X-axis data high byte                        */
#define ICM20648_REG_GYRO_XOUT_L_SH      (ICM20648_BANK_0 | 0x34)    /**< Gyroscope X-axis data low byte                         */
#define ICM20648_REG_GYRO_YOUT_H_SH      (ICM20648_BANK_0 | 0x35)    /**< Gyroscope Y-axis data high byte                        */
#define ICM20648_REG_GYRO_YOUT_L_SH      (ICM20648_BANK_0 | 0x36)    /**< Gyroscope Y-axis data low byte                         */
#define ICM20648_REG_GYRO_ZOUT_H_SH      (ICM20648_BANK_0 | 0x37)    /**< Gyroscope Z-axis data high byte                        */
#define ICM20648_REG_GYRO_ZOUT_L_SH      (ICM20648_BANK_0 | 0x38)    /**< Gyroscope Z-axis data low byte                         */

#define ICM20648_REG_TEMPERATURE_H       (ICM20648_BANK_0 | 0x39)    /**< Temperature data high byte                             */
#define ICM20648_REG_TEMPERATURE_L       (ICM20648_BANK_0 | 0x3A)    /**< Temperature data low byte                              */
#define ICM20648_REG_TEMP_CONFIG         (ICM20648_BANK_0 | 0x53)    /**< Temperature Configuration register                     */

#define ICM20648_REG_FIFO_EN_1           (ICM20648_BANK_0 | 0x66)    /**< FIFO Enable 1 register                                 */

#define ICM20648_REG_FIFO_EN_2           (ICM20648_BANK_0 | 0x67)    /**< FIFO Enable 2 register                                 */
#define ICM20648_BIT_ACCEL_FIFO_EN       0x10                        /**< Enable writing acceleration data to FIFO bit           */
#define ICM20648_BITS_GYRO_FIFO_EN       0x0E                        /**< Enable writing gyroscope data to FIFO bit              */

#define ICM20648_REG_FIFO_RST            (ICM20648_BANK_0 | 0x68)    /**< FIFO Reset register                                    */
#define ICM20648_REG_FIFO_MODE           (ICM20648_BANK_0 | 0x69)    /**< FIFO Mode register                                     */

#define ICM20648_REG_FIFO_COUNT_H        (ICM20648_BANK_0 | 0x70)    /**< FIFO data count high byte                              */
#define ICM20648_REG_FIFO_COUNT_L        (ICM20648_BANK_0 | 0x71)    /**< FIFO data count low byte                               */
#define ICM20648_REG_FIFO_R_W            (ICM20648_BANK_0 | 0x72)    /**< FIFO Read/Write register                               */

#define ICM20648_REG_DATA_RDY_STATUS     (ICM20648_BANK_0 | 0x74)    /**< Data Ready Status register                             */
#define ICM20648_BIT_RAW_DATA_0_RDY      0x01                        /**< Raw Data Ready bit                                     */

#define ICM20648_REG_FIFO_CFG            (ICM20648_BANK_0 | 0x76)    /**< FIFO Configuration register                            */
#define ICM20648_BIT_MULTI_FIFO_CFG      0x01                        /**< Interrupt status for each sensor is required           */
#define ICM20648_BIT_SINGLE_FIFO_CFG     0x00                        /**< Interrupt status for only a single sensor is required  */

/***********************/
/* Bank 1 register map */
/***********************/
#define ICM20648_REG_XA_OFFSET_H         (ICM20648_BANK_1 | 0x14)    /**< Acceleration sensor X-axis offset cancellation high byte  */
#define ICM20648_REG_XA_OFFSET_L         (ICM20648_BANK_1 | 0x15)    /**< Acceleration sensor X-axis offset cancellation low byte   */
#define ICM20648_REG_YA_OFFSET_H         (ICM20648_BANK_1 | 0x17)    /**< Acceleration sensor Y-axis offset cancellation high byte  */
#define ICM20648_REG_YA_OFFSET_L         (ICM20648_BANK_1 | 0x18)    /**< Acceleration sensor Y-axis offset cancellation low byte   */
#define ICM20648_REG_ZA_OFFSET_H         (ICM20648_BANK_1 | 0x1A)    /**< Acceleration sensor Z-axis offset cancellation high byte  */
#define ICM20648_REG_ZA_OFFSET_L         (ICM20648_BANK_1 | 0x1B)    /**< Acceleration sensor Z-axis offset cancellation low byte   */

#define ICM20648_REG_TIMEBASE_CORR_PLL   (ICM20648_BANK_1 | 0x28)    /**< PLL Timebase Correction register                          */

/***********************/
/* Bank 2 register map */
/***********************/
#define ICM20648_REG_GYRO_SMPLRT_DIV     (ICM20648_BANK_2 | 0x00)    /**< Gyroscope Sample Rate Divider regiser      */

#define ICM20648_REG_GYRO_CONFIG_1       (ICM20648_BANK_2 | 0x01)    /**< Gyroscope Configuration 1 register         */
#define ICM20648_BIT_GYRO_FCHOICE        0x01                        /**< Gyro Digital Low-Pass Filter enable bit    */
#define ICM20648_SHIFT_GYRO_FS_SEL       1                           /**< Gyro Full Scale Select bit shift           */
#define ICM20648_SHIFT_GYRO_DLPCFG       3                           /**< Gyro DLPF Config bit shift                 */
#define ICM20648_MASK_GYRO_FULLSCALE     0x06                        /**< Gyro Full Scale Select bitmask             */
#define ICM20648_MASK_GYRO_BW            0x39                        /**< Gyro Bandwidth Select bitmask              */
#define ICM20648_GYRO_FULLSCALE_250DPS   (0x00 << ICM20648_SHIFT_GYRO_FS_SEL)    /**< Gyro Full Scale = 250 deg/sec  */
#define ICM20648_GYRO_FULLSCALE_500DPS   (0x01 << ICM20648_SHIFT_GYRO_FS_SEL)    /**< Gyro Full Scale = 500 deg/sec  */
#define ICM20648_GYRO_FULLSCALE_1000DPS  (0x02 << ICM20648_SHIFT_GYRO_FS_SEL)    /**< Gyro Full Scale = 1000 deg/sec */
#define ICM20648_GYRO_FULLSCALE_2000DPS  (0x03 << ICM20648_SHIFT_GYRO_FS_SEL)    /**< Gyro Full Scale = 2000 deg/sec */
#define ICM20648_GYRO_BW_12100HZ         (0x00 << ICM20648_SHIFT_GYRO_DLPCFG)                                     /**< Gyro Bandwidth = 12100 Hz */
#define ICM20648_GYRO_BW_360HZ           ( (0x07 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 360 Hz   */
#define ICM20648_GYRO_BW_200HZ           ( (0x00 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 200 Hz   */
#define ICM20648_GYRO_BW_150HZ           ( (0x01 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 150 Hz   */
#define ICM20648_GYRO_BW_120HZ           ( (0x02 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 120 Hz   */
#define ICM20648_GYRO_BW_51HZ            ( (0x03 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 51 Hz    */
#define ICM20648_GYRO_BW_24HZ            ( (0x04 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 24 Hz    */
#define ICM20648_GYRO_BW_12HZ            ( (0x05 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 12 Hz    */
#define ICM20648_GYRO_BW_6HZ             ( (0x06 << ICM20648_SHIFT_GYRO_DLPCFG) | ICM20648_BIT_GYRO_FCHOICE)      /**< Gyro Bandwidth = 6 Hz     */

#define ICM20648_REG_GYRO_CONFIG_2       (ICM20648_BANK_2 | 0x02)    /**< Gyroscope Configuration 2 register                     */
#define ICM20648_BIT_GYRO_CTEN           0x38                        /**< Gyroscope Self-Test Enable bits                        */

#define ICM20648_REG_XG_OFFS_USRH        (ICM20648_BANK_2 | 0x03)    /**< Gyroscope sensor X-axis offset cancellation high byte  */
#define ICM20648_REG_XG_OFFS_USRL        (ICM20648_BANK_2 | 0x04)    /**< Gyroscope sensor X-axis offset cancellation low byte   */
#define ICM20648_REG_YG_OFFS_USRH        (ICM20648_BANK_2 | 0x05)    /**< Gyroscope sensor Y-axis offset cancellation high byte  */
#define ICM20648_REG_YG_OFFS_USRL        (ICM20648_BANK_2 | 0x06)    /**< Gyroscope sensor Y-axis offset cancellation low byte   */
#define ICM20648_REG_ZG_OFFS_USRH        (ICM20648_BANK_2 | 0x07)    /**< Gyroscope sensor Z-axis offset cancellation high byte  */
#define ICM20648_REG_ZG_OFFS_USRL        (ICM20648_BANK_2 | 0x08)    /**< Gyroscope sensor Z-axis offset cancellation low byte   */

#define ICM20648_REG_ODR_ALIGN_EN        (ICM20648_BANK_2 | 0x09)    /**< Output Data Rate start time alignment                  */

#define ICM20648_REG_ACCEL_SMPLRT_DIV_1  (ICM20648_BANK_2 | 0x10)    /**< Acceleration Sensor Sample Rate Divider 1 register     */
#define ICM20648_REG_ACCEL_SMPLRT_DIV_2  (ICM20648_BANK_2 | 0x11)    /**< Acceleration Sensor Sample Rate Divider 2 register     */

#define ICM20648_REG_ACCEL_INTEL_CTRL    (ICM20648_BANK_2 | 0x12)    /**< Accelerometer Hardware Intelligence Control register   */
#define ICM20648_BIT_ACCEL_INTEL_EN      0x02                        /**< Wake-up On Motion enable bit                           */
#define ICM20648_BIT_ACCEL_INTEL_MODE    0x01                        /**< WOM algorithm selection bit                            */

#define ICM20648_REG_ACCEL_WOM_THR       (ICM20648_BANK_2 | 0x13)    /**< Wake-up On Motion Threshold register                   */

#define ICM20648_REG_ACCEL_CONFIG        (ICM20648_BANK_2 | 0x14)    /**< Accelerometer Configuration register                   */
#define ICM20648_BIT_ACCEL_FCHOICE       0x01                        /**< Accel Digital Low-Pass Filter enable bit               */
#define ICM20648_SHIFT_ACCEL_FS          1                           /**< Accel Full Scale Select bit shift                      */
#define ICM20648_SHIFT_ACCEL_DLPCFG      3                           /**< Accel DLPF Config bit shift                            */
#define ICM20648_MASK_ACCEL_FULLSCALE    0x06                        /**< Accel Full Scale Select bitmask                        */
#define ICM20648_MASK_ACCEL_BW           0x39                        /**< Accel Bandwidth Select bitmask                         */
#define ICM20648_ACCEL_FULLSCALE_2G      (0x00 << ICM20648_SHIFT_ACCEL_FS)    /**< Accel Full Scale = 2 g  */
#define ICM20648_ACCEL_FULLSCALE_4G      (0x01 << ICM20648_SHIFT_ACCEL_FS)    /**< Accel Full Scale = 4 g  */
#define ICM20648_ACCEL_FULLSCALE_8G      (0x02 << ICM20648_SHIFT_ACCEL_FS)    /**< Accel Full Scale = 8 g  */
#define ICM20648_ACCEL_FULLSCALE_16G     (0x03 << ICM20648_SHIFT_ACCEL_FS)    /**< Accel Full Scale = 16 g */
#define ICM20648_ACCEL_BW_1210HZ         (0x00 << ICM20648_SHIFT_ACCEL_DLPCFG)                                    /**< Accel Bandwidth = 1210 Hz  */
#define ICM20648_ACCEL_BW_470HZ          ( (0x07 << ICM20648_SHIFT_ACCEL_DLPCFG) | ICM20648_BIT_ACCEL_FCHOICE)    /**< Accel Bandwidth = 470 Hz   */
#define ICM20648_ACCEL_BW_246HZ          ( (0x00 << ICM20648_SHIFT_ACCEL_DLPCFG) | ICM20648_BIT_ACCEL_FCHOICE)    /**< Accel Bandwidth = 246 Hz   */
#define ICM20648_ACCEL_BW_111HZ          ( (0x02 << ICM20648_SHIFT_ACCEL_DLPCFG) | ICM20648_BIT_ACCEL_FCHOICE)    /**< Accel Bandwidth = 111 Hz   */
#define ICM20648_ACCEL_BW_50HZ           ( (0x03 << ICM20648_SHIFT_ACCEL_DLPCFG) | ICM20648_BIT_ACCEL_FCHOICE)    /**< Accel Bandwidth = 50 Hz    */
#define ICM20648_ACCEL_BW_24HZ           ( (0x04 << ICM20648_SHIFT_ACCEL_DLPCFG) | ICM20648_BIT_ACCEL_FCHOICE)    /**< Accel Bandwidth = 24 Hz    */
#define ICM20648_ACCEL_BW_12HZ           ( (0x05 << ICM20648_SHIFT_ACCEL_DLPCFG) | ICM20648_BIT_ACCEL_FCHOICE)    /**< Accel Bandwidth = 12 Hz    */
#define ICM20648_ACCEL_BW_6HZ            ( (0x06 << ICM20648_SHIFT_ACCEL_DLPCFG) | ICM20648_BIT_ACCEL_FCHOICE)    /**< Accel Bandwidth = 6 Hz     */

#define ICM20648_REG_ACCEL_CONFIG_2      (ICM20648_BANK_2 | 0x15)    /**< Accelerometer Configuration 2 register              */
#define ICM20648_BIT_ACCEL_CTEN          0x1C                        /**< Accelerometer Self-Test Enable bits                 */

/***********************/
/* Bank 3 register map */
/***********************/
#define ICM20648_REG_I2C_MST_ODR_CONFIG  (ICM20648_BANK_3 | 0x00)    /**< I2C Master Output Data Rate Configuration register  */

#define ICM20648_REG_I2C_MST_CTRL        (ICM20648_BANK_3 | 0x01)    /**< I2C Master Control register                         */
#define ICM20648_BIT_I2C_MST_P_NSR       0x10                        /**< Stop between reads enabling bit                     */

#define ICM20648_REG_I2C_MST_DELAY_CTRL  (ICM20648_BANK_3 | 0x02)    /**< I2C Master Delay Control register                   */
#define ICM20648_BIT_SLV0_DLY_EN         0x01                        /**< I2C Slave0 Delay Enable bit                         */
#define ICM20648_BIT_SLV1_DLY_EN         0x02                        /**< I2C Slave1 Delay Enable bit                         */
#define ICM20648_BIT_SLV2_DLY_EN         0x04                        /**< I2C Slave2 Delay Enable bit                         */
#define ICM20648_BIT_SLV3_DLY_EN         0x08                        /**< I2C Slave3 Delay Enable bit                         */

#define ICM20648_REG_I2C_SLV0_ADDR       (ICM20648_BANK_3 | 0x03)    /**< I2C Slave0 Physical Address register                */
#define ICM20648_REG_I2C_SLV0_REG        (ICM20648_BANK_3 | 0x04)    /**< I2C Slave0 Register Address register                */
#define ICM20648_REG_I2C_SLV0_CTRL       (ICM20648_BANK_3 | 0x05)    /**< I2C Slave0 Control register                         */
#define ICM20648_REG_I2C_SLV0_DO         (ICM20648_BANK_3 | 0x06)    /**< I2C Slave0 Data Out register                        */

#define ICM20648_REG_I2C_SLV1_ADDR       (ICM20648_BANK_3 | 0x07)    /**< I2C Slave1 Physical Address register                */
#define ICM20648_REG_I2C_SLV1_REG        (ICM20648_BANK_3 | 0x08)    /**< I2C Slave1 Register Address register                */
#define ICM20648_REG_I2C_SLV1_CTRL       (ICM20648_BANK_3 | 0x09)    /**< I2C Slave1 Control register                         */
#define ICM20648_REG_I2C_SLV1_DO         (ICM20648_BANK_3 | 0x0A)    /**< I2C Slave1 Data Out register                        */

#define ICM20648_REG_I2C_SLV2_ADDR       (ICM20648_BANK_3 | 0x0B)    /**< I2C Slave2 Physical Address register                */
#define ICM20648_REG_I2C_SLV2_REG        (ICM20648_BANK_3 | 0x0C)    /**< I2C Slave2 Register Address register                */
#define ICM20648_REG_I2C_SLV2_CTRL       (ICM20648_BANK_3 | 0x0D)    /**< I2C Slave2 Control register                         */
#define ICM20648_REG_I2C_SLV2_DO         (ICM20648_BANK_3 | 0x0E)    /**< I2C Slave2 Data Out register                        */

#define ICM20648_REG_I2C_SLV3_ADDR       (ICM20648_BANK_3 | 0x0F)    /**< I2C Slave3 Physical Address register                */
#define ICM20648_REG_I2C_SLV3_REG        (ICM20648_BANK_3 | 0x10)    /**< I2C Slave3 Register Address register                */
#define ICM20648_REG_I2C_SLV3_CTRL       (ICM20648_BANK_3 | 0x11)    /**< I2C Slave3 Control register                         */
#define ICM20648_REG_I2C_SLV3_DO         (ICM20648_BANK_3 | 0x12)    /**< I2C Slave3 Data Out register                        */

#define ICM20648_REG_I2C_SLV4_ADDR       (ICM20648_BANK_3 | 0x13)    /**< I2C Slave4 Physical Address register                */
#define ICM20648_REG_I2C_SLV4_REG        (ICM20648_BANK_3 | 0x14)    /**< I2C Slave4 Register Address register                */
#define ICM20648_REG_I2C_SLV4_CTRL       (ICM20648_BANK_3 | 0x15)    /**< I2C Slave4 Control register                         */
#define ICM20648_REG_I2C_SLV4_DO         (ICM20648_BANK_3 | 0x16)    /**< I2C Slave4 Data Out register                        */
#define ICM20648_REG_I2C_SLV4_DI         (ICM20648_BANK_3 | 0x17)    /**< I2C Slave4 Data In register                         */

#define ICM20648_BIT_I2C_SLV_EN          0x80                        /**< I2C Slave Enable bit                                */
#define ICM20648_BIT_I2C_BYTE_SW         0x40                        /**< I2C Slave Byte Swap enable bit                      */
#define ICM20648_BIT_I2C_REG_DIS         0x20                        /**< I2C Slave Do Not Write Register Value bit           */
#define ICM20648_BIT_I2C_GRP             0x10                        /**< I2C Slave Group bit                                 */
#define ICM20648_BIT_I2C_READ            0x80                        /**< I2C Slave R/W bit                                   */

/* Register common for all banks */
#define ICM20648_REG_BANK_SEL            0x7F                        /**< Bank Select register                                */

#define ICM20648_DEVICE_ID               0xE0                        /**< ICM20648 Device ID value                            */
#define ICM20948_DEVICE_ID               0xEA                        /**< ICM20948 Device ID value                            */
/**@}*/

/** @endcond */


ICM20648::ICM20648(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName irq) : m_SPI(mosi, miso, sclk), m_CS(cs, 1), m_IRQ(irq)
{
    m_IRQ.disable_irq();
    m_IRQ.fall(Callback<void(void)>(this, &ICM20648::irq_handler));
}

ICM20648::~ICM20648(void)
{
}

bool ICM20648::open()
{
    uint8_t data;

    reset();

    /* Disable I2C interface, use SPI */
    write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_I2C_IF_DIS);

    /* Read Who am I register, should get 0x71 */
    read_register(ICM20648_REG_WHO_AM_I, 1, &data);

    /* If not - return */
    if ( (data != ICM20648_DEVICE_ID) && (data != ICM20948_DEVICE_ID) ) {
        return false;
    }

    /* Auto selects the best available clock source  PLL if ready, else use the Internal oscillator */
    write_register(ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_CLK_PLL);

    /* PLL startup time - maybe it is too long but better be on the safe side, no spec in the datasheet */
    wait_ms(30);

    /* INT pin: active low, open drain, IT status read clears. It seems that latched mode does not work, the INT pin cannot be cleared if set */
    write_register(ICM20648_REG_INT_PIN_CFG, ICM20648_BIT_INT_ACTL | ICM20648_BIT_INT_OPEN);

    return true;
}

/** Perform a measurement
 *
 * @returns true if measurement was successful
 */
bool ICM20648::measure()
{

}

/** Do a measurement on the gyroscope
 *
 * @param[out] gyr_x Gyroscope measurement on X axis
 * @param[out] gyr_y Gyroscope measurement on Y axis
 * @param[out] gyr_z Gyroscope measurement on Z axis
 *
 * @returns true if measurement was successful
 */
bool ICM20648::get_gyroscope(float *gyr_x, float *gyr_y, float *gyr_z)
{
    float buf[3];
    if(read_gyro_data(buf)) {
        return false;
    }

    *gyr_x = buf[0];
    *gyr_y = buf[1];
    *gyr_z = buf[2];
}

/** Do a measurement on the accelerometer
 *
 * @param[out] acc_x Accelerometer measurement on X axis
 * @param[out] acc_y Accelerometer measurement on Y axis
 * @param[out] acc_z Accelerometer measurement on Z axis
 *
 * @returns true if measurement was successful
 */
bool ICM20648::get_accelerometer(float *acc_x, float *acc_y, float *acc_z)
{
    float buf[3];
    if(read_accel_data(buf)) {
        return false;
    }

    *acc_x = buf[0];
    *acc_y = buf[1];
    *acc_z = buf[2];
}

bool ICM20648::get_temperature(float *temperature)
{
    read_temperature(temperature);
    return true;
}

/***************************************************************************//**
 * @brief
 *    Reads register from the ICM20648 device
 *
 * @param[in] addr
 *    The register address to read from in the sensor
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] numBytes
 *    The number of bytes to read
 *
 * @param[out] data
 *    The data read from the register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20648::read_register(uint16_t addr, int numBytes, uint8_t *data)
{
    uint8_t regAddr;
    uint8_t bank;

    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);

    /* Enable chip select */
    m_CS = 0;

    /* Set R/W bit to 1 - read */
    m_SPI.write(regAddr | 0x80);
    /* Transmit 0's to provide clock and read the data */
    m_SPI.write(NULL, 0, (char*)data, numBytes);

    /* Disable chip select */
    m_CS = 1;

    return;
}

/***************************************************************************//**
 * @brief
 *    Writes a register in the ICM20648 device
 *
 * @param[in] addr
 *    The register address to write
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] data
 *    The data to write to the register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20648::write_register(uint16_t addr, uint8_t data)
{
    uint8_t regAddr;
    uint8_t bank;

    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);

    /* Enable chip select */
    m_CS = 0;

    /* clear R/W bit - write, send the address */
    m_SPI.write(regAddr & 0x7F);
    m_SPI.write(data);

    /* Disable chip select */
    m_CS = 1;

    return;
}

/***************************************************************************//**
 * @brief
 *    Select the desired register bank
 *
 * @param[in] bank
 *    The address of the register bank (0..3)
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20648::select_bank(uint8_t bank)
{
    /* Enable chip select */
    m_CS = 0;

    /* clear R/W bit - write, send the address */
    m_SPI.write(ICM20648_REG_BANK_SEL);
    m_SPI.write((uint8_t)(bank << 4));

    /* Disable chip select */
    m_CS = 1;

    return;
}

/***************************************************************************//**
 * @brief
 *    Performs soft reset on the ICM20648 chip
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::reset(void)
{
    /* Set H_RESET bit to initiate soft reset */
    write_register(ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_H_RESET);

    /* Wait 100ms to complete the reset sequence */
    wait_ms(100);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the sample rate both of the accelerometer and the gyroscope.
 *
 * @param[in] sampleRate
 *    The desired sample rate in Hz. Since the resolution of the sample rate
 *    divider is different in the accel and gyro stages it is possible that
 *    the two sensor will have different sample rate set.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_sample_rate(float sampleRate)
{
    set_gyro_sample_rate(sampleRate);
    set_accel_sample_rate(sampleRate);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the sample rate of the accelerometer
 *
 * @param[in] sampleRate
 *    The desired sample rate in Hz
 *
 * @return
 *    The actual sample rate. May be different from the desired value because
 *    of the finite and discrete number of divider settings
 ******************************************************************************/
float ICM20648::set_gyro_sample_rate(float sampleRate)
{
    uint8_t gyroDiv;
    float gyroSampleRate;

    /* Calculate the sample rate divider */
    gyroSampleRate = (1125.0 / sampleRate) - 1.0;

    /* Check if it fits in the divider register */
    if ( gyroSampleRate > 255.0 ) {
        gyroSampleRate = 255.0;
    }

    if ( gyroSampleRate < 0 ) {
        gyroSampleRate = 0.0;
    }

    /* Write the value to the register */
    gyroDiv = (uint8_t) gyroSampleRate;
    write_register(ICM20648_REG_GYRO_SMPLRT_DIV, gyroDiv);

    /* Calculate the actual sample rate from the divider value */
    gyroSampleRate = 1125.0 / (gyroDiv + 1);

    return gyroSampleRate;
}

/***************************************************************************//**
 * @brief
 *    Sets the sample rate of the gyroscope
 *
 * @param[in] sampleRate
 *    The desired sample rate in Hz
 *
 * @return
 *    The actual sample rate. May be different from the desired value because
 *    of the finite and discrete number of divider settings
 ******************************************************************************/
float ICM20648::set_accel_sample_rate(float sampleRate)
{
    uint16_t accelDiv;
    float accelSampleRate;

    /* Calculate the sample rate divider */
    accelSampleRate = (1125.0 / sampleRate) - 1.0;

    /* Check if it fits in the divider registers */
    if ( accelSampleRate > 4095.0 ) {
        accelSampleRate = 4095.0;
    }

    if ( accelSampleRate < 0 ) {
        accelSampleRate = 0.0;
    }

    /* Write the value to the registers */
    accelDiv = (uint16_t) accelSampleRate;
    write_register(ICM20648_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accelDiv >> 8) );
    write_register(ICM20648_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF) );

    /* Calculate the actual sample rate from the divider value */
    accelSampleRate = 1125.0 / (accelDiv + 1);

    return accelSampleRate;
}

/***************************************************************************//**
 * @brief
 *    Sets the bandwidth of the gyroscope
 *
 * @param[in] gyroBw
 *    The desired bandwidth value. Use the ICM20648_GYRO_BW_xHZ macros, which
 *    are defined in the icm20648.h file. The value of x can be
 *    6, 12, 24, 51, 120, 150, 200, 360 or 12100.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_gyro_bandwidth(uint8_t gyroBw)
{
    uint8_t reg;

    /* Read the GYRO_CONFIG_1 register */
    read_register(ICM20648_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM20648_MASK_GYRO_BW);

    /* Write the new bandwidth value to the gyro config register */
    reg |= (gyroBw & ICM20648_MASK_GYRO_BW);
    write_register(ICM20648_REG_GYRO_CONFIG_1, reg);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the bandwidth of the accelerometer
 *
 * @param[in] accelBw
 *    The desired bandwidth value. Use the ICM20648_ACCEL_BW_yHZ macros, which
 *    are defined in the icm20648.h file. The value of y can be
 *    6, 12, 24, 50, 111, 246, 470 or 1210.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_accel_bandwidth(uint8_t accelBw)
{
    uint8_t reg;

    /* Read the GYRO_CONFIG_1 register */
    read_register(ICM20648_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM20648_MASK_ACCEL_BW);

    /* Write the new bandwidth value to the gyro config register */
    reg |= (accelBw & ICM20648_MASK_ACCEL_BW);
    write_register(ICM20648_REG_ACCEL_CONFIG, reg);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the raw acceleration value and converts to g value based on
 *    the actual resolution
 *
 * @param[out] accel
 *    A 3-element array of float numbers containing the acceleration values
 *    for the x, y and z axes in g units.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::read_accel_data(float *accel)
{
    uint8_t rawData[6];
    float accelRes;
    int16_t temp;

    /* Retrieve the current resolution */
    get_accel_resolution(&accelRes);

    /* Read the six raw data registers into data array */
    read_register(ICM20648_REG_ACCEL_XOUT_H_SH, 6, &rawData[0]);

    /* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the G value */
    temp = ( (int16_t) rawData[0] << 8) | rawData[1];
    accel[0] = (float) temp * accelRes;
    temp = ( (int16_t) rawData[2] << 8) | rawData[3];
    accel[1] = (float) temp * accelRes;
    temp = ( (int16_t) rawData[4] << 8) | rawData[5];
    accel[2] = (float) temp * accelRes;

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the raw gyroscope value and converts to deg/sec value based on
 *    the actual resolution
 *
 * @param[out] gyro
 *    A 3-element array of float numbers containing the gyroscope values
 *    for the x, y and z axes in deg/sec units.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::read_gyro_data(float *gyro)
{
    uint8_t rawData[6];
    float gyroRes;
    int16_t temp;

    /* Retrieve the current resolution */
    get_gyro_resolution(&gyroRes);

    /* Read the six raw data registers into data array */
    read_register(ICM20648_REG_GYRO_XOUT_H_SH, 6, &rawData[0]);

    /* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the dps value */
    temp = ( (int16_t) rawData[0] << 8) | rawData[1];
    gyro[0] = (float) temp * gyroRes;
    temp = ( (int16_t) rawData[2] << 8) | rawData[3];
    gyro[1] = (float) temp * gyroRes;
    temp = ( (int16_t) rawData[4] << 8) | rawData[5];
    gyro[2] = (float) temp * gyroRes;

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Gets the actual resolution of the accelerometer
 *
 * @param[out] accelRes
 *    The resolution in g/bit units
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::get_accel_resolution(float *accelRes)
{
    uint8_t reg;

    /* Read the actual acceleration full scale setting */
    read_register(ICM20648_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ICM20648_MASK_ACCEL_FULLSCALE;

    /* Calculate the resolution */
    switch ( reg ) {
        case ICM20648_ACCEL_FULLSCALE_2G:
            *accelRes = 2.0 / 32768.0;
            break;

        case ICM20648_ACCEL_FULLSCALE_4G:
            *accelRes = 4.0 / 32768.0;
            break;

        case ICM20648_ACCEL_FULLSCALE_8G:
            *accelRes = 8.0 / 32768.0;
            break;

        case ICM20648_ACCEL_FULLSCALE_16G:
            *accelRes = 16.0 / 32768.0;
            break;
    }

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Gets the actual resolution of the gyroscope
 *
 * @param[out] gyroRes
 *    The actual resolution in (deg/sec)/bit units
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::get_gyro_resolution(float *gyroRes)
{
    uint8_t reg;

    /* Read the actual gyroscope full scale setting */
    read_register(ICM20648_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ICM20648_MASK_GYRO_FULLSCALE;

    /* Calculate the resolution */
    switch ( reg ) {
        case ICM20648_GYRO_FULLSCALE_250DPS:
            *gyroRes = 250.0 / 32768.0;
            break;

        case ICM20648_GYRO_FULLSCALE_500DPS:
            *gyroRes = 500.0 / 32768.0;
            break;

        case ICM20648_GYRO_FULLSCALE_1000DPS:
            *gyroRes = 1000.0 / 32768.0;
            break;

        case ICM20648_GYRO_FULLSCALE_2000DPS:
            *gyroRes = 2000.0 / 32768.0;
            break;
    }

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the full scale value of the accelerometer
 *
 * @param[in] accelFs
 *    The desired full scale value. Use the ICM20648_ACCEL_FULLSCALE_xG
 *    macros, which are defined in the icm20648.h file. The value of x can be
 *    2, 4, 8 or 16.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_accel_fullscale(uint8_t accelFs)
{
    uint8_t reg;

    accelFs &= ICM20648_MASK_ACCEL_FULLSCALE;
    read_register(ICM20648_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM20648_MASK_ACCEL_FULLSCALE);
    reg |= accelFs;
    write_register(ICM20648_REG_ACCEL_CONFIG, reg);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the full scale value of the gyroscope
 *
 * @param[in] gyroFs
 *    The desired full scale value. Use the ICM20648_GYRO_FULLSCALE_yDPS
 *    macros, which are defined in the icm20648.h file. The value of y can be
 *    250, 500, 1000 or 2000.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_gyro_fullscale(uint8_t gyroFs)
{
    uint8_t reg;

    gyroFs &= ICM20648_MASK_GYRO_FULLSCALE;
    read_register(ICM20648_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM20648_MASK_GYRO_FULLSCALE);
    reg |= gyroFs;
    write_register(ICM20648_REG_GYRO_CONFIG_1, reg);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sleep mode of the device
 *
 * @param[in] enable
 *    If true, sleep mode is enabled. Set to false to disable sleep mode.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_sleepmode(bool enable)
{
    uint8_t reg;

    read_register(ICM20648_REG_PWR_MGMT_1, 1, &reg);

    if ( enable ) {
        /* Sleep: set the SLEEP bit */
        reg |= ICM20648_BIT_SLEEP;
    } else {
        /* Wake up: clear the SLEEP bit */
        reg &= ~(ICM20648_BIT_SLEEP);
    }

    write_register(ICM20648_REG_PWR_MGMT_1, reg);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the cycle mode operation of the accel and gyro
 *
 * @param[in] enable
 *    If true both the accel and gyro sensors will operate in cycle mode. If
 *    false the senors working in continuous mode.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_cyclemode(bool enable)
{
    uint8_t reg;

    reg = 0x00;

    if ( enable ) {
        reg = ICM20648_BIT_ACCEL_CYCLE | ICM20648_BIT_GYRO_CYCLE;
    }

    write_register(ICM20648_REG_LP_CONFIG, reg);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sensors in the ICM20648 chip
 *
 * @param[in] accel
 *    If true enables the acceleration sensor
 *
 * @param[in] gyro
 *    If true enables the gyroscope sensor
 *
 * @param[in] temp
 *    If true enables the temperature sensor
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_sensor(bool accel, bool gyro, bool temp)
{
    uint8_t pwrManagement1;
    uint8_t pwrManagement2;

    read_register(ICM20648_REG_PWR_MGMT_1, 1, &pwrManagement1);
    pwrManagement2 = 0;

    /* To enable the accelerometer clear the DISABLE_ACCEL bits in PWR_MGMT_2 */
    if ( accel ) {
        pwrManagement2 &= ~(ICM20648_BIT_PWR_ACCEL_STBY);
    } else {
        pwrManagement2 |= ICM20648_BIT_PWR_ACCEL_STBY;
    }

    /* To enable gyro clear the DISABLE_GYRO bits in PWR_MGMT_2 */
    if ( gyro ) {
        pwrManagement2 &= ~(ICM20648_BIT_PWR_GYRO_STBY);
    } else {
        pwrManagement2 |= ICM20648_BIT_PWR_GYRO_STBY;
    }

    /* To enable the temperature sensor clear the TEMP_DIS bit in PWR_MGMT_1 */
    if ( temp ) {
        pwrManagement1 &= ~(ICM20648_BIT_TEMP_DIS);
    } else {
        pwrManagement1 |= ICM20648_BIT_TEMP_DIS;
    }

    /* Write back the modified values */
    write_register(ICM20648_REG_PWR_MGMT_1, pwrManagement1);
    write_register(ICM20648_REG_PWR_MGMT_2, pwrManagement2);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sensors in low power mode in the ICM20648 chip
 *
 * @param[in] enAccel
 *    If true enables the acceleration sensor in low power mode
 *
 * @param[in] enGyro
 *    If true enables the gyroscope sensor in low power mode
 *
 * @param[in] enTemp
 *    If true enables the temperature sensor in low power mode
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enter_lowpowermode(bool enAccel, bool enGyro, bool enTemp)
{
    uint8_t data;

    read_register(ICM20648_REG_PWR_MGMT_1, 1, &data);

    if ( enAccel || enGyro || enTemp ) {
        /* Make sure that the chip is not in sleep */
        enable_sleepmode(false);

        /* And in continuous mode */
        enable_cyclemode(false);

        /* Enable the accelerometer and the gyroscope*/
        enable_sensor(enAccel, enGyro, enTemp);
        wait_ms(50);

        /* Enable cycle mode */
        enable_cyclemode(true);

        /* Set the LP_EN bit to enable low power mode */
        data |= ICM20648_BIT_LP_EN;
    } else {
        /* Enable continuous mode */
        enable_cyclemode(false);

        /* Clear the LP_EN bit to disable low power mode */
        data &= ~ICM20648_BIT_LP_EN;
    }

    /* Write the updated value to the PWR_MGNT_1 register */
    write_register(ICM20648_REG_PWR_MGMT_1, data);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the interrupts in the ICM20648 chip
 *
 * @param[in] dataReadyEnable
 *    If true enables the Raw Data Ready interrupt, otherwise disables.
 *
 * @param[in] womEnable
 *    If true enables the Wake-up On Motion interrupt, otherwise disables.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_irq(bool dataReadyEnable, bool womEnable)
{
    uint8_t intEnable;

    /* All interrupts disabled by default */
    intEnable = 0;

    /* Enable one or both of the interrupt sources if required */
    if ( womEnable ) {
        intEnable = ICM20648_BIT_WOM_INT_EN;
    }
    /* Write value to register */
    write_register(ICM20648_REG_INT_ENABLE, intEnable);

    /* All interrupts disabled by default */
    intEnable = 0;

    if ( dataReadyEnable ) {
        intEnable = ICM20648_BIT_RAW_DATA_0_RDY_EN;
    }

    /* Write value to register */
    write_register(ICM20648_REG_INT_ENABLE_1, intEnable);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the interrupt status registers of the ICM20648 chip
 *
 * @param[out] intStatus
 *    The content the four interrupt registers. LSByte is INT_STATUS, MSByte is
 *    INT_STATUS_3
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::read_irqstatus(uint32_t *int_status)
{
    uint8_t reg[4];

    read_register(ICM20648_REG_INT_STATUS, 4, reg);
    *int_status = (uint32_t) reg[0];
    *int_status |= ( ( (uint32_t) reg[1]) << 8);
    *int_status |= ( ( (uint32_t) reg[2]) << 16);
    *int_status |= ( ( (uint32_t) reg[3]) << 24);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Checks if new data is available for read
 *
 * @return
 *    Returns true if the Raw Data Ready interrupt bit set, false otherwise
 ******************************************************************************/
bool ICM20648::is_data_ready(void)
{
    uint8_t status;
    bool ret;

    ret = false;
    read_register(ICM20648_REG_INT_STATUS_1, 1, &status);

    if ( status & ICM20648_BIT_RAW_DATA_0_RDY_INT ) {
        ret = true;
    }

    return ret;
}

/***************************************************************************//**
 * @brief
 *    Sets up and enables the Wake-up On Motion feature
 *
 * @param[in] enable
 *    If true enables the WOM feature, disables otherwise
 *
 * @param[in] womThreshold
 *    Threshold value for the Wake on Motion Interrupt for ACCEL x/y/z axes.
 *    LSB = 4mg. Range is 0mg to 1020mg
 *
 * @param[in] sampleRate
 *    The desired sample rate of the accel sensor in Hz
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_wake_on_motion(bool enable, uint8_t womThreshold, float sampleRate)
{
    if ( enable ) {
        /* Make sure that the chip is not in sleep */
        enable_sleepmode(false);

        /* And in continuous mode */
        enable_cyclemode(false);

        /* Enable only the accelerometer */
        enable_sensor(true, false, false);

        /* Set sample rate */
        set_sample_rate(sampleRate);

        /* Set the bandwidth to 1210Hz */
        set_accel_bandwidth(ICM20648_ACCEL_BW_1210HZ);

        /* Accel: 2G full scale */
        set_accel_fullscale(ICM20648_ACCEL_FULLSCALE_2G);

        /* Enable the Wake On Motion interrupt */
        enable_irq(false, true);
        wait_ms(50);

        /* Enable Wake On Motion feature */
        write_register(ICM20648_REG_ACCEL_INTEL_CTRL, ICM20648_BIT_ACCEL_INTEL_EN | ICM20648_BIT_ACCEL_INTEL_MODE);

        /* Set the wake on motion threshold value */
        write_register(ICM20648_REG_ACCEL_WOM_THR, womThreshold);

        /* Enable low power mode */
        enter_lowpowermode(true, false, false);
    } else {
        /* Disable Wake On Motion feature */
        write_register(ICM20648_REG_ACCEL_INTEL_CTRL, 0x00);

        /* Disable the Wake On Motion interrupt */
        enable_irq(false, false);

        /* Disable cycle mode */
        enable_cyclemode(false);
    }

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Accelerometer and gyroscope calibration function. Reads the gyroscope
 *    and accelerometer values while the device is at rest and in level. The
 *    resulting values are loaded to the accel and gyro bias registers to cancel
 *    the static offset error.
 *
 * @param[out] accelBiasScaled
 *    The mesured acceleration sensor bias in mg
 *
 * @param[out] gyroBiasScaled
 *    The mesured gyro sensor bias in deg/sec
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::calibrate(float *accelBiasScaled, float *gyroBiasScaled)
{
    uint8_t data[12];
    uint16_t i, packetCount, fifoCount;
    int32_t gyroBias[3] = { 0, 0, 0 };
    int32_t accelBias[3] = { 0, 0, 0 };
    int32_t accelTemp[3];
    int32_t gyroTemp[3];
    int32_t accelBiasFactory[3];
    int32_t gyroBiasStored[3];
    float gyroRes, accelRes;

    /* Enable the accelerometer and the gyro */
    enable_sensor(true, true, false);

    /* Set 1kHz sample rate */
    set_sample_rate(1100.0);

    /* 246Hz BW for the accelerometer and 200Hz for the gyroscope */
    set_accel_bandwidth(ICM20648_ACCEL_BW_246HZ);
    set_gyro_bandwidth(ICM20648_GYRO_BW_12HZ);

    /* Set the most sensitive range: 2G full scale and 250dps full scale */
    set_accel_fullscale(ICM20648_ACCEL_FULLSCALE_2G);
    set_gyro_fullscale(ICM20648_GYRO_FULLSCALE_250DPS);

    /* Retrieve the resolution per bit */
    get_accel_resolution(&accelRes);
    get_gyro_resolution(&gyroRes);

    /* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
    /* Experiments show that the gyro needs more time to get reliable results */
    wait_ms(50);

    /* Disable the FIFO */
    write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);
    write_register(ICM20648_REG_FIFO_MODE, 0x0F);

    /* Enable accelerometer and gyro to store the data in FIFO */
    write_register(ICM20648_REG_FIFO_EN_2, ICM20648_BIT_ACCEL_FIFO_EN | ICM20648_BITS_GYRO_FIFO_EN);

    /* Reset the FIFO */
    write_register(ICM20648_REG_FIFO_RST, 0x0F);
    write_register(ICM20648_REG_FIFO_RST, 0x00);

    /* Enable the FIFO */
    write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);

    /* The max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
    /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
    /* Loop until at least 4080 samples gathered */
    fifoCount = 0;
    while ( fifoCount < 4080 ) {
        wait_ms(5);
        /* Read FIFO sample count */
        read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);
        /* Convert to a 16 bit value */
        fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);
    }

    /* Disable accelerometer and gyro to store the data in FIFO */
    write_register(ICM20648_REG_FIFO_EN_2, 0x00);

    /* Read FIFO sample count */
    read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);

    /* Convert to a 16 bit value */
    fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);

    /* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
    packetCount = fifoCount / 12;

    /* Retrieve the data from the FIFO */
    for ( i = 0; i < packetCount; i++ ) {
        read_register(ICM20648_REG_FIFO_R_W, 12, &data[0]);
        /* Convert to 16 bit signed accel and gyro x,y and z values */
        accelTemp[0] = ( (int16_t) (data[0] << 8) | data[1]);
        accelTemp[1] = ( (int16_t) (data[2] << 8) | data[3]);
        accelTemp[2] = ( (int16_t) (data[4] << 8) | data[5]);
        gyroTemp[0] = ( (int16_t) (data[6] << 8) | data[7]);
        gyroTemp[1] = ( (int16_t) (data[8] << 8) | data[9]);
        gyroTemp[2] = ( (int16_t) (data[10] << 8) | data[11]);

        /* Sum the values */
        accelBias[0] += accelTemp[0];
        accelBias[1] += accelTemp[1];
        accelBias[2] += accelTemp[2];
        gyroBias[0] += gyroTemp[0];
        gyroBias[1] += gyroTemp[1];
        gyroBias[2] += gyroTemp[2];
    }

    /* Divide by packet count to get the average */
    accelBias[0] /= packetCount;
    accelBias[1] /= packetCount;
    accelBias[2] /= packetCount;
    gyroBias[0] /= packetCount;
    gyroBias[1] /= packetCount;
    gyroBias[2] /= packetCount;

    /* Acceleormeter: add or remove (depending on the orientation of the chip) 1G (gravity) from the Z axis value */
    if ( accelBias[2] > 0L ) {
        accelBias[2] -= (int32_t) (1.0 / accelRes);
    } else {
        accelBias[2] += (int32_t) (1.0 / accelRes);
    }

    /* Convert the values to degrees per sec for displaying */
    gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
    gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
    gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

    /* Read stored gyro trim values. After reset these values are all 0 */
    read_register(ICM20648_REG_XG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[0] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20648_REG_YG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[1] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20648_REG_ZG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[2] = ( (int16_t) (data[0] << 8) | data[1]);

    /* The gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
    /* the best sensitivity, so need to divide by 4 */
    /* Substract from the stored calibration value */
    gyroBiasStored[0] -= gyroBias[0] / 4;
    gyroBiasStored[1] -= gyroBias[1] / 4;
    gyroBiasStored[2] -= gyroBias[2] / 4;

    /* Split the values into two bytes */
    data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
    data[1] = (gyroBiasStored[0]) & 0xFF;
    data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
    data[3] = (gyroBiasStored[1]) & 0xFF;
    data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
    data[5] = (gyroBiasStored[2]) & 0xFF;

    /* Write the  gyro bias values to the chip */
    write_register(ICM20648_REG_XG_OFFS_USRH, data[0]);
    write_register(ICM20648_REG_XG_OFFS_USRL, data[1]);
    write_register(ICM20648_REG_YG_OFFS_USRH, data[2]);
    write_register(ICM20648_REG_YG_OFFS_USRL, data[3]);
    write_register(ICM20648_REG_ZG_OFFS_USRH, data[4]);
    write_register(ICM20648_REG_ZG_OFFS_USRL, data[5]);

    /* Calculate the accelerometer bias values to store in the hardware accelerometer bias registers. These registers contain */
    /* factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold */
    /* non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature */
    /* compensation calculations(? the datasheet is not clear). Accelerometer bias registers expect bias input */
    /* as 2048 LSB per g, so that the accelerometer biases calculated above must be divided by 8. */

    /* Read factory accelerometer trim values */
    read_register(ICM20648_REG_XA_OFFSET_H, 2, &data[0]);
    accelBiasFactory[0] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20648_REG_YA_OFFSET_H, 2, &data[0]);
    accelBiasFactory[1] = ( (int16_t) (data[0] << 8) | data[1]);
    read_register(ICM20648_REG_ZA_OFFSET_H, 2, &data[0]);
    accelBiasFactory[2] = ( (int16_t) (data[0] << 8) | data[1]);

    /* Construct total accelerometer bias, including calculated average accelerometer bias from above */
    /* Scale the 2g full scale (most sensitive range) results to 16g full scale - divide by 8 */
    /* Clear the last bit (temperature compensation? - the datasheet is not clear) */
    /* Substract from the factory calibration value */

    accelBiasFactory[0] -= ( (accelBias[0] / 8) & ~1);
    accelBiasFactory[1] -= ( (accelBias[1] / 8) & ~1);
    accelBiasFactory[2] -= ( (accelBias[2] / 8) & ~1);

    /* Split the values into two bytes */
    data[0] = (accelBiasFactory[0] >> 8) & 0xFF;
    data[1] = (accelBiasFactory[0]) & 0xFF;
    data[2] = (accelBiasFactory[1] >> 8) & 0xFF;
    data[3] = (accelBiasFactory[1]) & 0xFF;
    data[4] = (accelBiasFactory[2] >> 8) & 0xFF;
    data[5] = (accelBiasFactory[2]) & 0xFF;

    /* Store them in the accelerometer offset registers */
    write_register(ICM20648_REG_XA_OFFSET_H, data[0]);
    write_register(ICM20648_REG_XA_OFFSET_L, data[1]);
    write_register(ICM20648_REG_YA_OFFSET_H, data[2]);
    write_register(ICM20648_REG_YA_OFFSET_L, data[3]);
    write_register(ICM20648_REG_ZA_OFFSET_H, data[4]);
    write_register(ICM20648_REG_ZA_OFFSET_L, data[5]);

    /* Convert the values to G for displaying */
    accelBiasScaled[0] = (float) accelBias[0] * accelRes;
    accelBiasScaled[1] = (float) accelBias[1] * accelRes;
    accelBiasScaled[2] = (float) accelBias[2] * accelRes;

    /* Turn off FIFO */
    write_register(ICM20648_REG_USER_CTRL, 0x00);

    /* Disable all sensors */
    enable_sensor(false, false, false);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Gyroscope calibration function. Reads the gyroscope
 *    values while the device is at rest and in level. The
 *    resulting values are loaded to the gyro bias registers to cancel
 *    the static offset error.
 *
 * @param[out] gyroBiasScaled
 *    The mesured gyro sensor bias in deg/sec
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::calibrate_gyro(float *gyroBiasScaled)
{
    uint8_t data[12];
    uint16_t i, packetCount, fifoCount;
    int32_t gyroBias[3] = { 0, 0, 0 };
    int32_t gyroTemp[3];
    int32_t gyroBiasStored[3];
    float gyroRes;

    /* Enable the accelerometer and the gyro */
    enable_sensor(true, true, false);

    /* Set 1kHz sample rate */
    set_sample_rate(1100.0);

    /* Configure bandwidth for gyroscope to 12Hz */
    set_gyro_bandwidth(ICM20648_GYRO_BW_12HZ);

    /* Configure sensitivity to 250dps full scale */
    set_gyro_fullscale(ICM20648_GYRO_FULLSCALE_250DPS);

    /* Retrieve the resolution per bit */
    get_gyro_resolution(&gyroRes);

    /* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
    /* Experiments show that the gyro needs more time to get reliable results */
    wait_ms(50);

    /* Disable the FIFO */
    write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);
    write_register(ICM20648_REG_FIFO_MODE, 0x0F);

    /* Enable accelerometer and gyro to store the data in FIFO */
    write_register(ICM20648_REG_FIFO_EN_2, ICM20648_BITS_GYRO_FIFO_EN);

    /* Reset the FIFO */
    write_register(ICM20648_REG_FIFO_RST, 0x0F);
    write_register(ICM20648_REG_FIFO_RST, 0x00);

    /* Enable the FIFO */
    write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);

    /* The max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
    /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
    /* Loop until at least 4080 samples gathered */
    fifoCount = 0;
    while ( fifoCount < 4080 ) {
        wait_ms(5);

        /* Read FIFO sample count */
        read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);

        /* Convert to a 16 bit value */
        fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);
    }

    /* Disable accelerometer and gyro to store the data in FIFO */
    write_register(ICM20648_REG_FIFO_EN_2, 0x00);

    /* Read FIFO sample count */
    read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);

    /* Convert to a 16 bit value */
    fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);

    /* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
    packetCount = fifoCount / 12;

    /* Retrieve the data from the FIFO */
    for ( i = 0; i < packetCount; i++ ) {
        read_register(ICM20648_REG_FIFO_R_W, 12, &data[0]);
        /* Convert to 16 bit signed accel and gyro x,y and z values */
        gyroTemp[0] = ( (int16_t) (data[6] << 8) | data[7]);
        gyroTemp[1] = ( (int16_t) (data[8] << 8) | data[9]);
        gyroTemp[2] = ( (int16_t) (data[10] << 8) | data[11]);

        /* Sum the values */
        gyroBias[0] += gyroTemp[0];
        gyroBias[1] += gyroTemp[1];
        gyroBias[2] += gyroTemp[2];
    }

    /* Divide by packet count to get the average */
    gyroBias[0] /= packetCount;
    gyroBias[1] /= packetCount;
    gyroBias[2] /= packetCount;

    /* Convert the values to degrees per sec for displaying */
    gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
    gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
    gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

    /* Read stored gyro trim values. After reset these values are all 0 */
    read_register(ICM20648_REG_XG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[0] = ( (int16_t) (data[0] << 8) | data[1]);

    read_register(ICM20648_REG_YG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[1] = ( (int16_t) (data[0] << 8) | data[1]);

    read_register(ICM20648_REG_ZG_OFFS_USRH, 2, &data[0]);
    gyroBiasStored[2] = ( (int16_t) (data[0] << 8) | data[1]);

    /* The gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
    /* the best sensitivity, so need to divide by 4 */
    /* Substract from the stored calibration value */
    gyroBiasStored[0] -= gyroBias[0] / 4;
    gyroBiasStored[1] -= gyroBias[1] / 4;
    gyroBiasStored[2] -= gyroBias[2] / 4;

    /* Split the values into two bytes */
    data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
    data[1] = (gyroBiasStored[0]) & 0xFF;
    data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
    data[3] = (gyroBiasStored[1]) & 0xFF;
    data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
    data[5] = (gyroBiasStored[2]) & 0xFF;

    /* Write the  gyro bias values to the chip */
    write_register(ICM20648_REG_XG_OFFS_USRH, data[0]);
    write_register(ICM20648_REG_XG_OFFS_USRL, data[1]);
    write_register(ICM20648_REG_YG_OFFS_USRH, data[2]);
    write_register(ICM20648_REG_YG_OFFS_USRL, data[3]);
    write_register(ICM20648_REG_ZG_OFFS_USRH, data[4]);
    write_register(ICM20648_REG_ZG_OFFS_USRL, data[5]);

    /* Turn off FIFO */
    write_register(ICM20648_REG_USER_CTRL, 0x00);

    /* Disable all sensors */
    enable_sensor(false, false, false);

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the temperature sensor raw value and converts to Celsius.
 *
 * @param[out] temperature
 *    The mesured temperature in Celsius
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::read_temperature(float *temperature)
{
    uint8_t data[2];
    int16_t raw_temp;

    /* Read temperature registers */
    read_register(ICM20648_REG_TEMPERATURE_H, 2, data);

    /* Convert to int16 */
    raw_temp = (int16_t) ( (data[0] << 8) + data[1]);

    /* Calculate the Celsius value from the raw reading */
    *temperature = ( (float) raw_temp / 333.87) + 21.0;

    return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the device ID of the ICM20648
 *
 * @param[out] devID
 *    The ID of the device read from teh WHO_AM_I register. Expected value? 0xE0
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::get_device_id(uint8_t *device_id)
{
    read_register(ICM20648_REG_WHO_AM_I, 1, device_id);

    return ICM20648_OK;
}

void ICM20648::irq_handler(void)
{

}
