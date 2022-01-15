/****************************************************************************
 * include/nuttx/sensors/lsm6dsl.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DSL_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DSL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Addresses ************************************************************/

/* Accelerometer addresses */

#define LSM6DSLACCEL_ADDR0  (0xD4 >> 1) /* 0x6a low */
#define LSM6DSLACCEL_ADDR1  (0xD6 >> 1) /* 0x6B .. high */

/* Gyroscope addresses */

#define LSM6DSLGYRO_ADDR0   0xd4
#define LSM6DSLGYRO_ADDR1   0xd6

/* Register Addresses *******************************************************/

/* Accelerometer and gyroscope registers */

#define LSM6DSL_FUNC_CFG_ACCESS                         0x01 /* Enable embedded functions register (r/w).*/
#define LSM6DSL_SENSOR_SYNC_TIME_FRAME                  0x04 /* Sensor synchronization time frame register (r/w). */
#define LSM6DSL_SENSOR_SYNC_RES_RATIO                   0x05 /* Sensor synchronization resolution ratio (r/w) */
#define LSM6DSL_FIFO_CTRL1                              0x06 /* FIFO control register (r/w). */
#define LSM6DSL_FIFO_CTRL2                              0x07 /* FIFO control register (r/w). */
#define LSM6DSL_FIFO_CTRL3                              0x08 /* FIFO control register (r/w). */
#define LSM6DSL_FIFO_CTRL4                              0x09 /* FIFO control register (r/w). */
#define LSM6DSL_FIFO_CTRL5                              0x0A /* FIFO control register (r/w). */
#define LSM6DSL_DRDY_PULSE_CFG_G                        0x0B /* DataReady configuration register (r/w).*/
#define LSM6DSL_INT1_CTRL                               0x0D /* INT1 pad control register (r/w). */
#define LSM6DSL_INT2_CTRL                               0x0E /* INT2 pad control register (r/w). */
#define LSM6DSL_WHO_AM_I                                0x0F /* Who_AM_I register (r). This register is a read-only register. */
#define LSM6DSL_WHO_AM_I_VALUE                          0x6A
#define LSM6DSL_CTRL1_XL                                0x10 /* Linear acceleration sensor control register 1 (r/w). */
#define LSM6DSL_CTRL2_G                                 0x11 /* Angular rate sensor control register 2 (r/w). */
#define LSM6DSL_CTRL3_C                                 0x12 /* Control register 3 (r/w). */
#define LSM6DSL_CTRL4_C                                 0x13 /* Control register 4 (r/w).*/
#define LSM6DSL_CTRL5_C                                 0x14 /* Control register 5 (r/w). */
#define LSM6DSL_CTRL6_C                                 0x15 /* Angular rate sensor control register 6 (r/w). */
#define LSM6DSL_CTRL7_G                                 0x16 /* Angular rate sensor control register 7 (r/w). */
#define LSM6DSL_CTRL8_XL                                0x17 /* Linear acceleration sensor control register 8 (r/w). */
#define LSM6DSL_CTRL9_XL                                0x18 /* Linear acceleration sensor control register 9 (r/w). */
#define LSM6DSL_CTRL10_C                                0x19 /* Control register 10 (r/w). */
#define LSM6DSL_MASTER_CONFIG                           0x1A /* Master configuration register (r/w). */
#define LSM6DSL_WAKE_UP_SRC                             0x1B /* Wake up interrupt source register (r). */
#define LSM6DSL_TAP_SRC                                 0x1C /* Tap source register (r). */
#define LSM6DSL_D6D_SRC                                 0x1D /* Portrait, landscape, face-up and face-down source register (r). */
#define LSM6DSL_STATUS_REG                              0x1E /* The STATUS_REG register is read by the SPI/I2C interface (r). */
#define LSM6DSL_OUT_TEMP_L                              0x20 /* Temperature data output register (r). */
#define LSM6DSL_OUT_TEMP_H                              0x21 /* Temperature data output register (r). */
#define LSM6DSL_OUTX_L_G                                0x22 /* Angular rate sensor pitch axis (X) angular rate output register (r). */
#define LSM6DSL_OUTX_H_G                                0x23 /* Angular rate sensor pitch axis (X) angular rate output register (r). */
#define LSM6DSL_OUTY_L_G                                0x24 /* Angular rate sensor roll axis (Y) angular rate output register (r). */
#define LSM6DSL_OUTY_H_G                                0x25 /* Angular rate sensor roll axis (Y) angular rate output register (r). */
#define LSM6DSL_OUTZ_L_G                                0x26 /* Angular rate sensor roll axis (Z) angular rate output register (r). */
#define LSM6DSL_OUTZ_H_G                                0x27 /* Angular rate sensor roll axis (Z) angular rate output register (r). */
#define LSM6DSL_OUTX_L_XL                               0x28 /* Linear acceleration sensor X-axis output register (r). */
#define LSM6DSL_OUTX_H_XL                               0x29 /* Linear acceleration sensor X-axis output register (r). */
#define LSM6DSL_OUTY_L_XL                               0x2A /* Linear acceleration sensor Y-axis output register (r). */
#define LSM6DSL_OUTY_H_XL                               0x2B /* Linear acceleration sensor Y-axis output register (r). */
#define LSM6DSL_OUTZ_L_XL                               0x2C /* Linear acceleration sensor Z-axis output register (r). */
#define LSM6DSL_OUTZ_H_XL                               0x2D /* Linear acceleration sensor Z-axis output register (r). */
#define LSM6DSL_SENSORHUB1_REG                          0x2E /* First byte associated to external sensors. */
#define LSM6DSL_SENSORHUB2_REG                          0x2F /* Second byte associated to external sensors. */
#define LSM6DSL_SENSORHUB3_REG                          0x30 /* Third byte associated to external sensors. */
#define LSM6DSL_SENSORHUB4_REG                          0x31 /* Fourth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB5_REG                          0x32 /* Fifth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB6_REG                          0x33 /* Sixth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB7_REG                          0x34 /* Seventh byte associated to external sensors. */
#define LSM6DSL_SENSORHUB8_REG                          0x35 /* Eighth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB9_REG                          0x36 /* Ninth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB10_REG                         0x37 /* Tenth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB11_REG                         0x38 /* Eleventh byte associated to external sensors. */
#define LSM6DSL_SENSORHUB12_REG                         0x39 /* Twelfth byte associated to external sensors. */
#define LSM6DSL_FIFO_STATUS1                            0x3A /* FIFO status control register (r). */
#define LSM6DSL_FIFO_STATUS2                            0x3B /* FIFO status control register (r). */
#define LSM6DSL_FIFO_STATUS3                            0x3C /* FIFO status control register (r). */
#define LSM6DSL_FIFO_STATUS4                            0x3D /* FIFO status control register (r). */
#define LSM6DSL_FIFO_DATA_OUT_L                         0x3E /* FIFO data output register (r). */
#define LSM6DSL_FIFO_DATA_OUT_H                         0x3F /* FIFO data output register (r). */
#define LSM6DSL_TIMESTAMP0_REG                          0x40 /* Timestamp first (least significant) byte data output register (r). */
#define LSM6DSL_TIMESTAMP1_REG                          0x41 /* Timestamp second byte data output register (r). */
#define LSM6DSL_TIMESTAMP2_REG                          0x42 /* Timestamp third (most significant) byte data output register (r). */
#define LSM6DSL_STEP_TIMESTAMP_L                        0x49 /* Step counter timestamp information register (r). */
#define LSM6DSL_STEP_TIMESTAMP_H                        0x4A /* Step counter timestamp information register (r). */
#define LSM6DSL_STEP_COUNTER_L                          0x4B /* Step counter output register (r). */
#define LSM6DSL_STEP_COUNTER_H                          0x4C /* Step counter output register (r). */
#define LSM6DSL_SENSORHUB13_REG                         0x4D /* Thirteenth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB14_REG                         0x4E /* Fourteenth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB15_REG                         0x4F /* Fifteenth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB16_REG                         0x50 /* Sixteenth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB17_REG                         0x51 /* Seventeenth byte associated to external sensors. */
#define LSM6DSL_SENSORHUB18_REG                         0x52 /* Eighteenth byte associated to external sensors. */
#define LSM6DSL_FUNC_SRC1                               0x53 /* Significant motion, tilt, step detector, hard/soft-iron and sensor hub interrupt source register (r). */
#define LSM6DSL_FUNC_SRC2                               0x54 /* Wrist tilt interrupt source register (r). */
#define LSM6DSL_WRIST_TILT_IA                           0x55 /* Wrist tilt interrupt source register (r). */
#define LSM6DSL_TAP_CFG                                 0x58 /* Enables interrupt and inactivity functions, configuration of filtering and tap recognition functions (r/w). */
#define LSM6DSL_TAP_THS_6D                              0x59 /* Portrait/landscape position and tap function threshold register (r/w). */
#define LSM6DSL_INT_DUR2                                0x5A /* Tap recognition function setting register (r/w). */
#define LSM6DSL_WAKE_UP_THS                             0x5B /* Single and double-tap function threshold register (r/w). */
#define LSM6DSL_WAKE_UP_DUR                             0x5C /* Free-fall, wakeup, timestamp and sleep mode functions duration setting register (r/w). */
#define LSM6DSL_FREE_FALL                               0x5D /* Free-fall function duration setting register (r/w). */
#define LSM6DSL_MD1_CFG                                 0x5E /* Functions routing on INT1 register (r/w). */
#define LSM6DSL_MD2_CFG                                 0x5F /* Functions routing on INT2 register (r/w). */
#define LSM6DSL_MASTER_CMD_CODE                         0x60 /* Master command code used for stamping for sensor sync. */
#define LSM6DSL_SENS_SYNC_SPI_ERROR_CODE                0x61 /* Error code used for sensor synchronization. */
#define LSM6DSL_OUT_MAG_RAW_X_L                         0x66 /* External magnetometer raw data (r). */
#define LSM6DSL_OUT_MAG_RAW_X_H                         0x67 /* External magnetometer raw data (r). */
#define LSM6DSL_OUT_MAG_RAW_Y_L                         0x68 /* External magnetometer raw data (r). */
#define LSM6DSL_OUT_MAG_RAW_Y_H                         0x69 /* External magnetometer raw data (r). */
#define LSM6DSL_OUT_MAG_RAW_Z_L                         0x6A /* External magnetometer raw data (r). */
#define LSM6DSL_OUT_MAG_RAW_Z_H                         0x6B /* External magnetometer raw data (r). */
#define LSM6DSL_X_OFS_USR                               0x73 /* Accelerometer X-axis user offset correction (r/w). */
#define LSM6DSL_Y_OFS_USR                               0x74 /* Accelerometer Y-axis user offset correction (r/w). */
#define LSM6DSL_Z_OFS_USR                               0x75 /* Accelerometer Z-axis user offset correction (r/w). */

/* Embedded functions registers description - Bank A */

#define LSM6DSL_SLV0_ADD                                0x02 /* I2C slave address of the first external sensor (Sensor1) register (r/w). */
#define LSM6DSL_SLV0_SUBADD                             0x03 /* Address of register on the first external sensor (Sensor1) register (r/w). */
#define LSM6DSL_SLAVE0_CONFIG                           0x04 /* First external sensor (Sensor1) configuration and sensor hub settings register (r/w). */
#define LSM6DSL_SLV1_ADD                                0x05 /* I2C slave address of the second external sensor (Sensor2) register (r/w). */
#define LSM6DSL_SLV1_SUBADD                             0x06 /* Address of register on the second external sensor (Sensor2) register (r/w). */
#define LSM6DSL_SLAVE1_CONFIG                           0x07 /* Second external sensor (Sensor2) configuration register (r/w). */
#define LSM6DSL_SLV2_ADD                                0x08 /* I2C slave address of the third external sensor (Sensor3) register (r/w). */
#define LSM6DSL_SLV2_SUBADD                             0x09 /* Address of register on the third external sensor (Sensor3) register (r/w). */
#define LSM6DSL_SLAVE2_CONFIG                           0x0A /* Third external sensor (Sensor3) configuration register (r/w). */
#define LSM6DSL_SLV3_ADD                                0x0B /* I2C slave address of the fourth external sensor (Sensor4) register (r/w). */
#define LSM6DSL_SLV3_SUBADD                             0x0C /* Address of register on the fourth external sensor (Sensor4) register (r/w). */
#define LSM6DSL_SLAVE3_CONFIG                           0x0D /* Fourth external sensor (Sensor4) configuration register (r/w). */
#define LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0             0x0E /* Data to be written into the slave device register (r/w). */
#define LSM6DSL_CONFIG_PEDO_THS_MIN                     0x0F
#define LSM6DSL_SM_THS                                  0x13 /* Significant motion configuration register (r/w). */
#define LSM6DSL_PEDO_DEB_REG                            0x14
#define LSM6DSL_STEP_COUNT_DELTA                        0x15 /* Time period register for step detection on delta time (r/w). */
#define LSM6DSL_MAG_SI_XX                               0x24 /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_XY                               0x25 /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_XZ                               0x26 /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_YX                               0x27 /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_YY                               0x28 /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_YZ                               0x29 /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_ZX                               0x2A /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_ZY                               0x2B /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_SI_ZZ                               0x2C /* Soft-iron matrix correction register (r/w). */
#define LSM6DSL_MAG_OFFX_L                              0x2D /* Offset for X-axis hard-iron compensation register (r/w). */
#define LSM6DSL_MAG_OFFX_H                              0x2E /* Offset for X-axis hard-iron compensation register (r/w). */
#define LSM6DSL_MAG_OFFY_L                              0x2F /* Offset for Y-axis hard-iron compensation register (r/w). */
#define LSM6DSL_MAG_OFFY_H                              0x30 /* Offset for Y-axis hard-iron compensation register (r/w). */
#define LSM6DSL_MAG_OFFZ_L                              0x31 /* Offset for Z-axis hard-iron compensation register (r/w). */
#define LSM6DSL_MAG_OFFZ_H                              0x32 /* Offset for Z-axis hard-iron compensation register (r/w). */

/* Embedded functions registers description - Bank B */

#define LSM6DSL_A_WRIST_TILT_LAT                        0x50 /* Absolute Wrist Tilt latency register (r/w). */
#define LSM6DSL_A_WRIST_TILT_THS                        0x54 /* Absolute Wrist Tilt threshold register (r/w). */
#define LSM6DSL_A_WRIST_TILT_Mask                       0x59 /* Absolute Wrist Tilt mask register (r/w). */

/****************************************************************************
 * Register Bit Definitions
 *
 * For this sensor it is chosen to not define each pin individually...its set
 * bitwise like:
 * 0b000[0]0000 with preferred hex value!
 *              Where [] is showing the [not defined in datasheet] bit.
 * A complete definition is written below, just not all registers are
 * validated!!
 *
 ****************************************************************************/

#define LSM6DSL_FUNC_CFG_ACCESS_FUNC_CFG_EN         (1 << 5)
#define LSM6DSL_FUNC_CFG_ACCESS_FUNC_CFG_EN_B       (1 << 7)

#define LSM6DSL_SENSOR_SYNC_TIME_FRAME_TPH_SHIFT    0
#define LSM6DSL_SENSOR_SYNC_TIME_FRAME_TPH_MASK     (15 << LSM6DSL_SENSOR_SYNC_TIME_FRAME_TPH_SHIFT)

#define LSM6DSL_SENSOR_SYNC_RES_RATIO_SHIFT         0
#define LSM6DSL_SENSOR_SYNC_RES_RATIO_MASK          (3 << LSM6DSL_SENSOR_SYNC_RES_RATIO_SHIFT)
#define LSM6DSL_SENSOR_SYNC_RES_RATIO_RR_2_11       (0 << LSM6DSL_SENSOR_SYNC_RES_RATIO_SHIFT)
#define LSM6DSL_SENSOR_SYNC_RES_RATIO_RR_2_12       (1 << LSM6DSL_SENSOR_SYNC_RES_RATIO_SHIFT)
#define LSM6DSL_SENSOR_SYNC_RES_RATIO_RR_2_13       (2 << LSM6DSL_SENSOR_SYNC_RES_RATIO_SHIFT)
#define LSM6DSL_SENSOR_SYNC_RES_RATIO_RR_2_14       (3 << LSM6DSL_SENSOR_SYNC_RES_RATIO_SHIFT)

#define LSM6DSL_FIFO_CTRL1_SHIFT                    0
#define LSM6DSL_FIFO_CTRL1_MASK                     (255 << LSM6DSL_FIFO_CTRL1_SHIFT)

#define LSM6DSL_FIFO_CTRL2_SHIFT                    255
#define LSM6DSL_FIFO_CTRL2_MASK                     (255 << LSM6DSL_FIFO_CTRL2_SHIFT)
#define LSM6DSL_FIFO_CTRL2_FTH_8                    (1 << 0)
#define LSM6DSL_FIFO_CTRL2_FTH_9                    (1 << 1)
#define LSM6DSL_FIFO_CTRL2_FTH_10                   (1 << 2)
#define LSM6DSL_FIFO_CTRL2_FIFO_TEMP_EN             (1 << 3)
#define LSM6DSL_FIFO_CTRL2_PEDO_FIFO_DRDY           (1 << 6)
#define LSM6DSL_FIFO_CTRL2_PEDO_FIFO_EN             (1 << 7)

#define LSM6DSL_FIFO_CTRL3_DEC_FIFO_XL_SHIFT        0
#define LSM6DSL_FIFO_CTRL3_DEC_FIFO_XL_MASK         (3 << LSM6DSL_FIFO_CTRL3_DEC_FIFO_XL_SHIFT)
#define LSM6DSL_FIFO_CTRL3_DEC_FIFO_GYRO_SHIFT      3
#define LSM6DSL_FIFO_CTRL3_DEC_FIFO_GYRO_MASK       (3 << LSM6DSL_FIFO_CTRL3_DEC_FIFO_GYRO_SHIFT)

#define LSM6DSL_FIFO_CTRL4_DEC_DS3_FIFO_SHIFT       0
#define LSM6DSL_FIFO_CTRL4_DEC_DS3_FIFO_MASK        (3 << LSM6DSL_FIFO_CTRL4_DEC_DS3_FIFO_SHIFT)
#define LSM6DSL_FIFO_CTRL4_DEC_DS4_FIFO_SHIFT       3
#define LSM6DSL_FIFO_CTRL4_DEC_DS4_FIFO_MASK        (3 << LSM6DSL_FIFO_CTRL4_DEC_DS4_FIFO_SHIFT)
#define LSM6DSL_FIFO_CTRL4_ONLY_HIGH_DATA           (1 << 6) /*  */
#define LSM6DSL_FIFO_CTRL4_STOP_ON_FTH              (1 << 7) /* .*/

#define LSM6DSL_FIFO_CTRL5_FIFO_MODE_SHIFT          0
#define LSM6DSL_FIFO_CTRL5_FIFO_MODE_MASK           (3 << LSM6DSL_FIFO_CTRL5_FIFO_MODE_SHIFT)
#define LSM6DSL_FIFO_CTRL5_FMODE_BYPASS             (0 << LSM6DSL_FIFO_CTRL5_FIFO_MODE_SHIFT)
#define LSM6DSL_FIFO_CTRL5_FMODE_FIFO               (1 << LSM6DSL_FIFO_CTRL5_FIFO_MODE_SHIFT)
#define LSM6DSL_FIFO_CTRL5_FMODE_CONT_FIFO          (3 << LSM6DSL_FIFO_CTRL5_FIFO_MODE_SHIFT)
#define LSM6DSL_FIFO_CTRL5_FMODE_BYPASS_CONT        (4 << LSM6DSL_FIFO_CTRL5_FIFO_MODE_SHIFT)
#define LSM6DSL_FIFO_CTRL5_FMODE_CONT               (6 << LSM6DSL_FIFO_CTRL5_FIFO_MODE_SHIFT)
#define LSM6DSL_FIFO_CTRL5_ODR_FIFO_SHIFT           3
#define LSM6DSL_FIFO_CTRL5_ODR_FIFO_MASK            (15 << LSM6DSL_FIFO_CTRL5_ODR_FIFO_SHIFT)

#define LSM6DSL_DRDY_PULSE_CFG_G_INT2_WRIST_TILT    (1 << 0)
#define LSM6DSL_DRDY_PULSE_CFG_G_DRDY_PULSED        (1 << 7)

#define LSM6DSL_INT1_CTRL_INT1_DRDY_XL              (1 << 0)
#define LSM6DSL_INT1_CTRL_INT1_DRDY_G               (1 << 1)
#define LSM6DSL_INT1_CTRL_INT1_BOOT                 (1 << 2)
#define LSM6DSL_INT1_CTRL_INT1_FTH                  (1 << 3)
#define LSM6DSL_INT1_CTRL_INT1_FIFO_OVT             (1 << 4)
#define LSM6DSL_INT1_CTRL_INT1_FULL_FLAG            (1 << 5)
#define LSM6DSL_INT1_CTRL_INT1_SIGN_MOT             (1 << 6)
#define LSM6DSL_INT1_CTRL_INT1_STEP_DETECTOR        (1 << 7)

#define LSM6DSL_INT2_CTRL_INT2_DRDY_XL              (1 << 0)
#define LSM6DSL_INT2_CTRL_INT2_DRDY_G               (1 << 1)
#define LSM6DSL_INT2_CTRL_INT2_DRDY_TEMP            (1 << 2)
#define LSM6DSL_INT2_CTRL_INT2_FTH                  (1 << 3)
#define LSM6DSL_INT2_CTRL_INT2_FIFO_OVT             (1 << 4)
#define LSM6DSL_INT2_CTRL_INT2_FULL_FLAG            (1 << 5)
#define LSM6DSL_INT2_CTRL_INT2_STEP_COUNT_OV        (1 << 6)
#define LSM6DSL_INT2_CTRL_INT2_STEP_DELTA           (1 << 7)

#define LSM6DSL_FIFO_CTRL5_ODR_FIFO_MASK            (15 << LSM6DSL_FIFO_CTRL5_ODR_FIFO_SHIFT)

#define LSM6DSL_CTRL1_XL_DRDY_XL                    (1 << 0)
#define LSM6DSL_CTRL1_XL_SHIFT                      0
#define LSM6DSL_CTRL1_XL_BW0_XL                     (1 << 0)
#define LSM6DSL_CTRL1_XL_LPF1_BW_SEL                (1 << 1)
#define LSM6DSL_CTRL1_XL_FS_XL_SHIFT                2
#define LSM6DSL_CTRL1_XL_FS_XL_MASK                 (3 << LSM6DSL_CTRL1_XL_FS_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_FS_XL_2G                   (0 << LSM6DSL_CTRL1_XL_FS_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_FS_XL_16G                  (1 << LSM6DSL_CTRL1_XL_FS_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_FS_XL_4G                   (2 << LSM6DSL_CTRL1_XL_FS_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_FS_XL_8G                   (3 << LSM6DSL_CTRL1_XL_FS_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_SHIFT               4
#define LSM6DSL_CTRL1_XL_ODR_XL_MASK                (15 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)

#define LSM6DSL_CTRL1_XL_ODR_XL_POWER_DOWN          (0 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_1_6HZ_12_5HZ        (11 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_12_5HZ_12_5HZ       (1 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_26HZ_26HZ           (2 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_52HZ_52HZ           (3 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_104HZ_104HZ         (4 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_208HZ_208HZ         (5 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_416HZ_416HZ         (6 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_833HZ_833HZ         (7 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_1_6kHZ_1_6kHZ       (8 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_3_3kHz_3_3kHZ       (9 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)
#define LSM6DSL_CTRL1_XL_ODR_XL_6_6kHZ_6_6kHZ       (10 << LSM6DSL_CTRL1_XL_ODR_XL_SHIFT)

#define LSM6DSL_CTRL2_G_SHIFT                       0
#define LSM6DSL_CTRL2_G_FS_125                      (1 << 1)
#define LSM6DSL_CTRL2_G_FS_G_SHIFT                  2
#define LSM6DSL_CTRL2_G_FS_G_MASK                   (3 << LSM6DSL_CTRL2_G_FS_G_SHIFT)
#define LSM6DSL_CTRL2_G_FS_G_250DPS                 (0 << LSM6DSL_CTRL2_G_FS_G_SHIFT)
#define LSM6DSL_CTRL2_G_FS_G_500DPS                 (1 << LSM6DSL_CTRL2_G_FS_G_SHIFT)
#define LSM6DSL_CTRL2_G_FS_G_1000DPS                (2 << LSM6DSL_CTRL2_G_FS_G_SHIFT)
#define LSM6DSL_CTRL2_G_FS_G_2000DPS                (3 << LSM6DSL_CTRL2_G_FS_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_SHIFT                 4
#define LSM6DSL_CTRL2_G_ODR_G_MASK                  (15 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)

#define LSM6DSL_CTRL2_G_ODR_G_POWER_DOWN            (0 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_1_6HZ_12_5HZ          (11 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_12_5HZ_12_5HZ         (1 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_26HZ_26HZ             (2 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_52HZ_52HZ             (3 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_104HZ_104HZ           (4 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_208HZ_208HZ           (5 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_416HZ_416HZ           (6 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_833HZ_833HZ           (7 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_1_6kHZ_1_6kHZ         (8 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_3_3kHz_3_3kHZ         (9 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)
#define LSM6DSL_CTRL2_G_ODR_G_6_6kHZ_6_6kHZ         (10 << LSM6DSL_CTRL2_G_ODR_G_SHIFT)

#define LSM6DSL_CTRL3_C_SHIFT                       0
#define LSM6DSL_CTRL3_C_MASK                        (0 << LSM6DSL_CTRL3_C_SHIFT)
#define LSM6DSL_CTRL3_C_SW_RESET                    (1 << 0)
#define LSM6DSL_CTRL3_C_BLE                         (1 << 1)
#define LSM6DSL_CTRL3_C_IF_INC                      (1 << 2)
#define LSM6DSL_CTRL3_C_SIM                         (1 << 3)
#define LSM6DSL_CTRL3_C_PP_OD                       (1 << 4)
#define LSM6DSL_CTRL3_C_H_LACTIVE                   (1 << 5)
#define LSM6DSL_CTRL3_C_BDU                         (1 << 6)
#define LSM6DSL_CTRL3_C_BOOT                        (1 << 7)

#define LSM6DSL_CTRL4_C_LPF1_SEL_G                  (1 << 1)
#define LSM6DSL_CTRL4_C_I2C_disable                 (1 << 2)
#define LSM6DSL_CTRL4_C_DRDY_MASK                   (1 << 3)
#define LSM6DSL_CTRL4_C_DEN_DRDY_INT1               (1 << 4)
#define LSM6DSL_CTRL4_C_INT2_on_INT1                (1 << 5)
#define LSM6DSL_CTRL4_C_SLEEP                       (1 << 6)
#define LSM6DSL_CTRL4_C_DEN_XL_EN                   (1 << 7)

#define LSM6DSL_CTRL5_C_SHIFT                       0
#define LSM6DSL_CTRL5_C_ST_XL_MASK                  (3 << LSM6DSL_CTRL5_C_SHIFT)
#define LSM6DSL_CTRL5_C_ST_G_SHIFT                  2
#define LSM6DSL_CTRL5_C_ST_G_MASK                   (3 << LSM6DSL_CTRL5_C_ST_G_SHIFT)
#define LSM6DSL_CTRL5_C_DEN_LH                      (1 << 4)
#define LSM6DSL_CTRL5_C_ROUNDING_SHIFT              2
#define LSM6DSL_CTRL5_C_ROUNDING_MASK               (7 << LSM6DSL_CTRL5_C_ROUNDING_SHIFT)

#define LSM6DSL_CTRL6_C_SHIFT                       0
#define LSM6DSL_CTRL6_C_FTYPE_MASK                  (3 << LSM6DSL_CTRL6_C_SHIFT)
#define LSM6DSL_CTRL6_C_USR_OFF_W                   (1 << 3)
#define LSM6DSL_CTRL6_C_XL_HM_MODE                  (1 << 4)
#define LSM6DSL_CTRL6_C_LVL2_EN                     (1 << 5)
#define LSM6DSL_CTRL6_C_LEL_EN                      (1 << 6)
#define LSM6DSL_CTRL6_C_TRIG_EN                     (1 << 7)

#define LSM6DSL_CTRL7_G_SHIFT                       0
#define LSM6DSL_CTRL7_G_MASK                        (0 << LSM6DSL_CTRL7_G_SHIFT)
#define LSM6DSL_CTRL7_G_ROUNDING_STATUS             (1 << 2)
#define LSM6DSL_CTRL7_G_HPM0_G                      (1 << 4)
#define LSM6DSL_CTRL7_G_HPM1_G                      (1 << 5)
#define LSM6DSL_CTRL7_G_HP_EN_G                     (1 << 6)
#define LSM6DSL_CTRL7_G_G_HM_MODE                   (1 << 7)

#define LSM6DSL_CTRL8_XL_LOW_PASS_ON_6D             (1 << 0)
#define LSM6DSL_CTRL8_XL_HP_SLOPE_XL_EN             (1 << 2)
#define LSM6DSL_CTRL8_XL_INPUT_COMPOSITE            (1 << 3)
#define LSM6DSL_CTRL8_XL_HP_REF_MODE                (1 << 4)
#define LSM6DSL_CTRL8_XL_HPCF_XL_SHIFT              5
#define LSM6DSL_CTRL8_XL_HPCF_XL_MASK               (3 << LSM6DSL_CTRL8_XL_HPCF_XL_SHIFT)
#define LSM6DSL_CTRL8_XL_LPF2_XL_EN                 (1 << 7)

#define LSM6DSL_CTRL9_XL_SOFT_EN                    (1 << 2)
#define LSM6DSL_CTRL9_XL_DEN_XL_G                   (1 << 4)
#define LSM6DSL_CTRL9_XL_DEN_Z                      (1 << 5)
#define LSM6DSL_CTRL9_XL_DEN_Y                      (1 << 6)
#define LSM6DSL_CTRL9_XL_DEN_X                      (1 << 7)

#define LSM6DSL_CTRL10_C_SIGN_MOTION_EN             (1 << 0)
#define LSM6DSL_CTRL10_C_PEDO_RST_STEP              (1 << 1)
#define LSM6DSL_CTRL10_C_FUNC_EN                    (1 << 2)
#define LSM6DSL_CTRL10_C_TILT_EN                    (1 << 3)
#define LSM6DSL_CTRL10_C_PEDO_EN                    (1 << 4)
#define LSM6DSL_CTRL10_C_TIMER_EN                   (1 << 5)
#define LSM6DSL_CTRL10_C_WRIST_TILT_EN              (1 << 7)

#define LSM6DSL_MASTER_CONFIG_MASTER_ON             (1 << 0)
#define LSM6DSL_MASTER_CONFIG_IRON_EN               (1 << 1)
#define LSM6DSL_MASTER_CONFIG_PASS_THROUGH_MODE     (1 << 2)
#define LSM6DSL_MASTER_CONFIG_PULL_UP_EN            (1 << 3)
#define LSM6DSL_MASTER_CONFIG_START_CONFIG          (1 << 4)
#define LSM6DSL_MASTER_CONFIG_DATA_VALID_SEL_FIFO   (1 << 6)
#define LSM6DSL_MASTER_CONFIG_RDY_ON_INT1           (1 << 7)

#define LSM6DSL_WAKE_UP_SRC_Z_WU                    (1 << 0)
#define LSM6DSL_WAKE_UP_SRC_Y_WU                    (1 << 1)
#define LSM6DSL_WAKE_UP_SRC_X_WU                    (1 << 2)
#define LSM6DSL_WAKE_UP_SRC_WU_IA                   (1 << 3)
#define LSM6DSL_WAKE_UP_SRC_SLEEP_STATE_IA          (1 << 4)
#define LSM6DSL_WAKE_UP_SRC_FF_IA                   (1 << 5)

#define LSM6DSL_TAP_SRC_Z_TAP                       (1 << 0)
#define LSM6DSL_TAP_SRC_Y_TAP                       (1 << 1)
#define LSM6DSL_TAP_SRC_X_TAP                       (1 << 2)
#define LSM6DSL_TAP_SRC_TAP_SIGN                    (1 << 3)
#define LSM6DSL_TAP_SRC_DOUBLE_TAP                  (1 << 4)
#define LSM6DSL_TAP_SRC_SINGLE_TAP                  (1 << 5)
#define LSM6DSL_TAP_SRC_TAP_IA                      (1 << 6)

#define LSM6DSL_D6D_SRC_XL                          (1 << 0)
#define LSM6DSL_D6D_SRC_XH                          (1 << 1)
#define LSM6DSL_D6D_SRC_YL                          (1 << 2)
#define LSM6DSL_D6D_SRC_YH                          (1 << 3)
#define LSM6DSL_D6D_SRC_ZL                          (1 << 4)
#define LSM6DSL_D6D_SRC_ZH                          (1 << 5)
#define LSM6DSL_D6D_SRC_D6D_IA                      (1 << 6)
#define LSM6DSL_D6D_SRC_DEN_DRDY                    (1 << 7)

#define LSM6DSL_STATUS_REG_SHIFT                    0
#define LSM6DSL_STATUS_REG_MASK                     (0 << LSM6DSL_STATUS_REG_SHIFT)
#define LSM6DSL_STATUS_REG_XLDA                     (1 << 0)
#define LSM6DSL_STATUS_REG_GDA                      (1 << 1)
#define LSM6DSL_STATUS_REG_TDA                      (1 << 2)

#define LSM6DSL_CTRL5_C_SHIFT                       0
#define LSM6DSL_CTRL5_C_ST_XL_MASK                  (3 << LSM6DSL_CTRL5_C_SHIFT)

#define LSM6DSL_OUT_TEMP_L_TEMP_SHIFT               0
#define LSM6DSL_OUT_TEMP_L_TEMP_MASK                (255 << LSM6DSL_OUT_TEMP_L_TEMP_SHIFT)
#define LSM6DSL_OUT_TEMP_H_TEMP_SHIFT               0
#define LSM6DSL_OUT_TEMP_H_TEMP_MASK                (255 << LSM6DSL_OUT_TEMP_H_TEMP_SHIFT)

#define LSM6DSL_OUTX_L_G_SHIFT                      0
#define LSM6DSL_OUTX_L_G_MASK                       (255 << LSM6DSL_OUTX_L_G_SHIFT)

#define LSM6DSL_OUTX_H_G_SHIFT                      0
#define LSM6DSL_OUTX_H_G_MASK                       (255 << LSM6DSL_OUTX_H_G_SHIFT)

#define LSM6DSL_OUTY_L_G_SHIFT                      0
#define LSM6DSL_OUTY_L_G_MASK                       (255 << LSM6DSL_OUTY_L_G_SHIFT)

#define LSM6DSL_OUTY_H_G_SHIFT                      0
#define LSM6DSL_OUTY_H_G_MASK                       (255 << LSM6DSL_OUTY_H_G_SHIFT)

#define LSM6DSL_OUTZ_L_G_SHIFT                      0
#define LSM6DSL_OUTZ_L_G_MASK                       (255 << LSM6DSL_OUTZ_L_G_SHIFT)

#define LSM6DSL_OUTZ_H_G_SHIFT                      0
#define LSM6DSL_OUTZ_H_G_MASK                       (255 << LSM6DSL_OUTZ_H_G_SHIFT)

#define LSM6DSL_OUTX_L_XL_SHIFT                     0
#define LSM6DSL_OUTX_L_XL_MASK                      (255 << LSM6DSL_OUTX_L_XL_SHIFT)

#define LSM6DSL_OUTX_H_XL_SHIFT                     0
#define LSM6DSL_OUTX_H_XL_MASK                      (255 << LSM6DSL_OUTX_H_XL_SHIFT)

#define LSM6DSL_OUTY_L_XL_SHIFT                     0
#define LSM6DSL_OUTY_L_XL_MASK                      (255 << LSM6DSL_OUTY_L_XL_SHIFT)

#define LSM6DSL_OUTY_H_XL_SHIFT                     0
#define LSM6DSL_OUTY_H_XL_MASK                      (255 << LSM6DSL_OUTY_H_XL_SHIFT)

#define LSM6DSL_OUTZ_L_XL_SHIFT                     0
#define LSM6DSL_OUTZ_L_XL_MASK                      (255 << LSM6DSL_OUTZ_L_XL_SHIFT)

#define LSM6DSL_OUTZ_H_XL_SHIFT                     0
#define LSM6DSL_OUTZ_H_XL_MASK                      (255 << LSM6DSL_OUTZ_H_XL_SHIFT)

#define LSM6DSL_SENSORHUB1_REG_SHIFT                0
#define LSM6DSL_SENSORHUB1_REG_MASK                 (255 << LSM6DSL_SENSORHUB1_REG_SHIFT)

#define LSM6DSL_SENSORHUB2_REG_SHIFT                0
#define LSM6DSL_SENSORHUB2_REG_MASK                 (255 << LSM6DSL_SENSORHUB2_REG_SHIFT)

#define LSM6DSL_SENSORHUB3_REG_SHIFT                0
#define LSM6DSL_SENSORHUB3_REG_MASK                 (255 << LSM6DSL_SENSORHUB3_REG_SHIFT)

#define LSM6DSL_SENSORHUB4_REG_SHIFT                0
#define LSM6DSL_SENSORHUB4_REG_MASK                 (255 << LSM6DSL_SENSORHUB4_REG_SHIFT)

#define LSM6DSL_SENSORHUB5_REG_SHIFT                0
#define LSM6DSL_SENSORHUB5_REG_MASK                 (255 << LSM6DSL_SENSORHUB5_REG_SHIFT)

#define LSM6DSL_SENSORHUB6_REG_SHIFT                0
#define LSM6DSL_SENSORHUB6_REG_MASK                 (255 << LSM6DSL_SENSORHUB6_REG_SHIFT)

#define LSM6DSL_SENSORHUB7_REG_SHIFT                0
#define LSM6DSL_SENSORHUB7_REG_MASK                 (255 << LSM6DSL_SENSORHUB7_REG_SHIFT)

#define LSM6DSL_SENSORHUB8_REG_SHIFT                0
#define LSM6DSL_SENSORHUB8_REG_MASK                 (255 << LSM6DSL_SENSORHUB8_REG_SHIFT)

#define LSM6DSL_SENSORHUB9_REG_SHIFT                0
#define LSM6DSL_SENSORHUB9_REG_MASK                 (255 << LSM6DSL_SENSORHUB9_REG_SHIFT)

#define LSM6DSL_SENSORHUB10_REG_SHIFT               0
#define LSM6DSL_SENSORHUB10_REG_MASK                (255 << LSM6DSL_SENSORHUB10_REG_SHIFT)

#define LSM6DSL_SENSORHUB11_REG_SHIFT               0
#define LSM6DSL_SENSORHUB11_REG_MASK                (255 << LSM6DSL_SENSORHUB11_REG_SHIFT)

#define LSM6DSL_SENSORHUB12_REG_SHIFT               0
#define LSM6DSL_SENSORHUB12_REG_MASK                (255 << LSM6DSL_SENSORHUB12_REG_SHIFT)

#define LSM6DSL_FIFO_STATUS1_SHIFT                  0
#define LSM6DSL_FIFO_STATUS1_MASK                   (255 << LSM6DSL_FIFO_STATUS1_SHIFT)

#define LSM6DSL_FIFO_STATUS2_DIFF_FIFO_SHIFT        0
#define LSM6DSL_FIFO_STATUS2_DIFF_FIFO_MASK         (7 << LSM6DSL_FIFO_STATUS2_DIFF_FIFO_SHIFT)
#define LSM6DSL_FIFO_STATUS2_FIFO_EMPTY             (1 << 1)
#define LSM6DSL_FIFO_STATUS2_FIFO_FULL_ART          (1 << 1)
#define LSM6DSL_FIFO_STATUS2_OVER_RUN               (1 << 2)
#define LSM6DSL_FIFO_STATUS2_WaterM                 (1 << 3)

#define LSM6DSL_FIFO_STATUS3_SHIFT                  0
#define LSM6DSL_FIFO_STATUS3_MASK                   (255 << LSM6DSL_FIFO_STATUS3_SHIFT)

#define LSM6DSL_FIFO_STATUS4_SHIFT                  0
#define LSM6DSL_FIFO_STATUS4_MASK                   (3 << LSM6DSL_FIFO_STATUS4_SHIFT)

#define LSM6DSL_FIFO_DATA_OUT_L_SHIFT               0
#define LSM6DSL_FIFO_DATA_OUT_L_MASK                (255 << LSM6DSL_FIFO_DATA_OUT_L_SHIFT)

#define LSM6DSL_FIFO_DATA_OUT_H_SHIFT               0
#define LSM6DSL_FIFO_DATA_OUT_H_MASK                (255 << LSM6DSL_FIFO_DATA_OUT_H_SHIFT)

#define LSM6DSL_TIMESTAMP0_REG_SHIFT                0
#define LSM6DSL_TIMESTAMP0_REG_MASK                 (255 << LSM6DSL_TIMESTAMP0_REG_SHIFT)

#define LSM6DSL_TIMESTAMP1_REG_SHIFT                0
#define LSM6DSL_TIMESTAMP1_REG_MASK                 (255 << LSM6DSL_TIMESTAMP1_REG_SHIFT)

#define LSM6DSL_TIMESTAMP2_REG_SHIFT                0
#define LSM6DSL_TIMESTAMP2_REG_MASK                 (255 << LSM6DSL_TIMESTAMP2_REG_SHIFT)

#define LSM6DSL_STEP_TIMESTAMP_L_SHIFT              0
#define LSM6DSL_STEP_TIMESTAMP_L_MASK               (255 << LSM6DSL_STEP_TIMESTAMP_L_SHIFT)

#define LSM6DSL_STEP_TIMESTAMP_H_SHIFT              0
#define LSM6DSL_STEP_TIMESTAMP_H_MASK               (255 << LSM6DSL_STEP_TIMESTAMP_H_SHIFT)

#define LSM6DSL_STEP_COUNTER_L_SHIFT                0
#define LSM6DSL_STEP_COUNTER_L_MASK                 (255 << LSM6DSL_STEP_COUNTER_L_SHIFT)

#define LSM6DSL_SENSORHUB13_REG_SHIFT               0
#define LSM6DSL_SENSORHUB13_REG_MASK                (255 << LSM6DSL_SENSORHUB13_REG_SHIFT)

#define LSM6DSL_SENSORHUB14_REG_SHIFT               0
#define LSM6DSL_SENSORHUB14_REG_MASK                (255 << LSM6DSL_SENSORHUB14_REG_SHIFT)

#define LSM6DSL_SENSORHUB15_REG_SHIFT               0
#define LSM6DSL_SENSORHUB15_REG_MASK                (255 << LSM6DSL_SENSORHUB15_REG_SHIFT)

#define LSM6DSL_SENSORHUB16_REG_SHIFT               0
#define LSM6DSL_SENSORHUB16_REG_MASK                (255 << LSM6DSL_SENSORHUB16_REG_SHIFT)

#define LSM6DSL_SENSORHUB17_REG_SHIFT               0
#define LSM6DSL_SENSORHUB17_REG_MASK                (255 << LSM6DSL_SENSORHUB17_REG_SHIFT)

#define LSM6DSL_SENSORHUB18_REG_SHIFT               0
#define LSM6DSL_SENSORHUB18_REG_MASK                (255 << LSM6DSL_SENSORHUB18_REG_SHIFT)

#define LSM6DSL_FUNC_SRC1_SENSORHUB_END_OP          (1 << 0)
#define LSM6DSL_FUNC_SRC1_SI_END_OP                 (1 << 1)
#define LSM6DSL_FUNC_SRC1_HI_FAIL                   (1 << 2)
#define LSM6DSL_FUNC_SRC1_STEP_OVERFLOW             (1 << 3)
#define LSM6DSL_FUNC_SRC1_STEP_DETECTED             (1 << 4)
#define LSM6DSL_FUNC_SRC1_TILT_IA                   (1 << 5)
#define LSM6DSL_FUNC_SRC1_SIGN_MOTION_IA            (1 << 6)
#define LSM6DSL_FUNC_SRC1_STEP_COUNT_DELTA_IA       (1 << 7)

#define LSM6DSL_FUNC_SRC2_WRIST_TILT_IA             (1 << 0)
#define LSM6DSL_FUNC_SRC2_SLAVE0_NACK               (1 << 3)
#define LSM6DSL_FUNC_SRC2_SLAVE1_NACK               (1 << 4)
#define LSM6DSL_FUNC_SRC2_SLAVE2_NACK               (1 << 5)
#define LSM6DSL_FUNC_SRC2_SLAVE3_NACK               (1 << 6)

#define LSM6DSL_WRIST_TILT_IA_SHIFT                 2
#define LSM6DSL_WRIST_TILT_IA_MASK                  (63 << LSM6DSL_WRIST_TILT_IA_SHIFT)

#define LSM6DSL_TAP_CFG_LIR                         (1 << 0)
#define LSM6DSL_TAP_CFG_TAP_Z_EN                    (1 << 1)
#define LSM6DSL_TAP_CFG_TAP_Y_EN                    (1 << 2)
#define LSM6DSL_TAP_CFG_TAP_X_EN                    (1 << 3)
#define LSM6DSL_TAP_CFG_SLOPE_FDS                   (1 << 4)
#define LSM6DSL_TAP_CFG_INACT_EN_SHIFT              2
#define LSM6DSL_TAP_CFG_INACT_EN_MASK               (3 << LSM6DSL_TAP_CFG_INACT_EN_SHIFT)
#define LSM6DSL_TAP_CFG_INTERRUPTS_ENABLE           (1 << 7)

#define LSM6DSL_TAP_THS_6D_TAP_THS_SHIFT            0
#define LSM6DSL_TAP_THS_6D_TAP_THS_MASK             (31 << LSM6DSL_TAP_THS_6D_TAP_THS_SHIFT)
#define LSM6DSL_TAP_THS_6D_SIXD_THS_SHIFT           5
#define LSM6DSL_TAP_THS_6D_SIXD_THS_MASK            (3 << LSM6DSL_TAP_THS_6D_SIXD_THS_SHIFT)
#define LSM6DSL_TAP_THS_6D_SIXD_THS_80DEGR          (0 << LSM6DSL_TAP_THS_6D_SIXD_THS_SHIFT)
#define LSM6DSL_TAP_THS_6D_SIXD_THS_70DEGR          (1 << LSM6DSL_TAP_THS_6D_SIXD_THS_SHIFT)
#define LSM6DSL_TAP_THS_6D_SIXD_THS_60DEGR          (2 << LSM6DSL_TAP_THS_6D_SIXD_THS_SHIFT)
#define LSM6DSL_TAP_THS_6D_SIXD_THS_50DEGR          (3 << LSM6DSL_TAP_THS_6D_SIXD_THS_SHIFT)
#define LSM6DSL_TAP_THS_6D_D4D_EN                   (1 << 7)

#define LSM6DSL_INT_DUR2_SHOCK_SHIFT                0
#define LSM6DSL_INT_DUR2_SHOCK_MASK                 (3 << LSM6DSL_INT_DUR2_SHOCK_SHIFT)
#define LSM6DSL_INT_DUR2_QUIET_SHIFT                2
#define LSM6DSL_INT_DUR2_QUIET_MASK                 (3 << LSM6DSL_INT_DUR2_QUIET_SHIFT)
#define LSM6DSL_INT_DUR2_DUR_SHIFT                  4
#define LSM6DSL_INT_DUR2_DUR_MASK                   (7 << LSM6DSL_INT_DUR2_QUIET_SHIFT)

#define LSM6DSL_WAKE_UP_THS_WK_THS_SHIFT            0
#define LSM6DSL_WAKE_UP_THS_WK_THS_MASK             (31 << LSM6DSL_WAKE_UP_THS_WK_THS_SHIFT)
#define LSM6DSL_WAKE_UP_THS_SINGLE_DOUBLE_TAP4D_EN  (1 << 7)

#define LSM6DSL_WAKE_UP_DUR_SLEEP_DUR_SHIFT         0
#define LSM6DSL_WAKE_UP_DUR_SLEEP_DUR_MASK          (15 << LSM6DSL_WAKE_UP_DUR_SLEEP_DUR_SHIFT)
#define LSM6DSL_WAKE_UP_DUR_TIMER_HR                (1 << 4)
#define LSM6DSL_WAKE_UP_DUR_WAKE_DUR_SHIFT          5
#define LSM6DSL_WAKE_UP_DUR_WAKE_DUR_MASK           (15 << LSM6DSL_WAKE_UP_DUR_WAKE_DUR_SHIFT)
#define LSM6DSL_WAKE_UP_DUR_FF_DUR5                 (1 << 7)

#define LSM6DSL_FREE_FALL_FF_THS_SHIFT              0
#define LSM6DSL_FREE_FALL_FF_THS_MASK               (7 << LSM6DSL_FREE_FALL_FF_THS_SHIFT)
#define LSM6DSL_FREE_FALL_FF_DUR_SHIFT              4
#define LSM6DSL_FREE_FALL_FF_DUR_MASK               (31 << LSM6DSL_FREE_FALL_FF_DUR_SHIFT)

#define LSM6DSL_MD1_CFG_INT1_TIMER                  (1 << 0)
#define LSM6DSL_MD1_CFG_INT1_TILT                   (1 << 1)
#define LSM6DSL_MD1_CFG_INT1_6D                     (1 << 2)
#define LSM6DSL_MD1_CFG_INT1_DOUBLE_TAP             (1 << 3)
#define LSM6DSL_MD1_CFG_INT1_FF                     (1 << 4)
#define LSM6DSL_MD1_CFG_INT1_WU                     (1 << 5)
#define LSM6DSL_MD1_CFG_INT1_SINGLE_TAP             (1 << 6)
#define LSM6DSL_MD1_CFG_INT1_INACT_STATE            (1 << 7)

#define LSM6DSL_MD2_CFG_INT1_IRON                   (1 << 0)
#define LSM6DSL_MD2_CFG_INT1_TILT                   (1 << 1)
#define LSM6DSL_MD2_CFG_INT1_6D                     (1 << 2)
#define LSM6DSL_MD2_CFG_INT1_DOUBLE_TAP             (1 << 3)
#define LSM6DSL_MD2_CFG_INT1_FF                     (1 << 4)
#define LSM6DSL_MD2_CFG_INT1_WU                     (1 << 5)
#define LSM6DSL_MD2_CFG_INT1_SINGLE_TAP             (1 << 6)
#define LSM6DSL_MD2_CFG_INT1_INACT_STATE            (1 << 7)

#define LSM6DSL_MASTER_CMD_CODE_SHIFT               0
#define LSM6DSL_MASTER_CMD_CODE_MASK                (255 << LSM6DSL_MASTER_CMD_CODE_SHIFT)

#define LSM6DSL_SENS_SYNC_SPI_ERROR_CODE_SHIFT      0
#define LSM6DSL_SENS_SYNC_SPI_ERROR_CODE_MASK       (255 << LSM6DSL_SENS_SYNC_SPI_ERROR_CODE_SHIFT)

#define LSM6DSL_OUT_MAG_RAW_X_L_SHIFT               0
#define LSM6DSL_OUT_MAG_RAW_X_L_MASK                (255 << LSM6DSL_OUT_MAG_RAW_X_L_SHIFT)

#define LSM6DSL_OUT_MAG_RAW_X_H_SHIFT               0
#define LSM6DSL_OUT_MAG_RAW_X_H_MASK                (255 << LSM6DSL_OUT_MAG_RAW_X_H_SHIFT)

#define LSM6DSL_OUT_MAG_RAW_Y_L_SHIFT               0
#define LSM6DSL_OUT_MAG_RAW_Y_L_MASK                (255 << LSM6DSL_OUT_MAG_RAW_Y_L_SHIFT)

#define LSM6DSL_OUT_MAG_RAW_Y_H_SHIFT               0
#define LSM6DSL_OUT_MAG_RAW_Y_H_MASK                (255 << LSM6DSL_OUT_MAG_RAW_Y_H_SHIFT)

#define LSM6DSL_OUT_MAG_RAW_Z_L_SHIFT               0
#define LSM6DSL_OUT_MAG_RAW_Z_L_MASK                (255 << LSM6DSL_OUT_MAG_RAW_Z_L_SHIFT)

#define LSM6DSL_OUT_MAG_RAW_Z_H_SHIFT               0
#define LSM6DSL_OUT_MAG_RAW_Z_H_MASK                (255 << LSM6DSL_OUT_MAG_RAW_Z_H_SHIFT)

#define LSM6DSL_X_OFS_USR_SHIFT                     0
#define LSM6DSL_X_OFS_USR_MASK                      (255 << LSM6DSL_X_OFS_USR_SHIFT)

#define LSM6DSL_Y_OFS_USR_SHIFT                     0
#define LSM6DSL_Y_OFS_USR_MASK                      (255 << LSM6DSL_Y_OFS_USR_SHIFT)

#define LSM6DSL_Z_OFS_USR_SHIFT                     0
#define LSM6DSL_Z_OFS_USR_MASK                      (255 << LSM6DSL_Z_OFS_USR_SHIFT)

/* Embedded functions registers description - Bank A */

#define LSM6DSL_SLV0_ADD_rw_0                       (1 << 0)
#define LSM6DSL_SLV0_ADD_Slave0_add_SHIFT           1
#define LSM6DSL_SLV0_ADD_Slave0_add_MASK            (127 << LSM6DSL_SLV0_ADD_Slave0_add_SHIFT)

#define LSM6DSL_SLV0_SUBADD_SHIFT                   0
#define LSM6DSL_SLV0_SUBADD_MASK                    (255 << LSM6DSL_SLV0_SUBADD_SHIFT)

#define LSM6DSL_SLAVE0_CONFIG_Slave0_numop_SHIFT    0
#define LSM6DSL_SLAVE0_CONFIG_Slave0_numop_MASK     (7 << LSM6DSL_SLAVE0_CONFIG_Slave0_numop_SHIFT)
#define LSM6DSL_SLAVE0_CONFIG_Src_mode              (1 << 3)
#define LSM6DSL_SLAVE0_CONFIG_Aux_sens_on_SHIFT     4
#define LSM6DSL_SLAVE0_CONFIG_Aux_sens_on_MASK      (3 << LSM6DSL_SLAVE0_CONFIG_Aux_sens_on_SHIFT)
#define LSM6DSL_SLAVE0_CONFIG_Slave0_rate_SHIFT     6
#define LSM6DSL_SLAVE0_CONFIG_Slave0_rate_MASK      (3 << LSM6DSL_SLAVE0_CONFIG_Slave0_rate_SHIFT)

#define LSM6DSL_SLV1_ADD_r_1                        (1 << 0)
#define LSM6DSL_SLV1_ADD_Slave1_add_SHIFT           1
#define LSM6DSL_SLV1_ADD_Slave1_add_MASK            (127 << LSM6DSL_SLV1_ADD_Slave1_add_SHIFT)

#define LSM6DSL_SLV1_SUBADD_SHIFT                   0
#define LSM6DSL_SLV1_SUBADD_MASK                    (255 << LSM6DSL_SLV1_SUBADD_SHIFT)

#define LSM6DSL_SLAVE1_CONFIG_Slave1_numop_SHIFT    0
#define LSM6DSL_SLAVE1_CONFIG_Slave1_numop_MASK     (7 << LSM6DSL_SLAVE1_CONFIG_Slave1_numop_SHIFT)
#define LSM6DSL_SLAVE1_CONFIG_write_once            (1 << 5)
#define LSM6DSL_SLAVE1_CONFIG_Slave1_rate_SHIFT     6
#define LSM6DSL_SLAVE1_CONFIG_Slave1_rate_MASK      (3 << LSM6DSL_SLAVE1_CONFIG_Slave1_rate_SHIFT)

#define LSM6DSL_SLV2_ADD_r_2                        (1 << 0)
#define LSM6DSL_SLV2_ADD_Slave2_add_SHIFT           1
#define LSM6DSL_SLV2_ADD_Slave2_add_MASK            (127 << LSM6DSL_SLV2_ADD_Slave2_add_SHIFT)

#define LSM6DSL_SLV2_SUBADD_SHIFT                   0
#define LSM6DSL_SLV2_SUBADD_MASK                    (255 << LSM6DSL_SLV2_SUBADD_SHIFT)

#define LSM6DSL_SLAVE2_CONFIG_Slave2_numop_SHIFT    0
#define LSM6DSL_SLAVE2_CONFIG_Slave2_numop_MASK     (7 << LSM6DSL_SLAVE1_CONFIG_Slave1_numop_SHIFT)
#define LSM6DSL_SLAVE2_CONFIG_Slave2_rate_SHIFT     6
#define LSM6DSL_SLAVE2_CONFIG_Slave2_rate_MASK      (3 << LSM6DSL_SLAVE1_CONFIG_Slave1_rate_SHIFT)

#define LSM6DSL_SLV3_ADD_r_3                        (1 << 0)
#define LSM6DSL_SLV3_ADD_Slave3_add_SHIFT           1
#define LSM6DSL_SLV3_ADD_Slave3_add_MASK            (127 << LSM6DSL_SLV3_ADD_Slave3_add_SHIFT)

#define LSM6DSL_SLV3_SUBADD_SHIFT                   0
#define LSM6DSL_SLV3_SUBADD_MASK                    (255 << LSM6DSL_SLV2_SUBADD_SHIFT)

#define LSM6DSL_SLAVE3_CONFIG_Slave3_numop_SHIFT    0
#define LSM6DSL_SLAVE3_CONFIG_Slave3_numop_MASK     (7 << LSM6DSL_SLAVE1_CONFIG_Slave1_numop_SHIFT)
#define LSM6DSL_SLAVE3_CONFIG_Slave3_rate_SHIFT     6
#define LSM6DSL_SLAVE3_CONFIG_Slave3_rate_MASK      (3 << LSM6DSL_SLAVE1_CONFIG_Slave1_rate_SHIFT)

#define LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0_SHIFT   0
#define LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0_MASK    (255 << LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0_SHIFT)

#define LSM6DSL_CONFIG_PEDO_THS_MIN_ths_min_SHIFT   0
#define LSM6DSL_CONFIG_PEDO_THS_MIN_ths_min_MASK    (31 << LSM6DSL_CONFIG_PEDO_THS_MIN_ths_min_SHIFT)
#define LSM6DSL_CONFIG_PEDO_THS_MIN_PEDO_FS         (1 << 7>

#define LSM6DSL_SM_THS_SHIFT                        0
#define LSM6DSL_SM_THS_MASK                         (255 << LSM6DSL_SM_THS_SHIFT)

#define LSM6DSL_PEDO_DEB_REG_DEB_STEP_SHIFT         0
#define LSM6DSL_PEDO_DEB_REG_DEB_STEP_MASK          (7 << LSM6DSL_PEDO_DEB_REG_DEB_STEP_SHIFT)
#define LSM6DSL_PEDO_DEB_REG_DEB_TIME_SHIFT         3
#define LSM6DSL_PEDO_DEB_REG_DEB_TIME_MASK          (31 << LSM6DSL_PEDO_DEB_REG_DEB_TIME_SHIFT)

#define LSM6DSL_STEP_COUNT_DELTA_SHIFT              0
#define LSM6DSL_STEP_COUNT_DELTA_MASK               (255 << LSM6DSL_STEP_COUNT_DELTA_SHIFT)

#define LSM6DSL_MAG_SI_XX_SHIFT                     0
#define LSM6DSL_MAG_SI_XX_MASK                      (255 << LSM6DSL_MAG_SI_XX_SHIFT)

#define LSM6DSL_MAG_SI_XY_SHIFT                     0
#define LSM6DSL_MAG_SI_XY_MASK                      (255 << LSM6DSL_MAG_SI_XY_SHIFT)

#define LSM6DSL_MAG_SI_XZ_SHIFT                     0
#define LSM6DSL_MAG_SI_XZ_MASK                      (255 << LSM6DSL_MAG_SI_XZ_SHIFT)

#define LSM6DSL_MAG_SI_YX_SHIFT                     0
#define LSM6DSL_MAG_SI_YX_MASK                      (255 << LSM6DSL_MAG_SI_YX_SHIFT)

#define LSM6DSL_MAG_SI_YY_SHIFT                     0
#define LSM6DSL_MAG_SI_YY_MASK                      (255 << LSM6DSL_MAG_SI_YY_SHIFT)

#define LSM6DSL_MAG_SI_YZ_SHIFT                     0
#define LSM6DSL_MAG_SI_YZ_MASK                      (255 << LSM6DSL_MAG_SI_YZ_SHIFT)

#define LSM6DSL_MAG_SI_ZX_SHIFT                     0
#define LSM6DSL_MAG_SI_ZX_MASK                      (255 << LSM6DSL_MAG_SI_ZX_SHIFT)

#define LSM6DSL_MAG_SI_ZY_SHIFT                     0
#define LSM6DSL_MAG_SI_ZY_MASK                      (255 << LSM6DSL_MAG_SI_ZY_SHIFT)

#define LSM6DSL_MAG_SI_ZZ_SHIFT                     0
#define LSM6DSL_MAG_SI_ZZ_MASK                      (255 << LSM6DSL_MAG_SI_ZZ_SHIFT)

#define LSM6DSL_MAG_OFFX_L_SHIFT                    0
#define LSM6DSL_MAG_OFFX_L_MASK                     (255 << LSM6DSL_MAG_OFFX_L_SHIFT)

#define LSM6DSL_MAG_OFFX_H_SHIFT                    0
#define LSM6DSL_MAG_OFFX_H_MASK                     (255 << LSM6DSL_MAG_OFFX_H_SHIFT)

#define LSM6DSL_MAG_OFFY_L_SHIFT                    0
#define LSM6DSL_MAG_OFFY_L_MASK                     (255 << LSM6DSL_MAG_OFFY_L_SHIFT)

#define LSM6DSL_MAG_OFFY_H_SHIFT                    0
#define LSM6DSL_MAG_OFFY_H_MASK                     (255 << LSM6DSL_MAG_OFFY_H_SHIFT)

#define LSM6DSL_MAG_OFFZ_L_SHIFT                    0
#define LSM6DSL_MAG_OFFZ_L_MASK                     (255 << LSM6DSL_MAG_OFFZ_L_SHIFT)

#define LSM6DSL_MAG_OFFZ_H_SHIFT                    0
#define LSM6DSL_MAG_OFFZ_H_MASK                     (255 << LSM6DSL_MAG_OFFZ_H_SHIFT)

/* Embedded functions registers description - Bank B */

#define LSM6DSL_A_WRIST_TILT_LAT_SHIFT              0
#define LSM6DSL_A_WRIST_TILT_LAT_MASK               (255 << LSM6DSL_A_WRIST_TILT_LAT_SHIFT)

#define LSM6DSL_A_WRIST_TILT_THS_SHIFT              0
#define LSM6DSL_A_WRIST_TILT_THS_MASK               (255 << LSM6DSL_A_WRIST_TILT_THS_SHIFT)

#define LSM6DSL_A_WRIST_TILT_Mask_SHIFT             2
#define LSM6DSL_A_WRIST_TILT_Mask_MASK              (63 << LSM6DSL_A_WRIST_TILT_Mask_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Container for sensor data */

struct lsm6dsl_sensor_data_s
{
  int16_t  x_data;
  int16_t  y_data;
  int16_t  z_data;
  uint16_t temperature;
  int16_t  g_x_data;
  int16_t  g_y_data;
  int16_t  g_z_data;
  uint16_t timestamp;
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lsm6dsl_dev_s;
struct lsm6dsl_ops_s
{
  CODE int (*config)(FAR struct lsm6dsl_dev_s *priv);
  CODE int (*start)(FAR struct lsm6dsl_dev_s *priv);
  CODE int (*stop)(FAR struct lsm6dsl_dev_s *priv);
  CODE int (*sensor_read)(FAR struct lsm6dsl_dev_s *priv,
                          FAR struct lsm6dsl_sensor_data_s *sensor_data);
  CODE int (*selftest)(FAR struct lsm6dsl_dev_s *priv,
                       uint32_t mode);
};

struct lsm6dsl_dev_s
{
  FAR struct i2c_master_s        *i2c;        /* I2C interface */
  uint8_t                         addr;       /* I2C address */

  FAR const struct lsm6dsl_ops_s *ops;

  uint8_t                         datareg;     /* Output data register of X low byte */
  struct lsm6dsl_sensor_data_s    sensor_data; /* Sensor data container     */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: lsm6dslaccel_register
 *
 * Description:
 *   Register the LSM6DSL accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/sensor0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dsl_sensor_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c,
                            uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM9DS1 */
#endif /* __INCLUDE_NUTTX_SENSORS_LSM6DSL_H */
