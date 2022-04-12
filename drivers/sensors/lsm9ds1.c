/****************************************************************************
 * drivers/sensors/lsm9ds1.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm9ds1.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM9DS1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LSM9DS1_I2C_FREQUENCY
#  define CONFIG_LSM9DS1_I2C_FREQUENCY 400000
#endif

/* Register Addresses *******************************************************/

/* Accelerometer and gyroscope registers */

#define LSM9DS1_ACT_THS                         0x04 /* Inactivity threshold */
#define LSM9DS1_ACT_DUR                         0x05 /* Inactivity duration */
#define LSM9DS1_INT_GEN_CFG_XL                  0x06 /* Accelerometer interrupt configuration */
#define LSM9DS1_INT_GEN_THS_X_XL                0x07 /* Accelerometer X interrupt threshold */
#define LSM9DS1_INT_GEN_THS_Y_XL                0x08 /* Accelerometer Y interrupt threshold */
#define LSM9DS1_INT_GEN_THS_Z_XL                0x09 /* Accelerometer Z interrupt threshold */
#define LSM9DS1_INT_GEN_DUR_XL                  0x0a /* Accelerometer interrupt duration */
#define LSM9DS1_REFERENCE_G                     0x0b /* Gyroscope reference value for high-pass filter */
#define LSM9DS1_INT1_CTRL                       0x0c /* INT1_A/G pin control */
#define LSM9DS1_INT2_CTRL                       0x0d /* INT2_A/G pin control */
#define LSM9DS1_WHO_AM_I                        0x0f /* Accelerometer and gyroscope device identification */
#define LSM9DS1_CTRL_REG1_G                     0x10 /* Gyroscope control register 1 */
#define LSM9DS1_CTRL_REG2_G                     0x11 /* Gyroscope control register 2 */
#define LSM9DS1_CTRL_REG3_G                     0x12 /* Gyroscope control register 3 */
#define LSM9DS1_ORIENT_CFG_G                    0x13 /* Gyroscope sign and orientation */
#define LSM9DS1_INT_GEN_SRC_G                   0x14 /* Gyroscope interrupt source */
#define LSM9DS1_OUT_TEMP_L                      0x15 /* Temperature low byte */
#define LSM9DS1_OUT_TEMP_H                      0x16 /* Temperature high byte */
#define LSM9DS1_STATUS_REG                      0x17 /* Status register */
#define LSM9DS1_OUT_X_L_G                       0x18 /* Gyroscope pitch (X) low byte */
#define LSM9DS1_OUT_X_H_G                       0x19 /* Gyroscope pitch (X) high byte */
#define LSM9DS1_OUT_Y_L_G                       0x1a /* Gyroscope roll (Y) low byte */
#define LSM9DS1_OUT_Y_H_G                       0x1b /* Gyroscope roll (Y) high byte */
#define LSM9DS1_OUT_Z_L_G                       0x1c /* Gyroscope yaw (Z) low byte */
#define LSM9DS1_OUT_Z_H_G                       0x1d /* Gyroscope yaw (Z) high byte */
#define LSM9DS1_CTRL_REG4                       0x1e /* Control register 4 */
#define LSM9DS1_CTRL_REG5_XL                    0x1f /* Accelerometer control register 5 */
#define LSM9DS1_CTRL_REG6_XL                    0x20 /* Accelerometer control register 6 */
#define LSM9DS1_CTRL_REG7_XL                    0x21 /* Accelerometer control register 7 */
#define LSM9DS1_CTRL_REG8                       0x22 /* Control register 8 */
#define LSM9DS1_CTRL_REG9                       0x23 /* Control register 9 */
#define LSM9DS1_CTRL_REG10                      0x24 /* Control register 10 */
#define LSM9DS1_INT_GEN_SRC_XL                  0x26 /* Accelerometer interrupt source */
#define LSM9DS1_STATUS_REG2                     0x27 /* Status register 2 */
#define LSM9DS1_OUT_X_L_XL                      0x28 /* Accelerometer X low byte */
#define LSM9DS1_OUT_X_H_XL                      0x29 /* Accelerometer X high byte */
#define LSM9DS1_OUT_Y_L_XL                      0x2a /* Accelerometer Y low byte */
#define LSM9DS1_OUT_Y_H_XL                      0x2b /* Accelerometer Y high byte */
#define LSM9DS1_OUT_Z_L_XL                      0x2c /* Accelerometer Z low byte */
#define LSM9DS1_OUT_Z_H_XL                      0x2d /* Accelerometer Z high byte */
#define LSM9DS1_FIFO_CTRL                       0x2e /* FIFO control register */
#define LSM9DS1_FIFO_SRC                        0x2f /* FIFO status control register */
#define LSM9DS1_INT_GEN_CFG_G                   0x30 /* Gyroscope interrupt configuration */
#define LSM9DS1_INT_GEN_THS_XH_G                0x31 /* Gyroscope pitch (X) interrupt threshold high byte */
#define LSM9DS1_INT_GEN_THS_XL_G                0x32 /* Gyroscope pitch (X) interrupt threshold low byte */
#define LSM9DS1_INT_GEN_THS_YH_G                0x33 /* Gyroscope roll (Y) interrupt threshold high byte */
#define LSM9DS1_INT_GEN_THS_YL_G                0x34 /* Gyroscope roll (Y) interrupt threshold low byte */
#define LSM9DS1_INT_GEN_THS_ZH_G                0x35 /* Gyroscope yaw (Z) interrupt threshold high byte */
#define LSM9DS1_INT_GEN_THS_ZL_G                0x36 /* Gyroscope yaw (Z) interrupt threshold low byte */
#define LSM9DS1_INT_GEN_DUR_G                   0x37 /* Gyroscope interrupt duration */

/* Magnetometer registers */

#define LSM9DS1_OFFSET_X_REG_L_M                0x05 /* X low byte offset */
#define LSM9DS1_OFFSET_X_REG_H_M                0x06 /* X high byte offset */
#define LSM9DS1_OFFSET_Y_REG_L_M                0x07 /* Y low byte offset */
#define LSM9DS1_OFFSET_Y_REG_H_M                0x08 /* Y high byte offset */
#define LSM9DS1_OFFSET_Z_REG_L_M                0x09 /* Z low byte offset */
#define LSM9DS1_OFFSET_Z_REG_H_M                0x0a /* Z high byte offset */
#define LSM9DS1_WHO_AM_I_M                      0x0f /* Device identification */
#define LSM9DS1_CTRL_REG1_M                     0x20 /* Control register 1 */
#define LSM9DS1_CTRL_REG2_M                     0x21 /* Control register 2 */
#define LSM9DS1_CTRL_REG3_M                     0x22 /* Control register 3 */
#define LSM9DS1_CTRL_REG4_M                     0x23 /* Control register 4 */
#define LSM9DS1_CTRL_REG5_M                     0x24 /* Control register 5 */
#define LSM9DS1_STATUS_REG_M                    0x27 /* Status register */
#define LSM9DS1_OUT_X_L_M                       0x28 /* X low byte */
#define LSM9DS1_OUT_X_H_M                       0x29 /* X high byte */
#define LSM9DS1_OUT_Y_L_M                       0x2a /* Y low byte */
#define LSM9DS1_OUT_Y_H_M                       0x2b /* Y high byte */
#define LSM9DS1_OUT_Z_L_M                       0x2c /* Z low byte */
#define LSM9DS1_OUT_Z_H_M                       0x2d /* Z high byte */
#define LSM9DS1_INT_CFG_M                       0x30 /* Interrupt configuration */
#define LSM9DS1_INT_SRC_M                       0x31 /* Interrupt source */
#define LSM9DS1_INT_THS_L_M                     0x32 /* Interrupt threshold low byte */
#define LSM9DS1_INT_THS_H_M                     0x33 /* Interrupt threshold high byte */

/* Register Bit Definitions *************************************************/

/* Inactivity threshold register */

#define LSM9DS1_ACT_THS_ACT_THS_SHIFT           0 /* Inactivity threshold */
#define LSM9DS1_ACT_THS_ACT_THS_MASK            (127 << LSM9DS1_ACT_THS_ACT_THS_SHIFT)
#define LSM9DS1_ACT_THS_SLEEP_ON_INACT_EN       (1 << 7) /* Gyroscope operating mode during inactivity */

/* Accelerometer interrupt configuration register */

#define LSM9DS1_INT_GEN_CFG_XL_XLIE_XL          (1 << 0) /* X-axis low byte interrupt enable */
#define LSM9DS1_INT_GEN_CFG_XL_XHIE_XL          (1 << 1) /* X-axis high byte interrupt enable */
#define LSM9DS1_INT_GEN_CFG_XL_YLIE_XL          (1 << 2) /* Y-axis low byte interrupt enable */
#define LSM9DS1_INT_GEN_CFG_XL_YHIE_XL          (1 << 3) /* Y-axis high byte interrupt enable */
#define LSM9DS1_INT_GEN_CFG_XL_ZLIE_XL          (1 << 4) /* Z-axis low byte interrupt enable */
#define LSM9DS1_INT_GEN_CFG_XL_ZHIE_XL          (1 << 5) /* Z-axis high byte interrupt enable */
#define LSM9DS1_INT_GEN_CFG_XL_6D               (1 << 6) /* 6-direction detection function for interrupt */
#define LSM9DS1_INT_GEN_CFG_XL_AOI_XL           (1 << 7) /* AND/OR combination of interrupt events */

/* Accelerometer interrupt duration register */

#define LSM9DS1_INT_GEN_DUR_XL_DUR_XL_SHIFT     0 /* Enter/exit interrupt duration */
#define LSM9DS1_INT_GEN_DUR_XL_DUR_XL_MASK      (127 << LSM9DS1_INT_GEN_DUR_XL_DUR_XL_SHIFT)
#define LSM9DS1_INT_GEN_DUR_XL_WAIT_XL          (1 << 7) /* Wait function enabled on duration counter */

/* INT1_A/G pin control register */

#define LSM9DS1_INT1_CTRL_INT1_DRDY_XL          (1 << 0) /* Accelerometer data ready */
#define LSM9DS1_INT1_CTRL_INT1_DRDY_G           (1 << 1) /* Gyroscope data ready */
#define LSM9DS1_INT1_CTRL_INT1_BOOT             (1 << 2) /* Boot status available */
#define LSM9DS1_INT1_CTRL_INT1_FTH              (1 << 3) /* FIFO threshold interrupt */
#define LSM9DS1_INT1_CTRL_INT1_OVR              (1 << 4) /* Overrun interrupt */
#define LSM9DS1_INT1_CTRL_INT1_FSS5             (1 << 5) /* FSS5 interrupt */
#define LSM9DS1_INT1_CTRL_INT1_IG_XL            (1 << 6) /* Accelerometer interrupt enable */
#define LSM9DS1_INT1_CTRL_INT1_IG_G             (1 << 7) /* Gyroscope interrupt enable */

/* INT2_A/G pin control register */

#define LSM9DS1_INT2_CTRL_INT2_DRDY_XL          (1 << 0) /* Accelerometer data ready */
#define LSM9DS1_INT2_CTRL_INT2_DRDY_G           (1 << 1) /* Gyroscope data ready */
#define LSM9DS1_INT2_CTRL_INT2_DRDY_TEMP        (1 << 2) /* Temperature data ready */
#define LSM9DS1_INT2_CTRL_INT2_FTH              (1 << 3) /* FIFO threshold interrupt */
#define LSM9DS1_INT2_CTRL_INT2_OVR              (1 << 4) /* Overrun interrupt */
#define LSM9DS1_INT2_CTRL_INT2_FSS5             (1 << 5) /* FSS5 interrupt */
#define LSM9DS1_INT2_CTRL_INT2_INACT            (1 << 7) /* Inactivity interrupt output signal */

/* Device identification register */

#define LSM9DS1_WHO_AM_I_VALUE                  0x68

/* Gyroscope control register 1 */

#define LSM9DS1_CTRL_REG1_G_BW_G_SHIFT          0 /* Gyroscope bandwidth selection */
#define LSM9DS1_CTRL_REG1_G_BW_G_MASK           (3 << LSM9DS1_CTRL_REG1_G_BW_G_SHIFT)
#define LSM9DS1_CTRL_REG1_G_FS_G_SHIFT          3 /* Gyroscope full-scale selection */
#define LSM9DS1_CTRL_REG1_G_FS_G_MASK           (3 << LSM9DS1_CTRL_REG1_G_FS_G_SHIFT)
#  define LSM9DS1_CTRL_REG1_G_FS_G_245DPS       (0 << LSM9DS1_CTRL_REG1_G_FS_G_SHIFT) /* 245 dps */
#  define LSM9DS1_CTRL_REG1_G_FS_G_500DPS       (1 << LSM9DS1_CTRL_REG1_G_FS_G_SHIFT) /* 500 dps */
#  define LSM9DS1_CTRL_REG1_G_FS_G_2000DPS      (3 << LSM9DS1_CTRL_REG1_G_FS_G_SHIFT) /* 2000 dps */

#define LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT         5 /* Gyroscope bandwidth selection */
#define LSM9DS1_CTRL_REG1_G_ODR_G_MASK          (7 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT)
#  define LSM9DS1_CTRL_REG1_G_ODR_G_POWERDOWN   (0 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT) /* Power-down mode */
#  define LSM9DS1_CTRL_REG1_G_ODR_G_14p9HZ      (1 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT) /* 14.9 Hz */
#  define LSM9DS1_CTRL_REG1_G_ODR_G_59p5HZ      (2 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT) /* 59.5 Hz */
#  define LSM9DS1_CTRL_REG1_G_ODR_G_119HZ       (3 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT) /* 119 Hz */
#  define LSM9DS1_CTRL_REG1_G_ODR_G_238HZ       (4 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT) /* 238 Hz */
#  define LSM9DS1_CTRL_REG1_G_ODR_G_476HZ       (5 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT) /* 476 Hz */
#  define LSM9DS1_CTRL_REG1_G_ODR_G_952HZ       (6 << LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT) /* 952 Hz */

/* Gyroscope control register 2 */

#define LSM9DS1_CTRL_REG2_G_OUT_SEL_SHIFT       0 /* Out selection configuration */
#define LSM9DS1_CTRL_REG2_G_OUT_SEL_MASK        (3 << LSM9DS1_CTRL_REG2_G_OUT_SEL_SHIFT)
#define LSM9DS1_CTRL_REG2_G_INT_SEL_SHIFT       2 /* INT selection configuration */
#define LSM9DS1_CTRL_REG2_G_INT_SEL_MASK        (3 << LSM9DS1_CTRL_REG2_G_INT_SEL_SHIFT)

/* Gyroscope control register 3 */

#define LSM9DS1_CTRL_REG3_G_HPCF_G_SHIFT        0 /* Gyroscope high-pass filter cutoff frequency selection */
#define LSM9DS1_CTRL_REG3_G_HPCF_G_MASK         (15 << LSM9DS1_CTRL_REG3_G_HPCF_G_SHIFT)
#define LSM9DS1_CTRL_REG3_G_HP_EN               (1 << 6) /* High-pass filter enable */
#define LSM9DS1_CTRL_REG3_G_LP_MODE             (1 << 7) /* Low-power mode enable */

/* Gyroscope sign and orientation register */

#define LSM9DS1_ORIENT_CFG_G_ORIENT_SHIFT       0 /* Directional user orientation selection */
#define LSM9DS1_ORIENT_CFG_G_ORIENT_MASK        (3 << LSM9DS1_ORIENT_CFG_G_ORIENT_SHIFT)
#define LSM9DS1_ORIENT_CFG_G_SIGNZ_G            (1 << 3) /* Yaw axis (Z) angular rate sign */
#define LSM9DS1_ORIENT_CFG_G_SIGNY_G            (1 << 4) /* Roll axis (Y) angular rate sign */
#define LSM9DS1_ORIENT_CFG_G_SIGNX_G            (1 << 5) /* Pitch axis (X) angular rate sign */

/* Gyroscope interrupt source register */

#define LSM9DS1_INT_GEN_SRC_G_XL_G              (1 << 0) /* Pitch (X) low */
#define LSM9DS1_INT_GEN_SRC_G_XH_G              (1 << 1) /* Pitch (X) high */
#define LSM9DS1_INT_GEN_SRC_G_YL_G              (1 << 2) /* Roll (Y) low */
#define LSM9DS1_INT_GEN_SRC_G_YH_G              (1 << 3) /* Roll (Y) high */
#define LSM9DS1_INT_GEN_SRC_G_ZL_G              (1 << 4) /* Yaw (Z) low */
#define LSM9DS1_INT_GEN_SRC_G_ZH_G              (1 << 5) /* Yaw (Z) high */
#define LSM9DS1_INT_GEN_SRC_G_IA_G              (1 << 6) /* Interrupt active */

/* Status register */

#define LSM9DS1_STATUS_REG_XLDA                 (1 << 0) /* Accelerometer new data available */
#define LSM9DS1_STATUS_REG_GDA                  (1 << 1) /* Gyroscope new data available */
#define LSM9DS1_STATUS_REG_TDA                  (1 << 2) /* Temperature sensor new data available */
#define LSM9DS1_STATUS_REG_BOOT_STATUS          (1 << 3) /* Boot running flag signal */
#define LSM9DS1_STATUS_REG_INACT                (1 << 4) /* Inactivity interrupt output signal */
#define LSM9DS1_STATUS_REG_IG_G                 (1 << 5) /* Gyroscope interrupt output signal */
#define LSM9DS1_STATUS_REG_IG_XL                (1 << 6) /* Accelerometer interrupt output signal */

/* Control register 4 */

#define LSM9DS1_CTRL_REG4_4D_XL1                (1 << 0) /* 4D option enabled on interrupt */
#define LSM9DS1_CTRL_REG4_LIR_XL1               (1 << 1) /* Latched interrupt */
#define LSM9DS1_CTRL_REG4_XEN_G                 (1 << 3) /* Gyroscope's pitch axis (X) output enable */
#define LSM9DS1_CTRL_REG4_YEN_G                 (1 << 4) /* Gyroscope's roll axis (Y) output enable */
#define LSM9DS1_CTRL_REG4_ZEN_G                 (1 << 5) /* Gyroscope's yaw axis (Z) output enable */

/* Accelerometer control register 5 */

#define LSM9DS1_CTRL_REG5_XL_XEN_XL             (1 << 3) /* Accelerometer's X-axis output enable */
#define LSM9DS1_CTRL_REG5_XL_YEN_XL             (1 << 4) /* Accelerometer's Y-axis output enable */
#define LSM9DS1_CTRL_REG5_XL_ZEN_XL             (1 << 5) /* Accelerometer's Z-axis output enable */

#define LSM9DS1_CTRL_REG5_XL_DEC_SHIFT          6 /* Decimation of acceleration data on OUT REG and FIFO  */
#define LSM9DS1_CTRL_REG5_XL_DEC_MASK           (3 << LSM9DS1_CTRL_REG5_XL_DEC_SHIFT)
#  define LSM9DS1_CTRL_REG5_XL_DEC_NODEC        (0 << LSM9DS1_CTRL_REG5_XL_DEC_SHIFT) /* No decimation */
#  define LSM9DS1_CTRL_REG5_XL_DEC_2SAMPLES     (1 << LSM9DS1_CTRL_REG5_XL_DEC_SHIFT) /* Update every 2 samples */
#  define LSM9DS1_CTRL_REG5_XL_DEC_4SAMPLES     (2 << LSM9DS1_CTRL_REG5_XL_DEC_SHIFT) /* Update every 4 samples */
#  define LSM9DS1_CTRL_REG5_XL_DEC_8SAMPLES     (3 << LSM9DS1_CTRL_REG5_XL_DEC_SHIFT) /* Update every 8 samples */

/* Accelerometer control register 6 */

#define LSM9DS1_CTRL_REG6_XL_BW_XL_SHIFT        0 /* Anti-aliasing filter bandwidth selection */
#define LSM9DS1_CTRL_REG6_XL_BW_XL_MASK         (3 << LSM9DS1_CTRL_REG6_XL_BW_XL_SHIFT)
#  define LSM9DS1_CTRL_REG6_XL_BW_XL_408HZ      (0 << LSM9DS1_CTRL_REG6_XL_BW_XL_SHIFT) /* 408 Hz */
#  define LSM9DS1_CTRL_REG6_XL_BW_XL_211HZ      (1 << LSM9DS1_CTRL_REG6_XL_BW_XL_SHIFT) /* 211 Hz */
#  define LSM9DS1_CTRL_REG6_XL_BW_XL_105HZ      (2 << LSM9DS1_CTRL_REG6_XL_BW_XL_SHIFT) /* 105 Hz */
#  define LSM9DS1_CTRL_REG6_XL_BW_XL_50HZ       (3 << LSM9DS1_CTRL_REG6_XL_BW_XL_SHIFT) /* 50 Hz */

#define LSM9DS1_CTRL_REG6_XL_BW_SCAL_ODR        (1 << 2) /* Bandwidth selection */

#define LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT        3 /* Accelerometer full-scale selection */
#define LSM9DS1_CTRL_REG6_XL_FS_XL_MASK         (3 << LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT)
#  define LSM9DS1_CTRL_REG6_XL_FS_XL_2G         (0 << LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT) /* +/- 2 g */
#  define LSM9DS1_CTRL_REG6_XL_FS_XL_16G        (1 << LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT) /* +/- 16 g */
#  define LSM9DS1_CTRL_REG6_XL_FS_XL_4G         (2 << LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT) /* +/- 4 g */
#  define LSM9DS1_CTRL_REG6_XL_FS_XL_8G         (3 << LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT) /* +/- 8 g */

#define LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT       5 /* Output data rate and power mode selection */
#define LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK        (7 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT)
#  define LSM9DS1_CTRL_REG6_XL_ODR_XL_POWERDOWN (0 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT) /* Power-down mode */
#  define LSM9DS1_CTRL_REG6_XL_ODR_XL_10HZ      (1 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT) /* 10 Hz */
#  define LSM9DS1_CTRL_REG6_XL_ODR_XL_50HZ      (2 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT) /* 50 Hz */
#  define LSM9DS1_CTRL_REG6_XL_ODR_XL_119HZ     (3 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT) /* 119 Hz */
#  define LSM9DS1_CTRL_REG6_XL_ODR_XL_238HZ     (4 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT) /* 238 Hz */
#  define LSM9DS1_CTRL_REG6_XL_ODR_XL_476HZ     (5 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT) /* 476 Hz */
#  define LSM9DS1_CTRL_REG6_XL_ODR_XL_952HZ     (6 << LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT) /* 952 Hz */

/* Accelerometer control register 7 */

#define LSM9DS1_CTRL_REG7_XL_HPIS1              (1 << 0) /* High-pass filter enabled */
#define LSM9DS1_CTRL_REG7_XL_FDS                (1 << 2) /* Filtered data selection */

#define LSM9DS1_CTRL_REG7_XL_DCF_SHIFT          5 /* Accelerometer digital filter cutoff frequency selection */
#define LSM9DS1_CTRL_REG7_XL_DCF_MASK           (3 << LSM9DS1_CTRL_REG7_XL_DCF_SHIFT)
#  define LSM9DS1_CTRL_REG7_XL_DCF_ODR_DIV50    (0 << LSM9DS1_CTRL_REG7_XL_DCF_SHIFT)
#  define LSM9DS1_CTRL_REG7_XL_DCF_ODR_DIV100   (1 << LSM9DS1_CTRL_REG7_XL_DCF_SHIFT)
#  define LSM9DS1_CTRL_REG7_XL_DCF_ODR_DIV9     (2 << LSM9DS1_CTRL_REG7_XL_DCF_SHIFT)
#  define LSM9DS1_CTRL_REG7_XL_DCF_ODR_DIV400   (3 << LSM9DS1_CTRL_REG7_XL_DCF_SHIFT)
#define LSM9DS1_CTRL_REG7_XL_HR                 (1 << 7) /* High resolution mode enable */

/* Control register 8 */

#define LSM9DS1_CTRL_REG8_SW_RESET              (1 << 0) /* Software reset */
#define LSM9DS1_CTRL_REG8_BLE                   (1 << 1) /* Big/little endian data selection */
#define LSM9DS1_CTRL_REG8_IF_ADD_INC            (1 << 2) /* Register address automatically incremented during a multibyte access */
#define LSM9DS1_CTRL_REG8_SIM                   (1 << 3) /* SPI serial interface mode selection */
#define LSM9DS1_CTRL_REG8_PP_OD                 (1 << 4) /* Push-pull/open-drain selection on the INT1_A/G and INT2_A/G pins */
#define LSM9DS1_CTRL_REG8_H_LACTIVE             (1 << 5) /* Interrupt activation level */
#define LSM9DS1_CTRL_REG8_BDU                   (1 << 6) /* Block data update */
#define LSM9DS1_CTRL_REG8_BOOT                  (1 << 7) /* Reboot memory content */

/* Control register 9 */

#define LSM9DS1_CTRL_REG9_STOP_ON_FTH           (1 << 0) /* Enable FIFO threshold level use */
#define LSM9DS1_CTRL_REG9_FIFO_EN               (1 << 1) /* FIFO memory enable */
#define LSM9DS1_CTRL_REG9_I2C_DISABLE           (1 << 2) /* Disable I2C interface */
#define LSM9DS1_CTRL_REG9_DRDY_MASK_BIT         (1 << 3) /* Data available enable bit */
#define LSM9DS1_CTRL_REG9_FIFO_TEMP_EN          (1 << 4) /* Temperature data storage in FIFO enable */
#define LSM9DS1_CTRL_REG9_SLEEP_G               (1 << 6) /* Gyroscope sleep mode enable */

/* Control register 10 */

#define LSM9DS1_CTRL_REG10_ST_XL                (1 << 0) /* Linear acceleration sensor self-test enable */
#define LSM9DS1_CTRL_REG10_ST_G                 (1 << 2) /* Angular rate sensor self-test enable */

/* Accelerometer interrupt source register */

#define LSM9DS1_INT_GEN_SRC_XL_XL_XL            (1 << 0) /* Accelerometer's X low event */
#define LSM9DS1_INT_GEN_SRC_XL_XH_XL            (1 << 1) /* Accelerometer's X high event */
#define LSM9DS1_INT_GEN_SRC_XL_YL_XL            (1 << 2) /* Accelerometer's Y low event */
#define LSM9DS1_INT_GEN_SRC_XL_YH_XL            (1 << 3) /* Accelerometer's Y high event */
#define LSM9DS1_INT_GEN_SRC_XL_ZL_XL            (1 << 4) /* Accelerometer's Z low event */
#define LSM9DS1_INT_GEN_SRC_XL_ZH_XL            (1 << 5) /* Accelerometer's Z high event */
#define LSM9DS1_INT_GEN_SRC_XL_IA_XL            (1 << 6) /* Interrupt active */

/* Status register 2 */

#define LSM9DS1_STATUS_REG2_XLDA                (1 << 0) /* Accelerometer new data available */
#define LSM9DS1_STATUS_REG2_GDA                 (1 << 1) /* Gyroscope new data available */
#define LSM9DS1_STATUS_REG2_TDA                 (1 << 2) /* Temperature sensor new data available */
#define LSM9DS1_STATUS_REG2_BOOT_STATUS         (1 << 3) /* Boot running flag signal */
#define LSM9DS1_STATUS_REG2_INACT               (1 << 4) /* Inactivity interrupt output signal */
#define LSM9DS1_STATUS_REG2_IG_G                (1 << 5) /* Gyroscope interrupt output signal */
#define LSM9DS1_STATUS_REG2_IG_XL               (1 << 6) /* Accelerometer interrupt output signal */

/* FIFO control register */

#define LSM9DS1_FIFO_CTRL_FTH_SHIFT             0 /* FIFO threshold level setting */
#define LSM9DS1_FIFO_CTRL_FTH_MASK              (31 << LSM9DS1_FIFO_CTRL_FTH_SHIFT)
#define LSM9DS1_FIFO_CTRL_FMODE_SHIFT           5 /* FIFO mode selection bits */
#define LSM9DS1_FIFO_CTRL_FMODE_MASK            (7 << LSM9DS1_FIFO_CTRL_FMODE_SHIFT)
#  define LSM9DS1_FIFO_CTRL_FMODE_BYPASS        (0 << LSM9DS1_FIFO_CTRL_FMODE_SHIFT) /* Bypass mode */
#  define LSM9DS1_FIFO_CTRL_FMODE_FIFO          (1 << LSM9DS1_FIFO_CTRL_FMODE_SHIFT) /* FIFO mode */
#  define LSM9DS1_FIFO_CTRL_FMODE_CONT_FIFO     (3 << LSM9DS1_FIFO_CTRL_FMODE_SHIFT) /* Continuous-to-FIFO mode */
#  define LSM9DS1_FIFO_CTRL_FMODE_BYPASS_CONT   (4 << LSM9DS1_FIFO_CTRL_FMODE_SHIFT) /* Bypass-to-continuous mode */
#  define LSM9DS1_FIFO_CTRL_FMODE_CONT          (5 << LSM9DS1_FIFO_CTRL_FMODE_SHIFT) /* Continuous mode */

/* FIFO status control register */

#define LSM9DS1_FIFO_SRC_FSS_SHIFT              0 /* Number of unread samples stored into FIFO */
#define LSM9DS1_FIFO_SRC_FSS_MASK               (63 << LSM9DS1_FIFO_SRC_FSS_SHIFT)
#define LSM9DS1_FIFO_SRC_OVRN                   (1 << 6) /* FIFO overrun status */
#define LSM9DS1_FIFO_SRC_FTH                    (1 << 7) /* FIFO threshold status */

/* Gyroscope interrupt configuration register */

#define LSM9DS1_INT_GEN_CFG_G_XLIE_G            (1 << 0) /* Pitch (X) axis low event interrupt enable */
#define LSM9DS1_INT_GEN_CFG_G_XHIE_G            (1 << 1) /* Pitch (X) axis high event interrupt enable */
#define LSM9DS1_INT_GEN_CFG_G_YLIE_G            (1 << 2) /* Roll (Y) axis low event interrupt enable */
#define LSM9DS1_INT_GEN_CFG_G_YHIE_G            (1 << 3) /* Roll (Y) axis high event interrupt enable */
#define LSM9DS1_INT_GEN_CFG_G_ZLIE_G            (1 << 4) /* Yaw (Z) axis low event interrupt enable */
#define LSM9DS1_INT_GEN_CFG_G_ZHIE_G            (1 << 5) /* Yaw (Z) axis high event interrupt enable */
#define LSM9DS1_INT_GEN_CFG_G_LIR_G             (1 << 6) /* Latch interrupt request */
#define LSM9DS1_INT_GEN_CFG_G_AOI_G             (1 << 7) /* AND/OR combination of interrupt events */

/* Gyroscope interrupt threshold registers */

#define LSM9DS1_INT_GEN_THS_XH_G_THS_XH_G_SHIFT 0 /* X interrupt threshold high byte */
#define LSM9DS1_INT_GEN_THS_XH_G_THS_XH_G_MASK  (127 << LSM9DS1_INT_GEN_THS_XH_G_THS_XH_G_SHIFT)
#define LSM9DS1_INT_GEN_THS_XH_G_DCRM_G         (1 << 7) /* Decrement or reset counter mode selection */

/* Gyroscope interrupt duration register */

#define LSM9DS1_INT_GEN_DUR_G_DUR_G_SHIFT       0 /* Enter/exit interrupt duration */
#define LSM9DS1_INT_GEN_DUR_G_DUR_G_MASK        (127 << LSM9DS1_INT_GEN_DUR_G_DUR_G_SHIFT)
#define LSM9DS1_INT_GEN_DUR_G_WAIT_G            (1 << 7) /* Exit from interrupt wait function enable */

/* Device identification register */

#define LSM9DS1_WHO_AM_I_M_VALUE                0x3d

/* Magnetometer control register 1 */

#define LSM9DS1_CTRL_REG1_M_ST                  (1 << 0) /* Self-test enable */
#define LSM9DS1_CTRL_REG1_M_FAST_ODR            (1 << 1) /* Enable data rates higher than 80 Hz */

#define LSM9DS1_CTRL_REG1_M_DO_SHIFT            2 /* Output data rate selection */
#define LSM9DS1_CTRL_REG1_M_DO_MASK             (7 << LSM9DS1_CTRL_REG1_M_DO_SHIFT)
#  define LSM9DS1_CTRL_REG1_M_DO_0p625HZ        (0 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 0.625 Hz */
#  define LSM9DS1_CTRL_REG1_M_DO_1p25HZ         (1 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 1.25 Hz */
#  define LSM9DS1_CTRL_REG1_M_DO_2p5HZ          (2 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 2.5 Hz */
#  define LSM9DS1_CTRL_REG1_M_DO_5HZ            (3 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 5 Hz */
#  define LSM9DS1_CTRL_REG1_M_DO_10HZ           (4 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 10 Hz */
#  define LSM9DS1_CTRL_REG1_M_DO_20HZ           (5 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 20 Hz */
#  define LSM9DS1_CTRL_REG1_M_DO_40HZ           (6 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 40 Hz */
#  define LSM9DS1_CTRL_REG1_M_DO_80HZ           (7 << LSM9DS1_CTRL_REG1_M_DO_SHIFT) /* 80 Hz */

#define LSM9DS1_CTRL_REG1_M_OM_SHIFT            5 /* X and Y axes operative mode selection */
#define LSM9DS1_CTRL_REG1_M_OM_MASK             (3 << LSM9DS1_CTRL_REG1_M_OM_SHIFT)
#  define LSM9DS1_CTRL_REG1_M_OM_LOW            (0 << LSM9DS1_CTRL_REG1_M_OM_SHIFT) /* Low-power mode */
#  define LSM9DS1_CTRL_REG1_M_OM_MEDIUM         (1 << LSM9DS1_CTRL_REG1_M_OM_SHIFT) /* Medium-performance mode */
#  define LSM9DS1_CTRL_REG1_M_OM_HIGH           (2 << LSM9DS1_CTRL_REG1_M_OM_SHIFT) /* High-performance mode */
#  define LSM9DS1_CTRL_REG1_M_OM_ULTRAHIGH      (3 << LSM9DS1_CTRL_REG1_M_OM_SHIFT) /* Ultra-high performance mode */

#define LSM9DS1_CTRL_REG1_M_TEMP_COMP           (1 << 7) /* Temperature compensation enable */

/* Magnetometer control register 2 */

#define LSM9DS1_CTRL_REG2_M_SOFT_RST            (1 << 2) /* Configuration register and user register reset */
#define LSM9DS1_CTRL_REG2_M_REBOOT              (1 << 3) /* Reboot memory content */

#define LSM9DS1_CTRL_REG2_M_FS_SHIFT            5 /* Full-scale configuration */
#define LSM9DS1_CTRL_REG2_M_FS_MASK             (3 << LSM9DS1_CTRL_REG2_M_FS_SHIFT)
#  define LSM9DS1_CTRL_REG2_M_FS_4GAUSS         (0 << LSM9DS1_CTRL_REG2_M_FS_SHIFT) /* +/- 4 gauss */
#  define LSM9DS1_CTRL_REG2_M_FS_8GAUSS         (1 << LSM9DS1_CTRL_REG2_M_FS_SHIFT) /* +/- 8 gauss */
#  define LSM9DS1_CTRL_REG2_M_FS_12GAUSS        (2 << LSM9DS1_CTRL_REG2_M_FS_SHIFT) /* +/- 12 gauss */
#  define LSM9DS1_CTRL_REG2_M_FS_16GAUSS        (3 << LSM9DS1_CTRL_REG2_M_FS_SHIFT) /* +/- 16 gauss */

/* Magnetometer control register 3 */

#define LSM9DS1_CTRL_REG3_M_MD_SHIFT            0 /* Operating mode selection */
#define LSM9DS1_CTRL_REG3_M_MD_MASK             (3 << LSM9DS1_CTRL_REG3_M_MD_SHIFT)
#  define LSM9DS1_CTRL_REG3_M_MD_CONT           (0 << LSM9DS1_CTRL_REG3_M_MD_SHIFT) /* Continuous-conversion mode */
#  define LSM9DS1_CTRL_REG3_M_MD_SINGLE         (1 << LSM9DS1_CTRL_REG3_M_MD_SHIFT) /* Single-conversion mode */
#  define LSM9DS1_CTRL_REG3_M_MD_POWERDOWN      (2 << LSM9DS1_CTRL_REG3_M_MD_SHIFT) /* Power-down mode */
#  define LSM9DS1_CTRL_REG3_M_MD_POWERDOWN2     (3 << LSM9DS1_CTRL_REG3_M_MD_SHIFT) /* Power-down mode */

#define LSM9DS1_CTRL_REG3_M_SIM                 (1 << 2) /* SPI serial interface mode selection */
#define LSM9DS1_CTRL_REG3_M_LP                  (1 << 5) /* Low-power mode configuration */
#define LSM9DS1_CTRL_REG3_M_I2C_DISABLE         (1 << 7) /* Disable I2C interface */

/* Magnetometer control register 4 */

#define LSM9DS1_CTRL_REG4_M_BLE                 (1 << 1) /* Big/little endian data selection */

#define LSM9DS1_CTRL_REG4_M_OMZ_SHIFT           2 /* Z-axis operative mode selection */
#define LSM9DS1_CTRL_REG4_M_OMZ_MASK            (3 << LSM9DS1_CTRL_REG4_M_OMZ_SHIFT)
#  define LSM9DS1_CTRL_REG4_M_OMZ_LOW           (0 << LSM9DS1_CTRL_REG4_M_OMZ_SHIFT) /* Low-power mode */
#  define LSM9DS1_CTRL_REG4_M_OMZ_MEDIUM        (1 << LSM9DS1_CTRL_REG4_M_OMZ_SHIFT) /* Medium-performance mode */
#  define LSM9DS1_CTRL_REG4_M_OMZ_HIGH          (2 << LSM9DS1_CTRL_REG4_M_OMZ_SHIFT) /* High-performance mode */
#  define LSM9DS1_CTRL_REG4_M_OMZ_ULTRAHIGH     (3 << LSM9DS1_CTRL_REG4_M_OMZ_SHIFT) /* Ultra-high performance mode */

/* Magnetometer control register 5 */

#define LSM9DS1_CTRL_REG5_M_BDU                 (1 << 6) /* Block data update */
#define LSM9DS1_CTRL_REG5_M_FAST_READ           (1 << 7) /* Fast read enable */

/* Magnetometer status register */

#define LSM9DS1_STATUS_REG_M_XDA                (1 << 0) /* X-axis new data available */
#define LSM9DS1_STATUS_REG_M_YDA                (1 << 1) /* Y-axis new data available */
#define LSM9DS1_STATUS_REG_M_ZDA                (1 << 2) /* Z-axis new data available */
#define LSM9DS1_STATUS_REG_M_ZYXDA              (1 << 3) /* X, Y and Z-axis new data available */
#define LSM9DS1_STATUS_REG_M_XOR                (1 << 4) /* X-axis data overrun */
#define LSM9DS1_STATUS_REG_M_YOR                (1 << 5) /* Y-axis data overrun */
#define LSM9DS1_STATUS_REG_M_ZOR                (1 << 6) /* Z-axis data overrun */
#define LSM9DS1_STATUS_REG_M_ZYXOR              (1 << 7) /* X, Y and Z-axis data overrun */

/* Magnetometer interrupt configuration register */

#define LSM9DS1_INT_CFG_M_IEN                   (1 << 0) /* Interrupt enable on the INT_M pin */
#define LSM9DS1_INT_CFG_M_IEL                   (1 << 1) /* Latch interrupt request */
#define LSM9DS1_INT_CFG_M_IEA                   (1 << 2) /* Interrupt active configuration on INT_MAG */
#define LSM9DS1_INT_CFG_M_ZIEN                  (1 << 5) /* Z-axis interrupt enable */
#define LSM9DS1_INT_CFG_M_YIEN                  (1 << 6) /* Y-axis interrupt enable */
#define LSM9DS1_INT_CFG_M_XIEN                  (1 << 7) /* X-axis interrupt enable */

/* Magnetometer interrupt source register */

#define LSM9DS1_INT_SRC_M_INT                   (1 << 0) /* Interrupt occurred */
#define LSM9DS1_INT_SRC_M_MROI                  (1 << 1) /* Internal measurement range overflow */
#define LSM9DS1_INT_SRC_M_NTH_Z                 (1 << 2) /* Value on Z-axis exceeds threshold on negative side */
#define LSM9DS1_INT_SRC_M_NTH_Y                 (1 << 3) /* Value on Y-axis exceeds threshold on negative side */
#define LSM9DS1_INT_SRC_M_NTH_X                 (1 << 4) /* Value on X-axis exceeds threshold on negative side */
#define LSM9DS1_INT_SRC_M_PTH_Z                 (1 << 5) /* Value on Z-axis exceeds threshold on positive side */
#define LSM9DS1_INT_SRC_M_PTH_Y                 (1 << 6) /* Value on Y-axis exceeds threshold on positive side */
#define LSM9DS1_INT_SRC_M_PTH_X                 (1 << 7) /* Value on X-axis exceeds threshold on positive side */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lsm9ds1_dev_s;
struct lsm9ds1_ops_s
{
  CODE int (*config)(FAR struct lsm9ds1_dev_s *priv);
  CODE int (*start)(FAR struct lsm9ds1_dev_s *priv);
  CODE int (*stop)(FAR struct lsm9ds1_dev_s *priv);
  CODE int (*setsamplerate)(FAR struct lsm9ds1_dev_s *priv,
                            uint32_t samplerate);
  CODE int (*setfullscale)(FAR struct lsm9ds1_dev_s *priv,
                           uint32_t fullscale);
};

struct lsm9ds1_dev_s
{
  FAR struct i2c_master_s        *i2c;        /* I2C interface */
  uint8_t                         addr;       /* I2C address */

  FAR const struct lsm9ds1_ops_s *ops;

  uint32_t                        samplerate; /* Output data rate */
  uint8_t                         datareg;    /* Output data register of X low byte */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int lsm9ds1_readreg8(FAR struct lsm9ds1_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regval);
static int lsm9ds1_writereg8(FAR struct lsm9ds1_dev_s *priv, uint8_t regaddr,
                             uint8_t regval);
static int lsm9ds1_modifyreg8(FAR struct lsm9ds1_dev_s *priv,
                              uint8_t regaddr, uint8_t clearbits,
                              uint8_t setbits);

/* Other Helpers */

static uint32_t lsm9ds1_midpoint(uint32_t a, uint32_t b);

/* Accelerometer Operations */

static int lsm9ds1accelgyro_config(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1accel_start(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1accel_stop(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1accelgyro_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                          uint32_t samplerate);
static int lsm9ds1accel_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                     uint32_t fullscale);

/* Gyroscope Operations */

static int lsm9ds1gyro_start(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1gyro_stop(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1gyro_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t fullscale);

/* Magnetometer Operations */

static int lsm9ds1mag_config(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1mag_start(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1mag_stop(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1mag_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                   uint32_t fullscale);
static int lsm9ds1mag_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t samplerate);

/* Character Driver Methods */

static ssize_t lsm9ds1_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t lsm9ds1_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     lsm9ds1_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/* Common Register Function */

static int lsm9ds1_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c, uint8_t addr,
                            FAR const struct lsm9ds1_ops_s *ops,
                            uint8_t datareg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  lsm9ds1_read,    /* read */
  lsm9ds1_write,   /* write */
  NULL,            /* seek */
  lsm9ds1_ioctl,   /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

static const struct lsm9ds1_ops_s g_lsm9ds1accel_ops =
{
  lsm9ds1accelgyro_config,
  lsm9ds1accel_start,
  lsm9ds1accel_stop,
  lsm9ds1accelgyro_setsamplerate,
  lsm9ds1accel_setfullscale,
};

static const struct lsm9ds1_ops_s g_lsm9ds1gyro_ops =
{
  lsm9ds1accelgyro_config,
  lsm9ds1gyro_start,
  lsm9ds1gyro_stop,
  lsm9ds1accelgyro_setsamplerate,
  lsm9ds1gyro_setfullscale,
};

static const struct lsm9ds1_ops_s g_lsm9ds1mag_ops =
{
  lsm9ds1mag_config,
  lsm9ds1mag_start,
  lsm9ds1mag_stop,
  lsm9ds1mag_setsamplerate,
  lsm9ds1mag_setfullscale,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1_readreg8
 *
 * Description:
 *   Read from an 8-bit register.
 *
 ****************************************************************************/

static int lsm9ds1_readreg8(FAR struct lsm9ds1_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM9DS1_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, *regval);
  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_writereg8
 *
 * Description:
 *   Write to an 8-bit register.
 *
 ****************************************************************************/

static int lsm9ds1_writereg8(FAR struct lsm9ds1_dev_s *priv, uint8_t regaddr,
                             uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM9DS1_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, regval);
  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_modifyreg8
 *
 * Description:
 *   Modify an 8-bit register.
 *
 ****************************************************************************/

static int lsm9ds1_modifyreg8(FAR struct lsm9ds1_dev_s *priv,
                              uint8_t regaddr,
                              uint8_t clearbits,
                              uint8_t setbits)
{
  int ret;
  uint8_t regval;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  ret = lsm9ds1_readreg8(priv, regaddr, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
      return ret;
    }

  regval &= ~clearbits;
  regval |= setbits;

  ret = lsm9ds1_writereg8(priv, regaddr, regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_writereg8 failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_midpoint
 *
 * Description:
 *   Find the midpoint between two numbers.
 *
 ****************************************************************************/

static uint32_t lsm9ds1_midpoint(uint32_t a, uint32_t b)
{
  return (uint32_t)(((uint64_t)a +
         (uint64_t)b + (uint64_t)1) / (uint64_t)2);
}

/****************************************************************************
 * Name: lsm9ds1accelgyro_config
 *
 * Description:
 *   Configure the accelerometer and gyroscope.
 *
 ****************************************************************************/

static int lsm9ds1accelgyro_config(FAR struct lsm9ds1_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get the device identification */

  ret = lsm9ds1_readreg8(priv, LSM9DS1_WHO_AM_I, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
      return ret;
    }

  if (regval != LSM9DS1_WHO_AM_I_VALUE)
    {
      snerr("ERROR: Invalid device identification %02x\n", regval);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1accel_start
 *
 * Description:
 *   Start the accelerometer.
 *
 ****************************************************************************/

static int lsm9ds1accel_start(FAR struct lsm9ds1_dev_s *priv)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (priv->samplerate < lsm9ds1_midpoint(10, 50))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_10HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(50, 119))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_50HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(119, 238))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_119HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(238, 476))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_238HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(476, 952))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_476HZ;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_952HZ;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG6_XL,
                            LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1accel_stop
 *
 * Description:
 *   Stop the accelerometer.
 *
 ****************************************************************************/

static int lsm9ds1accel_stop(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG6_XL,
                            LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK,
                            LSM9DS1_CTRL_REG6_XL_ODR_XL_POWERDOWN);
}

/****************************************************************************
 * Name: lsm9ds1accelgyro_setsamplerate
 *
 * Description:
 *   Set the accelerometer or gyroscope's sample rate.
 *
 ****************************************************************************/

static int lsm9ds1accelgyro_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                          uint32_t samplerate)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  priv->samplerate = samplerate;
  return OK;
}

/****************************************************************************
 * Name: lsm9ds1accel_setfullscale
 *
 * Description:
 *   Set the accelerometer's full-scale range.
 *
 ****************************************************************************/

static int lsm9ds1accel_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                     uint32_t fullscale)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (fullscale < lsm9ds1_midpoint(2, 4))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_2G;
    }
  else if (fullscale < lsm9ds1_midpoint(4, 8))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_4G;
    }
  else if (fullscale < lsm9ds1_midpoint(8, 16))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_8G;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_16G;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG6_XL,
                            LSM9DS1_CTRL_REG6_XL_FS_XL_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1gyro_start
 *
 * Description:
 *   Start the gyroscope.
 *
 ****************************************************************************/

static int lsm9ds1gyro_start(FAR struct lsm9ds1_dev_s *priv)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (priv->samplerate < lsm9ds1_midpoint(14, 59))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_14p9HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(59, 119))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_59p5HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(119, 238))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_119HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(238, 476))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_238HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(476, 952))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_476HZ;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_952HZ;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_G,
                            LSM9DS1_CTRL_REG1_G_ODR_G_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1gyro_stop
 *
 * Description:
 *   Stop the gyroscope.
 *
 ****************************************************************************/

static int lsm9ds1gyro_stop(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_G,
                            LSM9DS1_CTRL_REG1_G_ODR_G_MASK,
                            LSM9DS1_CTRL_REG1_G_ODR_G_POWERDOWN);
}

/****************************************************************************
 * Name: lsm9ds1gyro_setfullscale
 *
 * Description:
 *   Set the gyroscope's full-scale range.
 *
 ****************************************************************************/

static int lsm9ds1gyro_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t fullscale)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (fullscale < lsm9ds1_midpoint(245, 500))
    {
      setbits = LSM9DS1_CTRL_REG1_G_FS_G_245DPS;
    }
  else if (fullscale < lsm9ds1_midpoint(500, 2000))
    {
      setbits = LSM9DS1_CTRL_REG1_G_FS_G_500DPS;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG1_G_FS_G_2000DPS;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_G,
                            LSM9DS1_CTRL_REG1_G_FS_G_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1mag_config
 *
 * Description:
 *   Configure the magnetometer.
 *
 ****************************************************************************/

static int lsm9ds1mag_config(FAR struct lsm9ds1_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get the device identification */

  ret = lsm9ds1_readreg8(priv, LSM9DS1_WHO_AM_I_M, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
      return ret;
    }

  if (regval != LSM9DS1_WHO_AM_I_M_VALUE)
    {
      snerr("ERROR: Invalid device identification %02x\n", regval);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1mag_start
 *
 * Description:
 *   Start the magnetometer.
 *
 ****************************************************************************/

static int lsm9ds1mag_start(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG3_M,
                            LSM9DS1_CTRL_REG3_M_MD_MASK,
                            LSM9DS1_CTRL_REG3_M_MD_CONT);
}

/****************************************************************************
 * Name: lsm9ds1mag_stop
 *
 * Description:
 *   Stop the magnetometer.
 *
 ****************************************************************************/

static int lsm9ds1mag_stop(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG3_M,
                            LSM9DS1_CTRL_REG3_M_MD_MASK,
                            LSM9DS1_CTRL_REG3_M_MD_POWERDOWN2);
}

/****************************************************************************
 * Name: lsm9ds1mag_setfullscale
 *
 * Description:
 *   Set the magnetometer's full-scale range.
 *
 ****************************************************************************/

static int lsm9ds1mag_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                   uint32_t fullscale)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (fullscale < lsm9ds1_midpoint(4, 8))
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_4GAUSS;
    }
  else if (fullscale < lsm9ds1_midpoint(8, 12))
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_8GAUSS;
    }
  else if (fullscale < lsm9ds1_midpoint(12, 16))
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_12GAUSS;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_16GAUSS;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG2_M,
                            LSM9DS1_CTRL_REG2_M_FS_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1mag_setsamplerate
 *
 * Description:
 *   Set the magnetometer's sample rate.
 *
 ****************************************************************************/

static int lsm9ds1mag_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t samplerate)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* The magnetometer can change its sample rate without exiting
   * power-down mode, so we don't need to save the value for later,
   * unlike the accelerometer and gyroscope.
   */

  if (samplerate < lsm9ds1_midpoint(0, 1))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_0p625HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(1, 2))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_1p25HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(2, 5))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_2p5HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(5, 10))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_5HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(10, 20))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_10HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(20, 40))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_20HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(40, 80))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_40HZ;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_80HZ;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_M,
                            LSM9DS1_CTRL_REG1_M_DO_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1_read
 *
 * Description:
 *   The standard read method.
 *
 ****************************************************************************/

static ssize_t lsm9ds1_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode         *inode;
  FAR struct lsm9ds1_dev_s *priv;
  int                       ret;
  size_t                    i;
  size_t                    j;
  size_t                    samplesize;
  size_t                    nsamples;
  uint16_t                  data;
  FAR int16_t              *ptr;
  uint8_t                   regaddr;
  uint8_t                   lo;
  uint8_t                   hi;
  uint32_t                  merge = 0;

  /* Sanity check */

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct lsm9ds1_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->datareg == LSM9DS1_OUT_X_L_G ||
              priv->datareg == LSM9DS1_OUT_X_L_XL ||
              priv->datareg == LSM9DS1_OUT_X_L_M);
  DEBUGASSERT(buffer != NULL);

  samplesize = 3 * sizeof(*ptr);
  nsamples   = buflen / samplesize;
  ptr        = (FAR int16_t *)buffer;

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      /* Reset the register address to the X low byte register */

      regaddr = priv->datareg;

      /* Read the X, Y and Z data */

      for (j = 0; j < 3; j++)
        {
          /* Read the low byte */

          ret = lsm9ds1_readreg8(priv, regaddr, &lo);
          if (ret < 0)
            {
              snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
              return (ssize_t)ret;
            }

          regaddr++;

          /* Read the high byte */

          ret = lsm9ds1_readreg8(priv, regaddr, &hi);
          if (ret < 0)
            {
              snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
              return (ssize_t)ret;
            }

          regaddr++;

          /* The data is 16 bits in two's complement representation */

          data = ((uint16_t)hi << 8) | (uint16_t)lo;

          /* Collect entropy */

          merge += data ^ (merge >> 16);

          /* The value is positive */

          if (data < 0x8000)
            {
              ptr[j] = (int16_t)data;
            }

          /* The value is negative, so find its absolute value by taking the
           * two's complement
           */

          else if (data > 0x8000)
            {
              data = ~data + 1;
              ptr[j] = -(int16_t)data;
            }

          /* The value is negative and can't be represented as a positive
           * int16_t value
           */

          else
            {
              ptr[j] = (int16_t)(-32768);
            }
        }
    }

  /* Feed sensor data to entropy pool */

  add_sensor_randomness(merge);

  return nsamples * samplesize;
}

/****************************************************************************
 * Name: lsm9ds1_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t lsm9ds1_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lsm9ds1_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int lsm9ds1_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct lsm9ds1_dev_s *priv;
  int                       ret;

  /* Sanity check */

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct lsm9ds1_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Start converting. Arg: None. */

      case SNIOC_START:
        ret = priv->ops->start(priv);
        break;

      /* Stop converting. Arg: None. */

      case SNIOC_STOP:
        ret = priv->ops->stop(priv);
        break;

      /* Set the sample rate. Arg: uint32_t value. */

      case SNIOC_SETSAMPLERATE:
        ret = priv->ops->setsamplerate(priv, (uint32_t)arg);
        sninfo("sample rate: %08x ret: %d\n", (uint32_t)arg, ret);
        break;

      /* Set the full-scale range. Arg: uint32_t value. */

      case SNIOC_SETFULLSCALE:
        ret = priv->ops->setfullscale(priv, (uint32_t)arg);
        sninfo("full-scale range: %08x ret: %d\n", (uint32_t)arg, ret);
        break;

      /* Unrecognized commands */

      default:
        snerr("ERROR: Unrecognized cmd: %d arg: %lu\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: lsm9ds1_register
 *
 * Description:
 *   Register the LSM9DS1 accelerometer, gyroscope or magnetometer character
 *   device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0",
 *             "/dev/gyro0" or "/dev/mag0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 accelerometer, gyroscope or
 *             magnetometer.
 *   ops     - The device operations structure.
 *   datareg - The register address of the low byte of the X-coordinate data.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm9ds1_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c, uint8_t addr,
                            FAR const struct lsm9ds1_ops_s *ops,
                            uint8_t datareg)
{
  FAR struct lsm9ds1_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(datareg == LSM9DS1_OUT_X_L_XL ||
              datareg == LSM9DS1_OUT_X_L_G ||
              datareg == LSM9DS1_OUT_X_L_M);

  /* Initialize the device's structure */

  priv = (FAR struct lsm9ds1_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->ops        = ops;
  priv->samplerate = 0;
  priv->datareg    = datareg;

  /* Configure the device */

  ret = priv->ops->config(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure device: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1accel_register
 *
 * Description:
 *   Register the LSM9DS1 accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1accel_register(FAR const char *devpath,
                          FAR struct i2c_master_s *i2c,
                          uint8_t addr)
{
  /* Sanity check */

  DEBUGASSERT(addr == LSM9DS1ACCEL_ADDR0 || addr == LSM9DS1ACCEL_ADDR1);

  return lsm9ds1_register(devpath, i2c, addr, &g_lsm9ds1accel_ops,
                          LSM9DS1_OUT_X_L_XL);
}

/****************************************************************************
 * Name: lsm9ds1gyro_register
 *
 * Description:
 *   Register the LSM9DS1 gyroscope character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/gyro0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 gyroscope.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1gyro_register(FAR const char *devpath,
                         FAR struct i2c_master_s *i2c,
                         uint8_t addr)
{
  /* Sanity check */

  DEBUGASSERT(addr == LSM9DS1GYRO_ADDR0 || addr == LSM9DS1GYRO_ADDR1);

  return lsm9ds1_register(devpath, i2c, addr, &g_lsm9ds1gyro_ops,
                          LSM9DS1_OUT_X_L_G);
}

/****************************************************************************
 * Name: lsm9ds1mag_register
 *
 * Description:
 *   Register the LSM9DS1 magnetometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/mag0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 magnetometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1mag_register(FAR const char *devpath,
                        FAR struct i2c_master_s *i2c,
                        uint8_t addr)
{
  /* Sanity check */

  DEBUGASSERT(addr == LSM9DS1MAG_ADDR0 || addr == LSM9DS1MAG_ADDR1);

  return lsm9ds1_register(devpath, i2c, addr, &g_lsm9ds1mag_ops,
                          LSM9DS1_OUT_X_L_M);
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM9DS1 */
