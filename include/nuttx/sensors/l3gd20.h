/****************************************************************************
 * include/nuttx/sensors/l3gd20.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_L3GD20_H
#define __INCLUDE_NUTTX_SENSORS_L3GD20_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_L3GD20)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI BUS PARAMETERS *******************************************************/

#define L3GD20_SPI_FREQUENCY  (4000000)        /* 4 MHz */
#define L3GD20_SPI_MODE       (SPIDEV_MODE3)   /* Device uses SPI Mode 3:
                                                *  CPOL=1, CPHA=1
                                                */

/* Register Addresses *******************************************************/

/* Gyroscope registers */

#define L3GD20_WHO_AM_I                0x0F /* Accelerometer and gyroscope device identification */
#define L3GD20_CTRL_REG_1              0x20 /* Gyroscope control register 1 */
#define L3GD20_CTRL_REG_2              0x21 /* Gyroscope control register 2 */
#define L3GD20_CTRL_REG_3              0x22 /* Gyroscope control register 3 */
#define L3GD20_CTRL_REG_4              0x23 /* Gyroscope control register 4 */
#define L3GD20_CTRL_REG_5              0x24 /* Gyroscope control register 5 */
#define L3GD20_REF_REG                 0x25 /* Gyroscope reference value for interrupt generation */
#define L3GD20_OUT_TEMP_REG            0x26 /* Temperature data */
#define L3GD20_STATUS_REG              0x27 /* Status register */
#define L3GD20_OUT_X_L_REG             0x28 /* Gyroscope pitch (X) low byte */
#define L3GD20_OUT_X_H_REG             0x29 /* Gyroscope pitch (X) high byte */
#define L3GD20_OUT_Y_L_REG             0x2A /* Gyroscope roll (Y) low byte */
#define L3GD20_OUT_Y_H_REG             0x2B /* Gyroscope roll (Y) high byte */
#define L3GD20_OUT_Z_L_REG             0x2C /* Gyroscope yaw (Z) low byte */
#define L3GD20_OUT_Z_H_REG             0x2D /* Gyroscope yaw (Z) high byte */
#define L3GD20_FIFO_CTRL_REG           0x2E /* FIFO control register */
#define L3GD20_FIFO_SRC_REG            0x2f /* FIFO status control register */
#define L3GD20_INT_GEN_CFG_REG         0x30 /* Gyroscope interrupt configuration */
#define L3GD20_INT_GEN_SRC_REG         0x31 /* Gyroscope interrupt source */
#define L3GD20_INT_GEN_THS_X_H_REG     0x32 /* Gyroscope pitch (X) interrupt threshold high byte */
#define L3GD20_INT_GEN_THS_X_L_REG     0x33 /* Gyroscope pitch (X) interrupt threshold low byte */
#define L3GD20_INT_GEN_THS_Y_H_REG     0x34 /* Gyroscope roll (Y) interrupt threshold high byte */
#define L3GD20_INT_GEN_THS_Y_L_REG     0x35 /* Gyroscope roll (Y) interrupt threshold low byte */
#define L3GD20_INT_GEN_THS_Z_H_REG     0x36 /* Gyroscope yaw (Z) interrupt threshold high byte */
#define L3GD20_INT_GEN_THS_Z_L_REG     0x37 /* Gyroscope yaw (Z) interrupt threshold low byte */
#define L3GD20_INT_GEN_DUR_REG         0x38 /* Gyroscope interrupt duration */

/* Register Bit Definitions *************************************************/

/* Device identification register */

#define L3GD20_WHO_AM_I_VALUE          0xD4

/* Gyroscope control register 1 */

#define L3GD20_CTRL_REG_1_X_EN_BM          (1 << 0)
#define L3GD20_CTRL_REG_1_Y_EN_BM          (1 << 1)
#define L3GD20_CTRL_REG_1_Z_EN_BM          (1 << 2)
#define L3GD20_CTRL_REG_1_POWERDOWN_BM     (1 << 3)
#define L3GD20_CTRL_REG_1_BW_0_BM          (1 << 4)
#define L3GD20_CTRL_REG_1_BW_1_BM          (1 << 5)
#define L3GD20_CTRL_REG_1_DR_0_BM          (1 << 6)
#define L3GD20_CTRL_REG_1_DR_1_BM          (1 << 7)

/* Gyroscope control register 2 */

#define L3GD20_CTRL_REG_2_HPCF_0_BM        (1 << 0)
#define L3GD20_CTRL_REG_2_HPCF_1_BM        (1 << 1)
#define L3GD20_CTRL_REG_2_HPCF_2_BM        (1 << 2)
#define L3GD20_CTRL_REG_2_HPCF_3_BM        (1 << 3)
#define L3GD20_CTRL_REG_2_HPM_0_BM         (1 << 4)
#define L3GD20_CTRL_REG_2_HPM_1_BM         (1 << 5)
#define L3GD20_CTRL_REG_2_RES6_            (1 << 6)
#define L3GD20_CTRL_REG_2_RES7_            (1 << 7)

/* Gyroscope control register 3 */

#define L3GD20_CTRL_REG_3_I2_EMPTY_BM      (1 << 0)
#define L3GD20_CTRL_REG_3_I2_ORUN_BM       (1 << 1)
#define L3GD20_CTRL_REG_3_I2_WTM_BM        (1 << 2)
#define L3GD20_CTRL_REG_3_I2_DRDY_BM       (1 << 3)
#define L3GD20_CTRL_REG_3_PP_OD_BM         (1 << 4)
#define L3GD20_CTRL_REG_3_H_LACTIVE_BM     (1 << 5)
#define L3GD20_CTRL_REG_3_I1_BOOT_BM       (1 << 6)
#define L3GD20_CTRL_REG_3_I1_INT1_BM       (1 << 7)

/* Gyroscope control register 4 */

#define L3GD20_CTRL_REG_4_SIM_BM           (1 << 0)
#define L3GD20_CTRL_REG_4_RES1_            (1 << 1)
#define L3GD20_CTRL_REG_4_RES2_            (1 << 2)
#define L3GD20_CTRL_REG_4_RES3_            (1 << 3)
#define L3GD20_CTRL_REG_4_FS_0_BM          (1 << 4)
#define L3GD20_CTRL_REG_4_FS_1_BM          (1 << 5)
#define L3GD20_CTRL_REG_4_BLE_BM           (1 << 6)
#define L3GD20_CTRL_REG_4_BDU_BM           (1 << 7)

/* Gyroscope control register 5 */

#define L3GD20_CTRL_REG_5_OUT_SEL_0_BM     (1 << 0)
#define L3GD20_CTRL_REG_5_OUT_SEL_1_BM     (1 << 1)
#define L3GD20_CTRL_REG_5_INT1_SEL_0_BM    (1 << 2)
#define L3GD20_CTRL_REG_5_INT1_SEL_1_BM    (1 << 3)
#define L3GD20_CTRL_REG_5_HP_EN_BM         (1 << 4)
#define L3GD20_CTRL_REG_5_RES5_            (1 << 5)
#define L3GD20_CTRL_REG_5_FIFO_EN_BM       (1 << 6)
#define L3GD20_CTRL_REG_5_BOOT_BM          (1 << 7)

/* Status register */

#define L3GD20_STATUS_REG_X_DA_BM          (1 << 0)
#define L3GD20_STATUS_REG_Y_DA_BM          (1 << 1)
#define L3GD20_STATUS_REG_Z_DA_BM          (1 << 2)
#define L3GD20_STATUS_REG_ZYX_DA_BM        (1 << 3)
#define L3GD20_STATUS_REG_X_OR_BM          (1 << 4)
#define L3GD20_STATUS_REG_Y_OR_BM          (1 << 5)
#define L3GD20_STATUS_REG_Z_OR_BM          (1 << 6)
#define L3GD20_STATUS_REG_ZYX_OR_BM        (1 << 7)

/* FIFO control register */

#define L3GD20_FIFO_CTRL_WTM_0_BM          (1 << 0)
#define L3GD20_FIFO_CTRL_WTM_1_BM          (1 << 1)
#define L3GD20_FIFO_CTRL_WTM_2_BM          (1 << 2)
#define L3GD20_FIFO_CTRL_WTM_3_BM          (1 << 3)
#define L3GD20_FIFO_CTRL_WTM_4_BM          (1 << 4)
#define L3GD20_FIFO_CTRL_FM_0_BM           (1 << 5)
#define L3GD20_FIFO_CTRL_FM_1_BM           (1 << 6)
#define L3GD20_FIFO_CTRL_FM_2_BM           (1 << 7)
#define L3GD20_FIFO_CTRL_FMODE_BYPASS      (0)
#define L3GD20_FIFO_CTRL_FMODE_FIFO        (L3GD20_FIFO_CTRL_FM0)
#define L3GD20_FIFO_CTRL_FMODE_CONT        (L3GD20_FIFO_CTRL_FM1)
#define L3GD20_FIFO_CTRL_FMODE_CONT_FIFO   (L3GD20_FIFO_CTRL_FM1 | L3GD20_FIFO_CTRL_FM0)
#define L3GD20_FIFO_CTRL_FMODE_BYPASS_CONT (L3GD20_FIFO_CTRL_FM2 | L3GD20_FIFO_CTRL_FM1)

/* FIFO status control register */

#define L3GD20_FIFO_SRC_FSS_0_BM           (1 << 0)
#define L3GD20_FIFO_SRC_FSS_1_BM           (1 << 1)
#define L3GD20_FIFO_SRC_FSS_2_BM           (1 << 2)
#define L3GD20_FIFO_SRC_FSS_3_BM           (1 << 3)
#define L3GD20_FIFO_SRC_FSS_4_BM           (1 << 4)
#define L3GD20_FIFO_SRC_EMPTY_BM           (1 << 5)
#define L3GD20_FIFO_SRC_OVRUN_BM           (1 << 6)
#define L3GD20_FIFO_SRC_WTM_BM             (1 << 7)

/* Gyroscope interrupt configuration */

#define L3GD20_INT_GEN_CFG_X_L_IE_BM       (1 << 0)
#define L3GD20_INT_GEN_CFG_X_H_IE_BM       (1 << 1)
#define L3GD20_INT_GEN_CFG_Y_L_IE_BM       (1 << 2)
#define L3GD20_INT_GEN_CFG_Y_H_IE_BM       (1 << 3)
#define L3GD20_INT_GEN_CFG_Z_L_IE_BM       (1 << 4)
#define L3GD20_INT_GEN_CFG_Z_H_IE_BM       (1 << 5)
#define L3GD20_INT_GEN_CFG_LIR_BM          (1 << 6)
#define L3GD20_INT_GEN_CFG_AOI_BM          (1 << 7)

/* Gyroscope interrupt source */

#define L3GD20_INT_GEN_SRC_X_L_BM          (1 << 0)
#define L3GD20_INT_GEN_SRC_X_H_BM          (1 << 1)
#define L3GD20_INT_GEN_SRC_Y_L_BM          (1 << 2)
#define L3GD20_INT_GEN_SRC_Y_H_BM          (1 << 3)
#define L3GD20_INT_GEN_SRC_Z_L_BM          (1 << 4)
#define L3GD20_INT_GEN_SRC_Z_H_BM          (1 << 5)
#define L3GD20_INT_GEN_SRC_I_A_BM          (1 << 6)
#define L3GD20_INT_GEN_SRC_RES7_           (1 << 7)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the L3GD20
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct l3gd20_config_s
{
  /* Since multiple L3GD20 can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired L3GD20 chip via their chip select inputs.
   */

  int spi_devid;

  /* The IRQ number must be provided for each L3GD20 device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the L3GD20 interrupt handler to the GPIO interrupt of the
   * concrete L3GD20 instance.
   */

  int (*attach)(FAR struct l3gd20_config_s *, xcpt_t);
};

/* Data returned by reading from the L3GD20 is returned in this format. */

struct l3gd20_sensor_data_s
{
  int16_t x_gyr;              /* Measurement result for x axis */
  int16_t y_gyr;              /* Measurement result for y axis */
  int16_t z_gyr;              /* Measurement result for z axis */
  int8_t temperature;         /* Measurement result for temperature sensor */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: l3gd20_register
 *
 * Description:
 *   Register the L3DF20 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/gyr0".
 *   i2c     - An SPI driver instance.
 *   config  - configuration for the L3GD20 driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int l3gd20_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                    FAR struct l3gd20_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_L3GD20 */
#endif /* __INCLUDE_NUTTX_SENSORS_L3GD20_H */
