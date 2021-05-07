/****************************************************************************
 * include/nuttx/sensors/lis3mdl.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LIS3MDL_H
#define __INCLUDE_NUTTX_SENSORS_LIS3MDL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_LIS3MDL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LIS3MDL Register Definitions *********************************************/

#define LIS3MDL_WHO_AM_I_REG    (0x0F)
#define LIS3MDL_CTRL_REG_1      (0x20)
#define LIS3MDL_CTRL_REG_2      (0x21)
#define LIS3MDL_CTRL_REG_3      (0x22)
#define LIS3MDL_CTRL_REG_4      (0x23)
#define LIS3MDL_CTRL_REG_5      (0x24)
#define LIS3MDL_STATUS_REG      (0x27)
#define LIS3MDL_OUT_X_L_REG     (0x28)
#define LIS3MDL_OUT_X_H_REG     (0x29)
#define LIS3MDL_OUT_Y_L_REG     (0x2A)
#define LIS3MDL_OUT_Y_H_REG     (0x2B)
#define LIS3MDL_OUT_Z_L_REG     (0x2C)
#define LIS3MDL_OUT_Z_H_REG     (0x2D)
#define LIS3MDL_TEMP_OUT_L_REG  (0x2E)
#define LIS3MDL_TEMP_OUT_H_REG  (0x2F)
#define LIS3MDL_INT_CFG_REG     (0x30)
#define LIS3MDL_INT_SRC_REG     (0x31)
#define LIS3MDL_INT_THS_L_REG   (0x32)
#define LIS3MDL_INT_THS_H_REG   (0x33)

/* LIS3MDL CTRL_REG_1 Bit Definitions ***************************************/

#define LIS3MDL_CTRL_REG_1_TEMP_EN_BM      (1<<7)   /* Enable the temperature sensor */
#define LIS3MDL_CTRL_REG_1_OM_1_BM         (1<<6)   /* Select the operating mode of X and Y axis bit 1 */
#define LIS3MDL_CTRL_REG_1_OM_0_BM         (1<<5)   /* Select the operating mode of X and Y axis bit 0 */
#define LIS3MDL_CTRL_REG_1_DO2_BM          (1<<4)   /* Output data rate selection bit 2 */
#define LIS3MDL_CTRL_REG_1_DO1_BM          (1<<3)   /* Output data rate selection bit 1 */
#define LIS3MDL_CTRL_REG_1_DO0_BM          (1<<2)   /* Output data rate selection bit 2 */
#define LIS3MDL_CTRL_REG_1_FAST_ODR_BM     (1<<1)   /* Enable higher output data rates */

/* LIS3MDL CTRL_REG_2 Bit Definitions ***************************************/

#define LIS3MDL_CTRL_REG_2_FS_1_BM         (1<<6)   /* Full scale selection bit 1 */
#define LIS3MDL_CTRL_REG_2_FS_0_BM         (1<<5)   /* Full scale selection bit 0 */
#define LIS3MDL_CTRL_REG_2_REBOOT_BM       (1<<3)   /* Reboot Memory Content */
#define LIS3MDL_CTRL_REG_2_SOFT_RST_BM     (1<<2)   /* Soft Reset */

/* LIS3MDL CTRL_REG_4 Bit Definitions ***************************************/

#define LIS3MDL_CTRL_REG_4_OMZ_1_BM        (1<<3)   /* Select the operating mode of Z axis bit 1 */
#define LIS3MDL_CTRL_REG_4_OMZ_0_BM        (1<<2)   /* Select the operating mode of Z axis bit 0 */

/* LIS3MDL CTRL_REG_5 Bit Definitions ***************************************/

#define LIS3MDL_CTRL_REG_5_BDU_BM          (1<<6)   /* Enable block data update for magnetic data (prevent race conditions while reading) */

/* SPI BUS PARAMETERS *******************************************************/

#define LIS3MDL_SPI_FREQUENCY    (1000000)        /* 1 MHz */
#define LIS3MDL_SPI_MODE         (SPIDEV_MODE3)   /* Device uses SPI Mode 3: CPOL=1, CPHA=1 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the LIS3MDL
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct lis3mdl_config_s
{
  /* Since multiple LIS3MDL can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired LIS3MDL chip via their chip select inputs.
   */

  int spi_devid;

  /* The IRQ number must be provided for each so LIS3MDL device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the LIS3MDL interrupt handler to the GPIO interrupt of the
   * concrete LIS3MDL instance.
   */

  int (*attach)(FAR struct lis3mdl_config_s *, xcpt_t);
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
 * Name: lis3mdl_register
 *
 * Description:
 *   Register the LIS3MDL character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             LIS3MDL
 *   config  - configuration for the LIS3MDL driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis3mdl_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     FAR struct lis3mdl_config_s const *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_LIS3MDL */
#endif /* __INCLUDE_NUTTX_SENSORS_LIS3MDL_H */
