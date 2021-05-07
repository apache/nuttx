/****************************************************************************
 * include/nuttx/sensors/mlx90393.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MLX90393_H
#define __INCLUDE_NUTTX_SENSORS_MLX90393_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MLX90393)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MLX90393 Command Definitions *********************************************/

#define MLX90393_SB               (0x10)          /* SB = Start Burst Mode */
#define MLX90393_SW               (0x20)          /* SW = Start Wake-up on Change Mode */
#define MLX90393_SM               (0x30)          /* SB = Start Single Measurement Mode */
#define MLX90393_RM               (0x40)          /* RM = Read Measurement */
#define MLX90393_RR               (0x50)          /* RR = Read Register */
#define MLX90393_WR               (0x60)          /* WR = Write Register */
#define MLX90393_EX               (0x80)          /* EX = Exit Mode */
#define MLX90393_HR               (0xD0)          /* HR = Memory Recall */
#define MLX90393_HS               (0xE0)          /* HS = Memory Store */
#define MLX90393_RT               (0xF0)          /* RT = Reset */

/* MLX90393 Sensor Selection Bit Definitions ********************************/

#define MLX90393_T_BM             (1<<0)          /* Temperature Sensor Bitmask  */
#define MLX90393_X_BM             (1<<1)          /* Magnetometer X-Axis Bitmask */
#define MLX90393_Y_BM             (1<<2)          /* Magnetometer Y-Axis Bitmask */
#define MLX90393_Z_BM             (1<<3)          /* Magnetometer Z-Axis Bitmask */
#define MLX90393_ZYXT_BM          (MLX90393_Z_BM | MLX90393_Y_BM | MLX90393_X_BM | MLX90393_T_BM)

/* SPI BUS PARAMETERS *******************************************************/

#define MLX90393_SPI_FREQUENCY    (1000000)        /* 1 MHz */
#define MLX90393_SPI_MODE         (SPIDEV_MODE3)   /* Device uses SPI Mode 3: CPOL=1, CPHA=1 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the MLX90393
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct mlx90393_config_s
{
  /* Since multiple MLX90393 can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired MLX90393 chip via their chip select inputs.
   */

  int spi_devid;

  /* The IRQ number must be provided for each so MLX90393 device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the MLX90393 interrupt handler to the GPIO interrupt of the
   * concrete MLX90393 instance.
   */

  int (*attach)(FAR struct mlx90393_config_s *, xcpt_t);
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
 * Name: mlx90393_register
 *
 * Description:
 *   Register the MLX90393 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MLX90393
 *   config  - Describes the configuration of the MLX90393 part.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mlx90393_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                      FAR struct mlx90393_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_MLX90393 */

#endif /* __INCLUDE_NUTTX_SENSORS_MLX90393_H */
