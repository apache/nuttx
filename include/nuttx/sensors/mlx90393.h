/****************************************************************************
 * include/nuttx/sensors/mlx90393.h
 *
 *   Copyright (C) 2016 DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#define MLX90393_T_bm             (1<<0)          /* Temperature Sensor Bitmask  */
#define MLX90393_X_bm             (1<<1)          /* Magnetometer X-Axis Bitmask */
#define MLX90393_Y_bm             (1<<2)          /* Magnetometer Y-Axis Bitmask */
#define MLX90393_Z_bm             (1<<3)          /* Magnetometer Z-Axis Bitmask */
#define MLX90393_ZYXT_bm          (MLX90393_Z_bm | MLX90393_Y_bm | MLX90393_X_bm | MLX90393_T_bm)

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
