/****************************************************************************
 * include/nuttx/sensors/mpu60x0.h
 *
 * Support for the Invensense MPU6000 and MPU6050 MotionTracking(tm)
 * 6-axis accelerometer and gyroscope.
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright+
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

#ifndef __INCLUDE_NUTTX_SENSORS_MPU60X0_H
#define __INCLUDE_NUTTX_SENSORS_MPU60X0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures are defined elsewhere, and we don't need their
 * definitions here.
 */

#ifdef CONFIG_MPU60X0_SPI
struct spi_dev_s;
#else
struct i2c_master_s;
#endif

/* Specifies the initial chip configuration and location.
 *
 * The chip supports both SPI and I2C interfaces, but you wouldn't use
 * both at the same time on the same chip. It isn't an error to have
 * one chip of each flavor in the system, though, so it's not an
 * either-or configuration item.
 *
 * Important note :
 *
 * The driver determines which interface type to use according to
 * which of the two groups of fields is non-NULL.  Since support for
 * I2C and SPI are individually configurable, however, users should
 * let the compiler clear unused fields instead of setting unused
 * fields to NULL directly. For example, if using SPI and a
 * stack-allocated instance:
 *
 *    struct mpu_config_s mpuc;
 *    memset(&mpuc, 0, sizeof(mpuc)); * sets i2c to NULL, if present *
 *    mpuc.spi = ...;
 *
 * Or, if using dynamic memory allocation and I2C:
 *
 *    struct mpu_config_s* mpuc;
 *    mpuc = malloc(sizeof(*mpuc));
 *    memset(mpuc, 0, sizeof(*mpuc)); * sets spi to NULL, if present *
 *    mpuc.i2c = ...;
 *
 * The above examples will avoid compile-time errors unless the user
 * forgets to enable their preferred interface type, and will allow
 * them to disable or enable the unused interface type without
 * changing their code.
 *
 */

struct mpu_config_s
{
#ifdef CONFIG_MPU60X0_SPI
  /* For users on SPI.
   *
   *  spi_devid : the SPI master's slave-select number
   *              for the chip, as used in SPI_SELECT(..., dev_id, ...)
   *  spi       : the SPI master device, as used in SPI_SELECT(spi, ..., ...)
   */

  FAR struct spi_dev_s *spi;
  int spi_devid;
#else
  /* For users on I2C.
   *
   *  i2c  : the I2C master device
   *  addr : the I2C address.
   */

  FAR struct i2c_master_s *i2c;
  int addr;
#endif
  };

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Declares the existence of an mpu60x0 chip, wired according to
 * config; creates an interface to it at path.
 *
 * Returns 0 on success, or negative errno.
 */

int mpu60x0_register(FAR const char *path, FAR struct mpu_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_MPU60X0_H */
