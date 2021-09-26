/****************************************************************************
 * include/nuttx/sensors/mpu60x0.h
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
 *    mpuc = kmm_malloc(sizeof(*mpuc));
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
