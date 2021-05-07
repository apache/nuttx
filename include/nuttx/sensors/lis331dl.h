/****************************************************************************
 * include/nuttx/sensors/lis331dl.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LIS331DL_H
#define __INCLUDE_NUTTX_SENSORS_LIS331DL_H

#if defined(CONFIG_I2C) && defined(CONFIG_LIS331DL)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

struct lis331dl_dev_s;

struct lis331dl_vector_s
{
  int8_t x;
  int8_t y;
  int8_t z;
};

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lis331dl_init
 *
 * Description:
 *   Initialize ST LIS331DL Chip
 *
 * Input Parameters:
 *   i2c - I2C Device Structure
 *   address - I2C Address of the proposed device
 *
 * Returned Value:
 *   Pointer to newly allocated ST LIS331DL structure or NULL on error with
 *   errno set.
 *
 * Possible errno as set by this function on error:
 *  - ENODEV: When device addressed on given address is not compatible or it
 *    is not a LIS331DL
 *  - EFAULT: When there is no device at given address.
 *  - EBUSY: When device is already addressed by other device driver (not yet
 *    supported by low-level driver)
 *
 ****************************************************************************/

FAR struct lis331dl_dev_s *lis331dl_init(FAR struct i2c_master_s *i2c,
                                         uint16_t address);

/****************************************************************************
 * Name: lis331dl_deinit
 *
 * Description:
 *   Uninitialize ST LIS331DL Chip
 *
 * Input Parameters:
 *   dev - Device to LIS331DL device structure, as returned by the
 *         lis331dl_init()
 *
 * Returned Value:
 *   OK On success
 *
 ****************************************************************************/

int lis331dl_deinit(FAR struct lis331dl_dev_s *dev);

/****************************************************************************
 * Name: lis331dl_powerup
 *
 * Description:
 *   Power up device, start conversion
 *
 ****************************************************************************/

int lis331dl_powerup(FAR struct lis331dl_dev_s *dev);

/****************************************************************************
 * Name: lis331dl_powerdown
 *
 * Description:
 *   Power down device, stop conversion
 *
 ****************************************************************************/

int lis331dl_powerdown(FAR struct lis331dl_dev_s *dev);

/****************************************************************************
 * Name: lis331dl_setconversion
 *
 * Description:
 *   Configure conversion
 *
 * Input Parameters:
 *   dev  - Device to LIS331DL device structure
 *   full - When set, range of [-9g, 9g] is selected, otherwise [-2g, +2g]
 *   fast - When set, conversion operates at 400 Hz, otherwise at 100 Hz
 *
 * Returned Value:
 *   OK on success or errno is set
 *
 ****************************************************************************/

int lis331dl_setconversion(FAR struct lis331dl_dev_s *dev,
                           bool full,
                           bool fast);

/****************************************************************************
 * Name: lis331dl_getprecision
 *
 * Description:
 *   Get precision
 *
 * Returned Value:
 *   Precision of 1 LSB in terms of unit [mg]
 *
 ****************************************************************************/

int lis331dl_getprecision(FAR struct lis331dl_dev_s *dev);

/****************************************************************************
 * Name: lis331dl_getsamplerate
 *
 * Description:
 *   Get sample rate
 *
 * Returned Value:
 *   Sample rate in units of [Hz]
 *
 ****************************************************************************/

int lis331dl_getsamplerate(FAR struct lis331dl_dev_s *dev);

/****************************************************************************
 * Name: lis331dl_getreadings
 *
 * Description:
 *   Get readings, updates internal data structure
 *
 * Input Parameters:
 *   dev - Device to LIS331DL device structure
 *
 * Returned Value:
 *   Ptr to vector acceleration [x,y,z] on success, or NULL on error with
 *   errno set.  If data is not yet ready to be read from the LIS331 then
 *   errno is set to EAGAIN otherwise errno is set by I2C_TRANSFER().
 *
 ****************************************************************************/

FAR const struct lis331dl_vector_s *
  lis331dl_getreadings(FAR struct lis331dl_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_I2C && CONFIG_LIS331DL */
#endif /* __INCLUDE_NUTTX_SENSORS_LIS331DL_H */
