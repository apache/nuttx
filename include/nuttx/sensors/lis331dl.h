/****************************************************************************
 * include/nuttx/sensors/lis331dl.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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

#ifndef __INCLUDE_NUTTX_SENSORS_LIS331DL_H
#define __INCLUDE_NUTTX_SENSORS_LIS331DL_H

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_I2C) && defined(CONFIG_LIS331DL)

/************************************************************************************
 * Pre-Processor Declarations
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Data Types
 ************************************************************************************/

struct lis331dl_dev_s;

struct lis331dl_vector_s
{
  int8_t x;
  int8_t y;
  int8_t z;
};

struct i2c_master_s;

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
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
 *   Pointer to newly allocated ST LIS331DL structure or NULL on error with errno
 *   set.
 *
 * Possible errno as set by this function on error:
 *  - ENODEV: When device addressed on given address is not compatible or it is not
 *    a LIS331DL
 *  - EFAULT: When there is no device at given address.
 *  - EBUSY: When device is already addressed by other device driver (not yet
 *    supported by low-level driver)
 *
 ************************************************************************************/

FAR struct lis331dl_dev_s *lis331dl_init(FAR struct i2c_master_s *i2c,
                                         uint16_t address);

/************************************************************************************
 * Name: lis331dl_deinit
 *
 * Description:
 *   Uninitialize ST LIS331DL Chip
 *
 * Input Parameters:
 *   dev - Device to LIS331DL device structure, as returned by the lis331dl_init()
 *
 * Returned Value:
 *   OK On success
 *
 ************************************************************************************/

int lis331dl_deinit(FAR struct lis331dl_dev_s *dev);

/************************************************************************************
 * Name: lis331dl_powerup
 *
 * Description:
 *   Power up device, start conversion
 *
 ************************************************************************************/

int lis331dl_powerup(FAR struct lis331dl_dev_s *dev);

/************************************************************************************
 * Name: lis331dl_powerdown
 *
 * Description:
 *   Power down device, stop conversion
 *
 ************************************************************************************/

int lis331dl_powerdown(FAR struct lis331dl_dev_s *dev);

/************************************************************************************
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
 ************************************************************************************/

int lis331dl_setconversion(FAR struct lis331dl_dev_s *dev, bool full, bool fast);

/************************************************************************************
 * Name: lis331dl_getprecision
 *
 * Description:
 *   Get precision
 *
 * Returned Value:
 *   Precision of 1 LSB in terms of unit [mg]
 *
 ************************************************************************************/

int lis331dl_getprecision(FAR struct lis331dl_dev_s *dev);

/************************************************************************************
 * Name: lis331dl_getsamplerate
 *
 * Description:
 *   Get sample rate
 *
 * Returned Value:
 *   Sample rate in units of [Hz]
 *
 ************************************************************************************/

int lis331dl_getsamplerate(FAR struct lis331dl_dev_s *dev);

/************************************************************************************
 * Name: lis331dl_getreadings
 *
 * Description:
 *   Get readings, updates internal data structure
 *
 * Input Parameters:
 *   dev - Device to LIS331DL device structure
 *
 * Returned Value:
 *   Ptr to vector acceleration [x,y,z] on success, or NULL on error with errno
 *   set.  If data is not yet ready to be read from the LIS331 then errno is set
 *   to EAGAIN otherwise errno is set by I2C_TRANSFER().
 *
 ************************************************************************************/

FAR const struct lis331dl_vector_s *
  lis331dl_getreadings(FAR struct lis331dl_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_I2C && CONFIG_LIS331DL */
#endif /* __INCLUDE_NUTTX_SENSORS_LIS331DL_H */
