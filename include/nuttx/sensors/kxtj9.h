/****************************************************************************
 * include/nuttx/sensors/kxtj9.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This driver derives from the Motorola Moto Z MDK:
 *
 *   Copyright (c) 2016 Motorola Mobility, LLC.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_KXTJ9_H
#define __INCLUDE_NUTTX_SENSORS_KXTJ9_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_KXTJ9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Data control register bits */

enum kxtj9_odr_e
{
  ODR0_781F  = 8,
  ODR1_563F  = 9,
  ODR3_125F  = 10,
  ODR6_25F   = 11,
  ODR12_5F   = 0,
  ODR25F     = 1,
  ODR50F     = 2,
  ODR100F    = 3,
  ODR200F    = 4,
  ODR400F    = 5,
  ODR800F    = 6
};

/* Data returned by reading from the KXTJ9 is returned in this format.
 * In order for the read to be successful, a buffer of size >= sizeof(struct
 * kxtj9_sensor_data) must be provided with the read.
 */

struct kxtj9_sensor_data
{
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: kxtj9_register
 *
 * Description:
 *   Register the KXTJ9 accelerometer device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the KXTJ9 accelerometer, gyroscope or
 *             magnetometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int kxtj9_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t address);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_KXTJ9 */
#endif /* __INCLUDE_NUTTX_SENSORS_KXTJ9_H */
