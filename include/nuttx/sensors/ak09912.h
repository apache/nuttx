/****************************************************************************
 * include/nuttx/sensors/ak09912.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __INCLUDE_NUTTX_SENSORS_AK09912_H
#define __INCLUDE_NUTTX_SENSORS_AK09912_H

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_AK09912) || defined (CONFIG_SENSORS_AK09912_SCU))


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_AK09912
 *   Enables support for the AK09912 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

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

/* IOCTL Commands ***********************************************************/

/* Arg: 0: Disable compensated
 *      1: Enable compensated
 */
#define ENABLE_COMPENSATED (1)
#define DISABLE_COMPENSATED (0)
#define SNIOC_ENABLE_COMPENSATED   _SNIOC(0x0001)

#define SNIOC_GETADJ               _SNIOC(0x0002)

struct mag_data_s
{
  int16_t x; /* X raw data */
  int16_t y; /* Y raw data */
  int16_t z; /* Z raw data */
};

struct ak09912_sensadj_s
{
  uint8_t x;
  uint8_t y;
  uint8_t z;
};

#ifdef CONFIG_SENSORS_AK09912_SCU
/****************************************************************************
 * Name: ak09912_init
 *
 * Description:
 *   Initialize AK09912 magnetometer device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09912_init(FAR struct i2c_master_s *i2c, int port);
#endif

/****************************************************************************
 * Name: ak09912_register
 *
 * Description:
 *   Register the AK09912 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_SENSORS_AK09912_SCU
int ak09912_register(FAR const char *devpath, int minor,
                     FAR struct i2c_master_s *i2c, int port);
#else
int ak09912_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_AK09912 */
#endif /* __INCLUDE_NUTTX_SENSORS_AK09912_H */
