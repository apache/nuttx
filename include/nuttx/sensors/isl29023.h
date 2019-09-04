/****************************************************************************
 * include/nuttx/sensors/isl29023.h
 *
 *   Copyright (C) 2019 DataVision s.r.o. All rights reserved.
 *   Authors: Matous Pokorny <matous.pokorny@datavision.cz>
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

#ifndef __INCLUDE_NUTTX_SENSORS_ISL29023
#define __INCLUDE_NUTTX_SENSORS_ISL29023

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_ISL29023)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

enum isl29023_resolution_e
{
  ISL29023_RESOLUTION_16BITS =      0x0,
  ISL29023_RESOLUTION_12BITS =      0x1,
  ISL29023_RESOLUTION_8BITS =       0x2,
  ISL29023_RESOLUTION_4BITS =       0x3,
};

enum isl29023_als_range_e
{
  ISL29023_ALS_RANGE_1000 =         0x0,
  ISL29023_ALS_RANGE_4000 =         0x1,
  ISL29023_ALS_RANGE_16000 =        0x2,
  ISL29023_ALS_RANGE_64000 =        0x3,
};

/* ISL2923 goes to power dowm mode after mode *once */

enum isl29023_operational_mode_e
{
  ISL29023_OP_MODE_POWER_DOWN =     0x0,
  ISL29023_OP_MODE_ALS_ONCE =       0x1,
  ISL29023_OP_MODE_IR_ONCE =        0x2,
  ISL29023_OP_MODE_ALS_CONTINUES =  0x5,
  ISL29023_OP_MODE_IR_CONTINUES =   0x6,
};

/* Data transfer structure */

struct isl29023_data_s
{
  uint16_t lux;               /* Converted lux value */
  uint16_t raw;               /* Raw unconverted value */
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
 * Name: isl29023_register
 *
 * Description:
 *   Register the ISL29023 ALS device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/als0"
 *   i2c - An instance of the I2C interface to use to communicate with ALS
 *   addr - The I2C address of the ALS.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int isl29023_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_ISL29023 */
#endif /* __INCLUDE_NUTTX_SENSORS_ISL29023 */
