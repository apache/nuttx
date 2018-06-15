/****************************************************************************
 * include/nuttx/sensors/t67xx.h
 *
 *   Copyright (C) 2018 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_T67XX_H
#define __INCLUDE_NUTTX_SENSORS_T67XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_T67XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T67XX_I2C_ADDR 0x15

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct t67xx_value_s
{
  uint16_t gas_ppm;  /* CO2 level, parts per million */
  bool warming_up;   /* Warm-up mode, gas_ppm might be imprecise */
  bool calibrating;  /* Calibration in progress, gas_ppm might be imprecise */
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
 * Name: t67xx_register
 *
 * Description:
 *   Register the T67XX character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/co2_0"
 *   i2c - An instance of the I2C interface to use to communicate with T67XX
 *   addr - The I2C address of the T67XX. For T6713 this is initially 0x15
 *   but it can be changed by the SLAVE ADDRESS command.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int t67xx_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_T67XX */
#endif /* __INCLUDE_NUTTX_SENSORS_T67XX_H */
