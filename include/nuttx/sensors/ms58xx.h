/****************************************************************************
 * include/nuttx/sensors/ms58xx.h
 *
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#ifndef __INCLUDE_NUTTX_SENSORS_MS58XX
#define __INCLUDE_NUTTX_SENSORS_MS58XX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MS58XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_MS58XX
 *   Enables support for the MS58XX driver
 * CONFIG_MS58XX_VDD
 */

/* I2C Address **************************************************************/

#define MS58XX_ADDR0       0x76
#define MS58XX_ADDR1       0x77

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ms58xx_model_e
{
  MS58XX_MODEL_MS5803_02 = 0,
  MS58XX_MODEL_MS5803_05 = 1,
  MS58XX_MODEL_MS5803_07 = 2,
  MS58XX_MODEL_MS5803_14 = 3,
  MS58XX_MODEL_MS5803_30 = 4,
  MS58XX_MODEL_MS5805_02 = 5,
  MS58XX_MODEL_MS5806_02 = 6,
  MS58XX_MODEL_MS5837_30 = 7
};

struct ms58xx_measure_s
{
  int32_t temperature;  /* in Degree   x100    */
  int32_t pressure;     /* in mBar     x10     */
};

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

/****************************************************************************
 * Name: ms58xx_register
 *
 * Description:
 *   Register the MS58XX character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the MS58XX.
 *   osr     - The oversampling ratio.
 *   model   - The MS58XX model.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms58xx_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, uint16_t osr, enum ms58xx_model_e model);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MS58XX */
#endif /* __INCLUDE_NUTTX_SENSORS_MS58XX */
