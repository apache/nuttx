/****************************************************************************
 * drivers/sensors/vl53l1x.h
 *
 *   Copyright (C) 2019 Acutronics Robotics. All rights reserved.
 *   Author: Acutronics Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_VL53L1X_H
#define __INCLUDE_NUTTX_SENSORS_VL53L1X_H

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_VL53L1X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VL53L1X_I2C_PORTNO 1

#define SOFT_RESET                  0x0000
#define TIMEOUT_MACROP_LOOP_BOUND   0x0008
#define RANGE_OFFSET_MM             0x001e
#define INNER_OFFSET_MM             0x0020
#define OUTER_OFFSET_MM             0x0022
#define GPIO_MUX_CTRL               0x0030
#define GPIO_STATUS                 0x0031
#define PHASECAL_TIMEOUT_MACRO      0x004b
#define RANGE_CFG_TIMEOUT_MACRO_HI  0x005e
#define RANGE_VCSEL_PERIOD_A        0x0060
#define RANGE_VCSEL_PERIOD_B        0x0063
#define RANGE_TIMEOUT_MACRO_HI      0x0061
#define RANGE_CFG_VALID_PHASE       0x0069
#define SYSTEM__THRESH_HIGH         0x0072
#define SYSTEM__THRESH_LOW          0x0074
#define SD_CFG_WOI_SD0              0x0078
#define SD_CFG_INIT_PHASE           0x007a
#define INTERRUPT_CLEAR             0x0086
#define SYSTEM_MODE                 0x0087
#define EFFECTIVE_SPADS             0x008c
#define VL53L1_GET_DISTANCE         0x0096
#define SIGNAL_COUNT_RATE           0x0098
#define VL53L1_SYSTEM_STATUS        0x00e5
#define VL53L1_GET_ID               0x010f

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: vl53l1x_register
 *
 * Description:
 *   Register the VL53L1X character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/tof0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             VL53L1X
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vl53l1x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_VL53L1X */
#endif /* __INCLUDE_NUTTX_SENSORS_VL53L1X_H */
