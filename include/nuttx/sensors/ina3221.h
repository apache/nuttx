/****************************************************************************
 * include/nuttx/sensors/ina3221.h
 *
 *   Copyright (C) 2018 Verge Inc. All rights reserved.
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_INA3221_H
#define __INCLUDE_NUTTX_SENSORS_INA3221_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_INA3221)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* INA3221 Register Definitions ***********************************************/
/* INA3221 Config Register bits */

#define INA3221_CONFIG_MODE_POWERDOWN   0x000
#define INA3221_CONFIG_MODE_SHUNT_TRIG  0x001
#define INA3221_CONFIG_MODE_BUS_TRIG    0x010
#define INA3221_CONFIG_MODE_BOTH_TRIG   0x011
#define INA3221_CONFIG_MODE_POWERDOWN_1 0x100
#define INA3221_CONFIG_MODE_SHUNT_CONT  0x101
#define INA3221_CONFIG_MODE_BUS_CONT    0x110
#define INA3221_CONFIG_MODE_BOTH_CONT   0x111

#define INA3221_CONFIG_VSHUNTCT_140_US  (0x000 << 3)
#define INA3221_CONFIG_VSHUNTCT_204_US  (0x001 << 3)
#define INA3221_CONFIG_VSHUNTCT_332_US  (0x010 << 3)
#define INA3221_CONFIG_VSHUNTCT_588_US  (0x011 << 3)
#define INA3221_CONFIG_VSHUNTCT_1100_US (0x100 << 3)
#define INA3221_CONFIG_VSHUNTCT_2116_US (0x101 << 3)
#define INA3221_CONFIG_VSHUNTCT_4156_US (0x110 << 3)
#define INA3221_CONFIG_VSHUNTCT_8244_US (0x111 << 3)

#define INA3221_CONFIG_VBUSCT_140_US  (0x000 << 6)
#define INA3221_CONFIG_VBUSCT_204_US  (0x001 << 6)
#define INA3221_CONFIG_VBUSCT_332_US  (0x010 << 6)
#define INA3221_CONFIG_VBUSCT_588_US  (0x011 << 6)
#define INA3221_CONFIG_VBUSCT_1100_US (0x100 << 6)
#define INA3221_CONFIG_VBUSCT_2116_US (0x101 << 6)
#define INA3221_CONFIG_VBUSCT_4156_US (0x110 << 6)
#define INA3221_CONFIG_VBUSCT_8244_US (0x111 << 6)

#define INA3221_CONFIG_AVG_NSAMPLES_1     (0x000 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_4     (0x001 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_16    (0x010 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_64    (0x011 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_128   (0x100 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_256   (0x101 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_512   (0x110 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_1024  (0x111 << 9)

#define INA3221_CONFIG_CH1_EN (1 << 12)
#define INA3221_CONFIG_CH2_EN (1 << 13)
#define INA3221_CONFIG_CH3_EN (1 << 14)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct ina3221_config_s
{
  uint8_t addr;
  int32_t shunt_resistor[3];
  uint16_t cfgreg;
};

struct ina3221_channel_s
{
  uint32_t voltage;  /* [microvolt] max 4.2 kV - device max 26V */
  int32_t  current;  /* [microampere] max 2.1 kA - sensor is bidirectional */
};

struct ina3221_s
{
  struct ina3221_channel_s ch[3];
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
 * Name: ina3221_register
 *
 * Description:
 *   Register the ina3221 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *   i2c - An instance of the I2C interface to use to communicate with INA3221
 *   addr - The I2C address of the INA3221.  The base I2C address of the INA3221
 *   is 0x80.  Bits 0-1 can be controlled to get 4 unique addresses from 0x80
 *   through 0x83.
 *   shuntval - resistor value in microohms
 *   config - a combination of the constants defined earlier in this file
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ina3221_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     FAR const struct ina3221_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_INA3221 */
#endif /* __INCLUDE_NUTTX_SENSORS_INA3221_H */
