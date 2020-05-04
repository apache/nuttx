/****************************************************************************
 * include/nuttx/sensors/hdc1008.h
 *
 *   Copyright (C) 2020 Pelle Windestam. All rights reserved.
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

#ifndef __INCLUDE_NUTT_SENSORS_HDC1008_H
#define __INCLUDE_NUTT_SENSORS_HDC1008_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default address if both address pins are grounded */

#define CONFIG_HDC1008_ADDR 0x40

/* Modes of operation. Used in the calls to ioctl(). */

#define HDC1008_MEAS_TEMPERATURE  0x00
#define HDC1008_MEAS_HUMIDITY     0x01
#define HDC1008_MEAS_T_AND_RH     0x02

/* IOCTL commands */

#define SNIOC_RESET                 _SNIOC(0x0001) /* Soft Reset */
#define SNIOC_SET_OPERATIONAL_MODE  _SNIOC(0x0002) /* Mode: 0, 1, 2 */
#define SNIOC_SET_RESOLUTION_T      _SNIOC(0x0003) /* Resolution: 11, 14 */
#define SNIOC_SET_RESOLUTION_RH     _SNIOC(0x0004) /* Resolution: 8, 11, 14 */
#define SNIOC_SET_HEATER_MODE       _SNIOC(0x0005) /* Heater on or off */
#define SNIOC_GET_CONFIGURATION     _SNIOC(0x0006) /* Read config register */
#define SNIOC_MEASURE               _SNIOC(0x0007) /* Perform measurement */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

/* Structure with measurement data. Temperature is scaled by 100 and humidity
 * by 10.
 */

struct hdc1008_conv_data_s
{
  int temperature;
  int humidity;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hdc1008_register
 *
 * Description:
 *   Register the HDC1008 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the HDC1008
 *   addr    - The I2C address of the HDC1008. The I2C address is
 *             configurable by two address pins, in the range of 0x40-0x43
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hdc1008_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr);

#endif /* __INCLUDE_NUTT_SENSORS_HDC1008_H */
