/****************************************************************************
 * include/nuttx/sensors/lm92.h
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#ifndef __INCLUDE_NUTTX_SENSORS_LM92_H
#define __INCLUDE_NUTTX_SENSORS_LM92_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_LM92)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_LM92 - Enables support for the LM92 driver
 */

#define CONFIG_LM92_BASEADDR 0x48
#define CONFIG_LM92_ADDR0 (CONFIG_LM92_BASEADDR + 0)
#define CONFIG_LM92_ADDR1 (CONFIG_LM92_BASEADDR + 1)
#define CONFIG_LM92_ADDR2 (CONFIG_LM92_BASEADDR + 2)
#define CONFIG_LM92_ADDR3 (CONFIG_LM92_BASEADDR + 3)

/* IOCTL Commands ***********************************************************/

#define SNIOC_READCONF     _SNIOC(0x0001) /* Arg: uint8_t* pointer */
#define SNIOC_WRITECONF    _SNIOC(0x0002) /* Arg: uint8_t value */
#define SNIOC_SHUTDOWN     _SNIOC(0x0003) /* Arg: None */
#define SNIOC_POWERUP      _SNIOC(0x0004) /* Arg: None */
#define SNIOC_FAHRENHEIT   _SNIOC(0x0005) /* Arg: None */
#define SNIOC_CENTIGRADE   _SNIOC(0x0006) /* Arg: None */
#define SNIOC_READTHYS     _SNIOC(0x0007) /* Arg: b16_t* pointer */
#define SNIOC_WRITETHYS    _SNIOC(0x0008) /* Arg: b16_t value */
#define SNIOC_READTCRIT    _SNIOC(0x0009) /* Arg: b16_t* pointer */
#define SNIOC_WRITETCRIT   _SNIOC(0x000a) /* Arg: b16_t value */
#define SNIOC_READTLOW     _SNIOC(0x000b) /* Arg: b16_t* pointer */
#define SNIOC_WRITETLOW    _SNIOC(0x000c) /* Arg: b16_t value */
#define SNIOC_READTHIGH    _SNIOC(0x000d) /* Arg: b16_t* pointer */
#define SNIOC_WRITETHIGH   _SNIOC(0x000e) /* Arg: b16_t value */
#define SNIOC_READID       _SNIOC(0x000f) /* Arg: uint16_t* pointer */

/* LM92 Register Definitions ***********************************************/
/* LM92 Register Addresses */

#define LM92_TEMP_REG      0x00     /* Temperature Register */
#define LM92_CONF_REG      0x01     /* Configuration Register */
#define LM92_THYS_REG      0x02     /* Temperature Register */
#define LM92_TCRIT_REG     0x03     /* Critical Temperature Register */
#define LM92_TLOW_REG      0x04     /* Low Temperature Register */
#define LM92_THIGH_REG     0x05     /* High Temperature Register */
#define LM92_ID_REG        0x07     /* Manufacturer's Identification Register */

/* Configuration Register Bit Definitions */

#define LM92_CONF_SHUTDOWN      (1 << 0) /* Bit 0: Put LM92 goes in low power shutdown mode */
#define LM92_CONF_INTMODE       (1 << 1) /* Bit 1: 0=Comparator 1=Interrupt mode */
#define LM92_CONF_TCRITPOLARITY (1 << 2) /* Bit 2: 0=Active low 1=Active high */
#define LM92_CONF_INTPOLARITY   (1 << 3) /* Bit 3: 0=Active low 1=Active high */
#define LM92_CONF_FAULTQ        (1 << 4) /* Bit 4: 0=Disabled 1=Enabled */

/* NOTE: When temperature values are read, they are returned as b16_t, fixed
 * precision integer values (see include/fixedmath.h).
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

/****************************************************************************
 * Name: lm92_register
 *
 * Description:
 *   Register the LM92 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   addr    - The I2C address of the LM92.  The base I2C address of the LM92
 *             is 0x48.  Bits 0-2 can be controlled to get 4 unique addresses
 *             from 0x48 through 0x4b.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lm92_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                  uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_LM92 */
#endif /* __INCLUDE_NUTTX_SENSORS_LM92_H */
