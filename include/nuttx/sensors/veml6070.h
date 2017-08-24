/****************************************************************************
 * include/nuttx/input/veml6070.h
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_VEML6070_H
#define __INCLUDE_NUTTX_SENSORS_VEML6070_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SENSORS_VEML6070)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device I2C Address */

#define VEML6070_I2C_DATA_LSB_CMD_ADDR  0x38
#define VEML6070_I2C_DATA_MSB_ADDR      0x39

/* Command Register Format
 * Bits:
 *  7  |  6  |  5  |    4    |  3  |  2  |  1  |  0 |
 * RSV | RSV | ACK | ACK_THD | IT1 | IT0 | RSV | SD |
 *
 * NOTE: The RSV Bit 1 needs to be always 1
 */


#define VEML6070_CMD_SD           0x01 /* Shutdown command */
#define VEML6070_CMD_RSV          0x02
#define VEML6070_CMD_IT_0_5T      0x00 /* IT1=0 : IT0=0 */
#define VEML6070_CMD_IT_1T        0x04 /* IT1=0 : IT0=1 */
#define VEML6070_CMD_IT_2T        0x08 /* IT1=1 : IT0=0 */
#define VEML6070_CMD_IT_4T        0x0c /* IT1=1 : IT0=1 */
#define VEML6070_CMD_ACK_THD      0x10 /* Acknowledge thresold:
                                          0 = 102 steps
                                          1 = 145 steps */
#define VEML6070_CMD_ACK          0x20 /* Acknowledge activity */

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
 * Name: veml6070_register
 *
 * Description:
 *   Register the VEML6070 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/uvlight0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *              VEML6070
 *   addr    - The I2C address of the VEML6070.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int veml6070_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_VEML6070 */
#endif /* __INCLUDE_NUTTX_SENSORS_VEML6070_H */
