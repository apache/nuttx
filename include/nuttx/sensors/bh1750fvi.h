/****************************************************************************
 * include/nuttx/input/bh1750fvi.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BH1750FVI_H
#define __INCLUDE_NUTTX_SENSORS_BH1750FVI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SENSORS_BH1750FVI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device I2C Address */

#define BH1750FVI_I2C_ADDR        0x23

/* Instruction Set Architecture */

#define BH1750FVI_POWERDOWN       0x00
#define BH1750FVI_POWERON         0x01
#define BH1750FVI_RESET           0x07
#define BH1750FVI_CONTINUOUS_HRM  0x10 /* Continuously H-Resolution Mode */
#define BH1750FVI_CONTINUOUS_HRM2 0x11 /* Continuously H-Resolution Mode 2 */
#define BH1750FVI_CONTINUOUS_LRM  0x12 /* Continuously L-Resolution Mode */
#define BH1750FVI_ONETIME_HRM     0x20 /* One Time H-Resolution Mode */
#define BH1750FVI_ONETIME_HRM2    0x21 /* One Time H-Resolution Mode 2 */
#define BH1750FVI_ONETIME_LRM     0x23 /* One Time L-Resolution Mode */
#define BH1750FVI_MEASURE_TIMEH   0x40 /* Change Measure Time 01000_MT[7,6,5] */
#define BH1750FVI_MEASURE_TIMEL   0x60 /* Change Measute Time 011_MT[4,3,2,1,0] */

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
 * Name: bh1750fvi_register
 *
 * Description:
 *   Register the BH1750FVI character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *              BH1750FVI
 *   addr    - The I2C address of the BH1750FVI.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1750fvi_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_BH1750FVI */
#endif /* __INCLUDE_NUTTX_SENSORS_BH1750FVI_H */
