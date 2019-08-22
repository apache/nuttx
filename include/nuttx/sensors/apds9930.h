/****************************************************************************
 * include/nuttx/sensors/apds9930.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_APDS9930_H
#define __INCLUDE_NUTTX_SENSORS_APDS9930_H

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_APDS9930) || defined(CONFIG_SENSORS_APDS9930_SCU))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

/* Prerequisites:
 *
 * CONFIG_APDS9930
 *   Enables support for the APDS9930 driver
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

/* IOCTL Commands */

/* These commands are valid for CONFIG_APDS9930_PROXIMITY_INTERRUPT=y */

#define SNIOC_SETPSLTHRESHOLD    _SNIOC(0x0001)  /* Set low threshold */
#define SNIOC_SETPSHTHRESHOLD    _SNIOC(0x0002)  /* Set high threshold */
#define SNIOC_SETPSPERSISTENCE   _SNIOC(0x0003)  /* Set persistence(0-15) */
#define SNIOC_STARTPSMEASUREMENT _SNIOC(0x0004)  /* Start measurement */
#define SNIOC_STOPPSMEASUREMENT  _SNIOC(0x0005)  /* Stop measurement */
#define SNIOC_GETINTSTATUS       _SNIOC(0x0006)  /* Get interrupt status */
#define SNIOC_CLEARPSINT         _SNIOC(0x0007)  /* Clear interrupt */

/****************************************************************************
 * Name: apds9930_init
 *
 * Description:
 *   Initialize APDS9930 proximity and ambient light sensor device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9930
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9930_init(FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: apds9930als_register
 *
 * Description:
 *   Register the APDS9930 ambient light sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9930
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9930als_register(FAR const char *devpath, int minor,
                         FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: apds9930ps_register
 *
 * Description:
 *   Register the APDS9930 proximity sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/proxim0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9930
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9930ps_register(FAR const char *devpath, int minor,
                        FAR struct i2c_master_s *i2c, int port);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_APDS9930 */
#endif /* __INCLUDE_NUTTX_SENSORS_APDS9930_H */
