/****************************************************************************
 * include/nuttx/sensors/fxos8700cq.h
 * FXOS8700CQ Driver declaration
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/
#ifndef __INCLUDE_NUTTX_SENSORS_FXOS8700CQ_H
#define __INCLUDE_NUTTX_SENSORS_FXOS8700CQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#if defined(CONFIG_SENSORS_FXOS8700CQ)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Power mode */

#define FXOS8700CQ_PM_SUSPEND     (0x00)
#define FXOS8700CQ_PM_NORMAL      (0x01)
#define FXOS8700CQ_PM_LOWPOWER    (0x02)
#define FXOS8700CQ_PM_FASTSTARTUP (0x03)

/* IOCTL Commands ***********************************************************/

#define SNIOC_ENABLESC     _SNIOC(0x0001) /* Arg: uint8_t value */
#define SNIOC_READSC       _SNIOC(0x0002) /* Arg: int16_t* pointer */
#define SNIOC_SETACCPM     _SNIOC(0x0003) /* Arg: uint8_t value */
#define SNIOC_SETACCODR    _SNIOC(0x0004) /* Arg: uint8_t value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} v3int16_t;

typedef struct
{
    v3int16_t accel;
    v3int16_t magn;
} fxos8700cq_data;

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
 * Name: fxos8700cq_register
 *
 * Description:
 *   Register the FXOS8700CQ character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   dev     - An instance of the I2C interface to communicate with device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_SENSORS_FXOS8700CQ_SCU

int fxos8700cq_register(FAR const char *devpath,
                        FAR struct i2c_master_s *dev);

#else /* CONFIG_SENSORS_FXOS8700CQ_SCU */

int fxos8700cq_init(FAR struct i2c_master_s *dev, int port);
int fxos8700cqgyro_register(FAR const char *devpath, int minor,
                        FAR struct i2c_master_s *dev, int port);
int fxos8700cqaccel_register(FAR const char *devpath, int minor,
                         FAR struct i2c_master_s *dev, int port);

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_FXOS8700CQ */
#endif /* __INCLUDE_NUTTX_SENSORS_FXOS8700CQ_H */
