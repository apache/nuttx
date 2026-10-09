/****************************************************************************
 * include/nuttx/sensors/scl3300.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_SENSORS_SCL3300_H
#define __INCLUDE_NUTTX_SENSORS_SCL3300_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Operation modes (datasheet Table 2 and Table 12).  Used in
 * struct scl3300_config_s and as the argument of
 * SNIOC_SET_OPERATIONAL_MODE.
 */

enum scl3300_mode_e
{
  SCL3300_MODE_1 = 1,             /* +/-1.2 g, 40 Hz LPF (default) */
  SCL3300_MODE_2 = 2,             /* +/-2.4 g, 70 Hz LPF */
  SCL3300_MODE_3 = 3,             /* Inclination mode, 10 Hz LPF */
  SCL3300_MODE_4 = 4              /* Inclination mode, 10 Hz LPF, low noise */
};

/* Idle power policy.  Used in struct scl3300_config_s and as the argument
 * of SNIOC_SET_POWER_MODE.
 */

enum scl3300_power_e
{
  SCL3300_POWER_ALWAYS_ON = 0,    /* Stay powered when idle (default) */
  SCL3300_POWER_DOWN_IDLE = 1     /* Power down when no topic is active */
};

/* Platform data. Pass NULL to scl3300_register() for the defaults:
 * Mode 1 and SCL3300_POWER_ALWAYS_ON.
 */

struct scl3300_config_s
{
  uint8_t mode;                   /* enum scl3300_mode_e, 0 = default */
  uint8_t power;                  /* enum scl3300_power_e */
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
 * Name: scl3300_register
 *
 * Description:
 *   Register the Murata SCL3300 inclinometer as the uORB topics
 *   sensor_inclinometer<devno> and, if CONFIG_SENSORS_SCL3300_ACCEL is
 *   enabled, sensor_accel<devno>.  The chip select used is
 *   SPIDEV_ACCELEROMETER(devno).
 *
 * Input Parameters:
 *   devno  - Instance number of the uORB topics and of the chip select.
 *   spi    - An instance of the SPI interface to use to communicate with
 *            the SCL3300.
 *   config - Platform data, copied by the driver.  NULL selects Mode 1
 *            and SCL3300_POWER_ALWAYS_ON.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int scl3300_register(int devno, FAR struct spi_dev_s *spi,
                     FAR const struct scl3300_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_SCL3300_H */
