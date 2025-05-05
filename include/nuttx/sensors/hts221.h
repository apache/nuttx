/****************************************************************************
 * include/nuttx/sensors/hts221.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_HTS221_H
#define __INCLUDE_NUTTX_SENSORS_HTS221_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

/* Board interface: data-ready interrupt and power control.  All callbacks
 * are optional.  Without irq_attach the driver polls the sensor.
 */

typedef struct hts221_config_s
{
  int irq;
  CODE int  (*irq_attach)(FAR struct hts221_config_s *state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR const struct hts221_config_s *state,
                          bool enable);
  CODE void (*irq_clear)(FAR const struct hts221_config_s *state);
  CODE int  (*set_power)(FAR const struct hts221_config_s *state, bool on);
  bool drdy_active_low;         /* Data-ready pin polarity */
  bool drdy_open_drain;         /* Data-ready pin open drain, else push-pull */
} hts221_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hts221_register_uorb
 *
 * Description:
 *   Register the HTS221 as UORB devices: sensor_ambient_temp<devno> and
 *   sensor_humi<devno>.
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path.
 *   i2c    - The I2C bus driver instance.
 *   addr   - The I2C address of the HTS221.
 *   config - Board interface, or NULL.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hts221_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr, FAR hts221_config_t *config);

#endif /* __INCLUDE_NUTTX_SENSORS_HTS221_H */
