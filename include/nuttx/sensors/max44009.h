/****************************************************************************
 * include/nuttx/sensors/max44009.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MAX44009
#define __INCLUDE_NUTTX_SENSORS_MAX44009

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Integration time, ms */

enum max44009_integration_time_e
{
  MAX44009_INTEGR_TIME_800 = 0x0,     /* Preferred mode for boosting
                                       * low-light sensitivity */
  MAX44009_INTEGR_TIME_400,
  MAX44009_INTEGR_TIME_200,
  MAX44009_INTEGR_TIME_100,   /* Preferred mode for high-brightness
                               * applications */
  MAX44009_INTEGR_TIME_50,    /* Manual mode only */
  MAX44009_INTEGR_TIME_25,    /* Manual mode only */
  MAX44009_INTEGR_TIME_12_5,  /* Manual mode only */
  MAX44009_INTEGR_TIME_6_25   /* Manual mode only */
};

/* Board configuration data structure */

struct max44009_config_s
{
  CODE int (*irq_attach)(FAR struct max44009_config_s * state,
                         xcpt_t isr,
                         FAR void *arg);
  CODE void (*irq_enable)(FAR const struct max44009_config_s * state,
                          bool enable);
  CODE int (*set_power)(FAR const struct max44009_config_s * state,
                        bool on);
};

/* Configuration structure for MAX44009 */

struct max44009_init_s
{
  bool is_cont;               /* Needs more power, if it's in continuous
                               * mode. This one is useful in test-mode for
                               * instance */
  bool is_manual;             /* Timer's settings must be specified manually */
  bool is_cdr;                /* Current division ratio: false - All of the
                               * photodiode current goes to the ADC, true -
                               * 1/8 (must be used in high-brightness
                               * situations) */

  enum max44009_integration_time_e integr_time;  /* Integration time */
};

struct max44009_threshold_s
{
  uint8_t upper_threshold;    /* Upper threshold high-byte */
  uint8_t lower_threshold;    /* Lower threshold high-byte */
  uint8_t threshold_timer;    /* 0 - interrupt will be triggered as soon as
                               * the light level exceeds either threshold
                               */
};

/* Data transfer structure */

struct max44009_data_s
{
  uint32_t lux;               /* Converted lux value */
  uint16_t mlux;              /* Converted millilux (decimals for lux) */
  uint16_t raw_value;         /* Raw unconverted value */
  uint8_t  test_value;        /* For self-test only */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int max44009_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr, FAR struct max44009_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_MAX44009 */
