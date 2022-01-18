/****************************************************************************
 * include/nuttx/sensors/lps25h.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LPS25H_H
#define __INCLUDE_NUTTX_SENSORS_LPS25H_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPS25H_TEMPER_DIVIDER  1000

#define LPS25H_VALID_WHO_AM_I  0xbd

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct lps25h_temper_data_s
{
  int32_t int_temper;          /* int_temper value must be divided by
                                * LPS25H_TEMPER_DIVIDER in your app code */
  int16_t raw_data;
} lps25h_temper_data_t;

typedef struct lps25h_pressure_data_s
{
  uint32_t pressure_int_hp;
  uint32_t pressure_pa;
  uint32_t raw_data;
} lps25h_pressure_data_t;

typedef struct lps25h_who_am_i_data
{
  uint8_t who_am_i;
} lps25h_who_am_i_data;

typedef struct lps25h_config_s
{
  /* Device characterization */

  int irq;                    /* IRQ number received by interrupt handler. */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
   * to isolate the driver from differences in GPIO interrupt handling
   * by varying boards and MCUs.
   * irq_attach - Attach the interrupt handler to the GPIO interrupt
   * irq_enable - Enable or disable the GPIO
   * irq_clear - Acknowledge/clear any pending GPIO interrupt
   * set_power - Ask board to turn on regulator
   */

  CODE int (*irq_attach)(FAR struct lps25h_config_s *state, xcpt_t isr,
                         FAR void *arg);
  CODE void (*irq_enable)(FAR const struct lps25h_config_s *state,
                          bool enable);
  CODE void (*irq_clear)(FAR const struct lps25h_config_s *state);
  CODE int (*set_power)(FAR const struct lps25h_config_s *state, bool on);
} lps25h_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int lps25h_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, FAR lps25h_config_t *config);

#endif /* __INCLUDE_NUTTX_SENSORS_LPS25H_H */
