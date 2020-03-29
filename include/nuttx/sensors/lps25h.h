/****************************************************************************
 * include/nuttx/sensors/lps25h.h
 *
 *   Copyright (C) 2014, 2017 Haltian Ltd. All rights reserved.
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

#ifndef __INCLUDE_NUTT_SENSORS_LPS25H_H
#define __INCLUDE_NUTT_SENSORS_LPS25H_H

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

#endif /* __INCLUDE_NUTT_SENSORS_LPS25H_H */
