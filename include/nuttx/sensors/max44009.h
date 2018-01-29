/****************************************************************************
 * include/nuttx/sensors/max44009.h
 *
 *   Copyright (C) 2014-2018 Haltian Ltd. All rights reserved.
 *   Authors: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *            Juha Niskanen <juha.niskanen@haltian.com>
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
  CODE int (*irq_attach)(FAR struct max44009_config_s * state, xcpt_t isr, FAR void *arg);
  CODE void (*irq_enable)(FAR const struct max44009_config_s * state, bool enable);
  CODE int (*set_power)(FAR const struct max44009_config_s * state, bool on);
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
                               * the light level exceeds either threshold */
} ;

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
