/****************************************************************************
 * arch/arm/src/n32h7/n32_pwm.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_PWM_H
#define __ARCH_ARM_SRC_N32H7_N32_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include "chip.h"

#ifdef CONFIG_N32H7_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer type */
#define TIMTYPE_BASIC        0
#define TIMTYPE_GENERAL16    1
#define TIMTYPE_GENERAL32    2
#define TIMTYPE_ADVANCED     3
#define TIMTYPE_GTIMB        4   /* GTIMB (complementary, no break2) */

/* Timer modes */

enum n32_pwm_tim_mode_e
{
  N32_TIMMODE_COUNTUP   = 0,
  N32_TIMMODE_COUNTDOWN = 1,
  N32_TIMMODE_CENTER1   = 2,
  N32_TIMMODE_CENTER2   = 3,
  N32_TIMMODE_CENTER3   = 4,
};

/* Polarity */

enum n32_pwm_pol_e
{
  N32_POL_POS = 0,
  N32_POL_NEG = 1,
};

/* Idle state */

enum n32_pwm_idle_e
{
  N32_IDLE_INACTIVE = 0,
  N32_IDLE_ACTIVE   = 1,
};

/* PWM channel mode */

enum n32_pwm_chanmode_e
{
  N32_CHANMODE_FRZN        = 0,
  N32_CHANMODE_CHACT       = 1,
  N32_CHANMODE_CHINACT     = 2,
  N32_CHANMODE_OCREFTOG    = 3,
  N32_CHANMODE_OCREFLO     = 4,
  N32_CHANMODE_OCREFHI     = 5,
  N32_CHANMODE_PWM1        = 6,
  N32_CHANMODE_PWM2        = 7,
};

/* PWM channel numbers */

enum n32_pwm_chan_e
{
  N32_PWM_CHAN1 = 1,
  N32_PWM_CHAN2 = 2,
  N32_PWM_CHAN3 = 3,
  N32_PWM_CHAN4 = 4,
};

/* Output mask */

enum n32_pwm_output_e
{
  N32_PWM_OUT1  = (1 << 0),
  N32_PWM_OUT1N = (1 << 1),
  N32_PWM_OUT2  = (1 << 2),
  N32_PWM_OUT2N = (1 << 3),
  N32_PWM_OUT3  = (1 << 4),
  N32_PWM_OUT3N = (1 << 5),
  N32_PWM_OUT4  = (1 << 6),
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct n32_pwm_out_s
{
  uint8_t  in_use:1;
  uint8_t  pol:1;
  uint8_t  idle:1;
  uint32_t pincfg;
};

struct n32_pwm_break_s
{
  uint8_t en1:1;
  uint8_t pol1:1;
};

struct n32_pwmchan_s
{
  uint8_t                  channel:4;
  uint8_t                  mode:4;
  struct n32_pwm_out_s     out1;
  struct n32_pwm_break_s   brk;
  struct n32_pwm_out_s     out2;    /* complementary */
};

struct n32_pwmtimer_s
{
  const struct pwm_ops_s *ops;
  uint8_t  timid;
  uint8_t  timtype:3;
  uint8_t  mode:3;
  uint8_t  lock:2;
  uint8_t  deadtime;
  uint8_t  t_dts;
  uint32_t base;
  uint32_t pclk;
  struct n32_pwmchan_s *channels;
  uint32_t frequency;    /* current frequency */
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t  irq;
  uint8_t  prev;
  uint8_t  curr;
  uint32_t count;
  void    *handle;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct pwm_lowerhalf_s *n32_pwminitialize(int timer);

#endif /* CONFIG_N32H7_PWM */
#endif /* __ARCH_ARM_SRC_N32H7_N32_PWM_H */
