/****************************************************************************
 * arch/risc-v/src/bl602/bl602_oneshot_lowerhalf.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/oneshot.h>

#include "riscv_internal.h"

#include <hardware/bl602_timer.h>
#include "bl602_tim.h"
#include "bl602_oneshot_lowerhalf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Private definitions */
#define TIMER_MAX_VALUE (0xFFFFFFFF)
#define TIMER_CLK_DIV   (160)
#define TIMER_CLK_FREQ  (160000000UL / (TIMER_CLK_DIV))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half driver
 */

struct bl602_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct bl602_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh; /* Common lower-half driver fields */

  /* Private lower half data follows */

  uint32_t           freq;    /* Timer frequency */
  uint8_t            tim;     /* timer tim 0,1 */
  uint8_t            irq;     /* IRQ associated with this timer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t bl602_max_delay(struct oneshot_lowerhalf_s *lower);
static clkcnt_t bl602_current(struct oneshot_lowerhalf_s *lower);
static void bl602_start_absolute(struct oneshot_lowerhalf_s *lower,
                                 clkcnt_t expected);
static void bl602_start(struct oneshot_lowerhalf_s *lower,
                        clkcnt_t delta);
static void bl602_cancel(struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */

static const struct oneshot_operations_s g_bl602_ops =
{
  .current        = bl602_current,
  .start          = bl602_start,
  .start_absolute = bl602_start_absolute,
  .cancel         = bl602_cancel,
  .max_delay      = bl602_max_delay
};

static struct bl602_oneshot_lowerhalf_s g_bl602_lowerhalf =
{
  .lh.ops = &g_bl602_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint64_t bl602_get_nsec(void)
{
  struct timespec ts =
  {
    0
  };

  /* Since bl602 can not get current time, it must
   * rely on the other clocksource, such as mtime.
   */

  up_timer_gettime(&ts);

  return (uint64_t)ts.tv_nsec + (uint64_t)ts.tv_sec * NSEC_PER_SEC;
}

/****************************************************************************
 * Name: bl602_oneshot_handler
 *
 * Description:
 *   Timer expiration handler
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when bl602_oneshot_start()
 *         was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int bl602_oneshot_handler(int irq, void *context, void *arg)
{
  struct bl602_oneshot_lowerhalf_s *priv = &g_bl602_lowerhalf;

  /* Clear Interrupt Bits */

  uint32_t int_id;
  uint32_t ticr_val;
  uint32_t ticr_addr;

  bl602_cancel(&priv->lh);

  if (priv->tim == 0)
    {
      int_id = getreg32(BL602_TIMER_TMSR2);
      ticr_addr = BL602_TIMER_TICR2;
    }
  else
    {
      int_id = getreg32(BL602_TIMER_TMSR3);
      ticr_addr = BL602_TIMER_TICR3;
    }

  ticr_val  = getreg32(ticr_addr);

  /* Comparator 0 match interrupt */

  if ((int_id & TIMER_TMSR2_TMSR_0) != 0)
    {
      putreg32(ticr_val | TIMER_TICR2_TCLR_0, ticr_addr);
      oneshot_process_callback(&priv->lh);
    }

  /* Comparator 1 match interrupt */

  if ((int_id & TIMER_TMSR2_TMSR_1) != 0)
    {
      putreg32(ticr_val | TIMER_TICR2_TCLR_1, ticr_addr);
    }

  /* Comparator 2 match interrupt */

  if ((int_id & TIMER_TMSR2_TMSR_2) != 0)
    {
      putreg32(ticr_val | TIMER_TICR2_TCLR_2, ticr_addr);
    }

  return 0;
}

static clkcnt_t bl602_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t bl602_current(struct oneshot_lowerhalf_s *lower)
{
  return bl602_get_nsec();
}

static void bl602_start_absolute(struct oneshot_lowerhalf_s *lower,
                                 clkcnt_t expected)
{
  uint64_t   delay;
  uint64_t   curr;

  curr = bl602_current(lower);

  /* In case of overflow. */

  delay = expected - curr > expected ? 0 : expected - curr;

  bl602_start(lower, delay);
}

static void bl602_start(struct oneshot_lowerhalf_s *lower, clkcnt_t delta)
{
  struct bl602_oneshot_lowerhalf_s *priv = &g_bl602_lowerhalf;
  irqstate_t flags;
  uint64_t   usec;

  /* Save the callback information and start the timer */

  flags = enter_critical_section();

  /* Express the delay in usec */

  usec = delta / 1000u;

  bl602_timer_setcompvalue(
    priv->tim, TIMER_COMP_ID_0, usec / (TIMER_CLK_FREQ / priv->freq));

  bl602_timer_setpreloadvalue(priv->tim, 0);
  bl602_timer_intmask(priv->tim, TIMER_INT_COMP_0, 0);
  bl602_timer_enable(priv->tim);

  leave_critical_section(flags);
}

static void bl602_cancel(struct oneshot_lowerhalf_s *lower)
{
  struct bl602_oneshot_lowerhalf_s *priv = &g_bl602_lowerhalf;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  flags = enter_critical_section();

  bl602_timer_disable(priv->tim);
  bl602_timer_intmask(priv->tim, TIMER_INT_COMP_0, 1);

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Input Parameters:
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *oneshot_initialize(int      chan,
                                               uint16_t resolution)
{
  struct bl602_oneshot_lowerhalf_s *priv = &g_bl602_lowerhalf;
  struct timer_cfg_s                timstr;

  /* Initialize the lower-half driver structure */

  priv->freq    = TIMER_CLK_FREQ / resolution;
  priv->tim     = chan;
  if (priv->tim == TIMER_CH0)
    {
      priv->irq = BL602_IRQ_TIMER_CH0;
    }
  else
    {
      priv->irq = BL602_IRQ_TIMER_CH1;
    }

  /* Initialize the contained BL602 oneshot timer */

  timstr.timer_ch = chan;              /* Timer channel */
  timstr.clk_src  = TIMER_CLKSRC_FCLK; /* Timer clock source */
  timstr.pl_trig_src =
    TIMER_PRELOAD_TRIG_COMP0; /* Timer count register preload trigger source
                               * select */

  timstr.count_mode = TIMER_COUNT_PRELOAD; /* Timer count mode */

  timstr.clock_division =
    (TIMER_CLK_DIV * resolution) - 1; /* Timer clock division value */

  timstr.match_val0 = TIMER_MAX_VALUE; /* Timer match 0 value 0 */
  timstr.match_val1 = TIMER_MAX_VALUE; /* Timer match 1 value 0 */
  timstr.match_val2 = TIMER_MAX_VALUE; /* Timer match 2 value 0 */

  timstr.pre_load_val = TIMER_MAX_VALUE; /* Timer preload value */

  bl602_timer_intmask(chan, TIMER_INT_ALL, 1);

  /* timer disable */

  bl602_timer_disable(chan);

  bl602_timer_init(&timstr);

  irq_attach(priv->irq, bl602_oneshot_handler, (void *)priv);

  bl602_timer_intmask(priv->tim, TIMER_INT_COMP_0, 1);
  up_enable_irq(priv->irq);

  oneshot_count_init(&priv->lh, NSEC_PER_SEC);

  return &priv->lh;
}
