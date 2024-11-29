/****************************************************************************
 * arch/risc-v/src/bl808/bl808_timer.c
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>
#include <nuttx/fs/ioctl.h>

#include "hardware/bl808_timer.h"
#include "riscv_internal.h"
#include "chip.h"
#include "bl808_timer.h"

#ifdef CONFIG_BL808_TIMERS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_GET_BLK(n) (n >= 2)
#define TIMER_GET_CH(n)  (n % 2)

#define TIMER_CLK_SRC_XTAL 3
#define TIMER_CLK_SRC_NONE 5

#define XCLK_DIV 39 /* XCLK is 40 MHz, so divide by 40 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum bl808_timer_ch_e
  {
    TIMER0_CH0 = 0,
    TIMER0_CH1 = 1,
    TIMER1_CH0 = 2,
    TIMER1_CH1 = 3
  };

struct bl808_timer_ch_s
{
  const struct timer_ops_s *ops;
  enum bl808_timer_ch_e blk_ch;
  tccb_t callback;
  void *arg;
  bool started;
  uint32_t timeout;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

int bl808_timer_start(struct timer_lowerhalf_s *lower);
int bl808_timer_stop(struct timer_lowerhalf_s *lower);
int bl808_timer_getstatus(struct timer_lowerhalf_s *lower,
                          struct timer_status_s *status);
int bl808_timer_settimeout(struct timer_lowerhalf_s *lower,
                           uint32_t timeout);
void bl808_timer_setcallback(struct timer_lowerhalf_s *lower,
                            tccb_t callback, void *arg);
int bl808_timer_ioctl(struct timer_lowerhalf_s *lower,
                      int cmd, unsigned long arg);
int bl808_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                           uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timer_ops_s bl808_timer_ops =
  {
    .start = bl808_timer_start,
    .stop = bl808_timer_stop,
    .getstatus = bl808_timer_getstatus,
    .settimeout = bl808_timer_settimeout,
    .setcallback = bl808_timer_setcallback,
    .ioctl = bl808_timer_ioctl,
    .maxtimeout = bl808_timer_maxtimeout
  };

static struct bl808_timer_ch_s timer0_ch0 =
  {
    .ops = &bl808_timer_ops,
    .blk_ch = TIMER0_CH0,
    .callback = NULL,
    .arg = NULL,
    .started = false,
    .timeout = 0
  };

static struct bl808_timer_ch_s timer0_ch1 =
  {
    .ops = &bl808_timer_ops,
    .blk_ch = TIMER0_CH1,
    .callback = NULL,
    .arg = NULL,
    .started = false,
    .timeout = 0
  };

static struct bl808_timer_ch_s timer1_ch0 =
  {
    .ops = &bl808_timer_ops,
    .blk_ch = TIMER1_CH0,
    .callback = NULL,
    .arg = NULL,
    .started = false,
    .timeout = 0
  };

static struct bl808_timer_ch_s timer1_ch1 =
  {
    .ops = &bl808_timer_ops,
    .blk_ch = TIMER1_CH1,
    .callback = NULL,
    .arg = NULL,
    .started = false,
    .timeout = 0
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_interrupt
 *
 * Description:
 *   Timer interrupt handler. Clears the interrupt and
 *   Calls the attached callback if there is one.
 *
 ****************************************************************************/

static int __timer_interrupt(int irq, void *context, void *arg)
{
  struct bl808_timer_ch_s *priv = (struct bl808_timer_ch_s *)arg;
  uint32_t next_interval = 0;

  /* Clear IRQ */

  switch (irq)
    {
    case BL808_IRQ_TIMER0_CH0:
      modifyreg32(BL808_TIMER_CH0_ICLR(0), 0, TIMER_COMP0_INT);
      break;

    case BL808_IRQ_TIMER0_CH1:
      modifyreg32(BL808_TIMER_CH1_ICLR(0), 0, TIMER_COMP0_INT);
      break;

    case BL808_IRQ_TIMER1_CH0:
      modifyreg32(BL808_TIMER_CH0_ICLR(1), 0, TIMER_COMP0_INT);
      break;

    case BL808_IRQ_TIMER1_CH1:
      modifyreg32(BL808_TIMER_CH1_ICLR(1), 0, TIMER_COMP0_INT);
      break;

    default:
      return -EIO;
    }

  if (priv->callback != NULL)
    {
      if (priv->callback(&next_interval, priv->arg))
        {
          if (next_interval > 0)
            {
              bl808_timer_settimeout((struct timer_lowerhalf_s *)priv,
                                     next_interval);
            }

          bl808_timer_start((struct timer_lowerhalf_s *)priv);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bl808_timer_start
 *
 * Description:
 *   Reset the time to the current timeout and start the timer.
 *
 ****************************************************************************/

int bl808_timer_start(struct timer_lowerhalf_s *lower)
{
  struct bl808_timer_ch_s *priv = (struct bl808_timer_ch_s *)lower;
  if (TIMER_GET_CH(priv->blk_ch) == 0)
    {
      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH0_CLKSEL_MASK,
                  TIMER_CLK_SRC_XTAL << TIMER_CH0_CLKSEL_SHIFT);

      /* Clear timer */

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  0, TIMER_CH0_CLR);

      /* Wait for the counter to clear */

      while (getreg32(BL808_TIMER_CH0_COUNTER(TIMER_GET_BLK(priv->blk_ch)))
             != 0);

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH0_CLR, 0);
    }
  else /* Channel 1 */
    {
      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH1_CLKSEL_MASK,
                  TIMER_CLK_SRC_XTAL << TIMER_CH1_CLKSEL_SHIFT);

      /* Clear timer */

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  0, TIMER_CH1_CLR);

      /* Wait for the counter to clear */

      while (getreg32(BL808_TIMER_CH1_COUNTER(TIMER_GET_BLK(priv->blk_ch)))
             != 0);

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH1_CLR, 0);
    }

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: bl808_timer_stop
 *
 * Description:
 *   Stop the timer.
 *
 ****************************************************************************/

int bl808_timer_stop(struct timer_lowerhalf_s *lower)
{
  struct bl808_timer_ch_s *priv = (struct bl808_timer_ch_s *)lower;

  /* Timers are stopped by setting the input clock to NONE.
   * This is done to allow calling timer_stop and then get
   * the time left to timeout afterwards. If we used the
   * timer enable bits, the counter would reset to 0 when stopped.
   */

  if (TIMER_GET_CH(priv->blk_ch) == 0)
    {
      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH0_CLKSEL_MASK,
                  TIMER_CLK_SRC_NONE << TIMER_CH0_CLKSEL_SHIFT);
    }
  else
    {
      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH0_CLKSEL_MASK,
                  TIMER_CLK_SRC_NONE << TIMER_CH1_CLKSEL_SHIFT);
    }

  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: bl808_timer_getstatus
 *
 * Description:
 *   Get current timer status. Returns to status parameter.
 *
 ****************************************************************************/

int bl808_timer_getstatus(struct timer_lowerhalf_s *lower,
                          struct timer_status_s *status)
{
  struct bl808_timer_ch_s *priv = (struct bl808_timer_ch_s *)lower;
  uint32_t current_count;

  status->flags = priv->started
    | ((priv->callback != NULL) << 1);
  status->timeout = priv->timeout;

  if (TIMER_GET_CH(priv->blk_ch) == 0)
    {
      current_count =
        getreg32(BL808_TIMER_CH0_COUNTER(TIMER_GET_BLK(priv->blk_ch)));
    }
  else
    {
      current_count =
        getreg32(BL808_TIMER_CH1_COUNTER(TIMER_GET_BLK(priv->blk_ch)));
    }

  status->timeleft = priv->timeout - current_count;

  return OK;
}

/****************************************************************************
 * Name: bl808_timer_settimeout
 *
 * Description:
 *   Set a new timeout value and reset the timer.
 *
 ****************************************************************************/

int bl808_timer_settimeout(struct timer_lowerhalf_s *lower,
                           uint32_t timeout)
{
  struct bl808_timer_ch_s *priv = (struct bl808_timer_ch_s *)lower;
  if (TIMER_GET_CH(priv->blk_ch) == 0)
    {
      modifyreg32(BL808_TIMER_CH0_COMP0(TIMER_GET_BLK(priv->blk_ch)),
                  0xffffffff, timeout);
      priv->timeout = timeout;

      /* Clock is needed to clear counters */

      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH0_CLKSEL_MASK,
                  TIMER_CLK_SRC_XTAL << TIMER_CH0_CLKSEL_SHIFT);

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  0, TIMER_CH0_CLR);

      while (getreg32(BL808_TIMER_CH0_COUNTER(TIMER_GET_BLK(priv->blk_ch)))
             != 0);

      /* Disable clock to stop timer from running after clear */

      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH0_CLKSEL_MASK,
                  TIMER_CLK_SRC_NONE << TIMER_CH0_CLKSEL_SHIFT);

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH0_CLR, 0);
    }
  else
    {
      modifyreg32(BL808_TIMER_CH1_COMP0(TIMER_GET_BLK(priv->blk_ch)),
                  0xffffffff, timeout);
      priv->timeout = timeout;

      /* Clock is needed to clear counters */

      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH1_CLKSEL_MASK,
                  TIMER_CLK_SRC_XTAL << TIMER_CH1_CLKSEL_SHIFT);

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  0, TIMER_CH1_CLR);

      while (getreg32(BL808_TIMER_CH1_COUNTER(TIMER_GET_BLK(priv->blk_ch)))
             != 0);

      /* Disable clock to stop timer from running after clear */

      modifyreg32(BL808_TIMER_TCCR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH1_CLKSEL_MASK,
                  TIMER_CLK_SRC_NONE << TIMER_CH1_CLKSEL_SHIFT);

      modifyreg32(BL808_TIMER_EN_CLR(TIMER_GET_BLK(priv->blk_ch)),
                  TIMER_CH1_CLR, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: bl808_timer_setcallback
 *
 * Description:
 *   Sets a new callback to be run on timeout.
 *
 ****************************************************************************/

void bl808_timer_setcallback(struct timer_lowerhalf_s *lower,
                             tccb_t callback, void *arg)
{
  struct bl808_timer_ch_s *priv = (struct bl808_timer_ch_s *)lower;
  priv->callback = callback;
  priv->arg = arg;
}

/****************************************************************************
 * Name: bl808_timer_ioctl
 *
 * Description:
 *   Handle ioctl commands not recognized by upper-half.
 *
 ****************************************************************************/

int bl808_timer_ioctl(struct timer_lowerhalf_s *lower,
                      int cmd, unsigned long arg)
{
  /* No additional ioctl commands implemented */

  return -EIO;
}

/****************************************************************************
 * Name: bl808_timer_maxtimeout
 *
 * Description:
 *   Return the maximum allowed timeout value.
 *   Returns to maxtimeout parameter.
 *
 ****************************************************************************/

int bl808_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                           uint32_t *maxtimeout)
{
  /* Timer comparators are 32-bit */

  *maxtimeout = 0xffffffff;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bl808_timer_init(void)
{
  int ret = OK;

  /* The registered devpaths follow the recommended naming
   * convention, i.e. timer0, timer1, etc. The order chosen
   * for the numbering is the same as for bl808_timer_ch_e.
   */

  /* Timer 0, channel 0 */

  modifyreg32(BL808_TIMER_TCCR(0), TIMER_CH0_CLKSEL_MASK,
              TIMER_CLK_SRC_NONE << TIMER_CH0_CLKSEL_SHIFT);
  modifyreg32(BL808_TIMER_CH0_IE(0), 0, TIMER_COMP0_INT);
  modifyreg32(BL808_TIMER_DIV(0), TIMER_CH0_DIV_MASK,
              (XCLK_DIV << TIMER_CH0_DIV_SHIFT));
  modifyreg32(BL808_TIMER_MODE(0), 0, TIMER_CH0_MODE);
  modifyreg32(BL808_TIMER_EN_CLR(0), 0, TIMER_CH0_EN);
  ret |= irq_attach(BL808_IRQ_TIMER0_CH0, __timer_interrupt,
                   (void *)&timer0_ch0);
  if (ret == OK)
    {
      up_enable_irq(BL808_IRQ_TIMER0_CH0);
    }

  timer_register("/dev/timer0",
                 (struct timer_lowerhalf_s *)&timer0_ch0);

  /* Timer 0, channel 1 */

  modifyreg32(BL808_TIMER_TCCR(0), TIMER_CH1_CLKSEL_MASK,
              TIMER_CLK_SRC_NONE << TIMER_CH1_CLKSEL_SHIFT);
  modifyreg32(BL808_TIMER_CH1_IE(0), 0, TIMER_COMP0_INT);
  modifyreg32(BL808_TIMER_DIV(0), TIMER_CH1_DIV_MASK,
              (XCLK_DIV << TIMER_CH1_DIV_SHIFT));
  modifyreg32(BL808_TIMER_MODE(0), 0, TIMER_CH1_MODE);
  modifyreg32(BL808_TIMER_EN_CLR(0), 0, TIMER_CH1_EN);
  ret |= irq_attach(BL808_IRQ_TIMER0_CH1, __timer_interrupt,
                   (void *)&timer0_ch1);
  if (ret == OK)
    {
      up_enable_irq(BL808_IRQ_TIMER0_CH1);
    }

  timer_register("/dev/timer1",
                 (struct timer_lowerhalf_s *)&timer0_ch1);

  /* Timer 1, channel 0 */

  modifyreg32(BL808_TIMER_TCCR(1), TIMER_CH0_CLKSEL_MASK,
              TIMER_CLK_SRC_NONE << TIMER_CH0_CLKSEL_SHIFT);
  modifyreg32(BL808_TIMER_CH0_IE(1), 0, TIMER_COMP0_INT);
  modifyreg32(BL808_TIMER_DIV(1), TIMER_CH0_DIV_MASK,
              (XCLK_DIV << TIMER_CH0_DIV_SHIFT));
  modifyreg32(BL808_TIMER_MODE(1), 0, TIMER_CH0_MODE);
  modifyreg32(BL808_TIMER_EN_CLR(1), 0, TIMER_CH0_EN);
  ret |= irq_attach(BL808_IRQ_TIMER1_CH0, __timer_interrupt,
                   (void *)&timer1_ch0);
  if (ret == OK)
    {
      up_enable_irq(BL808_IRQ_TIMER1_CH0);
    }

  timer_register("/dev/timer2",
                 (struct timer_lowerhalf_s *)&timer1_ch0);

  /* Timer 1, channel 1 */

  modifyreg32(BL808_TIMER_TCCR(1), TIMER_CH1_CLKSEL_MASK,
              TIMER_CLK_SRC_NONE << TIMER_CH1_CLKSEL_SHIFT);
  modifyreg32(BL808_TIMER_CH1_IE(1), 0, TIMER_COMP0_INT);
  modifyreg32(BL808_TIMER_DIV(1), TIMER_CH1_DIV_MASK,
              (XCLK_DIV << TIMER_CH1_DIV_SHIFT));
  modifyreg32(BL808_TIMER_MODE(1), 0, TIMER_CH1_MODE);
  modifyreg32(BL808_TIMER_EN_CLR(1), 0, TIMER_CH1_EN);
  ret |= irq_attach(BL808_IRQ_TIMER1_CH1, __timer_interrupt,
                   (void *)&timer1_ch1);
  if (ret == OK)
    {
      up_enable_irq(BL808_IRQ_TIMER1_CH1);
    }

  timer_register("/dev/timer3",
                 (struct timer_lowerhalf_s *)&timer1_ch1);

  return ret;
}

#endif /* CONFIG_BL808_TIMERS */
