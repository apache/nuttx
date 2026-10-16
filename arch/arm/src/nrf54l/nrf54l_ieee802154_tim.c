/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ieee802154_tim.c
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

#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include "nrf54l_ieee802154_tim.h"
#include "nrf54l_ieee802154_trace.h"

#include "nrf54l_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF54L_TIMER0
#  error CONFIG_NRF54L_TIMER0 is needed to handle radio timings
#endif

/* Timer instance - 0 */

#define NRF54L_IEEE802154_TIMER0 (0)

/* Timer period set to 16us (symbol duration) */

#define NRF54L_TIMER_FREQUENCY   (1000000 / 16)

/* 16MHz / (2 ** 8) =  62500 */

#define NRF54L_TIMER_PRESCALER   (8)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC ops */

static int nrf54l_radioi8_tim(struct nrf54l_radioi8_dev_s *dev, uint8_t chan,
                             uint32_t val);
static void nrf54l_radioi8_tim_stop(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_tim_reset(struct nrf54l_radioi8_dev_s *dev);

/* Interrupts logic */

static void nrf54l_radioi8_tim_expire(struct nrf54l_radioi8_dev_s *dev);
static int nrf54l_radioi8_isr_tim(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Timer ops */

static struct nrf54l_radioi8_tim_ops_s g_radioi8_tim_ops =
{
  .setup = nrf54l_radioi8_tim,
  .stop  = nrf54l_radioi8_tim_stop,
  .reset = nrf54l_radioi8_tim_reset
};

/* Timer instance */

static struct nrf54l_radioi8_tim_s g_radioi8_tim;

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_tim
 *
 * Description:
 *   Configure TIMER event.
 *
 ****************************************************************************/

static int nrf54l_radioi8_tim(struct nrf54l_radioi8_dev_s *dev, uint8_t chan,
                             uint32_t val)
{
  struct nrf54l_radioi8_tim_s *tim = NULL;
  irqstate_t                  flags;

  DEBUGASSERT(dev != NULL);
  tim = dev->tim;

  flags = enter_critical_section();

  if (tim->tim_pending == true)
    {
      wlerr("TIMER busy! drop %" PRId8 " %" PRId32 " request\n", chan, val);
      ASSERT(0);
      return -EBUSY;
    }

  /* Stop timer and clear the counter */

  NRF54L_TIM_STOP(tim->tim);
  NRF54L_TIM_CLEAR(tim->tim);

  /* Clear the previous event */

  NRF54L_TIM_ACKINT(tim->tim, chan);

  /* Set compare register */

  NRF54L_TIM_SETCC(tim->tim, chan, val);

  /* Configure interrupt */

  NRF54L_TIM_ENABLEINT(tim->tim, chan);

  /* Set TIMER pending flag and used channel */

  tim->tim_pending = true;
  tim->tim_now     = chan;

  /* Start timer */

  nrf54l_radioi8_trace_put(RADIO_TRACE_TIMSTART, chan);

  if (val > 0)
    {
      NRF54L_TIM_START(tim->tim);
    }
  else
    {
      /* Call handler now */

      nrf54l_radioi8_tim_expire(dev);
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_tim_stop
 *
 * Description:
 *   Stop timer.
 *
 ****************************************************************************/

static void nrf54l_radioi8_tim_stop(struct nrf54l_radioi8_dev_s *dev)
{
  NRF54L_TIM_STOP(dev->tim->tim);

  /* Clear the event, it may have already been latched */

  if (dev->tim->tim_pending)
    {
      NRF54L_TIM_DISABLEINT(dev->tim->tim, dev->tim->tim_now);
      NRF54L_TIM_ACKINT(dev->tim->tim, dev->tim->tim_now);
    }

  /* Reset state */

  dev->tim->tim_pending = false;
  dev->tim->tim_now     = -1;
}

/****************************************************************************
 * Name: nrf54l_radioi8_tim_reset
 *
 * Description:
 *   Reset TIMER.
 *
 ****************************************************************************/

static void nrf54l_radioi8_tim_reset(struct nrf54l_radioi8_dev_s *dev)
{
  /* Configure TIMER - freq = 62500 */

  NRF54L_TIM_STOP(dev->tim->tim);
  NRF54L_TIM_CONFIGURE(dev->tim->tim, NRF54L_TIM_MODE_TIMER,
                      NRF54L_TIM_WIDTH_32B);
  NRF54L_TIM_SETPRE(dev->tim->tim, NRF54L_TIMER_PRESCALER);

  /* Reset state */

  dev->tim->tim_pending = false;
  dev->tim->tim_now     = -1;
}

/****************************************************************************
 * Name: nrf54l_radioi8_tim_expire
 *
 * Description:
 *   Handle the expired timer channel.
 *
 ****************************************************************************/

static void nrf54l_radioi8_tim_expire(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radioi8_tim_s *tim = dev->tim;
  irqstate_t                  flags;

  flags = enter_critical_section();

  switch (tim->tim_now)
    {
      /* RX ACK handler */

      case NRF54L_TIMER_CHAN_ACK:
        {
          nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_TIMACKTX, 0);

          /* Start TX */

          dev->radio->ops->txstart(dev);

          break;
        }

      /* Delayed TX handler */

      case NRF54L_TIMER_CHAN_TXDELAY:
        {
          nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_TIMTXDELAY, 0);

          /* Trigger TX */

          dev->radio->ops->norm_trigger(dev);

          break;
        }

      /* ACK wait handler */

      case NRF54L_TIMER_CHAN_WAITACK:
        {
          nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_TIMWAITACK, 0);

          /* Notify radio layer */

          dev->radio->ops->notify_noack(dev);

          break;
        }

      case NRF54L_TIMER_CHAN_CSMADELAY:
        {
          nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_TIMCSMADELAY, 0);

          /* Start CCA - we transmit immediately after the CCA procedure ends
           * (CCAIDLE_TXEN short), or CCABUSY interrupt happen.
           */

          dev->radio->ops->ccastart(dev);

          break;
        }

      default:
        {
          ASSERT(0);
          break;
        }
    }

  /* Stop timer */

  NRF54L_TIM_STOP(tim->tim);

  /* Disable and clear interrupts */

  NRF54L_TIM_DISABLEINT(tim->tim, tim->tim_now);
  NRF54L_TIM_ACKINT(tim->tim, tim->tim_now);

  /* Clear TIMER pending flag and used channel */

  tim->tim_pending = false;
  tim->tim_now     = -1;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_radioi8_isr_tim
 *
 * Description:
 *   Helper tim interrupt handler.
 *
 ****************************************************************************/

static int nrf54l_radioi8_isr_tim(int irq, void *context, void *arg)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)arg;
  struct nrf54l_radioi8_tim_s *tim = NULL;
  irqstate_t                  flags;

  DEBUGASSERT(dev != NULL);
  tim = dev->tim;

  flags = enter_critical_section();

  /* Ignore an interrupt raised before the timer was stopped or set again */

  if (tim->tim_pending && NRF54L_TIM_CHECKINT(tim->tim, tim->tim_now))
    {
      nrf54l_radioi8_tim_expire(dev);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_tim_init
 *
 * Description:
 *   Initialize high resolution timer for IEEE802154 operations.
 *   Used to handle short radio timeouts like ACK, IFS or delayed
 *   transmissions.
 *
 ****************************************************************************/

struct nrf54l_radioi8_tim_s *
nrf54l_radioi8_tim_init(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_tim_dev_s *tim = NULL;

  /* Reserve TIMER0 */

  tim = nrf54l_tim_init(NRF54L_IEEE802154_TIMER0);
  if (tim == NULL)
    {
      wlerr("nrf54l_tim_init(0) failed %d\n", -errno);
      return NULL;
    }

  /* Attach TIMER interrupt */

  NRF54L_TIM_SETISR(tim, nrf54l_radioi8_isr_tim, dev);

  /* Set interrupts priority */

  up_prioritize_irq(NRF54L_IRQ_TIMER20, 0);

  /* Connect timer */

  memset(&g_radioi8_tim, 0, sizeof(struct nrf54l_radioi8_tim_s));
  g_radioi8_tim.ops = &g_radioi8_tim_ops;
  g_radioi8_tim.tim = tim;

  return &g_radioi8_tim;
}
