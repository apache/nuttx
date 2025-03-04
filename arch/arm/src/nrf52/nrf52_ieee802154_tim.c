/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_tim.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include "nrf52_ieee802154_tim.h"
#include "nrf52_ieee802154_trace.h"

#include "nrf52_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF52_TIMER0
#  error CONFIG_NRF52_TIMER0 is needed to handle radio timings
#endif

/* Timer instance - 0 */

#define NRF52_IEEE802154_TIMER0 (0)

/* Timer period set to 16us (symbol duration) */

#define NRF52_TIMER_FREQUENCY   (1000000 / 16)

/* 16MHz / (2 ** 8) =  62500 */

#define NRF52_TIMER_PRESCALER   (8)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC ops */

static int nrf52_radioi8_tim(struct nrf52_radioi8_dev_s *dev, uint8_t chan,
                             uint32_t val);
static void nrf52_radioi8_tim_stop(struct nrf52_radioi8_dev_s *dev);
static void nrf52_radioi8_tim_reset(struct nrf52_radioi8_dev_s *dev);

/* Interrupts logic */

static int nrf52_radioi8_isr_tim(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Timer ops */

static struct nrf52_radioi8_tim_ops_s g_radioi8_tim_ops =
{
  .setup = nrf52_radioi8_tim,
  .stop  = nrf52_radioi8_tim_stop,
  .reset = nrf52_radioi8_tim_reset
};

/* Timer instance */

static struct nrf52_radioi8_tim_s g_radioi8_tim;

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_tim
 *
 * Description:
 *   Configure TIMER event.
 *
 ****************************************************************************/

static int nrf52_radioi8_tim(struct nrf52_radioi8_dev_s *dev, uint8_t chan,
                             uint32_t val)
{
  struct nrf52_radioi8_tim_s *tim = NULL;
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

  NRF52_TIM_STOP(tim->tim);
  NRF52_TIM_CLEAR(tim->tim);

  /* Clear the previous event */

  NRF52_TIM_ACKINT(tim->tim, chan);

  /* Set compare register */

  NRF52_TIM_SETCC(tim->tim, chan, val);

  /* Configure interupt */

  NRF52_TIM_ENABLEINT(tim->tim, chan);

  /* Set TIMER pending flag and used channel */

  tim->tim_pending = true;
  tim->tim_now     = chan;

  /* Start timer */

  nrf52_radioi8_trace_put(RADIO_TRACE_TIMSTART, chan);

  if (val > 0)
    {
      NRF52_TIM_START(tim->tim);
    }
  else
    {
      /* Call handler now */

      nrf52_radioi8_isr_tim(0, NULL, dev);
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_tim_stop
 *
 * Description:
 *   Stop timer.
 *
 ****************************************************************************/

static void nrf52_radioi8_tim_stop(struct nrf52_radioi8_dev_s *dev)
{
  NRF52_TIM_STOP(dev->tim->tim);
  NRF52_TIM_DISABLEINT(dev->tim->tim, dev->tim->tim_now);

  /* Reset state */

  dev->tim->tim_pending = false;
  dev->tim->tim_now     = -1;
}

/****************************************************************************
 * Name: nrf52_radioi8_tim_reset
 *
 * Description:
 *   Reset TIMER.
 *
 ****************************************************************************/

static void nrf52_radioi8_tim_reset(struct nrf52_radioi8_dev_s *dev)
{
  /* Configure TIMER - freq = 62500 */

  NRF52_TIM_STOP(dev->tim->tim);
  NRF52_TIM_CONFIGURE(dev->tim->tim, NRF52_TIM_MODE_TIMER,
                      NRF52_TIM_WIDTH_32B);
  NRF52_TIM_SETPRE(dev->tim->tim, NRF52_TIMER_PRESCALER);

  /* Reset state */

  dev->tim->tim_pending = false;
  dev->tim->tim_now     = -1;
}

/****************************************************************************
 * Name: nrf52_radioi8_isr_tim
 *
 * Description:
 *   Helper tim interrupt handler.
 *
 ****************************************************************************/

static int nrf52_radioi8_isr_tim(int irq, void *context, void *arg)
{
  struct nrf52_radioi8_dev_s *dev = (struct nrf52_radioi8_dev_s *)arg;
  struct nrf52_radioi8_tim_s *tim = NULL;
  irqstate_t                  flags;

  DEBUGASSERT(dev != NULL);
  tim = dev->tim;

  flags = enter_critical_section();

  switch (tim->tim_now)
    {
      /* RX ACK handler */

      case NRF52_TIMER_CHAN_ACK:
        {
          nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_TIMACKTX, 0);

          /* Start TX */

          dev->radio->ops->txstart(dev);

          break;
        }

      /* Delayed TX handler */

      case NRF52_TIMER_CHAN_TXDELAY:
        {
          nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_TIMTXDELAY, 0);

          /* Trigger TX */

          dev->radio->ops->norm_trigger(dev);

          break;
        }

      /* ACK wait handler */

      case NRF52_TIMER_CHAN_WAITACK:
        {
          nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_TIMWAITACK, 0);

          /* Notify radio layer */

          dev->radio->ops->notify_noack(dev);

          break;
        }

      case NRF52_TIMER_CHAN_CSMADELAY:
        {
          nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_TIMCSMADELAY, 0);

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

  NRF52_TIM_STOP(tim->tim);

  /* Disable and clear interrupts */

  NRF52_TIM_DISABLEINT(tim->tim, tim->tim_now);
  NRF52_TIM_ACKINT(tim->tim, tim->tim_now);

  /* Clear TIMER pending flag and used channel */

  tim->tim_pending = false;
  tim->tim_now     = -1;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_tim_init
 *
 * Description:
 *   Initialize high resoluton timer for IEEE802154 operations.
 *   Used to handle short radio timeouts like ACK, IFS or delayed
 *   transmitions.
 *
 ****************************************************************************/

struct nrf52_radioi8_tim_s *
nrf52_radioi8_tim_init(struct nrf52_radioi8_dev_s *dev)
{
  struct nrf52_tim_dev_s *tim = NULL;

  /* Reserve TIMER0 */

  tim = nrf52_tim_init(NRF52_IEEE802154_TIMER0);
  if (tim == NULL)
    {
      wlerr("nrf52_tim_init(0) failed %d\n", -errno);
      return NULL;
    }

  /* Atach TIMER interrupt */

  NRF52_TIM_SETISR(tim, nrf52_radioi8_isr_tim, dev);

  /* Set interrupts priority */

  up_prioritize_irq(NRF52_IRQ_TIMER0, 0);

  /* Connect timer */

  memset(&g_radioi8_tim, 0, sizeof(struct nrf52_radioi8_tim_s));
  g_radioi8_tim.ops = &g_radioi8_tim_ops;
  g_radioi8_tim.tim = tim;

  return &g_radioi8_tim;
}
