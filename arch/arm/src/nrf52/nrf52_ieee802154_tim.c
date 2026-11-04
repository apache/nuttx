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

#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include "arm_internal.h"
#include "nrf52_ppi.h"
#include "hardware/nrf52_tim.h"
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

/* Timer period set to 1 us, 16 MHz / 16 = 1 MHz */

#define NRF52_TIMER_PRESCALER    NRF52_TIM_PRE_1000000

/* The ACK of a received frame starts aTurnaroundTime after its last symbol
 * (IEEE 802.15.4-2006 7.5.6.4.2).  While the timer is idle, RADIO PHYEND
 * clears and starts it and it stops at the ACK compare.  For an ACK the
 * compare enables TX through RADIO TXEN, TXREADY starts the TX.
 */

#define NRF52_TIMER_BASE         NRF52_TIMER0_BASE
#define NRF52_TIMER_ACKTXEN_US   (IEEE802154_TURN_AROUND_TIME * \
                                  IEEE802154_SYMBOL_US - \
                                  NRF52_RADIOI8_RAMPUP_US - \
                                  NRF52_RADIOI8_PHYEND_US)
#define NRF52_TIMER_LINK_PHYEND  (14)
#define NRF52_TIMER_LINK_ACKTXEN (15)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC ops */

static int nrf52_radioi8_tim(struct nrf52_radioi8_dev_s *dev, uint8_t chan,
                             uint32_t val);
static void nrf52_radioi8_tim_stop(struct nrf52_radioi8_dev_s *dev);
static void nrf52_radioi8_tim_reset(struct nrf52_radioi8_dev_s *dev);
static bool nrf52_radioi8_tim_ackarm(struct nrf52_radioi8_dev_s *dev);
static void nrf52_radioi8_tim_ackdone(struct nrf52_radioi8_dev_s *dev);

/* Interrupts logic */

static void nrf52_radioi8_tim_expire(struct nrf52_radioi8_dev_s *dev);
static int nrf52_radioi8_isr_tim(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Timer ops */

static struct nrf52_radioi8_tim_ops_s g_radioi8_tim_ops =
{
  .setup   = nrf52_radioi8_tim,
  .stop    = nrf52_radioi8_tim_stop,
  .reset   = nrf52_radioi8_tim_reset,
  .ackarm  = nrf52_radioi8_tim_ackarm,
  .ackdone = nrf52_radioi8_tim_ackdone
};

/* Timer instance */

static struct nrf52_radioi8_tim_s g_radioi8_tim;

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_tim_count
 *
 * Description:
 *   Capture the timer, the TX delay channel is free when the timer is idle.
 *
 ****************************************************************************/

static uint32_t nrf52_radioi8_tim_count(void)
{
  putreg32(TIM_TASKS_CAPTURE, NRF52_TIMER_BASE +
           NRF52_TIM_TASKS_CAPTURE_OFFSET(NRF52_TIMER_CHAN_TXDELAY));

  return getreg32(NRF52_TIMER_BASE +
                  NRF52_TIM_CC_OFFSET(NRF52_TIMER_CHAN_TXDELAY));
}

/****************************************************************************
 * Name: nrf52_radioi8_tim_idle
 *
 * Description:
 *   Let RADIO PHYEND start the idle timer, or keep it for a timer event.
 *
 ****************************************************************************/

static void nrf52_radioi8_tim_idle(struct nrf52_radioi8_dev_s *dev,
                                   bool idle)
{
  struct nrf52_radioi8_tim_s *tim = dev->tim;

  if (idle)
    {
      NRF52_TIM_STOP(tim->tim);
      NRF52_TIM_CLEAR(tim->tim);
      NRF52_TIM_SHORTS(tim->tim, NRF52_TIM_SHORT_COMPARE_STOP,
                       NRF52_TIMER_CHAN_ACK, true);
      nrf52_ppi_channel_enable(NRF52_TIMER_LINK_PHYEND, true);
    }
  else
    {
      nrf52_ppi_channel_enable(NRF52_TIMER_LINK_PHYEND, false);
      NRF52_TIM_SHORTS(tim->tim, NRF52_TIM_SHORT_COMPARE_STOP,
                       NRF52_TIMER_CHAN_ACK, false);
    }
}

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

  nrf52_radioi8_tim_idle(dev, false);
  NRF52_TIM_STOP(tim->tim);
  NRF52_TIM_CLEAR(tim->tim);

  /* Clear the previous event */

  NRF52_TIM_ACKINT(tim->tim, chan);

  /* Set compare register */

  NRF52_TIM_SETCC(tim->tim, chan, val * IEEE802154_SYMBOL_US);

  /* Configure interrupt */

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

      nrf52_radioi8_tim_expire(dev);
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

  /* Clear the event, it may have already been latched */

  if (dev->tim->tim_pending)
    {
      NRF52_TIM_DISABLEINT(dev->tim->tim, dev->tim->tim_now);
      NRF52_TIM_ACKINT(dev->tim->tim, dev->tim->tim_now);
    }

  /* Reset state */

  dev->tim->tim_pending = false;
  dev->tim->tim_now     = -1;

  nrf52_radioi8_tim_idle(dev, true);
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
  /* Configure TIMER - freq = 1 MHz */

  NRF52_TIM_STOP(dev->tim->tim);
  NRF52_TIM_CONFIGURE(dev->tim->tim, NRF52_TIM_MODE_TIMER,
                      NRF52_TIM_WIDTH_32B);
  NRF52_TIM_SETPRE(dev->tim->tim, NRF52_TIMER_PRESCALER);
  NRF52_TIM_SETCC(dev->tim->tim, NRF52_TIMER_CHAN_ACK,
                NRF52_TIMER_ACKTXEN_US);

  /* RADIO PHYEND starts the timer, the ACK compare enables TX */

  nrf52_ppi_set_event_ep(NRF52_TIMER_LINK_PHYEND,
                         NRF52_RADIO_EVENTS_PHYEND);
  nrf52_ppi_set_task_ep(NRF52_TIMER_LINK_PHYEND,
                        NRF52_TIMER_BASE + NRF52_TIM_TASKS_CLEAR_OFFSET);
  nrf52_ppi_set_task2_ep(NRF52_TIMER_LINK_PHYEND,
                         NRF52_TIMER_BASE + NRF52_TIM_TASKS_START_OFFSET);
  nrf52_ppi_set_event_ep(NRF52_TIMER_LINK_ACKTXEN,
                         NRF52_TIMER_BASE +
                         NRF52_TIM_EVENTS_COMPARE_OFFSET(
                           NRF52_TIMER_CHAN_ACK));
  nrf52_ppi_set_task_ep(NRF52_TIMER_LINK_ACKTXEN, NRF52_RADIO_TASKS_TXEN);

  nrf52_ppi_channel_enable(NRF52_TIMER_LINK_ACKTXEN, false);

  /* Reset state */

  dev->tim->tim_pending = false;
  dev->tim->tim_now     = -1;

  nrf52_radioi8_tim_idle(dev, true);
}

/****************************************************************************
 * Name: nrf52_radioi8_tim_ackarm
 *
 * Description:
 *   Let the timer enable the ACK TX.  It runs since PHYEND only when it was
 *   idle, and it must not have reached the ACK compare yet.
 *
 ****************************************************************************/

static bool nrf52_radioi8_tim_ackarm(struct nrf52_radioi8_dev_s *dev)
{
  struct nrf52_radioi8_tim_s *tim = dev->tim;
  bool armed = false;
  uint32_t count;
  irqstate_t flags;

  flags = enter_critical_section();

  count = nrf52_radioi8_tim_count();
  if (!tim->tim_pending && count > 0 && count < NRF52_TIMER_ACKTXEN_US)
    {
      nrf52_ppi_channel_enable(NRF52_TIMER_LINK_ACKTXEN, true);

      /* The compare may pass while the channel is enabled */

      if (nrf52_radioi8_tim_count() < NRF52_TIMER_ACKTXEN_US ||
          (NRF52_RADIO_GETREG(dev->radio->lower, NRF52_RADIO_STATE_OFFSET) &
           RADIO_STATE_MASK) != RADIO_STATE_DISABLED)
        {
          tim->tim_pending = true;
          tim->tim_now     = NRF52_TIMER_CHAN_ACK;
          armed            = true;
        }
      else
        {
          nrf52_ppi_channel_enable(NRF52_TIMER_LINK_ACKTXEN, false);
        }
    }

  leave_critical_section(flags);
  return armed;
}

/****************************************************************************
 * Name: nrf52_radioi8_tim_ackdone
 *
 * Description:
 *   The ACK TX is done, the timer is idle again.
 *
 ****************************************************************************/

static void nrf52_radioi8_tim_ackdone(struct nrf52_radioi8_dev_s *dev)
{
  irqstate_t flags;

  flags = enter_critical_section();

  nrf52_ppi_channel_enable(NRF52_TIMER_LINK_ACKTXEN, false);

  if (dev->tim->tim_now == NRF52_TIMER_CHAN_ACK)
    {
      dev->tim->tim_pending = false;
      dev->tim->tim_now     = -1;
      nrf52_radioi8_tim_idle(dev, true);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf52_radioi8_tim_expire
 *
 * Description:
 *   Handle the expired timer channel.
 *
 ****************************************************************************/

static void nrf52_radioi8_tim_expire(struct nrf52_radioi8_dev_s *dev)
{
  struct nrf52_radioi8_tim_s *tim = dev->tim;
  irqstate_t                  flags;

  flags = enter_critical_section();

  switch (tim->tim_now)
    {
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

  nrf52_radioi8_tim_idle(dev, true);

  leave_critical_section(flags);
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

  /* Ignore an interrupt raised before the timer was stopped or set again */

  if (tim->tim_pending && NRF52_TIM_CHECKINT(tim->tim, tim->tim_now))
    {
      nrf52_radioi8_tim_expire(dev);
    }

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
 *   Initialize high resolution timer for IEEE802154 operations.
 *   Used to handle short radio timeouts like ACK, IFS or delayed
 *   transmissions.
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

  /* Attach TIMER interrupt */

  NRF52_TIM_SETISR(tim, nrf52_radioi8_isr_tim, dev);

  /* Connect timer */

  memset(&g_radioi8_tim, 0, sizeof(struct nrf52_radioi8_tim_s));
  g_radioi8_tim.ops = &g_radioi8_tim_ops;
  g_radioi8_tim.tim = tim;

  return &g_radioi8_tim;
}
