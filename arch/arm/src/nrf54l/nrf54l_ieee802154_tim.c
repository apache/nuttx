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

#include "arm_internal.h"
#include "hardware/nrf54l_dppic.h"
#include "hardware/nrf54l_tim.h"
#include "nrf54l_ieee802154_tim.h"
#include "nrf54l_ieee802154_trace.h"

#include "nrf54l_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF54L_TIMER6
#  error CONFIG_NRF54L_TIMER6 is needed to handle radio timings
#endif

/* Timer instance - TIMER10 */

#define NRF54L_IEEE802154_TIMER10 (6)

/* Timer period set to 1 us, 32 MHz / 32 = 1 MHz */

#define NRF54L_TIMER_PRESCALER    NRF54L_TIM_PRE_DIV32

/* The ACK of a received frame starts aTurnaroundTime after its last symbol
 * (IEEE 802.15.4-2006 7.5.6.4.2).  While the timer is idle, RADIO PHYEND
 * clears and starts it and it stops at the ACK compare.  For an ACK the
 * compare enables TX through RADIO TXEN, TXREADY starts the TX.
 */

#define NRF54L_TIMER_BASE         NRF54L_TIMER10_BASE
#define NRF54L_TIMER_ACKTXEN_US   (IEEE802154_TURN_AROUND_TIME * \
                                   IEEE802154_SYMBOL_US - \
                                   NRF54L_RADIOI8_RAMPUP_US - \
                                   NRF54L_RADIOI8_PHYEND_US)
#define NRF54L_TIMER_LINK_PHYEND  (14)
#define NRF54L_TIMER_LINK_ACKTXEN (15)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC ops */

static int nrf54l_radioi8_tim(struct nrf54l_radioi8_dev_s *dev, uint8_t chan,
                             uint32_t val);
static void nrf54l_radioi8_tim_stop(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_tim_reset(struct nrf54l_radioi8_dev_s *dev);
static bool nrf54l_radioi8_tim_ackarm(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_tim_ackdone(struct nrf54l_radioi8_dev_s *dev);

/* Interrupts logic */

static void nrf54l_radioi8_tim_expire(struct nrf54l_radioi8_dev_s *dev);
static int nrf54l_radioi8_isr_tim(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Timer ops */

static struct nrf54l_radioi8_tim_ops_s g_radioi8_tim_ops =
{
  .setup   = nrf54l_radioi8_tim,
  .stop    = nrf54l_radioi8_tim_stop,
  .reset   = nrf54l_radioi8_tim_reset,
  .ackarm  = nrf54l_radioi8_tim_ackarm,
  .ackdone = nrf54l_radioi8_tim_ackdone
};

/* Timer instance */

static struct nrf54l_radioi8_tim_s g_radioi8_tim;

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_tim_count
 *
 * Description:
 *   Capture the timer, the TX delay channel is free when the timer is idle.
 *
 ****************************************************************************/

static uint32_t nrf54l_radioi8_tim_count(void)
{
  putreg32(TIM_TASKS_CAPTURE, NRF54L_TIMER_BASE +
           NRF54L_TIM_TASKS_CAPTURE_OFFSET(NRF54L_TIMER_CHAN_TXDELAY));

  return getreg32(NRF54L_TIMER_BASE +
                  NRF54L_TIM_CC_OFFSET(NRF54L_TIMER_CHAN_TXDELAY));
}

/****************************************************************************
 * Name: nrf54l_radioi8_tim_idle
 *
 * Description:
 *   Let RADIO PHYEND start the idle timer, or keep it for a timer event.
 *
 ****************************************************************************/

static void nrf54l_radioi8_tim_idle(struct nrf54l_radioi8_dev_s *dev,
                                    bool idle)
{
  struct nrf54l_radioi8_tim_s *tim = dev->tim;

  if (idle)
    {
      NRF54L_TIM_STOP(tim->tim);
      NRF54L_TIM_CLEAR(tim->tim);
      NRF54L_TIM_SHORTS(tim->tim, NRF54L_TIM_SHORT_COMPARE_STOP,
                        NRF54L_TIMER_CHAN_ACK, true);
      putreg32(DPPIC_CHEN_CH(NRF54L_TIMER_LINK_PHYEND),
               NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENSET_OFFSET);
    }
  else
    {
      putreg32(DPPIC_CHEN_CH(NRF54L_TIMER_LINK_PHYEND),
               NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENCLR_OFFSET);
      NRF54L_TIM_SHORTS(tim->tim, NRF54L_TIM_SHORT_COMPARE_STOP,
                        NRF54L_TIMER_CHAN_ACK, false);
    }
}

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

  nrf54l_radioi8_tim_idle(dev, false);
  NRF54L_TIM_STOP(tim->tim);
  NRF54L_TIM_CLEAR(tim->tim);

  /* Clear the previous event */

  NRF54L_TIM_ACKINT(tim->tim, chan);

  /* Set compare register */

  NRF54L_TIM_SETCC(tim->tim, chan, val * IEEE802154_SYMBOL_US);

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

  nrf54l_radioi8_tim_idle(dev, true);
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
  /* Configure TIMER - freq = 1 MHz */

  NRF54L_TIM_STOP(dev->tim->tim);
  NRF54L_TIM_CONFIGURE(dev->tim->tim, NRF54L_TIM_MODE_TIMER,
                      NRF54L_TIM_WIDTH_32B);
  NRF54L_TIM_SETPRE(dev->tim->tim, NRF54L_TIMER_PRESCALER);
  NRF54L_TIM_SETCC(dev->tim->tim, NRF54L_TIMER_CHAN_ACK,
                 NRF54L_TIMER_ACKTXEN_US);

  /* RADIO PHYEND starts the timer, the ACK compare enables TX */

  putreg32(DPPI_EN | NRF54L_TIMER_LINK_PHYEND,
           NRF54L_TIMER_BASE + NRF54L_TIM_SUBSCRIBE_CLEAR_OFFSET);
  putreg32(DPPI_EN | NRF54L_TIMER_LINK_PHYEND,
           NRF54L_TIMER_BASE + NRF54L_TIM_SUBSCRIBE_START_OFFSET);
  putreg32(DPPI_EN | NRF54L_TIMER_LINK_ACKTXEN,
           NRF54L_TIMER_BASE +
           NRF54L_TIM_PUBLISH_COMPARE_OFFSET(NRF54L_TIMER_CHAN_ACK));
  NRF54L_RADIO_PUTREG(dev->radio->lower, NRF54L_RADIO_PUBLISH_PHYEND_OFFSET,
                      DPPI_EN | NRF54L_TIMER_LINK_PHYEND);
  NRF54L_RADIO_PUTREG(dev->radio->lower, NRF54L_RADIO_SUBSCRIBE_TXEN_OFFSET,
                      DPPI_EN | NRF54L_TIMER_LINK_ACKTXEN);

  putreg32(DPPIC_CHEN_CH(NRF54L_TIMER_LINK_ACKTXEN),
           NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENCLR_OFFSET);

  /* Reset state */

  dev->tim->tim_pending = false;
  dev->tim->tim_now     = -1;

  nrf54l_radioi8_tim_idle(dev, true);
}

/****************************************************************************
 * Name: nrf54l_radioi8_tim_ackarm
 *
 * Description:
 *   Let the timer enable the ACK TX.  It runs since PHYEND only when it was
 *   idle, and it must not have reached the ACK compare yet.
 *
 ****************************************************************************/

static bool nrf54l_radioi8_tim_ackarm(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radioi8_tim_s *tim = dev->tim;
  bool armed = false;
  uint32_t count;
  irqstate_t flags;

  flags = enter_critical_section();

  count = nrf54l_radioi8_tim_count();
  if (!tim->tim_pending && count > 0 && count < NRF54L_TIMER_ACKTXEN_US)
    {
      putreg32(DPPIC_CHEN_CH(NRF54L_TIMER_LINK_ACKTXEN),
               NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENSET_OFFSET);

      /* The compare may pass while the channel is enabled */

      if (nrf54l_radioi8_tim_count() < NRF54L_TIMER_ACKTXEN_US ||
          (NRF54L_RADIO_GETREG(dev->radio->lower,
                               NRF54L_RADIO_STATE_OFFSET) &
           RADIO_STATE_MASK) != RADIO_STATE_DISABLED)
        {
          tim->tim_pending = true;
          tim->tim_now     = NRF54L_TIMER_CHAN_ACK;
          armed            = true;
        }
      else
        {
          putreg32(DPPIC_CHEN_CH(NRF54L_TIMER_LINK_ACKTXEN),
                   NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENCLR_OFFSET);
        }
    }

  leave_critical_section(flags);
  return armed;
}

/****************************************************************************
 * Name: nrf54l_radioi8_tim_ackdone
 *
 * Description:
 *   The ACK TX is done, the timer is idle again.
 *
 ****************************************************************************/

static void nrf54l_radioi8_tim_ackdone(struct nrf54l_radioi8_dev_s *dev)
{
  irqstate_t flags;

  flags = enter_critical_section();

  putreg32(DPPIC_CHEN_CH(NRF54L_TIMER_LINK_ACKTXEN),
           NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENCLR_OFFSET);

  if (dev->tim->tim_now == NRF54L_TIMER_CHAN_ACK)
    {
      dev->tim->tim_pending = false;
      dev->tim->tim_now     = -1;
      nrf54l_radioi8_tim_idle(dev, true);
    }

  leave_critical_section(flags);
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

  nrf54l_radioi8_tim_idle(dev, true);

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

  /* Reserve TIMER10, RADIO PHYEND reaches it over DPPIC10 */

  tim = nrf54l_tim_init(NRF54L_IEEE802154_TIMER10);
  if (tim == NULL)
    {
      wlerr("nrf54l_tim_init(6) failed %d\n", -errno);
      return NULL;
    }

  /* Attach TIMER interrupt */

  NRF54L_TIM_SETISR(tim, nrf54l_radioi8_isr_tim, dev);

  /* Connect timer */

  memset(&g_radioi8_tim, 0, sizeof(struct nrf54l_radioi8_tim_s));
  g_radioi8_tim.ops = &g_radioi8_tim_ops;
  g_radioi8_tim.tim = tim;

  return &g_radioi8_tim;
}
