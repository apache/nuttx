/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_rtc.c
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

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "nrf52_ieee802154_rtc.h"
#include "nrf52_ieee802154_trace.h"

#include "nrf52_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF52_RTC0
#  error CONFIG_NRF52_RTC0 is needed to handle radio timings
#endif

/* RTC instance - 0 */

#define NRF52_IEEE802154_RTC0   (0)

/* RTC prescaler set to 0, freq = 32.768kHz, 1 tick = 15625 / 512 us */

#define NRF52_RTC_PRESCALER     (0)
#define NRF52_RTC_TICK_NUM      (15625)
#define NRF52_RTC_TICK_DEN      (512)

/* 24-bit counter */

#define NRF52_RTC_COUNTER_BITS  (24)
#define NRF52_RTC_COUNTER_MASK  ((1 << NRF52_RTC_COUNTER_BITS) - 1)

/* A compare value closer than 2 ticks to the counter may not match */

#define NRF52_RTC_CC_MARGIN     (2)
#define NRF52_RTC_CC_MARGIN_US  ((NRF52_RTC_CC_MARGIN * NRF52_RTC_TICK_NUM + \
                                  NRF52_RTC_TICK_DEN - 1) / NRF52_RTC_TICK_DEN)

/* Superframe events compare channel */

#define NRF52_RTC_CC            (NRF52_RTC_CC0)
#define NRF52_RTC_EVT           (NRF52_RTC_EVT_COMPARE0)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC ops */

static int nrf52_radioi8_rtc(struct nrf52_radioi8_dev_s *dev,
                             struct ieee802154_superframespec_s *sfspec);
static int nrf52_radioi8_rtc_start(struct nrf52_radioi8_dev_s *dev);
static int nrf52_radioi8_rtc_sync(struct nrf52_radioi8_dev_s *dev,
                                  uint8_t phr);
static int nrf52_radioi8_rtc_stop(struct nrf52_radioi8_dev_s *dev);
static void nrf52_radioi8_rtc_reset(struct nrf52_radioi8_dev_s *dev);
static bool nrf52_radioi8_rtc_elapsed(struct nrf52_radioi8_dev_s *dev,
                                      uint32_t *us);

/* Interrupts logic */

static void nrf52_radioi8_work_inactive(void *arg);
static int nrf52_radioi8_isr_rtc(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RTC ops */

static struct nrf52_radioi8_rtc_ops_s g_radioi8_rtc_ops =
{
  .setup   = nrf52_radioi8_rtc,
  .start   = nrf52_radioi8_rtc_start,
  .sync    = nrf52_radioi8_rtc_sync,
  .stop    = nrf52_radioi8_rtc_stop,
  .reset   = nrf52_radioi8_rtc_reset,
  .elapsed = nrf52_radioi8_rtc_elapsed
};

/* RTC instance */

static struct nrf52_radioi8_rtc_s g_radioi8_rtc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_rtc_ticks
 *
 * Description:
 *   Get the 64-bit RTC counter.  Must be called with the radio locked.
 *
 ****************************************************************************/

static uint64_t nrf52_radioi8_rtc_ticks(struct nrf52_radioi8_rtc_s *rtc)
{
  uint32_t ovf = rtc->ovf;
  uint32_t cnt = 0;

  NRF52_RTC_GETCOUNTER(rtc->rtc, &cnt);

  /* Overflow not handled yet */

  if (NRF52_RTC_CHECKINT(rtc->rtc, NRF52_RTC_EVT_OVRFLW))
    {
      NRF52_RTC_GETCOUNTER(rtc->rtc, &cnt);
      ovf += 1;
    }

  return ((uint64_t)ovf << NRF52_RTC_COUNTER_BITS) | cnt;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_now
 *
 * Description:
 *   Get the current time in us.  Must be called with the radio locked.
 *
 ****************************************************************************/

static uint64_t nrf52_radioi8_rtc_now(struct nrf52_radioi8_rtc_s *rtc)
{
  return nrf52_radioi8_rtc_ticks(rtc) * NRF52_RTC_TICK_NUM /
         NRF52_RTC_TICK_DEN;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_alarm
 *
 * Description:
 *   Schedule a superframe event.  The compare value matches on every
 *   counter wrap, the interrupt handler ignores the matches before the
 *   event time.  Must be called with the radio locked.
 *
 ****************************************************************************/

static void nrf52_radioi8_rtc_alarm(struct nrf52_radioi8_rtc_s *rtc,
                                    uint8_t evt, uint64_t time)
{
  uint64_t ticks = time * NRF52_RTC_TICK_DEN / NRF52_RTC_TICK_NUM;
  uint64_t now   = nrf52_radioi8_rtc_ticks(rtc);

  if (ticks < now + NRF52_RTC_CC_MARGIN)
    {
      ticks = now + NRF52_RTC_CC_MARGIN;
    }

  rtc->evt      = evt;
  rtc->evt_time = time;

  NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_EVT);
  NRF52_RTC_SETCC(rtc->rtc, NRF52_RTC_CC, ticks & NRF52_RTC_COUNTER_MASK);
  NRF52_RTC_ENABLEEVT(rtc->rtc, NRF52_RTC_EVT);
  NRF52_RTC_ENABLEINT(rtc->rtc, NRF52_RTC_EVT);
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_cancel
 *
 * Description:
 *   Cancel the scheduled superframe event.  Must be called with the radio
 *   locked.
 *
 ****************************************************************************/

static void nrf52_radioi8_rtc_cancel(struct nrf52_radioi8_rtc_s *rtc)
{
  NRF52_RTC_DISABLEINT(rtc->rtc, NRF52_RTC_EVT);
  NRF52_RTC_DISABLEEVT(rtc->rtc, NRF52_RTC_EVT);
  NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_EVT);

  rtc->evt = NRF52_SFEVT_NONE;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_next
 *
 * Description:
 *   Schedule the next superframe event after the superframe start.  Must
 *   be called with the radio locked.
 *
 ****************************************************************************/

static void nrf52_radioi8_rtc_next(struct nrf52_radioi8_dev_s *dev)
{
  struct nrf52_radioi8_rtc_s *rtc = dev->rtc;

  if (rtc->sd < rtc->bi)
    {
      nrf52_radioi8_rtc_alarm(rtc, NRF52_SFEVT_SDEND,
                              rtc->sf_start + rtc->sd);
    }
  else
    {
      /* No inactive portion, the MAC updates the beacon from now */

      if (work_available(&dev->rtc->inactive_work))
        {
          work_queue(HPWORK, &dev->rtc->inactive_work,
                     nrf52_radioi8_work_inactive, dev, 0);
        }

      nrf52_radioi8_rtc_alarm(rtc, NRF52_SFEVT_START,
                              rtc->sf_start + rtc->bi -
                              NRF52_RADIOI8_RAMPUP_US);
    }
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc
 *
 * Description:
 *   Configure the superframe timings according to superframe spec.
 *
 ****************************************************************************/

static int nrf52_radioi8_rtc(struct nrf52_radioi8_dev_s *dev,
                             struct ieee802154_superframespec_s *sfspec)
{
  struct nrf52_radioi8_rtc_s *rtc = dev->rtc;
  uint32_t                    slot;
  irqstate_t                  flags;

  if (sfspec->beaconorder > 14 ||
      (sfspec->sforder > sfspec->beaconorder && sfspec->sforder != 15))
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  /* BI = aBaseSuperframeDuration * 2^BO, SD = aBaseSuperframeDuration *
   * 2^SO, the CAP ends with the final CAP slot (IEEE 802.15.4-2003 7.5.1.1)
   */

  rtc->bi = (IEEE802154_BASE_SUPERFRAME_DURATION * IEEE802154_SYMBOL_US) <<
            sfspec->beaconorder;

  if (sfspec->sforder == 15)
    {
      /* The superframe is not active after the beacon */

      rtc->sd  = 0;
      rtc->cap = 0;
    }
  else
    {
      slot     = (IEEE802154_BASE_SLOT_DURATION * IEEE802154_SYMBOL_US) <<
                 sfspec->sforder;
      rtc->sd  = slot * IEEE802154_NUM_SUPERFRAME_SLOTS;
      rtc->cap = slot * (sfspec->final_capslot + 1);
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_start
 *
 * Description:
 *   Start transmitting beacons.
 *
 ****************************************************************************/

static int nrf52_radioi8_rtc_start(struct nrf52_radioi8_dev_s *dev)
{
  struct nrf52_radioi8_rtc_s *rtc = dev->rtc;
  irqstate_t                  flags;

  flags = enter_critical_section();

  rtc->sf_start = nrf52_radioi8_rtc_now(rtc) + NRF52_RTC_CC_MARGIN_US +
                  NRF52_RADIOI8_RAMPUP_US;
  rtc->running  = false;

  nrf52_radioi8_rtc_alarm(rtc, NRF52_SFEVT_START,
                          rtc->sf_start - NRF52_RADIOI8_RAMPUP_US);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_sync
 *
 * Description:
 *   Synchronize with the beacon just received, the superframe started
 *   with its SHR.
 *
 ****************************************************************************/

static int nrf52_radioi8_rtc_sync(struct nrf52_radioi8_dev_s *dev,
                                  uint8_t phr)
{
  struct nrf52_radioi8_rtc_s *rtc = dev->rtc;
  irqstate_t                  flags;

  flags = enter_critical_section();

  nrf52_radioi8_trace_put(RADIO_TRACE_SFSYNC, phr);

  rtc->sf_start = nrf52_radioi8_rtc_now(rtc) - NRF52_RADIOI8_FRAME_US(phr);
  rtc->capstart = NRF52_RADIOI8_FRAME_US(phr) + NRF52_RADIOI8_IFS_US(phr);
  rtc->running  = true;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_stop
 *
 * Description:
 *   Stop the superframe.
 *
 ****************************************************************************/

static int nrf52_radioi8_rtc_stop(struct nrf52_radioi8_dev_s *dev)
{
  irqstate_t flags;

  flags = enter_critical_section();

  nrf52_radioi8_rtc_cancel(dev->rtc);
  dev->rtc->running = false;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_reset
 *
 * Description:
 *   Reset RTC, the counter runs from now on.
 *
 ****************************************************************************/

static void nrf52_radioi8_rtc_reset(struct nrf52_radioi8_dev_s *dev)
{
  struct nrf52_radioi8_rtc_s *rtc = dev->rtc;
  irqstate_t                  flags;

  flags = enter_critical_section();

  /* Configure RTC - freq = 32.768 kHz */

  nrf52_radioi8_rtc_cancel(rtc);
  NRF52_RTC_STOP(rtc->rtc);
  NRF52_RTC_CLEAR(rtc->rtc);
  NRF52_RTC_SETPRE(rtc->rtc, NRF52_RTC_PRESCALER);

  /* Count overflows */

  NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_EVT_OVRFLW);
  NRF52_RTC_ENABLEEVT(rtc->rtc, NRF52_RTC_EVT_OVRFLW);
  NRF52_RTC_ENABLEINT(rtc->rtc, NRF52_RTC_EVT_OVRFLW);

  /* Reset data */

  rtc->ovf      = 0;
  rtc->sf_start = 0;
  rtc->running  = false;

  NRF52_RTC_START(rtc->rtc);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_elapsed
 *
 * Description:
 *   Time in us since the superframe start.
 *
 ****************************************************************************/

static bool nrf52_radioi8_rtc_elapsed(struct nrf52_radioi8_dev_s *dev,
                                      uint32_t *us)
{
  struct nrf52_radioi8_rtc_s *rtc = dev->rtc;
  uint64_t                    now;
  irqstate_t                  flags;
  bool                        ret = false;

  flags = enter_critical_section();

  if (rtc->running)
    {
      now = nrf52_radioi8_rtc_now(rtc);
      if (now < rtc->sf_start)
        {
          *us = 0;
        }
      else if (now - rtc->sf_start > UINT32_MAX)
        {
          *us = UINT32_MAX;
        }
      else
        {
          *us = now - rtc->sf_start;
        }

      ret = true;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: nrf52_radioi8_work_inactive
 *
 * Description:
 *   Work when we enter inactive state.
 *
 ****************************************************************************/

static void nrf52_radioi8_work_inactive(void *arg)
{
  struct nrf52_radioi8_dev_s *dev = (struct nrf52_radioi8_dev_s *)arg;
  enum ieee802154_sfevent_e   sfevent;

  /* Notify MAC */

  sfevent = IEEE802154_SFEVENT_ENDOFACTIVE;
  dev->radiocb->sfevent(dev->radiocb, sfevent);
}

/****************************************************************************
 * Name: nrf52_radioi8_isr_rtc
 *
 * Description:
 *   Helper RTC0 interrupt handler.
 *
 ****************************************************************************/

static int nrf52_radioi8_isr_rtc(int irq, void *context, void *arg)
{
  struct nrf52_radioi8_dev_s *dev = (struct nrf52_radioi8_dev_s *)arg;
  struct nrf52_radioi8_rtc_s *rtc = NULL;
  irqstate_t                  flags;
  uint8_t                     evt;

  DEBUGASSERT(dev != NULL);
  rtc = dev->rtc;

  flags = enter_critical_section();

  if (NRF52_RTC_CHECKINT(rtc->rtc, NRF52_RTC_EVT_OVRFLW))
    {
      NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_EVT_OVRFLW);
      rtc->ovf += 1;
    }

  if (NRF52_RTC_CHECKINT(rtc->rtc, NRF52_RTC_EVT))
    {
      NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_EVT);

      /* A match before the event time is a counter wrap */

      evt = rtc->evt;
      if (evt == NRF52_SFEVT_NONE ||
          nrf52_radioi8_rtc_ticks(rtc) <
          rtc->evt_time * NRF52_RTC_TICK_DEN / NRF52_RTC_TICK_NUM)
        {
          goto out;
        }

      rtc->evt = NRF52_SFEVT_NONE;

      /* Superframe start */

      if (evt == NRF52_SFEVT_START)
        {
          nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_RTCBI, 0);

          rtc->sf_start = rtc->evt_time + NRF52_RADIOI8_RAMPUP_US;
          rtc->capstart = NRF52_RADIOI8_FRAME_US(dev->radio->beaconbuf[0]) +
                          NRF52_RADIOI8_IFS_US(dev->radio->beaconbuf[0]);
          rtc->running  = true;

          /* Transmit data from beaconbuf */

          dev->radio->ops->beacon_tx(dev);

          nrf52_radioi8_rtc_next(dev);
        }

      /* End of active portion */

      else if (evt == NRF52_SFEVT_SDEND)
        {
          nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_RTCSD, 0);

          if (work_available(&dev->rtc->inactive_work))
            {
              work_queue(HPWORK, &dev->rtc->inactive_work,
                         nrf52_radioi8_work_inactive, dev, 0);
            }

          nrf52_radioi8_rtc_alarm(rtc, NRF52_SFEVT_START,
                                  rtc->sf_start + rtc->bi -
                                  NRF52_RADIOI8_RAMPUP_US);
        }
    }

out:
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_rtc_init
 *
 * Description:
 *   Initialize low resolution, low power timer for IEEE802154 operations.
 *   Used to handle superframe timings.
 *
 ****************************************************************************/

struct nrf52_radioi8_rtc_s *
nrf52_radioi8_rtc_init(struct nrf52_radioi8_dev_s *dev)
{
  struct nrf52_rtc_dev_s *rtc = NULL;

  /* Reserve RTC0 */

  rtc = nrf52_rtc_init(NRF52_IEEE802154_RTC0);
  if (rtc == NULL)
    {
      wlerr("nrf52_rtc_init(0) failed %d\n", -errno);
      return NULL;
    }

  /* Attach RTC interrupt */

  NRF52_RTC_SETISR(rtc, nrf52_radioi8_isr_rtc, dev);

  /* Connect RTC */

  memset(&g_radioi8_rtc, 0, sizeof(struct nrf52_radioi8_rtc_s));
  g_radioi8_rtc.ops = &g_radioi8_rtc_ops;
  g_radioi8_rtc.rtc = rtc;

  return &g_radioi8_rtc;
}
