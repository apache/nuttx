/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ieee802154_rtc.c
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

#include "nrf54l_ieee802154_rtc.h"
#include "nrf54l_ieee802154_trace.h"

#include "nrf54l_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF54L_GRTC
#  error CONFIG_NRF54L_GRTC is needed to handle radio timings
#endif

/* GRTC interrupt group of the radio, the system timer uses group 0 */

#define NRF54L_IEEE802154_GRTC  (1)

/* The SYSCOUNTER counts us, a compare value closer than this to the
 * counter may not match
 */

#define NRF54L_GRTC_CC_MARGIN   (10)

/* Superframe events compare channel, the system timer uses CC0 */

#define NRF54L_GRTC_CC          (NRF54L_GRTC_CC1)
#define NRF54L_GRTC_EVT         (NRF54L_GRTC_EVT_COMPARE1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC ops */

static int nrf54l_radioi8_rtc(struct nrf54l_radioi8_dev_s *dev,
                              struct ieee802154_superframespec_s *sfspec);
static int nrf54l_radioi8_rtc_start(struct nrf54l_radioi8_dev_s *dev);
static int nrf54l_radioi8_rtc_sync(struct nrf54l_radioi8_dev_s *dev,
                                   uint8_t phr);
static int nrf54l_radioi8_rtc_stop(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_rtc_reset(struct nrf54l_radioi8_dev_s *dev);
static bool nrf54l_radioi8_rtc_elapsed(struct nrf54l_radioi8_dev_s *dev,
                                       uint32_t *us);

/* Interrupts logic */

static void nrf54l_radioi8_work_inactive(void *arg);
static int nrf54l_radioi8_isr_rtc(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RTC ops */

static struct nrf54l_radioi8_rtc_ops_s g_radioi8_rtc_ops =
{
  .setup   = nrf54l_radioi8_rtc,
  .start   = nrf54l_radioi8_rtc_start,
  .sync    = nrf54l_radioi8_rtc_sync,
  .stop    = nrf54l_radioi8_rtc_stop,
  .reset   = nrf54l_radioi8_rtc_reset,
  .elapsed = nrf54l_radioi8_rtc_elapsed
};

/* RTC instance */

static struct nrf54l_radioi8_rtc_s g_radioi8_rtc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_now
 *
 * Description:
 *   Get the current time in us.
 *
 ****************************************************************************/

static uint64_t nrf54l_radioi8_rtc_now(struct nrf54l_radioi8_rtc_s *rtc)
{
  uint64_t now = 0;

  NRF54L_GRTC_GETCOUNTER(rtc->rtc, &now);

  return now;
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_alarm
 *
 * Description:
 *   Schedule a superframe event.  Must be called with the radio locked.
 *
 ****************************************************************************/

static void nrf54l_radioi8_rtc_alarm(struct nrf54l_radioi8_rtc_s *rtc,
                                     uint8_t evt, uint64_t time)
{
  uint64_t target = time;
  uint64_t now;

  rtc->evt      = evt;
  rtc->evt_time = time;

  NRF54L_GRTC_ACKINT(rtc->rtc, NRF54L_GRTC_EVT);

  /* Move a due compare value ahead of the counter, repeat until it is
   * ahead or the event fired
   */

  for (; ; )
    {
      now = nrf54l_radioi8_rtc_now(rtc);
      if (target <= now + NRF54L_GRTC_CC_MARGIN)
        {
          target = now + NRF54L_GRTC_CC_MARGIN;
        }

      NRF54L_GRTC_SETCC(rtc->rtc, NRF54L_GRTC_CC, target);

      if (target > nrf54l_radioi8_rtc_now(rtc) ||
          NRF54L_GRTC_CHECKINT(rtc->rtc, NRF54L_GRTC_EVT))
        {
          break;
        }
    }

  NRF54L_GRTC_ENABLEINT(rtc->rtc, NRF54L_GRTC_EVT);
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_cancel
 *
 * Description:
 *   Cancel the scheduled superframe event.  Must be called with the radio
 *   locked.
 *
 ****************************************************************************/

static void nrf54l_radioi8_rtc_cancel(struct nrf54l_radioi8_rtc_s *rtc)
{
  NRF54L_GRTC_DISABLEINT(rtc->rtc, NRF54L_GRTC_EVT);
  NRF54L_GRTC_DISABLECC(rtc->rtc, NRF54L_GRTC_CC);
  NRF54L_GRTC_ACKINT(rtc->rtc, NRF54L_GRTC_EVT);

  rtc->evt = NRF54L_SFEVT_NONE;
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_next
 *
 * Description:
 *   Schedule the next superframe event after the superframe start.  Must
 *   be called with the radio locked.
 *
 ****************************************************************************/

static void nrf54l_radioi8_rtc_next(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radioi8_rtc_s *rtc = dev->rtc;

  if (rtc->sd < rtc->bi)
    {
      nrf54l_radioi8_rtc_alarm(rtc, NRF54L_SFEVT_SDEND,
                               rtc->sf_start + rtc->sd);
    }
  else
    {
      /* No inactive portion, the MAC updates the beacon from now */

      if (work_available(&dev->rtc->inactive_work))
        {
          work_queue(HPWORK, &dev->rtc->inactive_work,
                     nrf54l_radioi8_work_inactive, dev, 0);
        }

      nrf54l_radioi8_rtc_alarm(rtc, NRF54L_SFEVT_START,
                               rtc->sf_start + rtc->bi -
                               NRF54L_RADIOI8_RAMPUP_US);
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc
 *
 * Description:
 *   Configure the superframe timings according to superframe spec.
 *
 ****************************************************************************/

static int nrf54l_radioi8_rtc(struct nrf54l_radioi8_dev_s *dev,
                              struct ieee802154_superframespec_s *sfspec)
{
  struct nrf54l_radioi8_rtc_s *rtc = dev->rtc;
  uint32_t                     slot;
  irqstate_t                   flags;

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
 * Name: nrf54l_radioi8_rtc_start
 *
 * Description:
 *   Start transmitting beacons.
 *
 ****************************************************************************/

static int nrf54l_radioi8_rtc_start(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radioi8_rtc_s *rtc = dev->rtc;
  irqstate_t                   flags;

  flags = enter_critical_section();

  rtc->sf_start = nrf54l_radioi8_rtc_now(rtc) + NRF54L_GRTC_CC_MARGIN +
                  NRF54L_RADIOI8_RAMPUP_US;
  rtc->running  = false;

  nrf54l_radioi8_rtc_alarm(rtc, NRF54L_SFEVT_START,
                           rtc->sf_start - NRF54L_RADIOI8_RAMPUP_US);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_sync
 *
 * Description:
 *   Synchronize with the beacon just received, the superframe started
 *   with its SHR.
 *
 ****************************************************************************/

static int nrf54l_radioi8_rtc_sync(struct nrf54l_radioi8_dev_s *dev,
                                   uint8_t phr)
{
  struct nrf54l_radioi8_rtc_s *rtc = dev->rtc;
  irqstate_t                   flags;

  flags = enter_critical_section();

  nrf54l_radioi8_trace_put(RADIO_TRACE_SFSYNC, phr);

  rtc->sf_start = nrf54l_radioi8_rtc_now(rtc) -
                  NRF54L_RADIOI8_FRAME_US(phr);
  rtc->capstart = NRF54L_RADIOI8_FRAME_US(phr) +
                  NRF54L_RADIOI8_IFS_US(phr);
  rtc->running  = true;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_stop
 *
 * Description:
 *   Stop the superframe.
 *
 ****************************************************************************/

static int nrf54l_radioi8_rtc_stop(struct nrf54l_radioi8_dev_s *dev)
{
  irqstate_t flags;

  flags = enter_critical_section();

  nrf54l_radioi8_rtc_cancel(dev->rtc);
  dev->rtc->running = false;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_reset
 *
 * Description:
 *   Reset the superframe, the SYSCOUNTER is shared and keeps running.
 *
 ****************************************************************************/

static void nrf54l_radioi8_rtc_reset(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radioi8_rtc_s *rtc = dev->rtc;
  irqstate_t                   flags;

  flags = enter_critical_section();

  nrf54l_radioi8_rtc_cancel(rtc);

  /* Reset data */

  rtc->sf_start = 0;
  rtc->running  = false;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_radioi8_rtc_elapsed
 *
 * Description:
 *   Time in us since the superframe start.
 *
 ****************************************************************************/

static bool nrf54l_radioi8_rtc_elapsed(struct nrf54l_radioi8_dev_s *dev,
                                       uint32_t *us)
{
  struct nrf54l_radioi8_rtc_s *rtc = dev->rtc;
  uint64_t                     now;
  irqstate_t                   flags;
  bool                         ret = false;

  flags = enter_critical_section();

  if (rtc->running)
    {
      now = nrf54l_radioi8_rtc_now(rtc);
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
 * Name: nrf54l_radioi8_work_inactive
 *
 * Description:
 *   Work when we enter inactive state.
 *
 ****************************************************************************/

static void nrf54l_radioi8_work_inactive(void *arg)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)arg;
  enum ieee802154_sfevent_e    sfevent;

  /* Notify MAC */

  sfevent = IEEE802154_SFEVENT_ENDOFACTIVE;
  dev->radiocb->sfevent(dev->radiocb, sfevent);
}

/****************************************************************************
 * Name: nrf54l_radioi8_isr_rtc
 *
 * Description:
 *   Helper GRTC interrupt handler.
 *
 ****************************************************************************/

static int nrf54l_radioi8_isr_rtc(int irq, void *context, void *arg)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)arg;
  struct nrf54l_radioi8_rtc_s *rtc = NULL;
  irqstate_t                   flags;
  uint8_t                      evt;

  DEBUGASSERT(dev != NULL);
  rtc = dev->rtc;

  flags = enter_critical_section();

  if (NRF54L_GRTC_CHECKINT(rtc->rtc, NRF54L_GRTC_EVT))
    {
      NRF54L_GRTC_ACKINT(rtc->rtc, NRF54L_GRTC_EVT);

      evt = rtc->evt;
      if (evt == NRF54L_SFEVT_NONE)
        {
          goto out;
        }

      rtc->evt = NRF54L_SFEVT_NONE;

      /* Superframe start */

      if (evt == NRF54L_SFEVT_START)
        {
          nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_RTCBI, 0);

          rtc->sf_start = rtc->evt_time + NRF54L_RADIOI8_RAMPUP_US;
          rtc->capstart = NRF54L_RADIOI8_FRAME_US(dev->radio->beaconbuf[0]) +
                          NRF54L_RADIOI8_IFS_US(dev->radio->beaconbuf[0]);
          rtc->running  = true;

          /* Transmit data from beaconbuf */

          dev->radio->ops->beacon_tx(dev);

          nrf54l_radioi8_rtc_next(dev);
        }

      /* End of active portion */

      else if (evt == NRF54L_SFEVT_SDEND)
        {
          nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_RTCSD, 0);

          if (work_available(&dev->rtc->inactive_work))
            {
              work_queue(HPWORK, &dev->rtc->inactive_work,
                         nrf54l_radioi8_work_inactive, dev, 0);
            }

          nrf54l_radioi8_rtc_alarm(rtc, NRF54L_SFEVT_START,
                                   rtc->sf_start + rtc->bi -
                                   NRF54L_RADIOI8_RAMPUP_US);
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
 * Name: nrf54l_radioi8_rtc_init
 *
 * Description:
 *   Initialize the GRTC for IEEE802154 operations.
 *   Used to handle superframe timings.
 *
 ****************************************************************************/

struct nrf54l_radioi8_rtc_s *
nrf54l_radioi8_rtc_init(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_grtc_dev_s *grtc = NULL;

  /* Reserve the GRTC interrupt group */

  grtc = nrf54l_grtc_init(NRF54L_IEEE802154_GRTC);
  if (grtc == NULL)
    {
      wlerr("nrf54l_grtc_init(%d) failed\n", NRF54L_IEEE802154_GRTC);
      return NULL;
    }

  /* The GRTC is not reset by a system reset, stop the superframe event
   * left by the previous firmware before the interrupt is enabled.
   */

  NRF54L_GRTC_DISABLEINT(grtc, NRF54L_GRTC_EVT);
  NRF54L_GRTC_DISABLECC(grtc, NRF54L_GRTC_CC);
  NRF54L_GRTC_ACKINT(grtc, NRF54L_GRTC_EVT);

  /* Attach GRTC interrupt */

  NRF54L_GRTC_SETISR(grtc, nrf54l_radioi8_isr_rtc, dev);

  /* The SYSCOUNTER runs from now on, it is not cleared as the system
   * timer may use it
   */

  NRF54L_GRTC_START(grtc);

  /* Connect RTC */

  memset(&g_radioi8_rtc, 0, sizeof(struct nrf54l_radioi8_rtc_s));
  g_radioi8_rtc.ops = &g_radioi8_rtc_ops;
  g_radioi8_rtc.rtc = grtc;

  return &g_radioi8_rtc;
}
