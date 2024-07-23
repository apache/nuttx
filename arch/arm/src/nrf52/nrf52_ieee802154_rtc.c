/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_rtc.c
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

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "nrf52_ieee802154_rtc.h"
#include "nrf52_ieee802154_trace.h"

#include "nrf52_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NRF52_RADIO_IEEE802154_SUPERFRAME
#  warning Beacon and superframe support work in progress
#endif

#ifndef CONFIG_NRF52_RTC0
#  error CONFIG_NRF52_RTC0 is needed to handle radio timings
#endif

/* RTC instance - 0 */

#define NRF52_IEEE802154_RTC0   (0)

/* RTC prescaler set to 0, freq = 32.768kHz, resolution = 30.517 us */

#define NRF52_RTC_PRESCALER     (0)
#define NRF52_RTC_FREQUENCY     (32768)

#define NRF52_RTC_RESOLUTION_NS (30517)

/* 31 * 30.517us = 946.027us */

#define NRF52_RTC_TIMESLOT_CC   (31)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC ops */

static int nrf52_radioi8_rtc(struct nrf52_radioi8_dev_s *dev,
                             struct ieee802154_superframespec_s *sfspec);
static int nrf52_radioi8_rtc_start(struct nrf52_radioi8_dev_s *dev);
static int nrf52_radioi8_rtc_stop(struct nrf52_radioi8_dev_s *dev);
static void nrf52_radioi8_rtc_reset(struct nrf52_radioi8_dev_s *dev);

/* Interrupts logic */

static void nrf52_radioi8_work_inactive(void *arg);
static int nrf52_radioi8_isr_rtc(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RTC ops */

static struct nrf52_radioi8_rtc_ops_s g_radioi8_rtc_ops =
{
  .setup = nrf52_radioi8_rtc,
  .start = nrf52_radioi8_rtc_start,
  .stop  = nrf52_radioi8_rtc_stop,
  .reset = nrf52_radioi8_rtc_reset
};

/* RTC instance */

static struct nrf52_radioi8_rtc_s g_radioi8_rtc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_rtc
 *
 * Description:
 *   Configure RTC events according to superframe spec.
 *
 ****************************************************************************/

static int nrf52_radioi8_rtc(struct nrf52_radioi8_dev_s *dev,
                             struct ieee802154_superframespec_s *sfspec)
{
  uint32_t   bi     = 0;
  uint32_t   sd     = 0;
  uint32_t   cap    = 0;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Stop RTC and clear the counter */

  NRF52_RTC_STOP(dev->rtc->rtc);
  NRF52_RTC_CLEAR(dev->rtc->rtc);

  /* Initialize BI counter */

  bi = ((IEEE802154_BASE_SUPERFRAME_DURATION *
        (2 << sfspec->beaconorder) *
         IEEE802154_SYMBOL_US * 1000)  /
        NRF52_RTC_RESOLUTION_NS);
  NRF52_RTC_SETCC(dev->rtc->rtc, NRF52_RTC_BI, bi);

  /* Initialize SD counter */

  sd = ((IEEE802154_BASE_SUPERFRAME_DURATION *
         (2 << sfspec->sforder) *
         IEEE802154_SYMBOL_US * 1000) /
        NRF52_RTC_RESOLUTION_NS);
  NRF52_RTC_SETCC(dev->rtc->rtc, NRF52_RTC_SD, sd);

  /* Initialize CAP counter */

  cap = ((sfspec->final_capslot *
          IEEE802154_TIMESLOT_US * 1000) /
         NRF52_RTC_RESOLUTION_NS);
  NRF52_RTC_SETCC(dev->rtc->rtc, NRF52_RTC_CAP, cap);

  /* Initialize timeslot counter */

  dev->rtc->rtc_timeslot = NRF52_RTC_TIMESLOT_CC;
  NRF52_RTC_SETCC(dev->rtc->rtc, NRF52_RTC_TIMESLOT, dev->rtc->rtc_timeslot);

  /* Configure interupts */

  NRF52_RTC_ENABLEINT(dev->rtc->rtc, NRF52_RTC_BI);
  NRF52_RTC_ENABLEINT(dev->rtc->rtc, NRF52_RTC_SD);
  NRF52_RTC_ENABLEINT(dev->rtc->rtc, NRF52_RTC_CAP);
  NRF52_RTC_ENABLEINT(dev->rtc->rtc, NRF52_RTC_TIMESLOT);

  /* Configure events */

  NRF52_RTC_ENABLEEVT(dev->rtc->rtc, NRF52_RTC_BI);
  NRF52_RTC_ENABLEEVT(dev->rtc->rtc, NRF52_RTC_SD);
  NRF52_RTC_ENABLEEVT(dev->rtc->rtc, NRF52_RTC_CAP);
  NRF52_RTC_ENABLEEVT(dev->rtc->rtc, NRF52_RTC_TIMESLOT);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_start
 *
 * Description:
 *   Start RTC.
 *
 ****************************************************************************/

static int nrf52_radioi8_rtc_start(struct nrf52_radioi8_dev_s *dev)
{
  NRF52_RTC_START(dev->rtc->rtc);
  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_stop
 *
 * Description:
 *   Stop RTC.
 *
 ****************************************************************************/

static int nrf52_radioi8_rtc_stop(struct nrf52_radioi8_dev_s *dev)
{
  NRF52_RTC_STOP(dev->rtc->rtc);
  return OK;
}

/****************************************************************************
 * Name: nrf52_radioi8_rtc_reset
 *
 * Description:
 *   Reset RTC.
 *
 ****************************************************************************/

static void nrf52_radioi8_rtc_reset(struct nrf52_radioi8_dev_s *dev)
{
  /* Configure RTC - freq = 32.768 kHz */

  NRF52_RTC_STOP(dev->rtc->rtc);
  NRF52_RTC_SETPRE(dev->rtc->rtc, NRF52_RTC_PRESCALER);

  /* Reset data */

  dev->rtc->rtc_timeslot = 0;
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

  DEBUGASSERT(dev != NULL);
  rtc = dev->rtc;

  flags = enter_critical_section();

  /* End of CAP */

  if (NRF52_RTC_CHECKINT(rtc->rtc, NRF52_RTC_CAP))
    {
      nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_RTCCAP, 0);

      /* TODO */

      /* Clear event */

      NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_CAP);
    }

  /* End of active portion */

  else if (NRF52_RTC_CHECKINT(rtc->rtc, NRF52_RTC_SD))
    {
      nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_RTCSD, 0);

      /* Schedule work */

      DEBUGASSERT(work_available(&dev->rtc->inactive_work));
      work_queue(HPWORK, &dev->rtc->inactive_work,
                 nrf52_radioi8_work_inactive, dev, 0);

      /* Reset timeslot */

      rtc->rtc_timeslot = NRF52_RTC_TIMESLOT_CC;

      /* Clear event */

      NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_SD);
    }

  /* Beacon interval */

  else if (NRF52_RTC_CHECKINT(rtc->rtc, NRF52_RTC_BI))
    {
      nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_RTCBI, 0);

      /* Transmit beacon if we are not endpoint */

      if (dev->state.devmode != NRF52_DEVMODE_ENDPOINT)
        {
          /* Transmit data from beaconbuf */

          dev->radio->ops->beacon_tx(dev);
        }

      /* Clear event */

      NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_BI);

      /* Next cycle */

      NRF52_RTC_TRGOVRFLW(rtc->rtc);

      /* TODO: resync with beacon ? */
    }

  /* Timeslot in active portion */

  if (NRF52_RTC_CHECKINT(rtc->rtc, NRF52_RTC_TIMESLOT))
    {
      nrf52_radioi8_trace_put(RADIO_TRACE_IRQ_RTCTIMESLOT, 0);

      /* TODO: how to sync transmition with timeslot ?
       *       do we need count every timeslot here ?
       *       or wait for the timeslot we are interested in ?
       *       or just use txdelay ?
       */

      /* Update timeslot */

      rtc->rtc_timeslot += NRF52_RTC_TIMESLOT_CC;
      NRF52_RTC_SETCC(rtc->rtc, NRF52_RTC_TIMESLOT, rtc->rtc_timeslot);

      /* Clear event */

      NRF52_RTC_ACKINT(rtc->rtc, NRF52_RTC_TIMESLOT);
    }

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
 *   Initialize low resoluton, low power timer for IEEE802154 operations.
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

  /* Atach RTC interrupt */

  NRF52_RTC_SETISR(rtc, nrf52_radioi8_isr_rtc, dev);

  /* Set interrupts priority */

  up_prioritize_irq(NRF52_IRQ_RTC0, 0);

  /* Connect RTC */

  memset(&g_radioi8_rtc, 0, sizeof(struct nrf52_radioi8_rtc_s));
  g_radioi8_rtc.ops = &g_radioi8_rtc_ops;
  g_radioi8_rtc.rtc = rtc;

  return &g_radioi8_rtc;
}
