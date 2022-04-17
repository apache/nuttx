/****************************************************************************
 * arch/arm/src/nrf52/nrf52_rtc.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/nrf52_rtc.h"

#include "nrf52_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_rtc_priv_s
{
  struct nrf52_rtc_ops_s *ops;
  uint32_t                base;
  uint32_t                irq;
  uint8_t                 chan;
  bool                    inuse;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RTC registers access *****************************************************/

static uint32_t nrf52_rtc_getreg(struct nrf52_rtc_dev_s *dev,
                                 uint32_t offset);
static void nrf52_rtc_putreg(struct nrf52_rtc_dev_s *dev,
                             uint32_t offset,
                             uint32_t value);

/* RTC helpers **************************************************************/

static uint32_t nrf52_rtc_irq2reg(struct nrf52_rtc_dev_s *dev,
                                  uint8_t s);
static uint32_t nrf52_rtc_evt2reg(struct nrf52_rtc_dev_s *dev,
                                  uint8_t evt);

/* RTC operations ***********************************************************/

static int nrf52_rtc_start(struct nrf52_rtc_dev_s *dev);
static int nrf52_rtc_stop(struct nrf52_rtc_dev_s *dev);
static int nrf52_rtc_clear(struct nrf52_rtc_dev_s *dev);
static int nrf52_rtc_trgovrflw(struct nrf52_rtc_dev_s *dev);
static int nrf52_rtc_getcounter(struct nrf52_rtc_dev_s *dev,
                                uint32_t *cc);
static int nrf52_rtc_setcc(struct nrf52_rtc_dev_s *dev, uint8_t i,
                           uint32_t cc);
static int nrf52_rtc_getcc(struct nrf52_rtc_dev_s *dev, uint8_t i,
                           uint32_t *cc);
static int nrf52_rtc_setpre(struct nrf52_rtc_dev_s *dev, uint16_t pre);
static int nrf52_rtc_setisr(struct nrf52_rtc_dev_s *dev, xcpt_t handler,
                            void * arg);
static int nrf52_rtc_enableint(struct nrf52_rtc_dev_s *dev, uint8_t s);
static int nrf52_rtc_disableint(struct nrf52_rtc_dev_s *dev, uint8_t s);
static int nrf52_rtc_checkint(struct nrf52_rtc_dev_s *dev, uint8_t s);
static int nrf52_rtc_ackint(struct nrf52_rtc_dev_s *dev, uint8_t s);
static int nrf52_rtc_enableevt(struct nrf52_rtc_dev_s *dev, uint8_t evt);
static int nrf52_rtc_disableevt(struct nrf52_rtc_dev_s *dev,
                                uint8_t evt);
static uint32_t nrf52_rtc_getbase(struct nrf52_rtc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NRF52 RTC ops */

struct nrf52_rtc_ops_s nrf52_rtc_ops =
{
  .start      = nrf52_rtc_start,
  .stop       = nrf52_rtc_stop,
  .clear      = nrf52_rtc_clear,
  .trgovrflw  = nrf52_rtc_trgovrflw,
  .getcounter = nrf52_rtc_getcounter,
  .setcc      = nrf52_rtc_setcc,
  .getcc      = nrf52_rtc_getcc,
  .setpre     = nrf52_rtc_setpre,
  .setisr     = nrf52_rtc_setisr,
  .enableint  = nrf52_rtc_enableint,
  .disableint = nrf52_rtc_disableint,
  .checkint   = nrf52_rtc_checkint,
  .ackint     = nrf52_rtc_ackint,
  .enableevt  = nrf52_rtc_enableevt,
  .disableevt = nrf52_rtc_disableevt,
  .getbase    = nrf52_rtc_getbase,
};

#ifdef CONFIG_NRF52_RTC0
/* RTC0 */

struct nrf52_rtc_priv_s g_nrf52_rtc0_priv =
{
  .ops   = &nrf52_rtc_ops,
  .base  = NRF52_RTC0_BASE,
  .irq   = NRF52_IRQ_RTC0,
  .chan  = 3,
  .inuse = false,
};
#endif

#ifdef CONFIG_NRF52_RTC1
/* RTC1 */

struct nrf52_rtc_priv_s g_nrf52_rtc1_priv =
{
  .ops   = &nrf52_rtc_ops,
  .base  = NRF52_RTC1_BASE,
  .irq   = NRF52_IRQ_RTC1,
  .chan  = 4,
  .inuse = false,
};
#endif

#ifdef CONFIG_NRF52_RTC2
/* RTC2 */

struct nrf52_rtc_priv_s g_nrf52_rtc2_priv =
{
  .ops   = &nrf52_rtc_ops,
  .base  = NRF52_RTC2_BASE,
  .irq   = NRF52_IRQ_RTC2,
  .chan  = 4,
  .inuse = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_rtc_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static uint32_t nrf52_rtc_getreg(struct nrf52_rtc_dev_s *dev,
                                 uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct nrf52_rtc_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf52_rtc_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static void nrf52_rtc_putreg(struct nrf52_rtc_dev_s *dev,
                             uint32_t offset,
                             uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct nrf52_rtc_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf52_rtc_irq2reg
 *
 * Description:
 *   Get the value of the interrupt register corresponding to the given
 *   interrupt source
 *
 ****************************************************************************/

static uint32_t nrf52_rtc_irq2reg(struct nrf52_rtc_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;

  switch (s)
    {
      case NRF52_RTC_EVT_TICK:
        {
          regval = RTC_INT_TICK;
          break;
        }

      case NRF52_RTC_EVT_OVRFLW:
        {
          regval = RTC_INT_OVRFLW;
          break;
        }

      case NRF52_RTC_EVT_COMPARE0:
        {
          regval = RTC_INT_COMPARE(0);
          break;
        }

      case NRF52_RTC_EVT_COMPARE1:
        {
          regval = RTC_INT_COMPARE(1);
          break;
        }

      case NRF52_RTC_EVT_COMPARE2:
        {
          regval = RTC_INT_COMPARE(2);
          break;
        }

      case NRF52_RTC_EVT_COMPARE3:
        {
          regval = RTC_INT_COMPARE(3);
          break;
        }

      default:
        {
          rtcerr("ERROR: unsupported IRQ source %d\n", s);
          regval = 0;
          goto errout;
        }
    }

errout:
  return regval;
}

/****************************************************************************
 * Name: nrf52_rtc_evt2reg
 *
 * Description:
 *   Get the offset of the event register corresponding to the given event
 *
 ****************************************************************************/

static uint32_t nrf52_rtc_evt2reg(struct nrf52_rtc_dev_s *dev,
                                  uint8_t evt)
{
  uint32_t regval;

  switch (evt)
    {
      case NRF52_RTC_EVT_TICK:
        {
          regval = RTC_EVTEN_TICK;
          break;
        }

      case NRF52_RTC_EVT_OVRFLW:
        {
          regval = RTC_EVTEN_OVRFLW;
          break;
        }

      case NRF52_RTC_EVT_COMPARE0:
        {
          regval = RTC_EVTEN_COMPARE(0);
          break;
        }

      case NRF52_RTC_EVT_COMPARE1:
        {
          regval = RTC_EVTEN_COMPARE(1);
          break;
        }

      case NRF52_RTC_EVT_COMPARE2:
        {
          regval = RTC_EVTEN_COMPARE(2);
          break;
        }

      case NRF52_RTC_EVT_COMPARE3:
        {
          regval = RTC_EVTEN_COMPARE(3);
          break;
        }

      default:
        {
          rtcerr("ERROR: unsupported EVENT %d\n", evt);
          regval = 0;
          goto errout;
        }
    }

errout:
  return regval;
}

/****************************************************************************
 * Name: nrf52_rtc_start
 ****************************************************************************/

static int nrf52_rtc_start(struct nrf52_rtc_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf52_rtc_putreg(dev, NRF52_RTC_TASKS_START_OFFSET, RTC_TASKS_START);

  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_stop
 ****************************************************************************/

static int nrf52_rtc_stop(struct nrf52_rtc_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf52_rtc_putreg(dev, NRF52_RTC_TASKS_STOP_OFFSET, RTC_TASKS_STOP);

  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_clear
 ****************************************************************************/

static int nrf52_rtc_clear(struct nrf52_rtc_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf52_rtc_putreg(dev, NRF52_RTC_TASKS_CLEAR_OFFSET, RTC_TASKS_CLEAR);

  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_trgovrflw
 ****************************************************************************/

static int nrf52_rtc_trgovrflw(struct nrf52_rtc_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf52_rtc_putreg(dev, NRF52_RTC_TASKS_TRIGOVRFLW_OFFSET,
                   RTC_TASKS_TRIGOVRFLW);

  return OK;
}

static int nrf52_rtc_getcounter(struct nrf52_rtc_dev_s *dev,
                                uint32_t *ctr)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(ctr);

  *ctr = nrf52_rtc_getreg(dev, NRF52_RTC_COUNTER_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_setcc
 ****************************************************************************/

static int nrf52_rtc_setcc(struct nrf52_rtc_dev_s *dev, uint8_t i,
                           uint32_t cc)
{
  struct nrf52_rtc_priv_s *rtc = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  rtc = (struct nrf52_rtc_priv_s *)dev;

  /* Is the channel supported? */

  if (i > rtc->chan)
    {
      rtcerr("ERROR: unsupported RTCER channel %d\n", i);
      ret = -EINVAL;
      goto errout;
    }

  nrf52_rtc_putreg(dev, NRF52_RTC_CC_OFFSET(i), cc);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_getcc
 ****************************************************************************/

static int nrf52_rtc_getcc(struct nrf52_rtc_dev_s *dev, uint8_t i,
                           uint32_t *cc)
{
  struct nrf52_rtc_priv_s *rtc = NULL;
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cc);

  rtc = (struct nrf52_rtc_priv_s *)dev;

  /* Is the channel supported? */

  if (i > rtc->chan)
    {
      rtcerr("ERROR: unsupported RTCER channel %d\n", i);
      ret = -EINVAL;
      goto errout;
    }

  *cc = nrf52_rtc_getreg(dev, NRF52_RTC_CC_OFFSET(i));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_setpre
 ****************************************************************************/

static int nrf52_rtc_setpre(struct nrf52_rtc_dev_s *dev, uint16_t pre)
{
  int ret = OK;

  DEBUGASSERT(dev);

  if (pre > RTC_PRESCALER_MAX)
    {
      rtcerr("ERROR: unsupported RTC prescaler %d\n", pre);
      ret = -EINVAL;
      goto errout;
    }

  nrf52_rtc_putreg(dev, NRF52_RTC_PRESCALER_OFFSET, pre);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_setisr
 ****************************************************************************/

static int nrf52_rtc_setisr(struct nrf52_rtc_dev_s *dev, xcpt_t handler,
                            void *arg)
{
  struct nrf52_rtc_priv_s *rtc = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  rtc = (struct nrf52_rtc_priv_s *)dev;

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(rtc->irq);
      irq_detach(rtc->irq);
      ret = OK;
      goto errout;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(rtc->irq, handler, arg);
  up_enable_irq(rtc->irq);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_enableint
 ****************************************************************************/

static int nrf52_rtc_enableint(struct nrf52_rtc_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given interrupt source */

  regval = nrf52_rtc_irq2reg(dev, s);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf52_rtc_putreg(dev, NRF52_RTC_INTENSET_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_disableint
 ****************************************************************************/

static int nrf52_rtc_disableint(struct nrf52_rtc_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given interrupt source */

  regval = nrf52_rtc_irq2reg(dev, s);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf52_rtc_putreg(dev, NRF52_RTC_INTENCLR_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_checkint
 ****************************************************************************/

static int nrf52_rtc_checkint(struct nrf52_rtc_dev_s *dev, uint8_t s)
{
  int ret = 0;

  DEBUGASSERT(dev);

  switch (s)
    {
      case NRF52_RTC_EVT_TICK:
        {
          ret = nrf52_rtc_getreg(dev, NRF52_RTC_EVENTS_TICK_OFFSET);
          break;
        }

      case NRF52_RTC_EVT_OVRFLW:
        {
          ret = nrf52_rtc_getreg(dev, NRF52_RTC_EVENTS_OVRFLW_OFFSET);
          break;
        }

      case NRF52_RTC_EVT_COMPARE0:
        {
          ret = nrf52_rtc_getreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(0));
          break;
        }

      case NRF52_RTC_EVT_COMPARE1:
        {
          ret = nrf52_rtc_getreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(1));
          break;
        }

      case NRF52_RTC_EVT_COMPARE2:
        {
          ret = nrf52_rtc_getreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(2));
          break;
        }

      case NRF52_RTC_EVT_COMPARE3:
        {
          ret = nrf52_rtc_getreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(3));
          break;
        }

      default:
        {
          rtcerr("ERROR: unsupported IRQ source %d\n", s);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_ackint
 ****************************************************************************/

static int nrf52_rtc_ackint(struct nrf52_rtc_dev_s *dev, uint8_t s)
{
  int ret = 0;

  DEBUGASSERT(dev);

  switch (s)
    {
      case NRF52_RTC_EVT_TICK:
        {
          nrf52_rtc_putreg(dev, NRF52_RTC_EVENTS_TICK_OFFSET, 0);
          break;
        }

      case NRF52_RTC_EVT_OVRFLW:
        {
          nrf52_rtc_putreg(dev, NRF52_RTC_EVENTS_OVRFLW_OFFSET, 0);
          break;
        }

      case NRF52_RTC_EVT_COMPARE0:
        {
          nrf52_rtc_putreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(0), 0);
          break;
        }

      case NRF52_RTC_EVT_COMPARE1:
        {
          nrf52_rtc_putreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(1), 0);
          break;
        }

      case NRF52_RTC_EVT_COMPARE2:
        {
          nrf52_rtc_putreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(2), 0);
          break;
        }

      case NRF52_RTC_EVT_COMPARE3:
        {
          nrf52_rtc_putreg(dev, NRF52_RTC_EVENTS_COMPARE_OFFSET(3), 0);
          break;
        }

      default:
        {
          rtcerr("ERROR: unsupported IRQ source %d\n", s);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_enableevt
 ****************************************************************************/

static int nrf52_rtc_enableevt(struct nrf52_rtc_dev_s *dev, uint8_t evt)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given event */

  regval = nrf52_rtc_evt2reg(dev, evt);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf52_rtc_putreg(dev, NRF52_RTC_EVTENSET_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_disableevt
 ****************************************************************************/

static int nrf52_rtc_disableevt(struct nrf52_rtc_dev_s *dev, uint8_t evt)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given event */

  regval = nrf52_rtc_evt2reg(dev, evt);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf52_rtc_putreg(dev, NRF52_RTC_EVTENCLR_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_getbase
 ****************************************************************************/

static uint32_t nrf52_rtc_getbase(struct nrf52_rtc_dev_s *dev)
{
  struct nrf52_rtc_priv_s *rtc = (struct nrf52_rtc_priv_s *)dev;
  DEBUGASSERT(dev);

  return rtc->base;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_rtc_init
 *
 * Description:
 *   Initialize RTC device
 *
 ****************************************************************************/

struct nrf52_rtc_dev_s *nrf52_rtc_init(int rtc)
{
  struct nrf52_rtc_priv_s *priv = NULL;

  /* Get RTC instance */

  switch (rtc)
    {
#ifdef CONFIG_NRF52_RTC0
      case 0:
        {
          priv = &g_nrf52_rtc0_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF52_RTC1
      case 1:
        {
          priv = &g_nrf52_rtc1_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF52_RTC2
      case 2:
        {
          priv = &g_nrf52_rtc2_priv;
          break;
        }
#endif

      default:
        {
          rtcerr("ERROR: unsupported RTC %d\n", rtc);
          goto errout;
        }
    }

  if (priv->inuse != false)
    {
      /* RTC already in use */

      priv = NULL;
    }

errout:
  return (struct nrf52_rtc_dev_s *)priv;
}

/****************************************************************************
 * Name: nrf52_rtc_deinit
 *
 * Description:
 *   Deinit RTC device
 *
 ****************************************************************************/

int nrf52_rtc_deinit(struct nrf52_rtc_dev_s *dev)
{
  struct nrf52_rtc_priv_s *rtc = NULL;

  DEBUGASSERT(dev);

  rtc = (struct nrf52_rtc_priv_s *)dev;

  rtc->inuse = false;

  return OK;
}
