/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_grtc.c
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

#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/nrf54l_grtc.h"

#include "nrf54l_grtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_grtc_priv_s
{
  struct nrf54l_grtc_ops_s *ops;
  uint32_t                  base;
  uint32_t                  irq;
  uint8_t                   chan;
  bool                      inuse;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* GRTC registers access ****************************************************/

static uint32_t nrf54l_grtc_getreg(struct nrf54l_grtc_dev_s *dev,
                                   uint32_t offset);
static void nrf54l_grtc_putreg(struct nrf54l_grtc_dev_s *dev,
                               uint32_t offset,
                               uint32_t value);

/* GRTC helpers *************************************************************/

static uint32_t nrf54l_grtc_irq2reg(struct nrf54l_grtc_dev_s *dev,
                                    uint8_t s);
static void nrf54l_grtc_waitready(struct nrf54l_grtc_dev_s *dev);

/* GRTC operations **********************************************************/

static int nrf54l_grtc_start(struct nrf54l_grtc_dev_s *dev);
static int nrf54l_grtc_stop(struct nrf54l_grtc_dev_s *dev);
static int nrf54l_grtc_clear(struct nrf54l_grtc_dev_s *dev);
static int nrf54l_grtc_getcounter(struct nrf54l_grtc_dev_s *dev,
                                  uint64_t *cc);
static int nrf54l_grtc_setcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                             uint64_t cc);
static int nrf54l_grtc_getcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                             uint64_t *cc);
static int nrf54l_grtc_disablecc(struct nrf54l_grtc_dev_s *dev, uint8_t i);
static int nrf54l_grtc_setisr(struct nrf54l_grtc_dev_s *dev, xcpt_t handler,
                              void * arg);
static int nrf54l_grtc_enableint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static int nrf54l_grtc_disableint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static int nrf54l_grtc_checkint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static int nrf54l_grtc_ackint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static uint32_t nrf54l_grtc_getbase(struct nrf54l_grtc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NRF54L GRTC ops */

struct nrf54l_grtc_ops_s nrf54l_grtc_ops =
{
  .start      = nrf54l_grtc_start,
  .stop       = nrf54l_grtc_stop,
  .clear      = nrf54l_grtc_clear,
  .getcounter = nrf54l_grtc_getcounter,
  .setcc      = nrf54l_grtc_setcc,
  .getcc      = nrf54l_grtc_getcc,
  .disablecc  = nrf54l_grtc_disablecc,
  .setisr     = nrf54l_grtc_setisr,
  .enableint  = nrf54l_grtc_enableint,
  .disableint = nrf54l_grtc_disableint,
  .checkint   = nrf54l_grtc_checkint,
  .ackint     = nrf54l_grtc_ackint,
  .getbase    = nrf54l_grtc_getbase,
};

/* GRTC */

struct nrf54l_grtc_priv_s g_nrf54l_grtc_priv =
{
  .ops   = &nrf54l_grtc_ops,
  .base  = NRF54L_GRTC_BASE,
  .irq   = NRF54L_IRQ_GRTC_0,
  .chan  = 12,
  .inuse = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_grtc_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static uint32_t nrf54l_grtc_getreg(struct nrf54l_grtc_dev_s *dev,
                                   uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct nrf54l_grtc_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf54l_grtc_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static void nrf54l_grtc_putreg(struct nrf54l_grtc_dev_s *dev,
                               uint32_t offset,
                               uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct nrf54l_grtc_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf54l_grtc_irq2reg
 *
 * Description:
 *   Get the value of the interrupt register corresponding to the given
 *   interrupt source
 *
 ****************************************************************************/

static uint32_t nrf54l_grtc_irq2reg(struct nrf54l_grtc_dev_s *dev,
                                    uint8_t s)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  uint32_t regval = 0;

  if (s < grtc->chan)
    {
      regval = GRTC_INT_COMPARE(s);
    }
  else
    {
      rtcerr("ERROR: unsupported IRQ source %d\n", s);
    }

  return regval;
}

/****************************************************************************
 * Name: nrf54l_grtc_waitready
 *
 * Description:
 *   Wait until the low-frequency timer has completed the previous task
 *
 ****************************************************************************/

static void nrf54l_grtc_waitready(struct nrf54l_grtc_dev_s *dev)
{
  while ((nrf54l_grtc_getreg(dev, NRF54L_GRTC_STATUS_LFTIMER_OFFSET) &
          GRTC_STATUS_LFTIMER_READY) == 0)
    {
    }
}

/****************************************************************************
 * Name: nrf54l_grtc_start
 ****************************************************************************/

static int nrf54l_grtc_start(struct nrf54l_grtc_dev_s *dev)
{
  uint64_t counter;

  DEBUGASSERT(dev);

  nrf54l_grtc_waitready(dev);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_TASKS_START_OFFSET, GRTC_TASKS_START);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_MODE_OFFSET, GRTC_MODE_SYSCOUNTEREN);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_SYSCOUNTER_ACTIVE_OFFSET(0),
                     GRTC_SYSCOUNTER_ACTIVE);

  /* Reading the low word initiates synchronization of the snapshot */

  nrf54l_grtc_getcounter(dev, &counter);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_stop
 ****************************************************************************/

static int nrf54l_grtc_stop(struct nrf54l_grtc_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_SYSCOUNTER_ACTIVE_OFFSET(0), 0);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_MODE_OFFSET, 0);
  nrf54l_grtc_waitready(dev);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_TASKS_STOP_OFFSET, GRTC_TASKS_STOP);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_clear
 ****************************************************************************/

static int nrf54l_grtc_clear(struct nrf54l_grtc_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf54l_grtc_waitready(dev);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_TASKS_CLEAR_OFFSET, GRTC_TASKS_CLEAR);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_getcounter
 ****************************************************************************/

static int nrf54l_grtc_getcounter(struct nrf54l_grtc_dev_s *dev,
                                  uint64_t *ctr)
{
  irqstate_t flags;
  uint32_t low;
  uint32_t high;

  DEBUGASSERT(dev);
  DEBUGASSERT(ctr);

  /* The low word read snapshots the high word. Retry when synchronization
   * is busy or the low word rolled over between the reads.
   */

  flags = enter_critical_section();

  do
    {
      low  = nrf54l_grtc_getreg(dev, NRF54L_GRTC_SYSCOUNTERL_OFFSET(0));
      high = nrf54l_grtc_getreg(dev, NRF54L_GRTC_SYSCOUNTERH_OFFSET(0));
    }
  while ((high & (GRTC_SYSCOUNTERH_BUSY | GRTC_SYSCOUNTERH_OVERFLOW)) != 0);

  leave_critical_section(flags);

  *ctr = ((uint64_t)(high & GRTC_SYSCOUNTERH_MASK) << 32) | low;

  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_setcc
 ****************************************************************************/

static int nrf54l_grtc_setcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                             uint64_t cc)
{
  struct nrf54l_grtc_priv_s *grtc = NULL;
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(dev);

  grtc = (struct nrf54l_grtc_priv_s *)dev;

  /* Is the channel supported? */

  if (i >= grtc->chan || cc > GRTC_COUNTER_MAX)
    {
      rtcerr("ERROR: unsupported GRTC channel %d\n", i);
      ret = -EINVAL;
      goto errout;
    }

  /* The channel must be disabled while the two words are written */

  flags = enter_critical_section();

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCEN_OFFSET(i), 0);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCL_OFFSET(i), (uint32_t)cc);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCH_OFFSET(i), (uint32_t)(cc >> 32));
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCEN_OFFSET(i), GRTC_CCEN_ACTIVE);

  leave_critical_section(flags);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_getcc
 ****************************************************************************/

static int nrf54l_grtc_getcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                             uint64_t *cc)
{
  struct nrf54l_grtc_priv_s *grtc = NULL;
  uint32_t high;
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cc);

  grtc = (struct nrf54l_grtc_priv_s *)dev;

  /* Is the channel supported? */

  if (i >= grtc->chan)
    {
      rtcerr("ERROR: unsupported GRTC channel %d\n", i);
      ret = -EINVAL;
      goto errout;
    }

  high = nrf54l_grtc_getreg(dev, NRF54L_GRTC_CCH_OFFSET(i));
  *cc = ((uint64_t)(high & GRTC_CCH_MASK) << 32) |
        nrf54l_grtc_getreg(dev, NRF54L_GRTC_CCL_OFFSET(i));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_disablecc
 ****************************************************************************/

static int nrf54l_grtc_disablecc(struct nrf54l_grtc_dev_s *dev, uint8_t i)
{
  struct nrf54l_grtc_priv_s *grtc = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  grtc = (struct nrf54l_grtc_priv_s *)dev;

  /* Is the channel supported? */

  if (i >= grtc->chan)
    {
      rtcerr("ERROR: unsupported GRTC channel %d\n", i);
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCEN_OFFSET(i), 0);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_setisr
 ****************************************************************************/

static int nrf54l_grtc_setisr(struct nrf54l_grtc_dev_s *dev, xcpt_t handler,
                              void *arg)
{
  struct nrf54l_grtc_priv_s *grtc = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  grtc = (struct nrf54l_grtc_priv_s *)dev;

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(grtc->irq);
      irq_detach(grtc->irq);
      ret = OK;
      goto errout;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(grtc->irq, handler, arg);
  up_enable_irq(grtc->irq);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_enableint
 ****************************************************************************/

static int nrf54l_grtc_enableint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given interrupt source */

  regval = nrf54l_grtc_irq2reg(dev, s);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_INTENSET_OFFSET(0), regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_disableint
 ****************************************************************************/

static int nrf54l_grtc_disableint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given interrupt source */

  regval = nrf54l_grtc_irq2reg(dev, s);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_INTENCLR_OFFSET(0), regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_checkint
 ****************************************************************************/

static int nrf54l_grtc_checkint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  int ret = 0;

  DEBUGASSERT(dev);

  if (s >= grtc->chan)
    {
      rtcerr("ERROR: unsupported IRQ source %d\n", s);
      ret = -EINVAL;
      goto errout;
    }

  ret = nrf54l_grtc_getreg(dev, NRF54L_GRTC_EVENTS_COMPARE_OFFSET(s));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_ackint
 ****************************************************************************/

static int nrf54l_grtc_ackint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  int ret = 0;

  DEBUGASSERT(dev);

  if (s >= grtc->chan)
    {
      rtcerr("ERROR: unsupported IRQ source %d\n", s);
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_EVENTS_COMPARE_OFFSET(s), 0);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_getbase
 ****************************************************************************/

static uint32_t nrf54l_grtc_getbase(struct nrf54l_grtc_dev_s *dev)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  DEBUGASSERT(dev);

  return grtc->base;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_grtc_init
 *
 * Description:
 *   Initialize GRTC device
 *
 ****************************************************************************/

struct nrf54l_grtc_dev_s *nrf54l_grtc_init(int grtc)
{
  struct nrf54l_grtc_priv_s *priv = NULL;
  uint32_t regval;

  /* Get GRTC instance */

  switch (grtc)
    {
      case 0:
        {
          priv = &g_nrf54l_grtc_priv;
          break;
        }

      default:
        {
          rtcerr("ERROR: unsupported GRTC %d\n", grtc);
          goto errout;
        }
    }

  if (priv->inuse != false)
    {
      /* GRTC already in use */

      priv = NULL;
      goto errout;
    }

  priv->inuse = true;

  /* Clock the LF timer from LFCLK. This register is not affected by a
   * system reset so it is always configured here.
   */

  regval = getreg32(priv->base + NRF54L_GRTC_CLKCFG_OFFSET);
  regval &= ~GRTC_CLKCFG_CLKSEL_MASK;
  regval |= GRTC_CLKCFG_CLKSEL_LFCLK;
  putreg32(regval, priv->base + NRF54L_GRTC_CLKCFG_OFFSET);

errout:
  return (struct nrf54l_grtc_dev_s *)priv;
}

/****************************************************************************
 * Name: nrf54l_grtc_deinit
 *
 * Description:
 *   Deinit GRTC device
 *
 ****************************************************************************/

int nrf54l_grtc_deinit(struct nrf54l_grtc_dev_s *dev)
{
  struct nrf54l_grtc_priv_s *grtc = NULL;

  DEBUGASSERT(dev);

  grtc = (struct nrf54l_grtc_priv_s *)dev;

  grtc->inuse = false;

  return OK;
}
