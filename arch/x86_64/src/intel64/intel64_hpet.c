/****************************************************************************
 * arch/x86_64/src/intel64/intel64_hpet.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/io.h>
#include <arch/hpet.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>

#include "intel64_hpet.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* HPET timer channel */

struct intel64_hpet_chan_s
{
  uint8_t irq;
};

/* HPET timer driver */

struct intel64_hpet_s
{
  struct intel64_tim_ops_s   *ops;
  uint64_t                    base;
  uint32_t                    clk_per_fs;
  uint8_t                     timers;
  bool                        initialized;
  struct intel64_hpet_chan_s  chans[CONFIG_INTEL64_HPET_CHANNELS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void intel64_hpet_putreg(struct intel64_hpet_s *hpet, uint32_t offset,
                                uint64_t value);
static uint64_t intel64_hpet_getreg(struct intel64_hpet_s *hpet,
                                    uint32_t offset);

/* Ops */

static void intel64_hpet_enable(struct intel64_tim_dev_s *dev, bool en);
static void intel64_hpet_cmpset(struct intel64_tim_dev_s *dev, uint8_t timer,
                                uint64_t cmp);
static uint64_t intel64_hpet_cmpget(struct intel64_tim_dev_s *dev,
                                    uint8_t timer);
static uint64_t intel64_hpet_cntget(struct intel64_tim_dev_s *dev);
static uint64_t intel64_hpet_intget(struct intel64_tim_dev_s *dev,
                                    uint8_t timer);
static void intel64_hpet_intack(struct intel64_tim_dev_s *dev,
                                uint8_t timer);
static void intel64_hpet_cntset(struct intel64_tim_dev_s *dev,
                                uint64_t cntr);
static uint32_t intel64_hpet_perget(struct intel64_tim_dev_s *dev);
static int intel64_hpet_setisr(struct intel64_tim_dev_s *dev, uint8_t timer,
                               xcpt_t handler, void *arg, bool periodic);
static void intel64_hpet_enint(struct intel64_tim_dev_s *dev, uint8_t tim);
static void intel64_hpet_disint(struct intel64_tim_dev_s *dev, uint8_t tim);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HPET ops */

static struct intel64_tim_ops_s g_intel64_hpet_ops =
{
  .enable     = intel64_hpet_enable,
  .getperiod  = intel64_hpet_perget,
  .getcounter = intel64_hpet_cntget,
  .setcounter = intel64_hpet_cntset,
  .setcompare = intel64_hpet_cmpset,
  .getcompare = intel64_hpet_cmpget,
  .getint     = intel64_hpet_intget,
  .ackint     = intel64_hpet_intack,
  .setisr     = intel64_hpet_setisr,
  .enableint  = intel64_hpet_enint,
  .disableint = intel64_hpet_disint
};

/* HPET driver instance */

static struct intel64_hpet_s g_intel64_hpet =
{
  .ops         = &g_intel64_hpet_ops,
  .initialized = false,
  .chans =
  {
    /* Channel 0 (Timer 0) */

    {
      .irq = HPET0_IRQ,
    },

#if CONFIG_INTEL64_HPET_CHANNELS > 1
    /* Channel 1 (Timer 1) */

    {
      .irq = HPET1_IRQ,
    },
#endif

#if CONFIG_INTEL64_HPET_CHANNELS > 2
    /* Channel 2 (Timer 2) */

    {
      .irq = HPET2_IRQ
    },
#endif
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_hpet_putreg
 *
 * Description:
 *   Put a 64-bit register value by offset
 *
 ****************************************************************************/

static void intel64_hpet_putreg(struct intel64_hpet_s *hpet, uint32_t offset,
                                uint64_t value)
{
  return mmio_write64((void *)(hpet->base + offset), value);
}

/****************************************************************************
 * Name: intel64_hpet_getreg
 *
 * Description:
 *   Get a 64-bit register value by offset
 *
 ****************************************************************************/

static uint64_t intel64_hpet_getreg(struct intel64_hpet_s *hpet,
                                    uint32_t offset)
{
  return mmio_read64((void *)(hpet->base + offset));
}

/****************************************************************************
 * Name: intel64_hpet_enable
 *
 * Description:
 *   Allow main counter to run and allow timer interrupts
 *
 ****************************************************************************/

static void intel64_hpet_enable(struct intel64_tim_dev_s *dev, bool en)
{
  struct intel64_hpet_s *hpet   = (struct intel64_hpet_s *)dev;
  uint64_t               regval = 0;

  regval = intel64_hpet_getreg(hpet, HPET_GCONF_OFFSET);

  if (en)
    {
      regval |= HPET_GCONF_ENABLE;
    }
  else
    {
      regval &= ~HPET_GCONF_ENABLE;
    }

  intel64_hpet_putreg(hpet, HPET_GCONF_OFFSET, regval);
}

/****************************************************************************
 * Name: intel64_hpet_cmpset
 *
 * Description:
 *   Set a compare register to a given value.
 *
 ****************************************************************************/

static void intel64_hpet_cmpset(struct intel64_tim_dev_s *dev, uint8_t timer,
                                uint64_t cmp)
{
  struct intel64_hpet_s *hpet = (struct intel64_hpet_s *)dev;
  DEBUGASSERT(timer < hpet->timers);
  intel64_hpet_putreg(hpet, HPET_TCOMP_OFFSET(timer), cmp);
}

/****************************************************************************
 * Name: intel64_hpet_cmpget
 *
 * Description:
 *   Get a compare register to a given value.
 *
 ****************************************************************************/

static uint64_t intel64_hpet_cmpget(struct intel64_tim_dev_s *dev,
                                    uint8_t timer)
{
  struct intel64_hpet_s *hpet = (struct intel64_hpet_s *)dev;
  DEBUGASSERT(timer < hpet->timers);
  return intel64_hpet_getreg(hpet, HPET_TCOMP_OFFSET(timer));
}

/****************************************************************************
 * Name: intel64_hpet_intget
 *
 * Description:
 *   Get a interrupt status register.
 *
 ****************************************************************************/

static uint64_t intel64_hpet_intget(struct intel64_tim_dev_s *dev,
                                    uint8_t timer)
{
  struct intel64_hpet_s *hpet = (struct intel64_hpet_s *)dev;
  return (intel64_hpet_getreg(hpet, HPET_GISR_OFFSET) &
          HPET_GISR_TINT(timer));
}

/****************************************************************************
 * Name: intel64_hpet_intack
 *
 * Description:
 *   ACK interrupt.
 *
 ****************************************************************************/

static void intel64_hpet_intack(struct intel64_tim_dev_s *dev,
                                uint8_t timer)
{
  struct intel64_hpet_s *hpet = (struct intel64_hpet_s *)dev;
  intel64_hpet_putreg(hpet, HPET_GISR_OFFSET, HPET_GISR_TINT(timer));
}

/****************************************************************************
 * Name: intel64_hpet_cntget
 *
 * Description:
 *   Get the main counter.
 *
 ****************************************************************************/

static uint64_t intel64_hpet_cntget(struct intel64_tim_dev_s *dev)
{
  struct intel64_hpet_s *hpet = (struct intel64_hpet_s *)dev;
  return intel64_hpet_getreg(hpet, HPET_MCNTR_OFFSET);
}

/****************************************************************************
 * Name: intel64_hpet_cntset
 *
 * Description:
 *   Set the main counter.
 *
 ****************************************************************************/

static void intel64_hpet_cntset(struct intel64_tim_dev_s *dev,
                                uint64_t cntr)
{
  struct intel64_hpet_s *hpet = (struct intel64_hpet_s *)dev;
  return intel64_hpet_putreg(hpet, HPET_MCNTR_OFFSET, cntr);
}

/****************************************************************************
 * Name: intel64_hpet_perget
 *
 * Description:
 *   Get the main counter period in femtosecounds (1e-15 sec).
 *
 ****************************************************************************/

static uint32_t intel64_hpet_perget(struct intel64_tim_dev_s *dev)
{
  struct intel64_hpet_s *hpet = (struct intel64_hpet_s *)dev;
  return hpet->clk_per_fs;
}

/****************************************************************************
 * Name: intel64_hpet_setisr
 *
 * Description:
 *   Configure interrupt handler for a given timer
 *
 ****************************************************************************/

static int intel64_hpet_setisr(struct intel64_tim_dev_s *dev, uint8_t timer,
                               xcpt_t handler, void *arg, bool periodic)
{
  struct intel64_hpet_s *hpet   = (struct intel64_hpet_s *)dev;
  uint64_t               regval = 0;
  uint8_t                irq    = hpet->chans[timer].irq;

  DEBUGASSERT(timer < hpet->timers);

  regval = intel64_hpet_getreg(hpet, HPET_TCONF_OFFSET(timer));

  if (periodic)
    {
      if ((regval & HPET_TCONF_PERCAP) == 0)
        {
          tmrerr("Periodic not supported");
          return -EPERM;
        }

      regval |= HPET_TCONF_PERCAP;
    }
  else
    {
      regval &= ~HPET_TCONF_PERCAP;
    }

  /* Route interrupts */

  regval |= HPET_TCONF_INTROUTE(irq - IRQ0);

  /* Set level triggered mode.
   *
   * Edge triggered mode seems to work well on QEMU, but for real hardware,
   * unwanted interrupt is generated when we enable timer interrupts.
   */

  regval |= HPET_TCONF_INTTYPE;

  /* Set 64-bit mode */

  regval &= ~HPET_TCONF_32MODE;

  /* Write Timer configuration */

  intel64_hpet_putreg(hpet, HPET_TCONF_OFFSET(timer), regval);

  if (handler == NULL)
    {
      /* Disable interrupt */

      irq_attach(irq, handler, arg);
      up_disable_irq(irq);
    }
  else
    {
      /* Set callback and enable interrupt */

      irq_attach(irq, handler, arg);
      up_enable_irq(irq);
    }

  return OK;
}

/****************************************************************************
 * Name: intel64_hpet_enint
 *
 * Description:
 *   Enable interrupt
 *
 ****************************************************************************/

static void intel64_hpet_enint(struct intel64_tim_dev_s *dev, uint8_t tim)
{
  struct intel64_hpet_s *hpet   = (struct intel64_hpet_s *)dev;
  uint64_t               regval = 0;

  DEBUGASSERT(tim < hpet->timers);

  regval = intel64_hpet_getreg(hpet, HPET_TCONF_OFFSET(tim));
  regval |= HPET_TCONF_INTEN;
  intel64_hpet_putreg(hpet, HPET_TCONF_OFFSET(tim), regval);
}

/****************************************************************************
 * Name: intel64_hpet_disint
 *
 * Description:
 *   Disable interrupt
 *
 ****************************************************************************/

static void intel64_hpet_disint(struct intel64_tim_dev_s *dev, uint8_t tim)
{
  struct intel64_hpet_s *hpet   = (struct intel64_hpet_s *)dev;
  uint64_t               regval = 0;

  DEBUGASSERT(tim < hpet->timers);

  regval = intel64_hpet_getreg(hpet, HPET_TCONF_OFFSET(tim));
  regval &= ~HPET_TCONF_INTEN;
  intel64_hpet_putreg(hpet, HPET_TCONF_OFFSET(tim), regval);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_hpet_init
 *
 * Description:
 *   Initialize HPET timer with a given base address
 *
 ****************************************************************************/

struct intel64_tim_dev_s *intel64_hpet_init(uint64_t base)
{
  struct intel64_hpet_s *hpet   = &g_intel64_hpet;
  uint64_t               regval = 0;

  if (hpet->initialized == false)
    {
      /* Store HPET base */

      hpet->base = base;

      /* Map HPET region */

      up_map_region((void *)hpet->base, HPET_REGION_SIZE,
                    (X86_PAGE_PRESENT | X86_PAGE_WR | X86_PAGE_NOCACHE));

      /* Get capabilities */

      regval           = intel64_hpet_getreg(hpet, HPET_GCAPID_OFFSET);
      hpet->clk_per_fs = ((regval & HPET_GCAPID_CLKPER_MASK) >>
                          HPET_GCAPID_CLKPER_SHIFT);
      hpet->timers     = ((regval & HPET_GCAPID_NUMTIM_MASK) >>
                          HPET_GCAPID_NUMTIM_SHIFT);

      hpet->timers = (hpet->timers > CONFIG_INTEL64_HPET_CHANNELS ?
                      CONFIG_INTEL64_HPET_CHANNELS : hpet->timers);

      /* Connect ops */

      hpet->ops = &g_intel64_hpet_ops;

      if (regval & HPET_GCAPID_LEGROUTE)
        {
          /* Configure legacy mode.
           *
           * There is no way to disable PIT interrupts (?) other than enable
           * legacy mode for HPET. Otherwise unwanted PIT interupts will
           * interfere with HPET interrupts, making them useless.
           */

          intel64_hpet_putreg(hpet, HPET_GCONF_OFFSET, HPET_GCONF_LEGERT);
        }
      else
        {
          /* Not supported */

          ASSERT(0);
        }

      /* Enable HPET */

      intel64_hpet_enable((struct intel64_tim_dev_s *)hpet, true);

      tmrinfo("clk_per_fs = %" PRId32 " timers = %d\n",
              hpet->clk_per_fs, hpet->timers);

      /* Initialization done */

      hpet->initialized = true;
    }

  return (struct intel64_tim_dev_s *)hpet;
}
