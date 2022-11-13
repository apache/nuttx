/****************************************************************************
 * arch/arm/src/samv7/sam_tc.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2011, Atmel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* References:
 *
 *   SAMV71 Series Data Sheet
 *   NuttX SAMA5 timer/counter driver
 *   Atmel NoOS sample code for the SAMA5D3.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_periphclks.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_pmc.h"
#include "sam_gpio.h"
#include "sam_pck.h"
#include "sam_tc.h"

#if defined(CONFIG_SAMV7_TC0) || defined(CONFIG_SAMV7_TC1) || \
    defined(CONFIG_SAMV7_TC2) || defined(CONFIG_SAMV7_TC3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG_TIMER_INFO
#  undef CONFIG_SAMV7_TC_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the static configuration of a TC channel */

struct sam_chconfig_s
{
  uintptr_t base;          /* Channel register base address */
  uint8_t pid;             /* Peripheral ID */
  uint8_t irq;             /* IRQ number */
  gpio_pinset_t clkset;    /* CLK input PIO configuration */
  gpio_pinset_t tioaset;   /* Output A PIO configuration */
  gpio_pinset_t tiobset;   /* Output B PIO configuration */
};

/* This structure describes the static configuration of a TC */

struct sam_tcconfig_s
{
  uintptr_t base;          /* TC register base address */
  uint8_t tc;              /* Timer/counter number */

  /* Channels */

  struct sam_chconfig_s channel[SAM_TC_NCHANNELS];
};

/* This structure describes one timer counter channel */

struct sam_tc_s;
struct sam_chan_s
{
  struct sam_tc_s *tc;     /* Parent timer/counter */
  uintptr_t base;          /* Channel register base address */
  tc_handler_t handler;    /* User timer interrupt handler */
  void *arg;               /* User interrupt handler argument */
  uint8_t chan;            /* Channel number (0, 1, 2, ... 11} */
  bool inuse;              /* True: channel is in use */
};

/* This structure describes one timer/counter */

struct sam_tc_s
{
  mutex_t lock;            /* Assures mutually exclusive access to TC */
  uintptr_t base;          /* Register base address */
  uint8_t tc;              /* Timer/channel number {0, 1, 2, 3} */
  bool initialized;        /* True: Timer/counter has been initialized */

  /* Channels */

  struct sam_chan_s channel[SAM_TC_NCHANNELS];

  /* Debug stuff */

#ifdef CONFIG_SAMV7_TC_REGDEBUG
  bool wr;                /* True:Last was a write */
  uint32_t regaddr;       /* Last address */
  uint32_t regval;        /* Last value */
  int ntimes;             /* Number of times */
#endif
};

/* Type of the MCK divider lookup table */

struct mck_divsrc_s
{
  uint8_t log2;            /* Log2 of the divider */
  uint8_t tcclks;          /* CMR TCCLCKS setting */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TC_REGDEBUG
static void sam_regdump(struct sam_chan_s *chan, const char *msg);
static bool sam_checkreg(struct sam_tc_s *tc, bool wr, uint32_t regaddr,
                         uint32_t regval);
#else
#  define   sam_regdump(chan,msg)
#  define   sam_checkreg(tc,wr,regaddr,regval) (false)
#endif

static inline uint32_t sam_tc_getreg(struct sam_chan_s *chan,
                                     unsigned int offset);
static inline void sam_tc_putreg(struct sam_chan_s *chan,
                                 unsigned int offset, uint32_t regval);

static inline uint32_t sam_chan_getreg(struct sam_chan_s *chan,
                                       unsigned int offset);
static inline void sam_chan_putreg(struct sam_chan_s *chan,
                                   unsigned int offset, uint32_t regval);

/* Interrupt Handling *******************************************************/

static int sam_tc_interrupt(int irq, void *context, void *arg);

/* Initialization ***********************************************************/

static uint32_t sam_tc_mckfreq_lookup(uint32_t ftcin, int ndx);
static inline uint32_t sam_tc_tcclks_lookup(int ndx);
static int sam_tc_mcksrc(uint32_t frequency, uint32_t *tcclks,
                         uint32_t *actual);
static inline struct sam_chan_s *sam_tc_initialize(int channel);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Static timer configuration */

#ifdef CONFIG_SAMV7_TC0
static const struct sam_tcconfig_s g_tc012config =
{
  .base    = SAM_TC012_BASE,
  .tc      = 0,
  .channel =
  {
    [0] =
    {
      .base    = SAM_TC012_CHAN_BASE(0),
      .pid     = SAM_PID_TC0,
      .irq     = SAM_IRQ_TC0,

#ifdef CONFIG_SAMV7_TC0_CLK0
      .clkset  = GPIO_TC0_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC0_TIOA0
      .tioaset = GPIO_TC0_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC0_TIOB0
      .tiobset = GPIO_TC0_TIOB,
#else
      .tiobset = 0,
#endif
    },
    [1] =
    {
      .base    = SAM_TC012_CHAN_BASE(1),
      .pid     = SAM_PID_TC1,
      .irq     = SAM_IRQ_TC1,

#ifdef CONFIG_SAMV7_TC0_CLK1
      .clkset  = GPIO_TC1_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC0_TIOA1
      .tioaset = GPIO_TC1_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC0_TIOB1
      .tiobset = GPIO_TC1_TIOB,
#else
      .tiobset = 0,
#endif
    },
    [2] =
    {
      .base    = SAM_TC012_CHAN_BASE(2),
      .pid     = SAM_PID_TC2,
      .irq     = SAM_IRQ_TC2,

#ifdef CONFIG_SAMV7_TC0_CLK2
      .clkset  = GPIO_TC2_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC0_TIOA2
      .tioaset = GPIO_TC2_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC0_TIOB2
      .tiobset = GPIO_TC2_TIOB,
#else
      .tiobset = 0,
#endif
    },
  },
};
#endif

#ifdef CONFIG_SAMV7_TC1
static const struct sam_tcconfig_s g_tc345config =
{
  .base    = SAM_TC345_BASE,
  .tc      = 1,
  .channel =
  {
    [0] =
    {
      .base    = SAM_TC345_CHAN_BASE(3),
      .pid     = SAM_PID_TC3,
      .irq     = SAM_IRQ_TC3,

#ifdef CONFIG_SAMV7_TC1_CLK3
      .clkset  = GPIO_TC3_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC1_TIOA3
      .tioaset = GPIO_TC3_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC1_TIOB3
      .tiobset = GPIO_TC3_TIOB,
#else
      .tiobset = 0,
#endif
    },
    [1] =
    {
      .base    = SAM_TC345_CHAN_BASE(4),
      .pid     = SAM_PID_TC4,
      .irq     = SAM_IRQ_TC4,

#ifdef CONFIG_SAMV7_TC1_CLK4
      .clkset  = GPIO_TC4_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC1_TIOA4
      .tioaset = GPIO_TC4_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC1_TIOB4
      .tiobset = GPIO_TC4_TIOB,
#else
      .tiobset = 0,
#endif
    },
    [2] =
    {
      .base    = SAM_TC345_CHAN_BASE(5),
      .pid     = SAM_PID_TC5,
      .irq     = SAM_IRQ_TC5,

#ifdef CONFIG_SAMV7_TC1_CLK5
      .clkset  = GPIO_TC5_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC1_TIOA5
      .tioaset = GPIO_TC5_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC1_TIOB5
      .tiobset = GPIO_TC5_TIOB,
#else
      .tiobset = 0,
#endif
    },
  },
};
#endif

#ifdef CONFIG_SAMV7_TC2
static const struct sam_tcconfig_s g_tc678config =
{
  .base    = SAM_TC678_BASE,
  .tc      = 2,
  .channel =
  {
    [0] =
    {
      .base    = SAM_TC678_CHAN_BASE(6),
      .pid     = SAM_PID_TC6,
      .irq     = SAM_IRQ_TC6,

#ifdef CONFIG_SAMV7_TC2_CLK6
      .clkset  = GPIO_TC6_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOA6
      .tioaset = GPIO_TC6_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOB6
      .tiobset = GPIO_TC6_TIOB,
#else
      .tiobset = 0,
#endif
    },
    [1] =
    {
      .base    = SAM_TC678_CHAN_BASE(7),
      .pid     = SAM_PID_TC7,
      .irq     = SAM_IRQ_TC7,

#ifdef CONFIG_SAMV7_TC2_CLK7
      .clkset  = GPIO_TC7_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOA7
      .tioaset = GPIO_TC7_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOB7
      .tiobset = GPIO_TC7_TIOB,
#else
      .tiobset = 0,
#endif
    },
    [2] =
    {
      .base    = SAM_TC345_CHAN_BASE(8),
      .pid     = SAM_PID_TC8,
      .irq     = SAM_IRQ_TC8,

#ifdef CONFIG_SAMV7_TC2_CLK8
      .clkset  = GPIO_TC8_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOA8
      .tioaset = GPIO_TC8_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOB8
      .tiobset = GPIO_TC8_TIOB,
#else
      .tiobset = 0,
#endif
    },
  },
};
#endif

#ifdef CONFIG_SAMV7_TC3
static const struct sam_tcconfig_s g_tc901config =
{
  .base    = SAM_TC901_BASE,
  .tc      = 3,
  .channel =
  {
    [0] =
    {
      .base    = SAM_TC901_CHAN_BASE(9),
      .pid     = SAM_PID_TC9,
      .irq     = SAM_IRQ_TC9,

#ifdef CONFIG_SAMV7_TC2_CLK9
      .clkset  = GPIO_TC9_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOA9
      .tioaset = GPIO_TC9_TIOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOB9
      .tiobset = GPIO_TC9_TIOB,
#else
      .tiobset = 0,
#endif
    },
    [1] =
    {
      .base    = SAM_TC901_CHAN_BASE(10),
      .pid     = SAM_PID_TC10,
      .irq     = SAM_IRQ_TC10,

#ifdef CONFIG_SAMV7_TC2_CLK10
      .clkset  = GPIO_TC10_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOA10
      .tioaset = GPIO_TC10_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOB10
      .tiobset = GPIO_TC10_IOB,
#else
      .tiobset = 0,
#endif
    },
    [2] =
    {
      .base    = SAM_TC345_CHAN_BASE(11),
      .pid     = SAM_PID_TC11,
      .irq     = SAM_IRQ_TC11,

#ifdef CONFIG_SAMV7_TC2_CLK11
      .clkset  = GPIO_TC11_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOA11
      .tioaset = GPIO_TC11_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMV7_TC2_TIOB11
      .tiobset = GPIO_TC11_IOB,
#else
      .tiobset = 0,
#endif
    },
  },
};
#endif

/* Timer/counter state */

#ifdef CONFIG_SAMV7_TC0
static struct sam_tc_s g_tc012;
#endif

#ifdef CONFIG_SAMV7_TC1
static struct sam_tc_s g_tc345;
#endif

#ifdef CONFIG_SAMV7_TC2
static struct sam_tc_s g_tc678;
#endif

#ifdef CONFIG_SAMV7_TC3
static struct sam_tc_s g_tc901;
#endif

/* TC frequency data.  This table provides the frequency for each
 * selection of TCCLK
 */

#define TC_NDIVIDERS   3
#define TC_NDIVOPTIONS 4

/* This is the list of divider values: divider = (1 << value) */

static struct mck_divsrc_s g_log2divider[TC_NDIVOPTIONS] =
{
            /* TIMER_CLOCK1(0) -> PCK6 */

  {
    3, 1    /* TIMER_CLOCK2(1) -> MCK/8 */
  },
  {
    5, 2    /* TIMER_CLOCK3(2) -> MCK/32 */
  },
  {
    7, 3    /* TIMER_CLOCK4(3) -> MCK/128 */
  },
  {
    0, 4    /* TIMER_CLOCK5(4) -> SLCK (No MCK divider) */
  },
};

/* TC register lookup used by sam_tc_setregister */

#define TC_NREGISTERS 3

static const uint8_t g_regoffset[TC_NREGISTERS] =
{
  SAM_TC_RA_OFFSET,     /* Register A */
  SAM_TC_RB_OFFSET,     /* Register B */
  SAM_TC_RC_OFFSET      /* Register C */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_regdump
 *
 * Description:
 *   Dump all timer/counter channel and global registers
 *
 *   NOTE: The status register is not read because reading the status
 *   register clears bits and, hence, may cause lost interrupts.
 *
 * Input Parameters:
 *   chan  - The timer/counter channel state
 *   msg   - Message to print with the data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TC_REGDEBUG
static void sam_regdump(struct sam_chan_s *chan, const char *msg)
{
  struct sam_tc_s *tc = chan->tc;
  uintptr_t base;

  base = tc->base;
  tmrinfo("TC%d [%08x]: %s\n", tc->tc, (int)base, msg);
  tmrinfo("  BMR: %08x QIMR: %08x QISR: %08x WPMR: %08x\n",
          getreg32(base + SAM_TC_BMR_OFFSET),
          getreg32(base + SAM_TC_QIMR_OFFSET),
          getreg32(base + SAM_TC_QISR_OFFSET),
          getreg32(base + SAM_TC_WPMR_OFFSET));

  base = chan->base;
  tmrinfo("TC%d Channel %d [%08x]: %s\n",
          tc->tc, chan->chan, (int)base, msg);
  tmrinfo("  CMR: %08x SSMR: %08x  RAB: %08x   CV: %08x\n",
          getreg32(base + SAM_TC_CMR_OFFSET),
          getreg32(base + SAM_TC_SMMR_OFFSET),
          getreg32(base + SAM_TC_RAB_OFFSET),
          getreg32(base + SAM_TC_CV_OFFSET));
  tmrinfo("   RA: %08x   RB: %08x   RC: %08x  IMR: %08x\n",
          getreg32(base + SAM_TC_RA_OFFSET),
          getreg32(base + SAM_TC_RB_OFFSET),
          getreg32(base + SAM_TC_RC_OFFSET),
          getreg32(base + SAM_TC_IMR_OFFSET));
}
#endif

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   tc      - The timer/counter peripheral state
 *   wr      - True:write access false:read access
 *   regval  - The register value associated with the access
 *   regaddr - The address of the register being accessed
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TC_REGDEBUG
static bool sam_checkreg(struct sam_tc_s *tc, bool wr, uint32_t regaddr,
                         uint32_t regval)
{
  if (wr      == tc->wr &&      /* Same kind of access? */
      regaddr == tc->regaddr && /* Same register address? */
      regval  == tc->regval)    /* Same register value? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      tc->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (tc->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          tmrinfo("...[Repeats %d times]...\n", tc->ntimes);
        }

      /* Save information about the new access */

      tc->wr      = wr;
      tc->regval  = regval;
      tc->regaddr = regaddr;
      tc->ntimes  = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: sam_tc_getreg
 *
 * Description:
 *  Read an TC register
 *
 ****************************************************************************/

static inline uint32_t sam_tc_getreg(struct sam_chan_s *chan,
                                     unsigned int offset)
{
  struct sam_tc_s *tc = chan->tc;
  uint32_t regaddr    = tc->base + offset;
  uint32_t regval     = getreg32(regaddr);

#ifdef CONFIG_SAMV7_TC_REGDEBUG
  if (sam_checkreg(tc, false, regaddr, regval))
    {
      tmrinfo("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: sam_tc_putreg
 *
 * Description:
 *  Write a value to an TC register
 *
 ****************************************************************************/

static inline void sam_tc_putreg(struct sam_chan_s *chan,
                                 unsigned int offset, uint32_t regval)
{
  struct sam_tc_s *tc = chan->tc;
  uint32_t regaddr    = tc->base + offset;

#ifdef CONFIG_SAMV7_TC_REGDEBUG
  if (sam_checkreg(tc, true, regaddr, regval))
    {
      tmrinfo("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: sam_chan_getreg
 *
 * Description:
 *  Read an TC channel register
 *
 ****************************************************************************/

static inline uint32_t sam_chan_getreg(struct sam_chan_s *chan,
                                       unsigned int offset)
{
  uint32_t regaddr = chan->base + offset;
  uint32_t regval  = getreg32(regaddr);

#ifdef CONFIG_SAMV7_TC_REGDEBUG
  if (sam_checkreg(chan->tc, false, regaddr, regval))
    {
      tmrinfo("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: sam_chan_putreg
 *
 * Description:
 *  Write a value to an TC channel register
 *
 ****************************************************************************/

static inline void sam_chan_putreg(struct sam_chan_s *chan,
                                   unsigned int offset, uint32_t regval)
{
  uint32_t regaddr = chan->base + offset;

#ifdef CONFIG_SAMV7_TC_REGDEBUG
  if (sam_checkreg(chan->tc, true, regaddr, regval))
    {
      tmrinfo("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Interrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tc_interrupt
 *
 * Description:
 *  Common timer channel interrupt handling.
 *
 * Input Parameters:
 *   irq     The IRQ number that generated the interrupt
 *   context Architecture specific register save information.
 *   arg     Pointer to timer counter channel structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int sam_tc_interrupt(int irq, void *context, void *arg)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)arg;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;

  /* Get the interrupt status for this channel */

  sr      = sam_chan_getreg(chan, SAM_TC_SR_OFFSET);
  imr     = sam_chan_getreg(chan, SAM_TC_IMR_OFFSET);
  pending = sr & imr;

  tmrinfo("TC%u Channel %u: pending=%08" PRIx32 "\n",
          chan->tc->tc, chan->chan, pending);

  /* Are there any pending interrupts for this channel? */

  if (pending != 0)
    {
      /* Yes... if we have pending interrupts then interrupts must be
       * enabled and we must have a handler attached.
       */

      DEBUGASSERT(chan->handler);
      if (chan->handler)
        {
          /* Execute the callback */

          chan->handler(chan, chan->arg, sr);
        }
      else
        {
          /* Should never happen */

          sam_chan_putreg(chan, SAM_TC_IDR_OFFSET, TC_INT_ALL);
        }
    }

  return OK;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tc_mckfreq_lookup
 *
 * Description:
 *  Given the TC input frequency (Ftcin) and a divider index, return the
 *  value of the divided frequency.  The slow clock source is treated as
 *  though it were a divided down MCK frequency.
 *
 * Input Parameters:
 *   ftcin - TC input frequency
 *   ndx   - Divider index
 *
 * Returned Value:
 *   The divided frequency value
 *
 ****************************************************************************/

static uint32_t sam_tc_mckfreq_lookup(uint32_t ftcin, int ndx)
{
  /* The final option is to use the SLOW clock */

  if (ndx >= TC_NDIVIDERS)
    {
      return BOARD_SLOWCLK_FREQUENCY;
    }
  else
    {
      return ftcin >> g_log2divider[ndx].log2;
    }
}

/****************************************************************************
 * Name: sam_tc_tcclks_lookup
 *
 * Description:
 *  Given the TC input frequency (Ftcin) and a divider index, return the
 *  value of the divided frequency.  The slow clock source is treated as
 *  though it were a divided down MCK frequency.
 *
 * Input Parameters:
 *   ftcin - TC input frequency
 *   ndx   - Divider index
 *
 * Returned Value:
 *   The divided frequency value
 *
 ****************************************************************************/

static inline uint32_t sam_tc_tcclks_lookup(int ndx)
{
  unsigned int index = g_log2divider[ndx].tcclks;
  return TC_CMR_TCCLKS(index);
}

/****************************************************************************
 * Name: sam_tc_mcksrc
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (Ftcin / (div * 65536)) <= freq <= (Ftcin / div)
 *
 *   where:
 *     freq  - the desired frequency
 *     Ftcin - The timer/counter input frequency
 *     div   - With DIV being the highest possible value.
 *
 * Input Parameters:
 *   frequency  Desired timer frequency.
 *   tcclks     TCCLKS field value for divisor.
 *   actual     The actual frequency of the MCK
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

static int sam_tc_mcksrc(uint32_t frequency, uint32_t *tcclks,
                         uint32_t *actual)
{
  uint32_t fselect;
  uint32_t fnext;
  int ndx = 0;

  tmrinfo("frequency=%" PRId32 "\n", frequency);

  /* Satisfy lower bound.  That is, the value of the divider such that:
   *
   *   frequency >= (tc_input_frequency * 65536) / divider.
   */

  for (; ndx < TC_NDIVIDERS; ndx++)
    {
      fselect = sam_tc_mckfreq_lookup(BOARD_MCK_FREQUENCY, ndx);
      if (frequency >= (fselect >> 16))
        {
          break;
        }
    }

  if (ndx >= TC_NDIVIDERS)
    {
      /* If no divisor can be found, return -ERANGE */

      tmrerr("ERROR: Lower bound search failed\n");
      return -ERANGE;
    }

  /* Try to maximize DIV while still satisfying upper bound.  That the
   * value of the divider such that:
   *
   *   frequency < tc_input_frequency / divider.
   */

  for (; ndx < TC_NDIVIDERS; ndx++)
    {
      fnext = sam_tc_mckfreq_lookup(BOARD_MCK_FREQUENCY, ndx + 1);
      if (frequency > fnext)
        {
          break;
        }

      fselect = fnext;
    }

  /* Return the actual frequency and the TCCLKS selection */

  *actual = fselect;
  *tcclks = sam_tc_tcclks_lookup(ndx);
  return OK;
}

/****************************************************************************
 * Name: sam_tc_initialize
 *
 * Description:
 *  There is no global, one-time initialization of timer/counter data
 *  structures.  Rather, this function is called each time that a channel
 *  is allocated and, if the channel has not been initialized, it will be
 *  initialized then.
 *
 * Input Parameters:
 *   channel TC channel number (see TC_CHANx definitions)
 *
 * Returned Value:
 *   A pointer to the initialized timer channel structure associated with tc
 *   and channel.  NULL is returned on any failure.
 *
 *   On successful return, the caller holds the tc exclusive access
 *   semaphore.
 *
 ****************************************************************************/

static inline struct sam_chan_s *sam_tc_initialize(int channel)
{
  struct sam_tc_s *tc;
  const struct sam_tcconfig_s *tcconfig;
  struct sam_chan_s *chan;
  const struct sam_chconfig_s *chconfig;
  irqstate_t flags;
  uint32_t regval;
  int chndx;
  int ch;
  int chfirst;
  int ret;

  /* Select the timer/counter and get the index associated with the
   * channel.
   */

#ifdef CONFIG_SAMV7_TC0
  if (channel >= 0 && channel < 3)
    {
      tc       = &g_tc012;
      tcconfig = &g_tc012config;
      chfirst  = 0;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TC1
  if (channel >= 3 && channel < 6)
    {
      tc       = &g_tc345;
      tcconfig = &g_tc345config;
      chfirst  = 3;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TC2
  if (channel >= 6 && channel < 9)
    {
      tc       = &g_tc678;
      tcconfig = &g_tc678config;
      chfirst  = 6;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TC3
  if (channel >= 9 && channel < 12)
    {
      tc       = &g_tc901;
      tcconfig = &g_tc901config;
      chfirst  = 9;
    }
  else
#endif
    {
      /* Timer/counter is not invalid or not enabled */

      tmrerr("ERROR: Bad channel number: %d\n", channel);
      return NULL;
    }

  /* Has the timer/counter been initialized.  We have to be careful here
   * because there is no semaphore protection.
   */

  flags = enter_critical_section();
  if (!tc->initialized)
    {
      /* Initialize the timer counter data structure. */

      memset(tc, 0, sizeof(struct sam_tc_s));
      nxmutex_init(&tc->lock);
      tc->base = tcconfig->base;
      tc->tc   = tcconfig->tc;

      /* Initialize the channels */

      for (chndx = 0, ch = chfirst; chndx < SAM_TC_NCHANNELS; chndx++)
        {
          /* Initialize the channel data structure */

          chan       = &tc->channel[chndx];
          chconfig   = &tcconfig->channel[chndx];

          chan->base = chconfig->base;
          chan->tc   = tc;
          chan->chan = ch++;

          /* Disable and clear all channel interrupts */

          sam_chan_putreg(chan, SAM_TC_IDR_OFFSET, TC_INT_ALL);
          sam_chan_getreg(chan, SAM_TC_SR_OFFSET);
        }

      /* Now the timer/counter is initialized */

      tc->initialized = true;
    }

  /* Get exclusive access to the timer/count data structure */

  ret = nxmutex_lock(&tc->lock);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return NULL;
    }

  leave_critical_section(flags);

  /* Is the requested channel already in-use? */

  chndx = channel - chfirst;
  chan  = &tc->channel[chndx];

  if (chan->inuse)
    {
      /* Yes.. return a failure */

      tmrerr("ERROR: Channel %d is in-use\n", channel);
      nxmutex_unlock(&tc->lock);
      return NULL;
    }

  chconfig = &tcconfig->channel[chndx];

  /* Configure channel input/output pins */

  if (chconfig->clkset)
    {
      /* Configure clock input pin */

      sam_configgpio(chconfig->clkset);
    }

  if (chconfig->tioaset)
    {
      /* Configure output A pin */

      sam_configgpio(chconfig->tioaset);
    }

  if (chconfig->tiobset)
    {
      /* Configure output B pin */

      sam_configgpio(chconfig->tiobset);
    }

  /* Enable clocking to the timer counter */

  if (chconfig->pid < 32)
    {
      sam_enableperiph0(chconfig->pid);
    }
  else
    {
      sam_enableperiph1(chconfig->pid);
    }

  /* Set the maximum TC peripheral clock frequency.
   * REVISIT: This is from the SAMA5.  Does it apply here?
   */

  regval  = PMC_PCR_PID(chconfig->pid) | PMC_PCR_CMD | PMC_PCR_EN;
  putreg32(regval, SAM_PMC_PCR);

  /* Attach the timer interrupt handler and enable the timer interrupts */

  irq_attach(chconfig->irq, sam_tc_interrupt, chan);
  up_enable_irq(chconfig->irq);

  /* Mark the channel "inuse" */

  chan->inuse = true;

  /* And return the channel with the semaphore locked */

  sam_regdump(chan, "Initialized");
  return chan;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tc_allocate
 *
 * Description:
 *   Configures a Timer Counter to operate in the given mode.  The timer is
 *   stopped after configuration and must be restarted with sam_tc_start().
 *   All the interrupts of the timer are also disabled.
 *
 * Input Parameters:
 *   channel TC channel number (see TC_CHANx definitions)
 *   mode    Operating mode (TC_CMR value).
 *
 * Returned Value:
 *   On success, a non-NULL handle value is returned.  This handle may be
 *   used with subsequent timer/counter interfaces to manage the timer.  A
 *   NULL handle value is returned on a failure.
 *
 ****************************************************************************/

TC_HANDLE sam_tc_allocate(int channel, int mode)
{
  struct sam_chan_s *chan;

  /* Initialize the timer/counter data (if necessary) and get exclusive
   * access to the requested channel.
   */

  tmrinfo("channel=%d mode=%08x\n", channel, mode);

  chan = sam_tc_initialize(channel);
  if (chan)
    {
      /* Disable TC clock */

      sam_chan_putreg(chan, SAM_TC_CCR_OFFSET, TC_CCR_CLKDIS);

      /* Disable channel interrupts */

      sam_chan_putreg(chan, SAM_TC_IDR_OFFSET, TC_INT_ALL);

      /* Clear and pending status */

      sam_chan_getreg(chan, SAM_TC_SR_OFFSET);

      /* And set the requested mode */

      sam_chan_putreg(chan, SAM_TC_CMR_OFFSET, mode);
      sam_regdump(chan, "Allocated");
      nxmutex_unlock(&chan->tc->lock);
    }

  /* Return an opaque reference to the channel */

  tmrinfo("Returning %p\n", chan);
  return (TC_HANDLE)chan;
}

/****************************************************************************
 * Name: sam_tc_free
 *
 * Description:
 *   Release the handle previously allocated by sam_tc_allocate().
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_free(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  tmrinfo("Freeing %p channel=%d inuse=%d\n", chan, chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

  /* Make sure that interrupts are detached and disabled and that the channel
   * is stopped and disabled.
   */

  sam_tc_detach(handle);
  sam_tc_stop(handle);

  /* Mark the channel as available */

  chan->inuse = false;
}

/****************************************************************************
 * Name: sam_tc_start
 *
 * Description:
 *   Reset and Start the TC Channel.  Enables the timer clock and performs a
 *   software reset to start the counting.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_start(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  tmrinfo("Starting channel %d inuse=%d\n", chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

  /* Read the SR to clear any pending interrupts on this channel */

  sam_chan_getreg(chan, SAM_TC_SR_OFFSET);

  /* Then enable the timer (by setting the CLKEN bit).  Setting SWTRG
   * will also reset the timer counter and starting the timer.
   */

  sam_chan_putreg(chan, SAM_TC_CCR_OFFSET, TC_CCR_CLKEN | TC_CCR_SWTRG);
  sam_regdump(chan, "Started");
}

/****************************************************************************
 * Name: sam_tc_stop
 *
 * Description:
 *   Stop TC Channel.  Disables the timer clock, stopping the counting.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_stop(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  tmrinfo("Stopping channel %d inuse=%d\n", chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

  sam_chan_putreg(chan, SAM_TC_CCR_OFFSET, TC_CCR_CLKDIS);
  sam_regdump(chan, "Stopped");
}

/****************************************************************************
 * Name: sam_tc_attach
 *
 * Description:
 *   Attach or detach an interrupt handler to the timer interrupt.  The
 *   interrupt is detached if the handler argument is NULL.
 *
 * Input Parameters:
 *   handle  The handle that represents the timer state
 *   handler The interrupt handler that will be invoked when the interrupt
 *           condition occurs
 *   arg     An opaque argument that will be provided when the interrupt
 *           handler callback is executed.
 *   mask    The value of the timer interrupt mask register that defines
 *           which interrupts should be enabled.
 *
 * Returned Value:
 *   The old timer channel interrupt handler
 *
 ****************************************************************************/

tc_handler_t sam_tc_attach(TC_HANDLE handle, tc_handler_t handler,
                           void *arg, uint32_t mask)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  tc_handler_t oldhandler;
  irqstate_t flags;

  DEBUGASSERT(chan);

  /* Remember the old interrupt handler and set the new handler */

  flags         = enter_critical_section();
  oldhandler    = chan->handler;
  chan->handler = handler;

  /* Don't enable interrupt if we are detaching no matter what the caller
   * says.
   */

  if (handler == NULL)
    {
      arg  = NULL;
      mask = 0;
    }

  chan->arg = arg;

  /* Now enable interrupt as requested */

  sam_chan_putreg(chan, SAM_TC_IDR_OFFSET, TC_INT_ALL & ~mask);
  sam_chan_putreg(chan, SAM_TC_IER_OFFSET, TC_INT_ALL & mask);
  leave_critical_section(flags);

  return oldhandler;
}

/****************************************************************************
 * Name: sam_tc_getpending
 *
 * Description:
 *   Return the current contents of the interrupt status register, clearing
 *   all pending interrupts.
 *
 * Input Parameters:
 *   handle  The handle that represents the timer state
 *
 * Returned Value:
 *   The value of the channel interrupt status register.
 *
 ****************************************************************************/

uint32_t sam_tc_getpending(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  DEBUGASSERT(chan);
  return sam_chan_getreg(chan, SAM_TC_SR_OFFSET);
}

/****************************************************************************
 * Name: sam_tc_settcclks
 *
 * Description:
 *   Set the value of TCCLKS clock selection in TC_CMR register
 *
 * Input Parameters:
 *   handle  The handle that represents the timer state
 *   tcclks  The clock selection value to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_settcclks(TC_HANDLE handle, uint32_t tcclks)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  uint32_t regval;

  DEBUGASSERT(chan);

  regval  = sam_chan_getreg(chan, SAM_TC_CMR_OFFSET);
  regval &= ~TC_CMR_TCCLKS_MASK;
  regval |= tcclks;
  sam_chan_putreg(chan, SAM_TC_CMR_OFFSET, regval);
}

/****************************************************************************
 * Name: sam_tc_setregister
 *
 * Description:
 *    Set TC_REGA, TC_REGB, or TC_REGC register.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *   regid  One of {TC_REGA, TC_REGB, or TC_REGC}
 *   regval Then value to set in the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_setregister(TC_HANDLE handle, int regid, uint16_t regval)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  DEBUGASSERT(chan && regid < TC_NREGISTERS);

  tmrinfo("Channel %u: Set register RC%d to %04x\n",
          chan->chan, regid, regval);

  sam_chan_putreg(chan, g_regoffset[regid], regval);
  sam_regdump(chan, "Set register");
}

/****************************************************************************
 * Name: sam_tc_getregister
 *
 * Description:
 *    Get the current value of the TC_REGA, TC_REGB, or TC_REGC register.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *   regid  One of {TC_REGA, TC_REGB, or TC_REGC}
 *
 * Returned Value:
 *   The value of the specified register.
 *
 ****************************************************************************/

uint16_t sam_tc_getregister(TC_HANDLE handle, int regid)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  DEBUGASSERT(chan);
  return sam_chan_getreg(chan, g_regoffset[regid]);
}

/****************************************************************************
 * Name: sam_tc_getcounter
 *
 * Description:
 *   Return the current value of the timer counter register
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   The current value of the timer counter register for this channel.
 *
 ****************************************************************************/

uint16_t sam_tc_getcounter(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  DEBUGASSERT(chan);
  return sam_chan_getreg(chan, SAM_TC_CV_OFFSET);
}

/****************************************************************************
 * Name: sam_tc_setblockmode
 *
 * Description:
 *   Set the value of TC_BMR register
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *   regval Then value to set in the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_setblockmode(TC_HANDLE handle, uint32_t regval)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  DEBUGASSERT(chan);
  sam_tc_putreg(chan, SAM_TC_BMR_OFFSET, regval);
}

/****************************************************************************
 * Name: sam_tc_divfreq
 *
 * Description:
 *   Return the divided timer input frequency that is currently driving the
 *   the timer counter.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   The timer counter frequency.
 *
 ****************************************************************************/

uint32_t sam_tc_divfreq(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  uint32_t regval;
  int tcclks;

  DEBUGASSERT(chan);

  /* Get the TC_CMR register contents for this channel and extract the
   * TCCLKS index.
   */

  regval = sam_chan_getreg(chan, SAM_TC_CMR_OFFSET);
  tcclks = (regval & TC_CMR_TCCLKS_MASK) >> TC_CMR_TCCLKS_SHIFT;

  /* And use the TCCLKS index to calculate the timer counter frequency */

  if (tcclks == 0)
    {
      /* The tcclks value of 0 corresponds to PCK6 */

      return sam_pck_frequency(PCK6);
    }
  else
    {
      /* Values of tcclks in the range {1,5} correspond to the divided
       * down MCK or to the slow clock.
       */

      return sam_tc_mckfreq_lookup(BOARD_MCK_FREQUENCY, tcclks - 1);
    }
}

/****************************************************************************
 * Name: sam_tc_clockselect
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (Ftcin / (div * 65536)) <= freq <= (Ftcin / div)
 *
 *   where:
 *     freq  - the desired frequency
 *     Ftcin - The timer/counter input frequency
 *     div   - With DIV being the highest possible value.
 *
 * Input Parameters:
 *   frequency  Desired timer frequency.
 *   tcclks     TCCLKS field value for divisor.
 *   actual     The actual frequency of the MCK
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

int sam_tc_clockselect(uint32_t frequency, uint32_t *tcclks,
                       uint32_t *actual)
{
  uint32_t mck_actual = 0;
  uint32_t mck_tcclks = 0;
  uint32_t mck_error;
  int ret;

  /* Try to satisfy the requested frequency with the MCK or slow clock */

  ret = sam_tc_mcksrc(frequency, &mck_tcclks, &mck_actual);
  if (ret < 0)
    {
      mck_error = UINT32_MAX;
    }
  else
    {
      /* Get the absolute value of the frequency error */

      if (mck_actual > frequency)
        {
          mck_error = mck_actual - frequency;
        }
      else
        {
          mck_error = frequency - mck_actual;
        }
    }

  /* See if we do better with PCK6 */

  if (sam_pck_isenabled(PCK6))
    {
      uint32_t pck6_actual;
      uint32_t pck6_error;

      /* Get the absolute value of the frequency error */

      pck6_actual = sam_pck_frequency(PCK6);
      if (pck6_actual > frequency)
        {
          pck6_error = pck6_actual - frequency;
        }
      else
        {
          pck6_error = frequency - pck6_actual;
        }

      /* Return the PCK6 selection if the error is smaller */

      if (pck6_error < mck_error)
        {
          /* Return the PCK selection */

          if (actual)
            {
              tmrinfo("return actual=%lu\n", (unsigned long)pck6_actual);
              *actual = pck6_actual;
            }

          /* Return the TCCLKS selection */

          if (tcclks)
            {
              tmrinfo("return tcclks=%08lx\n",
                      (unsigned long)TC_CMR_TCCLKS_PCK6);
              *tcclks = TC_CMR_TCCLKS_PCK6;
            }

          /* Return success */

          return OK;
        }
    }

  /* Return the MCK/slow clock selection */

  if (actual)
    {
      tmrinfo("return actual=%lu\n", (unsigned long)mck_actual);
      *actual = mck_actual;
    }

  /* Return the TCCLKS selection */

  if (tcclks)
    {
      tmrinfo("return tcclks=%08lx\n", (unsigned long)mck_tcclks);
      *tcclks = mck_tcclks;
    }

  /* Return success */

  return ret;
}

#endif /* CONFIG_SAMV7_TC0 || CONFIG_SAMV7_TC1 || CONFIG_SAMV7_TC2 || CONFIG_SAMV7_TC3 */
