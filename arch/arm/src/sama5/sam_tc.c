/****************************************************************************
 * arch/arm/src/sama5/sam_tc.c
 *
 *   Copyright (C) 2013-2014, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_periphclks.h"
#include "chip/sam_pinmap.h"
#include "chip/sam_pmc.h"
#include "sam_pio.h"
#include "sam_tc.h"

#if defined(CONFIG_SAMA5_TC0) || defined(CONFIG_SAMA5_TC1) || \
    defined(CONFIG_SAMA5_TC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG_TIMER_INFO
#  undef CONFIG_SAMA5_TC_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure describes the static configuration of a TC channel */

struct sam_chconfig_s
{
  uintptr_t base;          /* Channel register base address */
  pio_pinset_t clkset;     /* CLK input PIO configuration */
  pio_pinset_t tioaset;    /* Output A PIO configuration */
  pio_pinset_t tiobset;    /* Output B PIO configuration */
};

/* This structure describes the static configuration of a TC */

struct sam_tcconfig_s
{
  uintptr_t base;          /* TC register base address */
  uint8_t pid;             /* Peripheral ID */
  uint8_t chfirst;         /* First channel number */
  uint8_t tc;              /* Timer/counter number */

  /* Channels */

  struct sam_chconfig_s channel[3];
};

/* This structure describes one timer counter channel */

struct sam_tc_s;
struct sam_chan_s
{
  struct sam_tc_s *tc;     /* Parent timer/counter */
  uintptr_t base;          /* Channel register base address */
  tc_handler_t handler;    /* Attached interrupt handler */
  void *arg;               /* Interrupt handler argument */
  uint8_t chan;            /* Channel number (0, 1, or 2 OR 3, 4, or 5) */
  bool inuse;              /* True: channel is in use */
};

/* This structure describes one timer/counter */

struct sam_tc_s
{
  sem_t exclsem;           /* Assures mutually exclusive access to TC */
  uintptr_t base;          /* Register base address */
  uint8_t pid;             /* Peripheral ID/irq number */
  uint8_t tc;              /* Timer/channel number (0 or 1) */
  bool initialized;        /* True: Timer data has been initialized */

  /* Channels */

  struct sam_chan_s channel[3];

  /* Debug stuff */

#ifdef CONFIG_SAMA5_TC_REGDEBUG
   bool wr;                /* True:Last was a write */
   uint32_t regaddr;       /* Last address */
   uint32_t regval;        /* Last value */
   int ntimes;             /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void sam_takesem(struct sam_tc_s *tc);
#define     sam_givesem(tc) (nxsem_post(&tc->exclsem))

#ifdef CONFIG_SAMA5_TC_REGDEBUG
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

static int sam_tc_interrupt(struct sam_tc_s *tc);
#ifdef CONFIG_SAMA5_TC0
static int sam_tc012_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_SAMA5_TC1
static int sam_tc345_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_SAMA5_TC2
static int sam_tc678_interrupt(int irq, void *context, FAR void *arg);
#endif

/* Initialization ***********************************************************/

#ifdef SAMA5_HAVE_PMC_PCR_DIV
static int sam_tc_mckdivider(uint32_t mck);
#endif
static int sam_tc_freqdiv_lookup(uint32_t ftcin, int ndx);
static uint32_t sam_tc_divfreq_lookup(uint32_t ftcin, int ndx);
static inline struct sam_chan_s *sam_tc_initialize(int channel);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Static timer configuration */

#ifdef CONFIG_SAMA5_TC0
static const struct sam_tcconfig_s g_tc012config =
{
  .base    = SAM_TC012_VBASE,
  .pid     = SAM_PID_TC0,
  .chfirst = 0,
  .tc      = 0,
  .channel =
  {
    [0] =
    {
      .base   = SAM_TC012_CHAN_BASE(0),
#ifdef CONFIG_SAMA5_TC0_CLK0
      .clkset = PIO_TC0_CLK,
#else
      .clkset = 0,
#endif
#ifdef CONFIG_SAMA5_TC0_TIOA0
      .tioaset = PIO_TC0_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC0_TIOB0
      .tiobset = PIO_TC0_IOB,
#else
      .tiobset = 0,
#endif
    },
    [1] =
    {
      .base    = SAM_TC012_CHAN_BASE(1),
#ifdef CONFIG_SAMA5_TC0_CLK1
      .clkset  = PIO_TC1_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC0_TIOA1
      .tioaset = PIO_TC1_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC0_TIOB1
      .tiobset = PIO_TC1_IOB,
#else
      .tiobset = 0,
#endif
    },
    [2] =
    {
      .base    = SAM_TC012_CHAN_BASE(2),
#ifdef CONFIG_SAMA5_TC0_CLK2
      .clkset  = PIO_TC2_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC0_TIOA2
      .tioaset = PIO_TC2_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC0_TIOB2
      .tiobset = PIO_TC2_IOB,
#else
      .tiobset = 0,
#endif
    },
  },
};
#endif

#ifdef CONFIG_SAMA5_TC1
static const struct sam_tcconfig_s g_tc345config =
{
  .base    = SAM_TC345_VBASE,
  .pid     = SAM_PID_TC1,
  .chfirst = 3,
  .tc      = 1,
  .channel =
  {
    [0] =
    {
      .base    = SAM_TC345_CHAN_BASE(3),
#ifdef CONFIG_SAMA5_TC1_CLK3
      .clkset  = PIO_TC3_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC1_TIOA3
      .tioaset = PIO_TC3_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC1_TIOB3
      .tiobset = PIO_TC3_IOB,
#else
      .tiobset = 0,
#endif
    },
    [1] =
    {
      .base    = SAM_TC345_CHAN_BASE(4),
#ifdef CONFIG_SAMA5_TC1_CLK4
      .clkset  = PIO_TC4_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC1_TIOA4
      .tioaset = PIO_TC4_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC1_TIOB4
      .tiobset = PIO_TC4_IOB,
#else
      .tiobset = 0,
#endif
    },
    [2] =
    {
      .base    = SAM_TC345_CHAN_BASE(5),
#ifdef CONFIG_SAMA5_TC1_CLK5
      .clkset  = PIO_TC5_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC1_TIOA5
      .tioaset = PIO_TC5_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC1_TIOB5
      .tiobset = PIO_TC5_IOB,
#else
      .tiobset = 0,
#endif
    },
  },
};
#endif

#ifdef CONFIG_SAMA5_TC2
static const struct sam_tcconfig_s g_tc678config =
{
  .base    = SAM_TC678_VBASE,
  .pid     = SAM_PID_TC2,
  .chfirst = 6,
  .tc      = 2,
  .channel =
  {
    [0] =
    {
      .base    = SAM_TC678_CHAN_BASE(6),
#ifdef CONFIG_SAMA5_TC2_CLK6
      .clkset  = PIO_TC6_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC2_TIOA6
      .tioaset = PIO_TC6_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC2_TIOB6
      .tiobset = PIO_TC6_IOB,
#else
      .tiobset = 0,
#endif
    },
    [1] =
    {
      .base    = SAM_TC678_CHAN_BASE(7),
#ifdef CONFIG_SAMA5_TC2_CLK7
      .clkset  = PIO_TC7_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC2_TIOA7
      .tioaset = PIO_TC7_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC2_TIOB7
      .tiobset = PIO_TC7_IOB,
#else
      .tiobset = 0,
#endif
    },
    [2] =
    {
      .base    = SAM_TC345_CHAN_BASE(8),
#ifdef CONFIG_SAMA5_TC2_CLK8
      .clkset  = PIO_TC8_CLK,
#else
      .clkset  = 0,
#endif
#ifdef CONFIG_SAMA5_TC2_TIOA8
      .tioaset = PIO_TC8_IOA,
#else
      .tioaset = 0,
#endif
#ifdef CONFIG_SAMA5_TC2_TIOB8
      .tiobset = PIO_TC8_IOB,
#else
      .tiobset = 0,
#endif
    },
  },
};
#endif

/* Timer/counter state */

#ifdef CONFIG_SAMA5_TC0
static struct sam_tc_s g_tc012;
#endif

#ifdef CONFIG_SAMA5_TC1
static struct sam_tc_s g_tc345;
#endif

#ifdef CONFIG_SAMA5_TC2
static struct sam_tc_s g_tc678;
#endif

/* TC frequency data.  This table provides the frequency for each selection of TCCLK */

#define TC_NDIVIDERS   4
#define TC_NDIVOPTIONS 5

/* This is the list of divider values: divider = (1 << value) */

static const uint8_t g_log2divider[TC_NDIVIDERS] =
{
  1,                     /* TIMER_CLOCK1 -> div2 */
  3,                     /* TIMER_CLOCK2 -> div8 */
  5,                     /* TIMER_CLOCK3 -> div32 */
  7                      /* TIMER_CLOCK4 -> div128 */
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
 * Name: sam_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_takesem(struct sam_tc_s *tc)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&tc->exclsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Name: sam_regdump
 *
 * Description:
 *   Dump all timer/counter channel and global registers
 *
 * Input Parameters:
 *   chan  - The timer/counter channel state
 *   msg   - Message to print with the data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_TC_REGDEBUG
static void sam_regdump(struct sam_chan_s *chan, const char *msg)
{
  struct sam_tc_s *tc = chan->tc;
  uintptr_t base;

  base = tc->base;
  tminfo("TC%d [%08x]: %s\n", tc->tc, (int)base, msg);
  tminfo("  BMR: %08x QIMR: %08x QISR: %08x WPMR: %08x\n",
        getreg32(base+SAM_TC_BMR_OFFSET), getreg32(base+SAM_TC_QIMR_OFFSET),
        getreg32(base+SAM_TC_QISR_OFFSET), getreg32(base+SAM_TC_WPMR_OFFSET));

  base = chan->base;
  tminfo("TC%d Channel %d [%08x]: %s\n", tc->tc, chan->chan, (int)base, msg);
  tminfo("  CMR: %08x SSMR: %08x  RAB: %08x   CV: %08x\n",
        getreg32(base+SAM_TC_CMR_OFFSET), getreg32(base+SAM_TC_SMMR_OFFSET),
        getreg32(base+SAM_TC_RAB_OFFSET), getreg32(base+SAM_TC_CV_OFFSET));
  tminfo("   RA: %08x   RB: %08x   RC: %08x   SR: %08x\n",
        getreg32(base+SAM_TC_RA_OFFSET), getreg32(base+SAM_TC_RB_OFFSET),
        getreg32(base+SAM_TC_RC_OFFSET), getreg32(base+SAM_TC_SR_OFFSET));
  tminfo("  IMR: %08x\n",
        getreg32(base+SAM_TC_IMR_OFFSET));
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
 *   regval  - The regiser value associated with the access
 *   regaddr - The address of the register being accessed
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_TC_REGDEBUG
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
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint32_t sam_tc_getreg(struct sam_chan_s *chan,
                                     unsigned int offset)
{
  struct sam_tc_s *tc = chan->tc;
  uint32_t regaddr    = tc->base + offset;
  uint32_t regval     = getreg32(regaddr);

#ifdef CONFIG_SAMA5_TC_REGDEBUG
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
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void sam_tc_putreg(struct sam_chan_s *chan, uint32_t regval,
                                 unsigned int offset)
{
  struct sam_tc_s *tc = chan->tc;
  uint32_t regaddr    = tc->base + offset;

#ifdef CONFIG_SAMA5_TC_REGDEBUG
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
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint32_t sam_chan_getreg(struct sam_chan_s *chan,
                                       unsigned int offset)
{
  uint32_t regaddr = chan->base + offset;
  uint32_t regval  = getreg32(regaddr);

#ifdef CONFIG_SAMA5_TC_REGDEBUG
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
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void sam_chan_putreg(struct sam_chan_s *chan, unsigned int offset,
                                   uint32_t regval)
{
  uint32_t regaddr = chan->base + offset;

#ifdef CONFIG_SAMA5_TC_REGDEBUG
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
 *   tc   Timer status instance
 *
 * Returned Value:
 *   A pointer to the initialized timer channel structure associated with tc
 *   and channel.  NULL is returned on any failure.
 *
 *   On successful return, the caller holds the tc exclusive access semaphore.
 *
 ****************************************************************************/

static int sam_tc_interrupt(struct sam_tc_s *tc)
{
  struct sam_chan_s *chan;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;
  int i;

  /* Process interrupts on each channel */

  for (i = 0; i < 3; i++)
    {
      /* Get the handy channel reference */

      chan = &tc->channel[i];

      /* Get the interrupt status for this channel */

      sr      = sam_chan_getreg(chan, SAM_TC_SR_OFFSET);
      imr     = sam_chan_getreg(chan, SAM_TC_IMR_OFFSET);
      pending = sr & imr;

      /* Are there any pending interrupts for this channel? */

      if (pending)
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
    }

  return OK;
}

/****************************************************************************
 * Name: sam_tcABC_interrupt
 *
 * Description:
 *  Timer block interrupt handlers
 *
 * Input Parameters:
 *   chan TC channel structure
 *   sr   The status register value that generated the interrupt
 *
 * Returned Value:
 *   A pointer to the initialized timer channel structure associated with tc
 *   and channel.  NULL is returned on any failure.
 *
 *   On successful return, the caller holds the tc exclusive access semaphore.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_TC0
static int sam_tc012_interrupt(int irq, void *context, void *arg)
{
  return sam_tc_interrupt(&g_tc012);
}
#endif

#ifdef CONFIG_SAMA5_TC1
static int sam_tc345_interrupt(int irq, void *context, FAR void *arg)
{
  return sam_tc_interrupt(&g_tc345);
}
#endif

#ifdef CONFIG_SAMA5_TC2
static int sam_tc678_interrupt(int irq, void *context, FAR void *arg)
{
  return sam_tc_interrupt(&g_tc678);
}
#endif

/****************************************************************************
 * Initialization
 ****************************************************************************/
/****************************************************************************
 * Name: sam_tc_mckdivider
 *
 * Description:
 *  Return the TC clock input divider value.  One of n=0..3 corresponding
 *  to divider values of {1, 2, 4, 8}.
 *
 *  NOTE: The SAMA5D4 has no clock input divider
 *
 * Input Parameters:
 *   mck - The MCK frequency to be divider.
 *
 * Returned Value:
 *   Log2 of the TC clock divider.
 *
 ****************************************************************************/

#ifdef SAMA5_HAVE_PMC_PCR_DIV
static int sam_tc_mckdivider(uint32_t mck)
{
  if (mck <= SAM_TC_MAXPERCLK)
    {
      return 0;
    }
  else if ((mck >> 1) <= SAM_TC_MAXPERCLK)
    {
      return 1;
    }
  else if ((mck >> 2) <= SAM_TC_MAXPERCLK)
    {
      return 2;
    }
  else /* if ((mck >> 3) <= SAM_TC_MAXPERCLK) */
    {
      DEBUGASSERT((mck >> 3) <= SAM_TC_MAXPERCLK);
      return 3;
    }
}
#endif

/****************************************************************************
 * Name: sam_tc_freqdiv_lookup
 *
 * Description:
 *  Given the TC input frequency (Ftcin) and a divider index, return the value of
 *  the Ftcin divider.
 *
 * Input Parameters:
 *   ftcin - TC input frequency
 *   ndx   - Divider index
 *
 * Returned Value:
 *   The Ftcin input divider value
 *
 ****************************************************************************/

static int sam_tc_freqdiv_lookup(uint32_t ftcin, int ndx)
{
  /* The final option is to use the SLOW clock */

  if (ndx >= TC_NDIVIDERS)
    {
      /* Not really a divider.  In this case, the board is actually driven
       * by the 32.768KHz slow clock.  This returns a value that looks like
       * correct divider if MCK were the input.
       */

      return ftcin / BOARD_SLOWCLK_FREQUENCY;
    }
  else
    {
      return 1 << g_log2divider[ndx];
    }
}

/****************************************************************************
 * Name: sam_tc_divfreq_lookup
 *
 * Description:
 *  Given the TC input frequency (Ftcin) and a divider index, return the
 *  value of the divided frequency
 *
 * Input Parameters:
 *   ftcin - TC input frequency
 *   ndx   - Divider index
 *
 * Returned Value:
 *   The divided frequency value
 *
 ****************************************************************************/

static uint32_t sam_tc_divfreq_lookup(uint32_t ftcin, int ndx)
{
  /* The final option is to use the SLOW clock */

  if (ndx >= TC_NDIVIDERS)
    {
      return BOARD_SLOWCLK_FREQUENCY;
    }
  else
    {
      return ftcin >> g_log2divider[ndx];
    }
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
 *   On successful return, the caller holds the tc exclusive access semaphore.
 *
 ****************************************************************************/

static inline struct sam_chan_s *sam_tc_initialize(int channel)
{
  struct sam_tc_s *tc;
  const struct sam_tcconfig_s *tcconfig;
  struct sam_chan_s *chan;
  const struct sam_chconfig_s *chconfig;
  irqstate_t flags;
  xcpt_t handler;
  uint32_t regval;
  uint8_t ch;
  int i;

  /* Select the timer/counter and get the index associated with the
   * channel.
   */

#ifdef CONFIG_SAMA5_TC0
  if (channel >= 0 && channel < 3)
    {
      tc       = &g_tc012;
      tcconfig = &g_tc012config;
      handler  = sam_tc012_interrupt;
    }
  else
#endif
#ifdef CONFIG_SAMA5_TC1
  if (channel >= 3 && channel < 6)
    {
      tc       = &g_tc345;
      tcconfig = &g_tc345config;
      handler  = sam_tc345_interrupt;
    }
  else
#endif
#ifdef CONFIG_SAMA5_TC2
  if (channel >= 6 && channel < 9)
    {
      tc       = &g_tc678;
      tcconfig = &g_tc678config;
      handler  = sam_tc678_interrupt;
    }
  else
#endif
    {
      /* Timer/counter is not invalid or not enabled */

      tmrerr("ERROR: Bad channel number: %d\n", channel);
      return NULL;
    }

  /* Has the timer counter been initialized.  We have to be careful here
   * because there is no semaphore protection.
   */

  flags = enter_critical_section();
  if (!tc->initialized)
    {
      /* Initialize the timer counter data structure. */

      memset(tc, 0, sizeof(struct sam_tc_s));
      nxsem_init(&tc->exclsem, 0, 1);
      tc->base = tcconfig->base;
      tc->tc   = channel < 3 ? 0 : 1;
      tc->pid  = tcconfig->pid;

      /* Initialize the channels */

      for (i = 0, ch = tcconfig->chfirst; i < SAM_TC_NCHANNELS; i++)
        {
          tmrerr("ERROR: Initializing TC%d channel %d\n", tcconfig->tc, ch);

          /* Initialize the channel data structure */

          chan       = &tc->channel[i];
          chconfig   = &tcconfig->channel[i];

          chan->tc   = tc;
          chan->base = chconfig->base;
          chan->chan = ch++;

          /* Configure channel input/output pins */

          if (chconfig->clkset)
            {
              /* Configure clock input pin */

              sam_configpio(chconfig->clkset);
            }

          if (chconfig->tioaset)
            {
              /* Configure output A pin */

              sam_configpio(chconfig->tioaset);
            }

          if (chconfig->tiobset)
            {
              /* Configure output B pin */

              sam_configpio(chconfig->tiobset);
            }

          /* Disable and clear all channel interrupts */

          sam_chan_putreg(chan, SAM_TC_IDR_OFFSET, TC_INT_ALL);
          (void)sam_chan_getreg(chan, SAM_TC_SR_OFFSET);
        }

      /* Set the maximum TC peripheral clock frequency */

      regval  = PMC_PCR_PID(tcconfig->pid) | PMC_PCR_CMD | PMC_PCR_EN;

#ifdef SAMA5_HAVE_PMC_PCR_DIV
      /* Set the MCK divider (if any) */

      regval |= PMC_PCR_DIV(sam_tc_mckdivider(BOARD_MCK_FREQUENCY));
#endif

      putreg32(regval, SAM_PMC_PCR);

      /* Enable clocking to the timer counter */

      sam_enableperiph0(tcconfig->pid);

      /* Attach the timer interrupt handler and enable the timer interrupts */

      (void)irq_attach(tc->pid, handler, NULL);
      up_enable_irq(tc->pid);

      /* Now the channel is initialized */

      tc->initialized = true;
    }

  /* Get exclusive access to the timer/count data structure */

  sam_takesem(tc);
  leave_critical_section(flags);

  /* Get the requested channel structure */

  chan = &tc->channel[channel - tcconfig->chfirst];

  /* Is it available? */

  if (chan->inuse)
    {
      /* No.. return a failure */

      tmrerr("ERROR: Channel %d is in-use\n", channel);
      sam_givesem(tc);
      return NULL;
    }

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

      (void)sam_chan_getreg(chan, SAM_TC_SR_OFFSET);

      /* And set the requested mode */

      sam_chan_putreg(chan, SAM_TC_CMR_OFFSET, mode);
      sam_regdump(chan, "Allocated");
      sam_givesem(chan->tc);
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

  sam_tc_attach(handle, NULL, NULL, 0);
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
 *
 ****************************************************************************/

void sam_tc_start(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  tmrinfo("Starting channel %d inuse=%d\n", chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

  /* Read the SR to clear any pending interrupts on this channel */

  (void)sam_chan_getreg(chan, SAM_TC_SR_OFFSET);

  /* Then enable the timer (by setting the CLKEN bit).  Setting SWTRIG
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
 *           which interrupts should be disabled.
 *
 * Returned Value:
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

  if (!handler)
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

void sam_tc_setregister(TC_HANDLE handle, int regid, uint32_t regval)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  DEBUGASSERT(chan && regid < TC_NREGISTERS);

  tmrinfo("Channel %d: Set register RC%d to %08lx\n",
          chan->chan, regid, (unsigned long)regval);

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

uint32_t sam_tc_getregister(TC_HANDLE handle, int regid)
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
 *  The current value of the timer counter register for this channel.
 *
 ****************************************************************************/

uint32_t sam_tc_getcounter(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  DEBUGASSERT(chan);
  return sam_chan_getreg(chan, SAM_TC_CV_OFFSET);
}

/****************************************************************************
 * Name: sam_tc_infreq
 *
 * Description:
 *   Return the timer input frequency (Ftcin), that is, the MCK frequency
 *   divided down so that the timer/counter is driven within its maximum
 *   frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  The timer input frequency.
 *
 ****************************************************************************/

uint32_t sam_tc_infreq(void)
{
#ifdef SAMA5_HAVE_PMC_PCR_DIV
  uint32_t mck = BOARD_MCK_FREQUENCY;
  int shift = sam_tc_mckdivider(mck);
  return mck >> shift;
#else
  return BOARD_MCK_FREQUENCY;
#endif
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
 *  The timer counter frequency.
 *
 ****************************************************************************/

uint32_t sam_tc_divfreq(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  uint32_t ftcin = sam_tc_infreq();
  uint32_t regval;
  int tcclks;

  DEBUGASSERT(chan);

  /* Get the TC_CMR register contents for this channel and extract the
   * TCCLKS index.
   */

  regval = sam_chan_getreg(chan, SAM_TC_CMR_OFFSET);
  tcclks = (regval & TC_CMR_TCCLKS_MASK) >> TC_CMR_TCCLKS_SHIFT;

  /* And use the TCCLKS index to calculate the timer counter frequency */

  return sam_tc_divfreq_lookup(ftcin, tcclks);
}

/****************************************************************************
 * Name: sam_tc_divisor
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (Ftcin / (div * 65536)) <= freq <= (Ftcin / dev)
 *
 *   where:
 *     freq  - the desired frequency
 *     Ftcin - The timer/counter input frequency
 *     div   - With DIV being the highest possible value.
 *
 * Input Parameters:
 *   frequency  Desired timer frequency.
 *   div        Divisor value.
 *   tcclks     TCCLKS field value for divisor.
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

int sam_tc_divisor(uint32_t frequency, uint32_t *div, uint32_t *tcclks)
{
  uint32_t ftcin = sam_tc_infreq();
  int ndx = 0;

  tmrinfo("frequency=%d\n", frequency);

  /* Satisfy lower bound.  That is, the value of the divider such that:
   *
   *   frequency >= (tc_input_frequency * 65536) / divider.
   */

  while (frequency < (sam_tc_divfreq_lookup(ftcin, ndx) >> 16))
    {
      if (++ndx > TC_NDIVOPTIONS)
        {
          /* If no divisor can be found, return -ERANGE */

          tmrerr("ERROR: Lower bound search failed\n");
          return -ERANGE;
        }
    }

  /* Try to maximize DIV while still satisfying upper bound.  That the
   * value of the divider such that:
   *
   *   frequency < tc_input_frequency / divider.
   */

  for (; ndx < (TC_NDIVOPTIONS-1); ndx++)
    {
      if (frequency > sam_tc_divfreq_lookup(ftcin, ndx + 1))
        {
          break;
        }
    }

  /* Return the divider value */

  if (div)
    {
      uint32_t value = sam_tc_freqdiv_lookup(ftcin, ndx);
      tmrinfo("return div=%lu\n", (unsigned long)value);
      *div = value;
    }

  /* Return the TCCLKS selection */

  if (tcclks)
    {
      tmrinfo("return tcclks=%08lx\n", (unsigned long)TC_CMR_TCCLKS(ndx));
      *tcclks = TC_CMR_TCCLKS(ndx);
    }

  return OK;
}

#endif /* CONFIG_SAMA5_TC0 || CONFIG_SAMA5_TC1 || CONFIG_SAMA5_TC2 */
