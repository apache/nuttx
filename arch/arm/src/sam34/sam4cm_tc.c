/****************************************************************************
 * arch/arm/src/sam34/sam4cm_tc.c
 *
 *   Copyright (C) 2013-2017 Gregory Nutt. All rights reserved.
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
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
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

#include "sam4cm_tc.h"

#if defined(CONFIG_SAM34_TC0) || defined(CONFIG_SAM34_TC1) || \
    defined(CONFIG_SAM34_TC2) || defined(CONFIG_SAM34_TC3) || \
    defined(CONFIG_SAM34_TC4) || defined(CONFIG_SAM34_TC5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG_TIMER_INFO
#  undef CONFIG_SAM34_TC_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the static configuration of a TC channel */

struct sam_chconfig_s
{
  uintptr_t base;          /* Channel register base address */
  uint8_t pid;             /* Peripheral ID number */
  uint8_t irq;             /* Channel IRQ number */
  uint8_t chan;
  gpio_pinset_t clkset;    /* CLK input PIO configuration */
  gpio_pinset_t tioaset;   /* Output A PIO configuration */
  gpio_pinset_t tiobset;   /* Output B PIO configuration */
};

/* This structure describes one timer counter channel */

struct sam_chan_s
{
  struct sam_tc_s *tc;     /* Parent timer/counter */
  uintptr_t base;          /* Channel register base address */
  uint8_t pid;             /* Peripheral ID number */
  uint8_t irq;             /* Channel IRQ number */
  tc_handler_t handler;    /* Attached interrupt handler */
  void *arg;               /* Interrupt handler argument */
  uint8_t chan;            /* Channel number (0, 1, or 2 OR 3, 4, or 5) */
  sem_t exclsem;           /* Assures mutually exclusive access to TC */
  bool initialized;        /* True: channel data has been initialized */
  bool inuse;              /* True: channel is in use */

  /* Debug stuff */

#ifdef CONFIG_SAM34_TC_REGDEBUG
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

static int  sam_takesem(struct sam_chan_s *chan);
#define     sam_givesem(chan) (nxsem_post(&chan->exclsem))

#ifdef CONFIG_SAM34_TC_REGDEBUG
static void sam_regdump(struct sam_chan_s *chan, const char *msg);
static bool sam_checkreg(struct sam_chan_s *chan, bool wr, uint32_t regaddr,
                         uint32_t regval);
#else
#  define   sam_regdump(chan,msg)
#  define   sam_checkreg(chan,wr,regaddr,regval) (false)
#endif

static inline uint32_t sam_chan_getreg(struct sam_chan_s *chan,
                                       unsigned int offset);
static inline void sam_chan_putreg(struct sam_chan_s *chan,
                                   unsigned int offset, uint32_t regval);

/* Interrupt Handling *******************************************************/

static int sam_tc_interrupt(int irq, void *context, void *arg);

/* Initialization ***********************************************************/

static int sam_tc_freqdiv_lookup(uint32_t ftcin, int ndx);
static uint32_t sam_tc_divfreq_lookup(uint32_t ftcin, int ndx);
static inline struct sam_chan_s *sam_tc_initialize(int channel);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Static timer configuration */

static const struct sam_chconfig_s g_configs[] =
{
#ifdef CONFIG_SAM34_TC0
  {
    .chan   = 0,
    .base   = SAM_TC0_BASE,
    .pid     = SAM_PID_TC0,
    .irq     = SAM_IRQ_TC0,
#ifdef CONFIG_SAM34_TC0_CLK
    .clkset = PIO_TC0_CLK,
#else
    .clkset = 0,
#endif
#ifdef CONFIG_SAM34_TC0_TIOA
    .tioaset = PIO_TC0_IOA,
#else
    .tioaset = 0,
#endif
#ifdef CONFIG_SAM34_TC0_TIOB
    .tiobset = PIO_TC0_IOB,
#else
    .tiobset = 0,
#endif
  },
#endif /* CONFIG_SAM34_TC0 */

#ifdef CONFIG_SAM34_TC1
  {
    .chan    = 1,
    .base    = SAM_TC1_BASE,
    .pid     = SAM_PID_TC1,
    .irq     = SAM_IRQ_TC1,
#ifdef CONFIG_SAM34_TC1_CLK
    .clkset  = PIO_TC1_CLK,
#else
    .clkset  = 0,
#endif
#ifdef CONFIG_SAM34_TC1_TIOA
    .tioaset = PIO_TC1_IOA,
#else
    .tioaset = 0,
#endif
#ifdef CONFIG_SAM34_TC1_TIOB
    .tiobset = PIO_TC1_IOB,
#else
    .tiobset = 0,
#endif
  },
#endif /* CONFIG_SAM34_TC1 */

#ifdef CONFIG_SAM34_TC2
  {
    .chan = 2,
    .base    = SAM_TC2_BASE,
    .pid     = SAM_PID_TC2,
    .irq     = SAM_IRQ_TC2,
#ifdef CONFIG_SAM34_TC2_CLK
    .clkset  = PIO_TC2_CLK,
#else
    .clkset  = 0,
#endif
#ifdef CONFIG_SAM34_TC2_TIOA
    .tioaset = PIO_TC2_IOA,
#else
    .tioaset = 0,
#endif
#ifdef CONFIG_SAM34_TC2_TIOB
    .tiobset = PIO_TC2_IOB,
#else
    .tiobset = 0,
#endif
  },
#endif /* CONFIG_SAM34_TC2 */

#ifdef CONFIG_SAM34_TC3
  {
    .chan    = 3,
    .base    = SAM_TC3_BASE,
    .pid     = SAM_PID_TC3,
    .irq     = SAM_IRQ_TC3,
#ifdef CONFIG_SAM34_TC3_CLK
    .clkset  = PIO_TC3_CLK,
#else
    .clkset  = 0,
#endif
#ifdef CONFIG_SAM34_TC3_TIOA
    .tioaset = PIO_TC3_IOA,
#else
    .tioaset = 0,
#endif
#ifdef CONFIG_SAM34_TC3_TIOB
    .tiobset = PIO_TC3_IOB,
#else
    .tiobset = 0,
#endif
  },
#endif /* CONFIG_SAM34_TC3 */

#ifdef CONFIG_SAM34_TC4
  {
    .chan    = 4,
    .base    = SAM_TC4_BASE,
    .pid     = SAM_PID_TC4,
    .irq     = SAM_IRQ_TC4,
#ifdef CONFIG_SAM34_TC4_CLK
    .clkset  = PIO_TC4_CLK,
#else
    .clkset  = 0,
#endif
#ifdef CONFIG_SAM34_TC4_TIOA
    .tioaset = PIO_TC4_IOA,
#else
    .tioaset = 0,
#endif
#ifdef CONFIG_SAM34_TC4_TIOB
    .tiobset = PIO_TC4_IOB,
#else
    .tiobset = 0,
#endif
  },
#endif /* CONFIG_SAM34_TC4 */

#ifdef CONFIG_SAM34_TC5
  {
    .chan    = 5,
    .base    = SAM_TC5_BASE,
    .pid     = SAM_PID_TC5,
    .irq     = SAM_IRQ_TC5,
#ifdef CONFIG_SAM34_TC5_CLK
    .clkset  = PIO_TC5_CLK,
#else
    .clkset  = 0,
#endif
#ifdef CONFIG_SAM34_TC5_TIOA
    .tioaset = PIO_TC5_IOA,
#else
    .tioaset = 0,
#endif
#ifdef CONFIG_SAM34_TC5_TIOB
    .tiobset = PIO_TC5_IOB,
#else
    .tiobset = 0,
#endif
  },
#endif /* CONFIG_SAM34_TC5 */
};

#define ENABLED_CHANNELS (sizeof(g_configs)/sizeof(struct sam_chconfig_s))

static struct sam_chan_s g_channels[ENABLED_CHANNELS];

/* TC frequency data.
 * This table provides the frequency for each selection of TCCLK
 */

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

static int sam_takesem(struct sam_chan_s *chan)
{
  return nxsem_wait_uninterruptible(&chan->exclsem);
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

#ifdef CONFIG_SAM34_TC_REGDEBUG
static void sam_regdump(struct sam_chan_s *chan, const char *msg)
{
  uintptr_t base;

  base = chan->base;
  tmrinfo("TC%d [%08x]: %s\n", chan->chan, (int)base, msg);
  tmrinfo("  BMR: %08x QIMR: %08x QISR: %08x WPMR: %08x\n",
          getreg32(base + SAM_TC_BMR_OFFSET),
          getreg32(base + SAM_TC_QIMR_OFFSET),
          getreg32(base + SAM_TC_QISR_OFFSET),
          getreg32(base + SAM_TC_WPMR_OFFSET));

  base = chan->base;
  tmrinfo("TC%d Channel %d [%08x]: %s\n",
          chan->chan, chan->chan, (int)base, msg);
  tmrinfo("  CMR: %08x SSMR: %08x  RAB: %08x   CV: %08x\n",
          getreg32(base + SAM_TC_CMR_OFFSET),
          getreg32(base + SAM_TC_SMMR_OFFSET),
          getreg32(base + SAM_TC_RAB_OFFSET),
          getreg32(base + SAM_TC_CV_OFFSET));
  tmrinfo("   RA: %08x   RB: %08x   RC: %08x   SR: %08x\n",
          getreg32(base + SAM_TC_RA_OFFSET),
          getreg32(base + SAM_TC_RB_OFFSET),
          getreg32(base + SAM_TC_RC_OFFSET),
          getreg32(base + SAM_TC_SR_OFFSET));
  tmrinfo("  IMR: %08x\n",
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
 *   chan    - The timer/counter peripheral state
 *   wr      - True:write access false:read access
 *   regval  - The register value associated with the access
 *   regaddr - The address of the register being accessed
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_TC_REGDEBUG
static bool sam_checkreg(struct sam_chan_s *chan, bool wr, uint32_t regaddr,
                         uint32_t regval)
{
  if (wr      == chan->wr &&      /* Same kind of access? */
      regaddr == chan->regaddr && /* Same register address? */
      regval  == chan->regval)    /* Same register value? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      chan->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (chan->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          tmrinfo("...[Repeats %d times]...\n", chan->ntimes);
        }

      /* Save information about the new access */

      chan->wr      = wr;
      chan->regval  = regval;
      chan->regaddr = regaddr;
      chan->ntimes  = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: sam_chan_getreg
 *
 * Description:
 *  Read an TC register
 *
 ****************************************************************************/

static inline uint32_t sam_chan_getreg(struct sam_chan_s *chan,
                                       unsigned int offset)
{
  uint32_t regaddr = chan->base + offset;
  uint32_t regval  = getreg32(regaddr);

#ifdef CONFIG_SAM34_TC_REGDEBUG
  if (sam_checkreg(chan, false, regaddr, regval))
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
 *  Write a value to an TC register
 *
 ****************************************************************************/

static inline void sam_chan_putreg(struct sam_chan_s *chan,
                                   unsigned int offset, uint32_t regval)
{
  uint32_t regaddr = chan->base + offset;

#ifdef CONFIG_SAM34_TC_REGDEBUG
  if (sam_checkreg(chan, true, regaddr, regval))
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
 *   On successful return, the caller holds the tc exclusive access
 *   semaphore.
 *
 ****************************************************************************/

static int sam_tc_interrupt(int irq, void *context, void *arg)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)arg;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;

  /* Process interrupts */

  DEBUGASSERT(chan != NULL);

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

  return OK;
}

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

#ifdef SAM34_HAVE_PMC_PCR_DIV
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
 *  Given the TC input frequency (Ftcin) and a divider index, return the
 *  value of the Ftcin divider.
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
 *   On successful return, the caller holds the tc exclusive access
 *   semaphore.
 *
 ****************************************************************************/

static inline struct sam_chan_s *sam_tc_initialize(int channel)
{
  struct sam_chan_s *chan;
  const struct sam_chconfig_s *chconfig;
  irqstate_t flags;
  int i;

  /* Select the timer/counter and get the index associated with the
   * channel.
   */

  chan = NULL;
  for (i = 0; i < ENABLED_CHANNELS; i++)
    {
      if (g_configs[i].chan == channel)
        {
          chan = &g_channels[i];
          chconfig = &g_configs[i];
          break;
        }
    }

  if (!chan)
    {
      /* Timer/counter is not invalid or not enabled */

      tmrerr("ERROR: Bad channel number: %d\n", channel);
      return NULL;
    }

  /* Has the timer counter been initialized.  We have to be careful here
   * because there is no semaphore protection.
   */

  flags = enter_critical_section();
  if (!chan->initialized)
    {
      /* Initialize the channel. */

      tmrerr("ERROR: Initializing TC%d\n", chconfig->chan);

      memset(chan, 0, sizeof(struct sam_chan_s));
      nxsem_init(&chan->exclsem, 0, 1);
      chan->base = chconfig->base;
      chan->pid  = chconfig->pid;
      chan->irq  = chconfig->irq;

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

      /* Disable and clear all channel interrupts */

      sam_chan_putreg(chan, SAM_TC_IDR_OFFSET, TC_INT_ALL);
      sam_chan_getreg(chan, SAM_TC_SR_OFFSET);

      /* Enable clocking to the timer counter */

      sam_enableperiph0(chan->pid);

      /* Attach the timer interrupt handler and enable the timer interrupts */

      irq_attach(chan->irq, sam_tc_interrupt, chan);
      up_enable_irq(chan->irq);

      /* Now the channel is initialized */

      chan->initialized = true;
    }

  /* Get exclusive access to the timer/count data structure */

  ret = sam_takesem(chan);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  leave_critical_section(flags);

  /* Is it available? */

  if (chan->inuse)
    {
      /* No.. return a failure */

      tmrerr("ERROR: Channel %d is in-use\n", channel);
      sam_givesem(chan);
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

      sam_chan_getreg(chan, SAM_TC_SR_OFFSET);

      /* And set the requested mode */

      sam_chan_putreg(chan, SAM_TC_CMR_OFFSET, mode);
      sam_regdump(chan, "Allocated");
      sam_givesem(chan);
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
 *
 ****************************************************************************/

void sam_tc_start(TC_HANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  tmrinfo("Starting channel %d inuse=%d\n", chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

  /* Read the SR to clear any pending interrupts on this channel */

  sam_chan_getreg(chan, SAM_TC_SR_OFFSET);

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
  return BOARD_MCK_FREQUENCY;
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

  for (; ndx < (TC_NDIVOPTIONS - 1); ndx++)
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

#endif
