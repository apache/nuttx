/****************************************************************************
 * arch/arm/src/sama5/sam_tc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_periphclks.h"
#include "chip/sam_pinmap.h"
#include "chip/sam_pmc.h"
#include "sam_pio.h"
#include "sam_tc.h"

#if defined(CONFIG_SAMA5_TC0) || defined(CONFIG_SAMA5_TC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking */

#if BOARD_MCK_FREQUENCY <= SAM_TC_MAXPERCLK
#  define TC_FREQUENCY BOARD_MCK_FREQUENCY
#  define TC_PCR_DIV PMC_PCR_DIV1
#elif (BOARD_MCK_FREQUENCY >> 1) <= SAM_TC_MAXPERCLK
#  define TC_FREQUENCY (BOARD_MCK_FREQUENCY >> 1)
#  define TC_PCR_DIV PMC_PCR_DIV2
#elif (BOARD_MCK_FREQUENCY >> 2) <= SAM_TC_MAXPERCLK
#  define TC_FREQUENCY (BOARD_MCK_FREQUENCY >> 2)
#  define TC_PCR_DIV PMC_PCR_DIV4
#elif (BOARD_MCK_FREQUENCY >> 3) <= SAM_TC_MAXPERCLK
#  define TC_FREQUENCY (BOARD_MCK_FREQUENCY >> 3)
#  define TC_PCR_DIV PMC_PCR_DIV8
#else
#  error Cannot realize TC input frequency
#endif

/* Timer debug is enabled if any timer client is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_ANALOG
#  undef CONFIG_SAMA5_TC_REGDEBUG
#endif

#undef DEBUG_TC
#if defined(CONFIG_SAMA5_ADC) && defined(CONFIG_DEBUG_ANALOG)
#  define DEBUG_TC 1
#endif

#ifdef DEBUG_TC
#  define tcdbg    dbg
#  define tcvdbg   vdbg
#else
#  define tcdbg(x...)
#  define tcvdbg(x...)
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
  uint8_t chan;            /* Channel number (0, 1, or 2 OR 3, 4, or 5) */
  bool inuse;              /* True: channel is in use */
};

/* This structure describes on timer/counter */

struct sam_tc_s
{
  sem_t exclsem;           /* Assures mutually exclusive access to TC */
  uintptr_t base;          /* Register base address */
  uint8_t pid;             /* Peripheral ID */
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
#define     sam_givesem(tc) (sem_post(&tc->exclsem))

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

/* Initialization ***********************************************************/

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

/* Timer/counter state */

#ifdef CONFIG_SAMA5_TC0
static struct sam_tc_s g_tc012;
#endif

#ifdef CONFIG_SAMA5_TC1
static struct sam_tc_s g_tc345;
#endif

/* TC frequency data.  This table provides the frequency for each selection of TCCLK */

#define TC_NDIVIDERS 5

/* This is the list of divider values */

static const uint16_t g_divider[TC_NDIVIDERS] =
{
  2,                     /* TIMER_CLOCK1 -> div2 */
  8,                     /* TIMER_CLOCK2 -> div8 */
  32,                    /* TIMER_CLOCK3 -> div32 */
  128,                   /* TIMER_CLOCK4 -> div128 */
  TC_FREQUENCY / 32768   /* TIMER_CLOCK5 -> slow clock (not really a divider) */
};

/* This is the list of divided down frequencies */

static const uint32_t g_divfreq[TC_NDIVIDERS] =
{
  TC_FREQUENCY / 2,      /* TIMER_CLOCK1 -> div2 */
  TC_FREQUENCY / 8,      /* TIMER_CLOCK2 -> div8 */
  TC_FREQUENCY / 32,     /* TIMER_CLOCK3 -> div32 */
  TC_FREQUENCY / 128,    /* TIMER_CLOCK4 -> div128 */
  32768                  /* TIMER_CLOCK5 -> slow clock */
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
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&tc->exclsem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
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
  lldbg("TC%d [%08x]: %s\n", tc->tc, (int)base, msg);
  lldbg("  BMR: %08x QIMR: %08x QISR: %08x WPMR: %08x\n",
        getreg32(base+SAM_TC_BMR_OFFSET), getreg32(base+SAM_TC_QIMR_OFFSET),
        getreg32(base+SAM_TC_QISR_OFFSET), getreg32(base+SAM_TC_WPMR_OFFSET));

  base = chan->base;
  lldbg("TC%d Channel %d [%08x]: %s\n", tc->tc, chan->chan, (int)base, msg);
  lldbg("  CMR: %08x SSMR: %08x  RAB: %08x   CV: %08x\n",
        getreg32(base+SAM_TC_CMR_OFFSET), getreg32(base+SAM_TC_SMMR_OFFSET),
        getreg32(base+SAM_TC_RAB_OFFSET), getreg32(base+SAM_TC_CV_OFFSET));
  lldbg("   RA: %08x   RB: %08x   RC: %08x   SR: %08x\n",
        getreg32(base+SAM_TC_RA_OFFSET), getreg32(base+SAM_TC_RB_OFFSET),
        getreg32(base+SAM_TC_RC_OFFSET), getreg32(base+SAM_TC_SR_OFFSET));
  lldbg("  IMR: %08x\n",
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

          lldbg("...[Repeats %d times]...\n", tc->ntimes);
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
      lldbg("%08x->%08x\n", regaddr, regval);
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
      lldbg("%08x<-%08x\n", regaddr, regval);
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
      lldbg("%08x->%08x\n", regaddr, regval);
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
      lldbg("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Initialization
 ****************************************************************************/
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
  FAR struct sam_tc_s *tc;
  FAR const struct sam_tcconfig_s *tcconfig;
  FAR struct sam_chan_s *chan;
  FAR const struct sam_chconfig_s *chconfig;
  irqstate_t flags;
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
    }
  else
#endif
#ifdef CONFIG_SAMA5_TC1
  if (channel >= 3 && channel < 5)
    {
      tc       = &g_tc345;
      tcconfig = &g_tc345config;
    }
  else
#endif
    {
      /* Timer/counter is not invalid or not enabled */

      tcdbg("ERROR: Bad channel number: %d\n", channel);
      return NULL;
    }

  /* Has the timer counter been initialized.  We have to be careful here
   * because there is no semaphore protection.
   */

  flags = irqsave();
  if (!tc->initialized)
    {
      /* Initialize the timer counter data structure. */

      memset(tc, 0, sizeof(struct sam_tc_s));
      sem_init(&tc->exclsem, 0, 1);
      tc->base = tcconfig->base;
      tc->tc   = channel < 3 ? 0 : 1;
      tc->pid  = tcconfig->pid;

      /* Initialize the channels */

      for (i = 0, ch = tcconfig->chfirst; i < SAM_TC_NCHANNELS; i++)
        {
          tcdbg("Initializing TC%d channel %d\n", tcconfig->tc, ch);

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
        }

      /* Set the maximum TC peripheral clock frequency */

      regval = PMC_PCR_PID(tcconfig->pid) | PMC_PCR_CMD | TC_PCR_DIV | PMC_PCR_EN;
      putreg32(regval, SAM_PMC_PCR);

      /* Enable clocking to the timer counter */

      sam_enableperiph0(tcconfig->pid);

      /* Now the channel is initialized */

      tc->initialized = true;
    }

  /* Get exclusive access to the timer/count data structure */

  sam_takesem(tc);
  irqrestore(flags);

  /* Get the requested channel structure */

  chan = &tc->channel[channel - tcconfig->chfirst];

  /* Is it available? */

  if (chan->inuse)
    {
      /* No.. return a failure */

      tcdbg("Channel %d is in-used\n", channel);
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

  tcvdbg("channel=%d mode=%08x\n", channel, mode);

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
    }

  /* Return an opaque reference to the channel */

  tcvdbg("Returning %p\n", chan);
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

  tcvdbg("Freeing %p channel=%d inuse=%d\n", chan, chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

  /* Make sure that the channel is stopped */

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

  tcvdbg("Starting channel %d inuse=%d\n", chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

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

  tcvdbg("Stopping channel %d inuse=%d\n", chan->chan, chan->inuse);
  DEBUGASSERT(chan && chan->inuse);

  sam_chan_putreg(chan, SAM_TC_CCR_OFFSET, TC_CCR_CLKDIS);
  sam_regdump(chan, "Stopped");
}

/****************************************************************************
 * Name: sam_tc_setregister
 *
 * Description:
 *    Set TC_RA, TC_RB, or TC_RB using the provided divisor.  The actual
 *    setting in the register will be the TC input frequency divided by
 *    the provided divider (which should derive from the divider returned
 *    by sam_tc_divider).
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tc_setregister(TC_HANDLE handle, int reg, unsigned int div)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
  uint32_t regval;

  DEBUGASSERT(reg < TC_NREGISTERS);

  regval = TC_FREQUENCY / div;
  tcvdbg("Channel %d: Set register %d to %d / %d = %d\n",
         chan->chan, reg, TC_FREQUENCY, div, (unsigned int)regval);

  sam_chan_putreg(chan, g_regoffset[reg], regval);
  sam_regdump(chan, "Set register");
}

/****************************************************************************
 * Name: sam_tc_frequency
 *
 * Description:
 *   Return the timer input frequency, that is, the MCK frequency divided
 *   down so that the timer/counter is driven within its maximum frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  The timer input frequency.
 *
 ****************************************************************************/

uint32_t sam_tc_frequency(void)
{
  return TC_FREQUENCY;
}

/****************************************************************************
 * Name: sam_tc_divisor
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (Ftc / (div * 65536)) <= freq <= (Ftc / dev)
 *
 *   where:
 *     freq - the desitred frequency
 *     Ftc  - The timer/counter input frequency
 *     div  - With DIV being the highest possible value.
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
  int ndx = 0;

  tcvdbg("frequency=%d\n", frequency);

  /* Satisfy lower bound */

  while (frequency < (g_divfreq[ndx] >> 16))
    {
      if (++ndx > TC_NDIVIDERS)
        {
          /* If no divisor can be found, return -ERANGE */

          tcdbg("Lower bound search failed\n");
          return -ERANGE;
        }
    }

  /* Try to maximize DIV while still satisfying upper bound */

  for (; ndx < (TC_NDIVIDERS-1); ndx++)
    {
      if (frequency > g_divfreq[ndx + 1])
        {
          break;
        }
    }

  /* Return the divider value */

  if (div)
    {
      tcvdbg("return div=%d\n", g_divider[ndx]);
      *div = g_divider[ndx];
    }

  /* REturn the TCCLKS selection */

  if (tcclks)
    {
      tcvdbg("return tcclks=%d\n", ndx);
      *tcclks = ndx;
    }

  return OK;
}

#endif /* CONFIG_SAMA5_TC0 || CONFIG_SAMA5_TC1 */
