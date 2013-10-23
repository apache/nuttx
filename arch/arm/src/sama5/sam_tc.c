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
 * The Atmel sample code has a BSD compatibile license that requires this
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

#include "up_arch.h"
#include "sam_periphclks.h"
#include "sam_tc.h"

#if defined(CONFIG_SAMA5_TC0) || defined(CONFIG_SAMA5_TC1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

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
  bool initialized;        /* True: Timer data has been initialized */

  /* Channels */

  struct sam_chan_s channel[3];

  /* Debug stuff */

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
   bool wrlast;            /* Last was a write */
   uint32_t addrlast;      /* Last address */
   uint32_t vallast;       /* Last value */
   int ntimes;             /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void sam_takesem(struct sam_tc_s *tc);
#define     sam_givesem(tc) (sem_post(&tc->exclsem))

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
static bool sam_checkreg(struct sam_tc_s *tc, bool wr,
                         uint32_t value, uint32_t regaddr, uint32_t regval);
#else
# define    sam_checkreg(tc,wr,value,regaddr) (false)
#endif

static inline uint32_t sam_tc_getreg(struct sam_chan_s *chan,
                                     unsigned int offset);
static inline void sam_tc_putreg(struct sam_chan_s *chan,
                                 unsigned int offset, uint32_t value);

static inline uint32_t sam_chan_getreg(struct sam_chan_s *chan,
                                       unsigned int offset);
static inline void sam_chan_putreg(struct sam_chan_s *chan,
                                   unsigned int offset, uint32_t value);

/* Initialization ***********************************************************/

static inline struct sam_chan_s *sam_tc_initialize(int channel);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMA5_TC0
static struct sam_tc_s g_tc012;
#endif
#ifdef CONFIG_SAMA5_TC1
static struct sam_tc_s g_tc345;
#endif

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
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
static bool sam_checkreg(struct sam_tc_s *tc, bool wr, uint32_t regaddr,
                         uint32_t value)
{
  if (wr      == tc->wrlast &&   /* Same kind of access? */
      value   == tc->vallast &&  /* Same value? */
      regaddr == tc->addrlast)   /* Same regaddr? */
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

      tc->wrlast   = wr;
      tc->vallast  = value;
      tc->addrlast = regaddr;
      tc->ntimes   = 0;
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

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
  if (sam_checkreg(tc, false, regval, regaddr))
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

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
  if (sam_checkreg(tc, true, regval, regaddr))
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

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
  if (sam_checkreg(chan->tc, false, regval, regaddr))
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

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
  if (sam_checkreg(chan->tc, true, regval, regaddr))
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
  static struct sam_tc_s *tc;
  static struct sam_chan_s *chan;
  irqstate_t flags;
  uintptr_t tcbase;
  uintptr_t chbase;
  int chfirst;
  int chndx;
  int pid;
  int i;

  /* Select the timer/counter and get the index associated with the
   * channel.
   */
 
#ifdef CONFIG_SAMA5_TC0
  if (channel >= 0 && channel < 3)
    {
      tc      = &g_tc012;
      chndx   = channel;

      /* These are only needed in the case where we need to initialize the
       * timer/counter.
       */

      chfirst = 0;
      tcbase  = SAM_TC012_VBASE;
      chbase  = SAM_TC012_CHAN_BASE(channel);
      pid     = SAM_PID_TC0;
    }
  else
#endif
#ifdef CONFIG_SAMA5_TC1
  if (channel >= 3 && channel < 5)
    {
      tc      = &g_tc345;
      chndx   = channel - 3;

      /* These are only needed in the case where we need to initialize the
       * timer/counter.
       */

      chfirst = 3;
      tcbase  = SAM_TC345_VBASE;
      chbase  = SAM_TC345_CHAN_BASE(channel)
      pid     = SAM_PID_TC0;
    }
  else
#endif
    {
      /* Timer/counter is not invalid or not enabled */

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
      tc->base = tcbase;
      tc->pid  = pid;

      /* Initialize the channels */

      for (i = 0; i < 3; i++)
        {
          chan       = &tc->channel[i];
          chan->base = chbase;
          chan->chan = chfirst++;
        }

      /* Enable clocking to the timer counter */

      sam_enableperiph0(pid);

      /* Now the channel is initialized */

      tc->initialized = true;
    }

  /* Get exclusive access to the timer/count data structure */

  sam_takesem(tc);
  irqrestore(flags);

  /* Get the requested channel structure */

  chan = &tc->channel[chndx];

  /* Is it available? */

  if (chan->inuse)
    {
      /* No.. return a failure */

      sam_givesem(tc);
      return NULL;
    }

  /* OK.. return the channel with the semaphore locked */

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

TCHANDLE sam_tc_allocate(int channel, int mode)
{
  struct sam_chan_s *chan;

  /* Initialize the timer/counter data (if necessary) and get exclusive
   * access to the requested channel.
   */

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
    }

  /* Return an opaque reference to the channel */

  return (TCHANDLE)chan;
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

void sam_tc_free(TCHANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;
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

void sam_tc_start(TCHANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  DEBUGASSERT(chan && chan->inuse);
  sam_chan_putreg(chan, SAM_TC_CCR_OFFSET, TC_CCR_CLKEN | TC_CCR_SWTRG);
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

void sam_tc_stop(TCHANDLE handle)
{
  struct sam_chan_s *chan = (struct sam_chan_s *)handle;

  DEBUGASSERT(chan && chan->inuse);
  sam_chan_putreg(chan, SAM_TC_CCR_OFFSET, TC_CCR_CLKDIS);
}

/****************************************************************************
 * Name: sam_tc_divisor
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (MCK / (DIV * 65536)) <= freq <= (MCK / DIV)
 *
 *   with DIV being the highest possible value.
 *
 * Input Parameters:
 *
 *   frequency  Desired timer frequency.
 *   mck        Master clock frequency.
 *   div        Divisor value.
 *   tcclks     TCCLKS field value for divisor.
 *   boardmck   Board clock frequency.
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

uint32_t sam_tc_divisor(uint32_t frequency, uint32_t mck, uint32_t *div,
                        uint32_t *tcclks, uint32_t boardmck)
{
  const uint32_t adivisors[5] = { 2, 8, 32, 128, boardmck / 32768 };
  int ndx = 0;

  /* Satisfy lower bound */

  while (frequency < ((mck / adivisors[ndx]) / 65536))
    {
      ndx++;

      /*  If no divisor can be found, return -ERANGE */

      if (ndx == (sizeof(adivisors)/sizeof(adivisors[0])))
        {
          return -ERANGE;
        }
    }

  /* Try to maximize DIV while satisfying upper bound */

  while (ndx < 4)
    {

      if (frequency > (mck / adivisors[ndx + 1]))
        {
          break;
        }

      ndx++;
    }

  /* Store results */

  if (div)
    {
      *div = adivisors[ndx];
    }

  if (tcclks)
    {
      *tcclks = ndx;
    }

  return OK;
}

#endif /* CONFIG_SAMA5_TC0 || CONFIG_SAMA5_TC1 */
