/****************************************************************************
 * arch/arm/src/samd5e5/sam_tc.c
 *
 *   Copyright 2020 Falker Automacao Agricola LTDA.
 *   Author: Leomar Mateus Radke <leomar@falker.com.br>
 *   Author: Ricardo Wartchow <wartchow@gmail.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <string.h>
#include <semaphore.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "sam_gclk.h"
#include "sam_periphclks.h"
#include "sam_port.h"
#include "sam_tc.h"

#include "hardware/sam_pac.h"

#include <arch/board/board.h>

#ifdef CONFIG_SAMD5E5_TC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Invariant attributes of an TC */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_TC0
static const struct tc_attr_s g_tc0attr =
{
  .tc        = 0,
  .irq       = SAM_IRQ_TC0,
  .coregen   = BOARD_TC0_GCLKGEN,
  .cc0       = BOARD_TC0_PINMAP_CC0,
  .cc1       = BOARD_TC0_PINMAP_CC1,
  .srcfreq   = BOARD_TC0_FREQUENCY,
  .base      = SAM_TC0_BASE,
};
static struct sam_tc_dev_s g_tc0 =
{
  .initialized = false,
  .inuse = false,
};
#endif

#ifdef CONFIG_SAMD5E5_TC1
static const struct tc_attr_s g_tc1attr =
{
  .tc        = 1,
  .irq       = SAM_IRQ_TC1,
  .coregen   = BOARD_TC1_GCLKGEN,
  .cc0       = BOARD_TC1_PINMAP_CC0,
  .cc1       = BOARD_TC1_PINMAP_CC1,
  .srcfreq   = BOARD_TC1_FREQUENCY,
  .base      = SAM_TC1_BASE,
};
static struct sam_tc_dev_s g_tc1 =
{
  .initialized = false,
  .inuse = false,
};
#endif

#ifdef CONFIG_SAMD5E5_TC2
static const struct tc_attr_s g_tc2attr =
{
  .tc        = 2,
  .irq       = SAM_IRQ_TC2,
  .coregen   = BOARD_TC2_GCLKGEN,
  .cc0       = BOARD_TC2_PINMAP_CC0,
  .cc1       = BOARD_TC2_PINMAP_CC1,
  .srcfreq   = BOARD_TC2_FREQUENCY,
  .base      = SAM_TC2_BASE,
};
static struct sam_tc_dev_s g_tc2 =
{
  .initialized = false,
  .inuse = false,
};
#endif

#ifdef CONFIG_SAMD5E5_TC3
static const struct tc_attr_s g_tc3attr =
{
  .tc        = 3,
  .irq       = SAM_IRQ_TC3,
  .coregen   = BOARD_TC3_GCLKGEN,
  .cc0       = BOARD_TC3_PINMAP_CC0,
  .cc1       = BOARD_TC3_PINMAP_CC1,
  .srcfreq   = BOARD_TC3_FREQUENCY,
  .base      = SAM_TC3_BASE,
};
static struct sam_tc_dev_s g_tc3 =
{
  .initialized = false,
  .inuse = false,
};
#endif

#ifdef CONFIG_SAMD5E5_TC4
static const struct tc_attr_s g_tc4attr =
{
  .tc        = 4,
  .irq       = SAM_IRQ_TC4,
  .coregen   = BOARD_TC4_GCLKGEN,
  .cc0       = BOARD_TC4_PINMAP_CC0,
  .cc1       = BOARD_TC4_PINMAP_CC1,
  .srcfreq   = BOARD_TC4_FREQUENCY,
  .base      = SAM_TC4_BASE,
};
static struct sam_tc_dev_s g_tc4 =
{
  .initialized = false,
  .inuse = false,
};
#endif

#ifdef CONFIG_SAMD5E5_TC5
static const struct tc_attr_s g_tc5attr =
{
  .tc        = 5,
  .irq       = SAM_IRQ_TC5,
  .coregen   = BOARD_TC5_GCLKGEN,
  .cc0       = BOARD_TC5_PINMAP_CC0,
  .cc1       = BOARD_TC5_PINMAP_CC1,
  .srcfreq   = BOARD_TC5_FREQUENCY,
  .base      = SAM_TC5_BASE,
};
static struct sam_tc_dev_s g_tc5 =
{
  .initialized = false,
  .inuse = false,
};
#endif

static const uint8_t g_corclk_channel[SAMD5E5_NTC] =
{
    GCLK_CHAN_TC0
  #if SAMD5E5_NTC > 1
    , GCLK_CHAN_TC1
  #endif
  #if SAMD5E5_NTC > 2
    , GCLK_CHAN_TC2
  #endif
  #if SAMD5E5_NTC > 3
    , GCLK_CHAN_TC3
  #endif
  #if SAMD5E5_NTC > 4
    , GCLK_CHAN_TC4
  #endif
  #if SAMD5E5_NTC > 5
    , GCLK_CHAN_TC5
  #endif
};

/* TC frequency data.
 * This table provides the frequency for each selection of TCCLK
 */

#define TC_NDIVIDERS   8

/* This is the list of divider values: divider = (1 << value) */

static const uint8_t g_log2divider[TC_NDIVIDERS] =
{
  0,                     /* TIMER_CLOCK -> div1 */
  1,                     /* TIMER_CLOCK -> div2 */
  2,                     /* TIMER_CLOCK -> div4 */
  3,                     /* TIMER_CLOCK -> div8 */
  4,                     /* TIMER_CLOCK -> div16 */
  6,                     /* TIMER_CLOCK -> div64 */
  8,                     /* TIMER_CLOCK -> div256 */
  10                     /* TIMER_CLOCK -> div1024 */
};

/* TC register lookup used by sam_tc_setregister */

#define TC_NREGISTERS 2

static const uint8_t g_regoffset[TC_NREGISTERS] =
{
  SAM_TC_COUNT32_CC0_OFFSET,     /* Channel 0 */
  SAM_TC_COUNT32_CC1_OFFSET,     /* Channel 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Initialization */

static void tc_takesem(struct sam_tc_dev_s *priv);
#define tc_givesem(priv) (nxsem_post(&priv->exclsem))
void tc_bridge_enable(int tc);
void sam_tc_dumpregs(struct sam_tc_dev_s *priv);
void sam_tc_setperiod(struct sam_tc_dev_s *priv);

static uint32_t sam_tc_divfreq_lookup(uint32_t ftcin, int ndx);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wake-ups due to
 *   the receipt of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tc_takesem(struct sam_tc_dev_s *priv)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&priv->exclsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      /* DEBUGASSERT(ret == OK || ret == -EINTR); */
    }

  while (ret == -EINTR);
}

/****************************************************************************
 * Name: tc_enable
 *
 * Description:
 *   Enable the Main Clock for Tc. (MCLK_APBxMASK)
 *
 ****************************************************************************/

void tc_bridge_enable(int tc)
{
  DEBUGASSERT((unsigned)tc < SAMD5E5_NTC);

  switch (tc)
    {
      case 0:
        sam_apb_tc0_enableperiph();
        break;

      case 1:
        sam_apb_tc1_enableperiph();
        break;

      case 2:
        sam_apb_tc2_enableperiph();
        break;

      case 3:
        sam_apb_tc3_enableperiph();
        break;

      case 4:
        sam_apb_tc4_enableperiph();
        break;

      case 5:
        sam_apb_tc5_enableperiph();
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: tc_wait_synchronization
 *
 * Description:
 *   Wait until the TC reports that it is synchronized.
 *
 ****************************************************************************/

static void tc_wait_synchronization(struct sam_tc_dev_s *priv)
{
  while ((getreg32(priv->attr->base +  SAM_TC_SYNCBUSY_OFFSET) & 0x7) != 0);
}

/****************************************************************************
 * Name: tc_coreclk_configure
 *
 * Description:
 *   Configure the TC source clock. PCHCTRL[id]
 *
 *   One generic clock is used by the TC: GCLK_TCx.
 *
 ****************************************************************************/

void tc_coreclk_configure(int tc, int coregen, bool wrlock)
{
  uint8_t corechan;

  DEBUGASSERT((unsigned)tc < SAMD5E5_NTC);

  /* Set up the TCn_GCLK_ID_CORE clock */

  corechan = g_corclk_channel[tc];
  sam_gclk_chan_enable(corechan, coregen, wrlock);
}

/****************************************************************************
 * Name: tc_interrupt
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
 * On successful return, the caller holds the tc exclusive access semaphore.
 *
 ****************************************************************************/

static int tc_interrupt(int irq, void *context, FAR void *arg)
{
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)arg;
  uint8_t flags;

  /* Get the interrupt status for this channel */

  flags = getreg8(priv->attr->base + SAM_TC_INTFLAG_OFFSET);
  if (flags & TC_INTFLAG_OVF)
    {
      tmrinfo("Overflow Interrupt Flag\n");
      putreg8(TC_INTFLAG_OVF, priv->attr->base + SAM_TC_INTFLAG_OFFSET);
    }

  if (flags & TC_INTFLAG_ERR)
    {
      tmrinfo("Error Interrupt Flag\n");
      putreg8(TC_INTFLAG_ERR, priv->attr->base + SAM_TC_INTFLAG_OFFSET);
    }

  if (flags & TC_INTFLAG_MC0)
    {
      tmrinfo("MC0 Interrupt Flag\n");
      putreg8(TC_INTFLAG_MC0, priv->attr->base + SAM_TC_INTFLAG_OFFSET);
    }

  if (flags & TC_INTFLAG_MC1)
    {
      tmrinfo("MC1 Interrupt Flag\n");
      putreg8(TC_INTFLAG_MC1, priv->attr->base + SAM_TC_INTFLAG_OFFSET);
    }

  if (priv->handler)
    {
      /* Execute the callback */

      priv->handler(priv, priv->arg, flags);
    }

  return OK;
}

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

      return ftcin / BOARD_OSC32K_FREQUENCY;
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
      return BOARD_OSC32K_FREQUENCY;
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
 *  On successful return, the caller holds the tc exclusive access semaphore.
 *
 ****************************************************************************/

static inline struct sam_tc_dev_s *sam_tc_initialize(int tc)
{
  struct sam_tc_dev_s *priv;

  /* Select the timer/counter and get the index associated with the TC. */

#ifdef CONFIG_SAMD5E5_TC0
  if (tc == 0)
    {
      /* Select up TC0 and setup invariant attributes */

      priv = &g_tc0;
      priv->attr = &g_tc0attr;
    }
  else
#endif
#ifdef CONFIG_SAMD5E5_TC1
  if (tc == 1)
    {
      /* Select up TC1 and setup invariant attributes */

      priv = &g_tc1;
      priv->attr = &g_tc1attr;
    }
  else
#endif
#ifdef CONFIG_SAMD5E5_TC2
  if (tc == 2)
    {
      /* Select up TC2 and setup invariant attributes */

      priv = &g_tc2;
      priv->attr = &g_tc2attr;
    }
  else
#endif
#ifdef CONFIG_SAMD5E5_TC3
  if (tc == 3)
    {
      /* Select up TC3 and setup invariant attributes */

      priv = &g_tc3;
      priv->attr = &g_tc3attr;
    }
  else
#endif
#ifdef CONFIG_SAMD5E5_TC4
  if (tc == 4)
    {
      /* Select up TC4 and setup invariant attributes */

      priv = &g_tc4;
      priv->attr = &g_tc4attr;
    }
  else
#endif
#ifdef CONFIG_SAMD5E5_TC5
  if (tc == 5)
    {
      /* Select up TC5 and setup invariant attributes */

      priv = &g_tc5;
      priv->attr = &g_tc5attr;
    }
  else
#endif
    {
      tmrerr("ERROR: Unsupported TC%d\n", tc);
      return NULL;
    }

  return priv;
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
 *   channel TC channel number
 *   mode    Operating mode (TC_CMR value).
 *
 * Returned Value:
 *   On success, a non-NULL handle value is returned.  This handle may be
 *   used with subsequent timer/counter interfaces to manage the timer.  A
 *   NULL handle value is returned on a failure.
 *
 ****************************************************************************/

TC_HANDLE sam_tc_allocate(int tc, int frequency)
{
  struct sam_tc_dev_s *priv;
  int ret;
  uint32_t divisor;
  uint32_t ctrla;
  uint32_t regval;
  irqstate_t flags;

  /* Initialize the timer/counter data (if necessary) and get exclusive
   * access to the requested tc.
   */

  tmrinfo("tc=%d frequency=%d\n", tc, frequency);

  priv = sam_tc_initialize(tc);
  if (priv)
    {
      /* The pre-calculate values to use when we start the timer */

      ret = sam_tc_divisor(priv, frequency, &divisor, &ctrla);
      if (ret < 0)
        {
          tmrerr("ERROR: sam_tc_divisor failed: %d\n", ret);
          return NULL;
        }

      tmrinfo("frequency=%lu, divisor=%lu, ctrla=0x%08lx\n",
          (unsigned long)frequency, (unsigned long)divisor,
          (unsigned long)ctrla);

      /* Perform one-time TC initialization */

      flags = enter_critical_section();

      /* Attach Interrupt Handler */

      ret = irq_attach(priv->attr->irq, tc_interrupt, priv);
      if (ret < 0)
        {
          tmrerr("ERROR: TC%d failed to attach irq %d\n",
                  tc, priv->attr->irq);
          leave_critical_section(flags);
          return NULL;
        }

      tmrinfo("TC%d attached irq %d\n", tc, priv->attr->irq);

      /* Initialize the TC driver structure */

      priv->flags = 0;
      (void)nxsem_init(&priv->exclsem, 0, 1);

      /* Enable clocking to the TC module in PCHCTRL */

      tc_bridge_enable(priv->attr->tc);
      tc_bridge_enable(priv->attr->tc +1); /* Slave TC in 32 bits mode */

      /* Configure the GCLKs for the TC module */

      tc_coreclk_configure(priv->attr->tc, priv->attr->coregen, false);

      /* Check if module is enabled */

      regval = getreg32(priv->attr->base + SAM_TC_CTRLA_OFFSET);
      if (regval & TC_CTRLA_ENABLE)
        {
          tmrerr("ERROR: Cannot initialize TC! It's already initialized!\n");
          leave_critical_section(flags);
          return NULL;
        }

      /* Check if reset is in progress */

      regval = getreg32(priv->attr->base + SAM_TC_CTRLA_OFFSET);
      if (regval & TC_CTRLA_SWRST)
        {
          tmrerr("ERROR: Module is in RESET process!\n");
          leave_critical_section(flags);
          return NULL;
        }

      /* see 48.7.1 | TC_CTRLA_PRESCSYNC_PRESC */

      putreg32(ctrla | TC_CTRLA_MODE_COUNT32,
               priv->attr->base + SAM_TC_CTRLA_OFFSET);
      tc_wait_synchronization(priv);

      /* Enable TC interrupts at the NVIC */

      up_enable_irq(priv->attr->irq);
      tmrinfo("enable TC%d irq=%d\n", tc, priv->attr->irq);

      /* Get exclusive access to the timer/count data structure */

      tc_takesem(priv);

      /* Is it available? */

      if (priv->inuse)
        {
          /* No.. return a failure */

          tmrerr("ERROR: TC%d is in-use\n", priv->attr->tc);
          tc_givesem(priv);
          leave_critical_section(flags);
          return NULL;
        }

      /* Mark the TC "inuse" */

      priv->inuse = true;

      /* Now the channel is initialized */

      priv->initialized = true;

      leave_critical_section(flags);
      tc_givesem(priv);
    }

  /* Return an opaque reference to the tc */

  tmrinfo("Returning 0x%p\n", priv);
  return (TC_HANDLE)priv;
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
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;

  tmrinfo("Freeing %p inuse=%d\n", priv, priv->inuse);
  DEBUGASSERT(priv && priv->inuse);

  /* Make sure that interrupts are detached and disabled and that the channel
   * is stopped and disabled.
   */

  sam_tc_attach(handle, NULL, NULL, 0);
  sam_tc_stop(handle);

  /* Mark the channel as available */

  priv->inuse = false;
}

/****************************************************************************
 * Name: sam_tc_start
 *
 * Description:
 *   Reset and Start the TC Channel. Enables the timer clock and performs a
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
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;
  uint32_t regval;

  tmrinfo("Starting TC%d inuse=%d COUNT=%d\n",
          priv->attr->tc, priv->inuse, sam_tc_getcounter(priv));
  DEBUGASSERT(priv && priv->inuse);

  putreg32(0, priv->attr->base + SAM_TC_COUNT_OFFSET);

  /* Clear any pending interrupts on this channel */

  putreg8(TC_INTFLAG_ALL, priv->attr->base + SAM_TC_INTFLAG_OFFSET);

  /* Then enable the timer */

  regval = getreg32(priv->attr->base + SAM_TC_CTRLA_OFFSET);
  regval |= TC_CTRLA_ENABLE;
  putreg32(regval, priv->attr->base + SAM_TC_CTRLA_OFFSET);
  tc_wait_synchronization(priv);
}

/****************************************************************************
 * Name: sam_tc_stop
 *
 * Description:
 *   Stop TC Channel. Disables the timer clock, stopping the counting.
 *
 * Input Parameters:
 *   handle Channel handle previously allocated by sam_tc_allocate()
 *
 * Returned Value:
 *
 ****************************************************************************/

void sam_tc_stop(TC_HANDLE handle)
{
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;
  uint32_t regval;

  tmrinfo("Stopping TC%d inuse=%d\n", priv->attr->tc, priv->inuse);
  DEBUGASSERT(priv && priv->inuse);

  regval = getreg32(priv->attr->base + SAM_TC_CTRLA_OFFSET);
  regval &= ~TC_CTRLA_ENABLE;
  putreg32(regval, priv->attr->base + SAM_TC_CTRLA_OFFSET);
  tc_wait_synchronization(priv);
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
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;
  tc_handler_t oldhandler;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* Remember the old interrupt handler and set the new handler */

  flags         = enter_critical_section();
  oldhandler    = priv->handler;
  priv->handler = handler;

  /* Don't enable interrupt if we are detaching no matter what the caller
   * says.
   */

  if (!handler)
    {
      arg  = NULL;
      mask = 0;
    }

  priv->arg = arg;

  /* Now enable interrupt as requested */

  putreg8(TC_INTFLAG_ALL, priv->attr->base + SAM_TC_INTENCLR_OFFSET);
  putreg8(mask, priv->attr->base + SAM_TC_INTENSET_OFFSET);
  if (mask & TC_INTFLAG_MC0)
    putreg8(TC_CTRLBSET_ONESHOT, priv->attr->base + SAM_TC_CTRLBSET_OFFSET);

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

uint8_t sam_tc_getpending(TC_HANDLE handle)
{
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;
  DEBUGASSERT(priv);
  return getreg8(priv->attr->base + SAM_TC_INTFLAG_OFFSET);
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
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;

  DEBUGASSERT(priv && regid < TC_NREGISTERS);

  tmrinfo("TC%d: Set register CC%d to 0x%08lx\n",
          priv->attr->tc, regid, (unsigned long)regval);

  putreg32(regval, priv->attr->base + g_regoffset[regid]);
}

/****************************************************************************
 * Name: sam_tc_getregister
 *
 * Description:
 *    Get the current value of the channel register.
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
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;

  DEBUGASSERT(priv && regid < TC_NREGISTERS);
  return getreg32(priv->attr->base + g_regoffset[regid]);
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
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;

  DEBUGASSERT(priv);
  putreg8(TC_CTRLBSET_CMD_READSYNC,
          priv->attr->base + SAM_TC_CTRLBSET_OFFSET);
  return getreg32(priv->attr->base + SAM_TC_COUNT_OFFSET);
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
  return BOARD_GCLK3_FREQUENCY;
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
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;
  uint32_t ftcin = priv->attr->srcfreq;
  uint32_t regval;
  int tcclks;

  DEBUGASSERT(priv);

  /* Get the SAM_TC_CTRLA_OFFSET register contents for this channel
   * and extract the PRESCALER index.
   */

  regval = getreg32(priv->attr->base + SAM_TC_CTRLA_OFFSET);
  tcclks = (regval & TC_CTRLA_PRESCALER_MASK) >> TC_CTRLA_PRESCALER_SHIFT;

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
 *   Zero (OK) if a proper divisor has been found, otherwise a negated
 *   errno value indicating the nature of the failure.
 *
 ****************************************************************************/

int sam_tc_divisor(struct sam_tc_dev_s *tc, uint32_t frequency,
                   uint32_t *div, uint32_t *tcclks)
{
  uint32_t ftcin = tc->attr->srcfreq;
  int ndx = 0;

  tmrinfo("frequency=%d\n", frequency);

  /* Satisfy lower bound.  That is, the value of the divider such that:
   *
   *   frequency >= (tc_input_frequency * 65536) / divider.
   */

  while (frequency < (sam_tc_divfreq_lookup(ftcin, ndx) >> 16))
    {
      if (++ndx > TC_NDIVIDERS)
        {
          /* If no divisor can be found, return -ERANGE */

          tmrerr("ERROR: Lower bound search failed\n");
          return -ERANGE;
        }
    }

  /* Try to maximize DIV while still satisfying upper bound.
   *  That the value of the divider such that:
   *
   *   frequency < tc_input_frequency / divider.
   */

  for (; ndx < (TC_NDIVIDERS - 1); ndx++)
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

  /* Return the PRESCALER selection */

  if (tcclks)
    {
      tmrinfo("return tcclks=0x%08lx\n",
             (unsigned long)TC_CTRLA_PRESCALER(ndx));
      *tcclks = TC_CTRLA_PRESCALER(ndx);
    }

  return OK;
}

uint32_t sam_tc_getctrla(TC_HANDLE handle)
{
  struct sam_tc_dev_s *priv = (struct sam_tc_dev_s *)handle;

  DEBUGASSERT(priv);
  return getreg32(priv->attr->base + SAM_TC_CTRLA_OFFSET);
}

#endif /* CONFIG_SAMD5E5_TC */
