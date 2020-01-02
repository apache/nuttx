/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_automount.c
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Anthony Merlino <anthony@vergeaero.com>
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

#if defined(CONFIG_FS_AUTOMOUNTER_DEBUG) && !defined(CONFIG_DEBUG_FS)
#  define CONFIG_DEBUG_FS 1
#endif

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/automount.h>

#include "clicker2-stm32.h"

#ifdef HAVE_AUTOMOUNTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure represents the changeable state of the automounter */

struct stm32_automount_state_s
{
  volatile automount_handler_t handler;    /* Upper half handler */
  FAR void *arg;                           /* Handler argument */
  bool enable;                             /* Fake interrupt enable */
  bool pending;                            /* Set if there an event while disabled */
};

/* This structure represents the static configuration of an automounter */

struct stm32_automount_config_s
{
  /* This must be first thing in structure so that we can simply cast from struct
   * automount_lower_s to struct stm32_automount_config_s
   */

  struct automount_lower_s lower;             /* Publicly visible part */
  uint8_t mmcsd;                              /* MB1_MMCSD_SLOTNO or MB2_MMCSD_SLOTNO */
  FAR struct stm32_automount_state_s *state;  /* Changeable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  stm32_attach(FAR const struct automount_lower_s *lower,
                       automount_handler_t isr, FAR void *arg);
static void stm32_enable(FAR const struct automount_lower_s *lower, bool enable);
static bool stm32_inserted(FAR const struct automount_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT
static struct stm32_automount_state_s g_mb1_mmcsdstate;
static const struct stm32_automount_config_s g_mb1_mmcsdconfig =
{
  .lower        =
  {
    .fstype     = CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT_UDELAY),
    .attach     = stm32_attach,
    .enable     = stm32_enable,
    .inserted   = stm32_inserted
  },
  .mmcsd        = MB1_MMCSD_SLOTNO,
  .state        = &g_mb1_mmcsdstate
};
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT
static struct stm32_automount_state_s g_mb2_mmcsdstate;
static const struct stm32_automount_config_s g_mb2_mmcsdconfig =
{
  .lower        =
  {
    .fstype     = CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT_UDELAY),
    .attach     = stm32_attach,
    .enable     = stm32_enable,
    .inserted   = stm32_inserted
  },
  .mmcsd        = MB2_MMCSD_SLOTNO,
  .state        = &g_mb2_mmcsdstate
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  stm32_attach
 *
 * Description:
 *   Attach a new MMCSD event handler
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *   isr   - The new event handler to be attach
 *   arg   - Client data to be provided when the event handler is invoked.
 *
 *  Returned Value:
 *    Always returns OK
 *
 ****************************************************************************/

static int stm32_attach(FAR const struct automount_lower_s *lower,
                      automount_handler_t isr, FAR void *arg)
{
  FAR const struct stm32_automount_config_s *config;
  FAR struct stm32_automount_state_s *state;

  /* Recover references to our structure */

  config = (FAR struct stm32_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  state = config->state;

  /* Save the new handler info (clearing the handler first to eliminate race
   * conditions).
   */

  state->handler = NULL;
  state->pending = false;
  state->arg     = arg;
  state->handler = isr;
  return OK;
}

/****************************************************************************
 * Name:  stm32_enable
 *
 * Description:
 *   Enable card insertion/removal event detection
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *   enable - True: enable event detection; False: disable
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void stm32_enable(FAR const struct automount_lower_s *lower, bool enable)
{
  FAR const struct stm32_automount_config_s *config;
  FAR struct stm32_automount_state_s *state;
  irqstate_t flags;

  /* Recover references to our structure */

  config = (FAR struct stm32_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  state = config->state;

  /* Save the fake enable setting */

  flags = enter_critical_section();
  state->enable = enable;

  /* Did an interrupt occur while interrupts were disabled? */

  if (enable && state->pending)
    {
      /* Yes.. perform the fake interrupt if the interrutp is attached */

      if (state->handler)
        {
          bool inserted = stm32_cardinserted(config->mmcsd);
          state->handler(&config->lower, state->arg, inserted);
        }

      state->pending = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_inserted
 *
 * Description:
 *   Check if a card is inserted into the slot.
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *
 *  Returned Value:
 *    True if the card is inserted; False otherwise
 *
 ****************************************************************************/

static bool stm32_inserted(FAR const struct automount_lower_s *lower)
{
  FAR const struct stm32_automount_config_s *config;

  config = (FAR struct stm32_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  return stm32_cardinserted(config->mmcsd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  stm32_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enabled MMCSD MikroBus slot
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

int stm32_automount_initialize(void)
{
  FAR void *handle;

  finfo("Initializing automounter(s)\n");

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT
  /* Initialize the MB1 MMCSD auto-mounter */

  handle = automount_initialize(&g_mb1_mmcsdconfig.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for MB1 MMCSD\n");
      return ERROR;
    }
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT
  /* Initialize the MB2 MMCSD auto-mounter */

  handle = automount_initialize(&g_mb2_mmcsdconfig.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for MB2 MMCSD\n");
      return ERROR;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name:  stm32_automount_event
 *
 * Description:
 *   The HSMCI card detection logic has detected an insertion or removal event.  It
 *   has already scheduled the MMC/SD block driver operations.  Now we need to
 *   schedule the auto-mount event which will occur with a substantial delay to make
 *   sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the MB slot: MB1_MMCSD_SLOTNO or MB2_MMCSD_SLOTNO.  There is a
 *      terminology problem here:  Each HSMCI supports two slots, slot A and slot B.
 *      Only slot A is used.  So this is not a really a slot, but an HSCMI peripheral
 *      number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

void stm32_automount_event(int slotno, bool inserted)
{
  FAR const struct stm32_automount_config_s *config;
  FAR struct stm32_automount_state_s *state;

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT
  /* Is this a change in the MB1 MMCSD slot insertion state? */

  if (slotno == MB1_MMCSD_SLOTNO)
    {
      /* Yes.. Select the MB1 MMCSD automounter */

      config = &g_mb1_mmcsdconfig;
      state  = &g_mb1_mmcsdstate;
    }
  else
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT
  /* Is this a change in the MB2 MMCSD slot insertion state? */

  if (slotno == MB2_MMCSD_SLOTNO)
    {
      /* Yes.. Select the MB2 MMCSD automounter */

      config = &g_mb2_mmcsdconfig;
      state  = &g_mb2_mmcsdstate;
    }
  else
#endif
    {
      ferr("ERROR: Unsupported MMCSD%d\n", slotno);
      return;
    }

  /* Is the auto-mounter interrupt attached? */

  if (state->handler)
    {
      /* Yes.. Have we been asked to hold off interrupts? */

      if (!state->enable)
        {
          /* Yes.. just remember that there is a pending interrupt. We will
           * deliver the interrupt when interrupts are "re-enabled."
           */

          state->pending = true;
        }
      else
        {
          /* No.. forward the event to the handler */

          state->handler(&config->lower, state->arg, inserted);
        }
    }
}

#endif /* HAVE_AUTOMOUNTER */
