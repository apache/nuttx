/****************************************************************************
 * boards/arm/kinetis/freedom-k66f/src/k66_automount.c
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

#if defined(CONFIG_FS_AUTOMOUNTER_DEBUG) && !defined(CONFIG_DEBUG_FS)
#  define CONFIG_DEBUG_FS 1
#endif

#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/automount.h>

#include "freedom-k66f.h"

#ifdef HAVE_AUTOMOUNTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL (void *)0
#endif

#ifndef OK
#  define OK 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the changeable state of the automounter */

struct k66_automount_state_s
{
  volatile automount_handler_t handler; /* Upper half handler */
  void *arg;                            /* Handler argument */
  bool enable;                          /* Fake interrupt enable */
  bool pending;                         /* Set if there an event while disabled */
};

/* This structure represents the static configuration of an automounter */

struct k66_automount_config_s
{
  /* This must be first thing in structure so that we can simply cast from
   * struct automount_lower_s to struct k66_automount_config_s
   */

  struct automount_lower_s lower;      /* Publicly visible part */
  struct k66_automount_state_s *state; /* Changeable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  k66_attach(const struct automount_lower_s *lower,
                       automount_handler_t isr, void *arg);
static void k66_enable(const struct automount_lower_s *lower,
                       bool enable);
static bool k66_inserted(const struct automount_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct k66_automount_state_s g_sdhc_state;
static const struct k66_automount_config_s g_sdhc_config =
{
  .lower        =
  {
    .fstype     = CONFIG_FRDMK66F_SDHC_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_FRDMK66F_SDHC_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_FRDMK66F_SDHC_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_FRDMK66F_SDHC_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_FRDMK66F_SDHC_AUTOMOUNT_UDELAY),
    .attach     = k66_attach,
    .enable     = k66_enable,
    .inserted   = k66_inserted
  },
  .state        = &g_sdhc_state
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  k66_attach
 *
 * Description:
 *   Attach a new SDHC event handler
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

static int k66_attach(const struct automount_lower_s *lower,
                      automount_handler_t isr, void *arg)
{
  const struct k66_automount_config_s *config;
  struct k66_automount_state_s *state;

  /* Recover references to our structure */

  config = (struct k66_automount_config_s *)lower;
  DEBUGASSERT(config != NULL && config->state != NULL);

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
 * Name:  k66_enable
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

static void k66_enable(const struct automount_lower_s *lower,
                       bool enable)
{
  const struct k66_automount_config_s *config;
  struct k66_automount_state_s *state;
  irqstate_t flags;

  /* Recover references to our structure */

  config = (struct k66_automount_config_s *)lower;
  DEBUGASSERT(config != NULL && config->state != NULL);

  state = config->state;

  /* Save the fake enable setting */

  flags = enter_critical_section();
  state->enable = enable;

  /* Did an interrupt occur while interrupts were disabled? */

  if (enable && state->pending)
    {
      /* Yes.. perform the fake interrupt if the interrupt is attached */

      if (state->handler)
        {
          bool inserted = k66_cardinserted();
          state->handler(&config->lower, state->arg, inserted);
        }

      state->pending = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: k66_inserted
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

static bool k66_inserted(const struct automount_lower_s *lower)
{
  return k66_cardinserted();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  k66_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured SDHC
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

void k66_automount_initialize(void)
{
  void *handle;

  finfo("Initializing automounter(s)\n");

  /* Initialize the SDHC0 auto-mounter */

  handle = automount_initialize(&g_sdhc_config.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for SDHC0\n");
    }
}

/****************************************************************************
 * Name:  k66_automount_event
 *
 * Description:
 *   The SDHC card detection logic has detected an insertion or removal
 *   event.
 *   It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the SDHC0 slot: SDHC0_SLOTNO or SDHC1_SLOTNO.
 *       There is a terminology problem here:  Each SDHC supports two slots,
 *       slot A and slot B. Only slot A is used.
 *       So this is not a really a slot, but an HSCMI peripheral number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

void k66_automount_event(bool inserted)
{
  const struct k66_automount_config_s *config = &g_sdhc_config;
  struct k66_automount_state_s *state = &g_sdhc_state;

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
