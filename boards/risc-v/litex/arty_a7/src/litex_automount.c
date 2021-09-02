/****************************************************************************
 * boards/risc-v/litex/arty_a7/src/litex_automount.c
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

#include "arty_a7.h"

#ifdef HAVE_AUTOMOUNTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the changeable state of the automounter */

struct litex_automount_state_s
{
  volatile automount_handler_t handler; /* Upper half handler */
  void *arg;                            /* Handler argument */
  bool enable;                          /* Fake interrupt enable */
  bool pending;                         /* Set if there an event while disabled */
};

/* This structure represents the static configuration of an automounter */

struct litex_automount_config_s
{
  /* This must be first thing in structure so that we can simply cast from
   * struct automount_lower_s to struct litex_automount_config_s
   */

  struct automount_lower_s lower;        /* Publicly visible part */
  uint8_t mmcsd;                         /* MB1_MMCSD_SLOTNO or MB2_MMCSD_SLOTNO */
  struct litex_automount_state_s *state; /* Changeable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  litex_attach(const struct automount_lower_s *lower,
                         automount_handler_t isr, void *arg);
static void litex_enable(const struct automount_lower_s *lower,
                         bool enable);
static bool litex_inserted(const struct automount_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LITEX_SDIO_MOUNT_MOUNTPOINT
static struct litex_automount_state_s g_mb1_mmcsdstate;
static const struct litex_automount_config_s g_mb1_mmcsdconfig =
{
  .lower        =
  {
    .fstype     = CONFIG_LITEX_SDIO_MOUNT_FSTYPE,
    .blockdev   = CONFIG_LITEX_SDIO_MOUNT_BLKDEV,
    .mountpoint = CONFIG_LITEX_SDIO_MOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(
                  100),
    .udelay     = MSEC2TICK(
                  100),
    .attach     = litex_attach,
    .enable     = litex_enable,
    .inserted   = litex_inserted
  },
  .mmcsd        = SDIO_SLOTNO,
  .state        = &g_mb1_mmcsdstate
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  litex_attach
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

static int litex_attach(const struct automount_lower_s *lower,
                      automount_handler_t isr, void *arg)
{
  const struct litex_automount_config_s *config;
  struct litex_automount_state_s *state;

  /* Recover references to our structure */

  config = (struct litex_automount_config_s *)lower;
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
 * Name:  litex_enable
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

static void litex_enable(const struct automount_lower_s *lower,
                         bool enable)
{
  const struct litex_automount_config_s *config;
  struct litex_automount_state_s *state;
  irqstate_t flags;

  /* Recover references to our structure */

  config = (struct litex_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

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
          bool inserted = litex_cardinserted(config->mmcsd);
          state->handler(&config->lower, state->arg, inserted);
        }

      state->pending = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: litex_inserted
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

static bool litex_inserted(const struct automount_lower_s *lower)
{
  const struct litex_automount_config_s *config;

  config = (struct litex_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  return litex_cardinserted(config->mmcsd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  litex_automount_initialize
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

int litex_automount_initialize(void)
{
  void *handle;

  finfo("Initializing automounter(s)\n");

#ifdef CONFIG_LITEX_SDIO_MOUNT_MOUNTPOINT
  /* Initialize the MB1 MMCSD auto-mounter */

  handle = automount_initialize(&g_mb1_mmcsdconfig.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for MB1 MMCSD\n");
      return ERROR;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name:  litex_automount_event
 *
 * Description:
 *   The HSMCI card detection logic has detected an insertion or removal
 *   event.  It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the MB slot: MB1_MMCSD_SLOTNO or MB2_MMCSD_SLOTNO.
 *      There is a terminology problem here:  Each HSMCI supports two slots,
 *      slot A and slot B. Only slot A is used.  So this is not a really a
 *      slot, but an HSCMI peripheral number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

void litex_automount_event(int slotno, bool inserted)
{
  const struct litex_automount_config_s *config;
  struct litex_automount_state_s *state;

#ifdef CONFIG_LITEX_SDIO_MOUNT_MOUNTPOINT
  /* Is this a change in the MB1 MMCSD slot insertion state? */

  if (slotno == SDIO_SLOTNO)
    {
      /* Yes.. Select the MB1 MMCSD automounter */

      config = &g_mb1_mmcsdconfig;
      state  = &g_mb1_mmcsdstate;
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
