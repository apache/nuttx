/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_automount.c
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

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/automount.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the changeable state of the automounter */

struct cxd56_automount_state_s
{
  volatile automount_handler_t handler; /* Upper half handler */
  void *arg;                            /* Handler argument */
  bool enable;                          /* Fake interrupt enable */
  bool pending;                         /* Set if there an event while disabled */
};

/* This structure represents the static configuration of an automounter */

struct cxd56_automount_config_s
{
  /* This must be first thing in structure so that we can simply cast from
   * struct automount_lower_s to struct cxd56_automount_config_s
   */

  struct automount_lower_s lower;        /* Publicly visible part */
  struct cxd56_automount_state_s *state; /* Changeable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sdcard_attach(const struct automount_lower_s *lower,
                          automount_handler_t isr, void *arg);
static void sdcard_enable(const struct automount_lower_s *lower,
                          bool enable);
static bool sdcard_inserted(const struct automount_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_CXD56_SDCARD_AUTOMOUNT
static struct cxd56_automount_state_s g_sdcard0state;
static const struct cxd56_automount_config_s g_sdcard0config =
{
  .lower        =
  {
    .fstype     = CONFIG_CXD56_SDCARD_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_CXD56_SDCARD_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_CXD56_SDCARD_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_CXD56_SDCARD_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_CXD56_SDCARD_AUTOMOUNT_UDELAY),
    .attach     = sdcard_attach,
    .enable     = sdcard_enable,
    .inserted   = sdcard_inserted
  },
  .state        = &g_sdcard0state
};
#endif /* CONFIG_CXD56_SDCARD_AUTOMOUNT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sdcard_attach
 *
 * Description:
 *   Attach a new SDCARD event handler
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

static int sdcard_attach(const struct automount_lower_s *lower,
                         automount_handler_t isr, void *arg)
{
  const struct cxd56_automount_config_s *config;
  struct cxd56_automount_state_s *state;

  /* Recover references to our structure */

  config = (struct cxd56_automount_config_s *)lower;
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
 * Name:  sdcard_enable
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

static void sdcard_enable(const struct automount_lower_s *lower,
                          bool enable)
{
  const struct cxd56_automount_config_s *config;
  struct cxd56_automount_state_s *state;
  irqstate_t flags;

  /* Recover references to our structure */

  config = (struct cxd56_automount_config_s *)lower;
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
          bool inserted = board_sdcard_inserted(0);
          state->handler(&config->lower, state->arg, inserted);
        }

      state->pending = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sdcard_inserted
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

static bool sdcard_inserted(const struct automount_lower_s *lower)
{
  const struct cxd56_automount_config_s *config;

  config = (struct cxd56_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  return board_sdcard_inserted(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured SDCARD
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

void board_automount_initialize(void)
{
  void *handle;

  finfo("Initializing automounter(s)\n");

#ifdef CONFIG_CXD56_SDCARD_AUTOMOUNT
  /* Initialize the SDCARD auto-mounter */

  handle = automount_initialize(&g_sdcard0config.lower);
  if (handle == NULL)
    {
      ferr("ERROR: Failed to initialize auto-mounter for SDCARD\n");
    }
#endif /* CONFIG_CXD56_SDCARD_AUTOMOUNT */
}

/****************************************************************************
 * Name:  board_automount_event
 *
 * Description:
 *   The SDCARD card detection logic has detected an insertion or removal
 *   event.
 *   It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the SDCARD slot: Only slot 0 is used.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

void board_automount_event(int slotno, bool inserted)
{
  const struct cxd56_automount_config_s *config;
  struct cxd56_automount_state_s *state;

#ifdef CONFIG_CXD56_SDCARD_AUTOMOUNT
  /* Is this a change in the SDCARD insertion state? */

  if (slotno == 0)
    {
      /* Yes.. Select the SDCARD automounter */

      config = &g_sdcard0config;
      state  = &g_sdcard0state;
    }
  else
#endif /* CONFIG_CXD56_SDCARD_AUTOMOUNT */
    {
      ferr("ERROR: Unsupported SDCARD%d\n", slotno);
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
