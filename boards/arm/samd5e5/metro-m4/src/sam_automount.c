/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_automount.c
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

#if defined(CONFIG_FS_AUTOMOUNTER_DEBUG) && !defined(CONFIG_DEBUG_FS)
#  define CONFIG_DEBUG_FS 1
#endif

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/automount.h>

#include "metro-m4.h"

#ifdef CONFIG_FS_AUTOMOUNTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL (FAR void *)0
#endif

#ifndef OK
#  define OK 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the changeable state of the automounter */

struct sam_automount_state_s
{
  volatile automount_handler_t handler;    /* Upper half handler */
  FAR void *arg;                           /* Handler argument */
  bool enable;                             /* Fake interrupt enable */
  bool pending;                            /* Set if there an event while disabled */
  bool inserted;
};

/* This structure represents the static configuration of an automounter */

struct sam_automount_config_s
{
  /* This must be first thing in structure so that we can simply cast from
   * struct automount_lower_s to struct sam_automount_config_s
   */

  struct automount_lower_s lower;          /* Publicly visible part */
  FAR struct sam_automount_state_s *state; /* Changeable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sam_attach(FAR const struct automount_lower_s *lower,
                       automount_handler_t isr, FAR void *arg);
static void sam_enable(FAR const struct automount_lower_s *lower,
                       bool enable);
static bool sam_inserted(FAR const struct automount_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_METRO_M4_USB_AUTOMOUNT
static struct sam_automount_state_s g_port0state;
static const struct sam_automount_config_s g_port0config =
{
  .lower        =
  {
    .fstype     = CONFIG_METRO_M4_USB_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_METRO_M4_USB_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_METRO_M4_USB_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_METRO_M4_USB_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_METRO_M4_USB_AUTOMOUNT_UDELAY),
    .attach     = sam_attach,
    .enable     = sam_enable,
    .inserted   = sam_inserted
  },
  .state        = &g_port0state
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_attach
 *
 * Description:
 *   Attach a new port event handler
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

static int sam_attach(FAR const struct automount_lower_s *lower,
                      automount_handler_t isr, FAR void *arg)
{
  FAR const struct sam_automount_config_s *config;
  FAR struct sam_automount_state_s *state;

  finfo("Entry\n");

  /* Recover references to our structure */

  config = (FAR struct sam_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  state = config->state;

  /* Save the new handler info (clearing the handler first to eliminate
   * race conditions).
   */

  state->handler = NULL;
  state->pending = false;
  state->arg     = arg;
  state->handler = isr;
  return OK;
}

/****************************************************************************
 * Name:  sam_enable
 *
 * Description:
 *   Enable port insertion/removal event detection
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *   enable - True: enable event detection; False: disable
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void sam_enable(FAR const struct automount_lower_s *lower,
                       bool enable)
{
  FAR const struct sam_automount_config_s *config;
  FAR struct sam_automount_state_s *state;
  irqstate_t flags;

  finfo("Entry\n");

  /* Recover references to our structure */

  config = (FAR struct sam_automount_config_s *)lower;
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
          (void)state->handler(&config->lower, state->arg, true);
        }

      state->pending = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_inserted
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

static bool sam_inserted(FAR const struct automount_lower_s *lower)
{
  FAR const struct sam_automount_config_s *config;

  config = (FAR struct sam_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);
  finfo("inserted:%d\n", config->state->inserted);
  return config->state->inserted;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured HSMCI
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

void sam_automount_initialize(void)
{
  FAR void *handle;

  finfo("Initializing automounter(s)\n");

#ifdef CONFIG_METRO_M4_USB_AUTOMOUNT
  /* Initialize the USB auto-mounter */

  handle = automount_initialize(&g_port0config.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for USB\n");
    }
#endif
}

/****************************************************************************
 * Name:  sam_automount_event
 *
 * Description:
 *   It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with
 *   a substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

void sam_automount_event(bool inserted)
{
  FAR const struct sam_automount_config_s *config;
  FAR struct sam_automount_state_s *state;

#ifdef CONFIG_METRO_M4_USB_AUTOMOUNT
  config = &g_port0config;
  state  = &g_port0state;
  state->inserted = inserted;
#endif

  finfo("event:%d\n", inserted);

  /* Is the auto-mounter interrupt attached? */

  if (state->handler)
    {
      /* Yes.. Have we been asked to hold off interrupts? */

      if (!state->enable)
        {
          /* Yes.. just remember that there is a pending interrupt.
           * We will deliver the interrupt when interrupts are
           * "re-enabled."
           */

          state->pending = true;
        }
      else
        {
          /* No.. forward the event to the handler */

          (void)state->handler(&config->lower, state->arg, state->inserted);
        }
    }
}

#endif /* CONFIG_FS_AUTOMOUNTER */
