/****************************************************************************
 * boards/arm/samv7/common/src/sam_hsmci.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "sam_hsmci.h"
#include "board_hsmci.h"

#ifdef CONFIG_FS_AUTOMOUNTER
#  include "sam_automount.h"
#endif /* CONFIG_FS_AUTOMOUNTER */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds static information unique to one HSMCI peripheral */

struct sam_hsmci_state_s
{
  struct sdio_dev_s *hsmci;   /* R/W device handle */
  gpio_pinset_t cdcfg;        /* Card detect PIO pin configuration */
  int cdirq;                  /* Interrupt number (same as pid) */
  uint8_t slotno;             /* Slot number */
  bool cd;                    /* TRUE: card is inserted */
  bool cdinvert;              /* Invert card detection to 0 signaling card */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HSCMI device state */

#ifdef CONFIG_SAMV7_HSMCI0
static struct sam_hsmci_state_s g_hsmci0 =
{
  .slotno  = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_cardinserted_internal
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

static bool sam_cardinserted_internal(struct sam_hsmci_state_s *state)
{
  bool inserted;

  /* Get the state of the PIO pin */

  inserted = sam_gpioread(state->cdcfg) != state->cdinvert;
  finfo("Slot %d inserted: %s\n", state->slotno, inserted ? "YES" : "NO");
  return inserted;
}

/****************************************************************************
 * Name: sam_hsmci_cardetect and sam_hsmci_cardetect_handler
 *
 * Description:
 *   Card detect interrupt handlers
 *
 ****************************************************************************/

static int sam_hsmci_cardetect(struct sam_hsmci_state_s *state)
{
  /* Get the current card insertion state */

  bool cd = sam_cardinserted_internal(state);

  /* Has the card detect state changed? */

  if (cd != state->cd)
    {
      /* Yes... remember that new state and inform the HSMCI driver */

      state->cd = cd;

      /* Report the new state to the SDIO driver */

      sdio_mediachange(state->hsmci, cd);
    }

  return OK;
}

static int sam_hsmci_cardetect_handler(int irq, void *context,
                                       void *arg)
{
  struct sam_hsmci_state_s *state = (struct sam_hsmci_state_s *)arg;
  int ret;

  /* Handle the card detect interrupt.  The interrupt level logic will
   * kick of the driver-level operations to initialize the MMC/SD block
   * device.
   */

  ret = sam_hsmci_cardetect(state);

#ifdef CONFIG_FS_AUTOMOUNTER
  /* Let the automounter know about the insertion event */

  sam_automount_event(state->slotno, sam_cardinserted(state->slotno));
#endif /* CONFIG_FS_AUTOMOUNTER */

  return ret;
}

/****************************************************************************
 * Name: sam_hsmci_state
 *
 * Description:
 *   Initialize HSMCI PIOs.
 *
 ****************************************************************************/

static inline struct sam_hsmci_state_s *sam_hsmci_state(int slotno)
{
  struct sam_hsmci_state_s *state = NULL;

#ifdef CONFIG_SAMV7_HSMCI0
  if (g_hsmci0.slotno == slotno)
    {
      state = &g_hsmci0;
    }
#endif

  return state;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int sam_hsmci_initialize(int slotno, int minor, gpio_pinset_t cdcfg,
                         int cdirq, bool cdinvert)
{
  struct sam_hsmci_state_s *state;
  int ret;

  /* Get the static HSMI description */

  state = sam_hsmci_state(slotno);
  if (state == NULL)
    {
      ferr("ERROR: No state for slotno %d\n", slotno);
      return -EINVAL;
    }

  state->cdcfg = cdcfg;
  state->cdirq = cdirq;
  state->cdinvert = cdinvert;

  /* Initialize card-detect, write-protect, and power enable PIOs */

  sam_configgpio(state->cdcfg);
  sam_dumpgpio(state->cdcfg, "HSMCI Card Detect");

  /* Mount the SDIO-based MMC/SD block driver */

  /* First, get an instance of the SDIO interface */

  state->hsmci = sdio_initialize(slotno);
  if (state->hsmci == NULL)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n",  slotno);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(minor, state->hsmci);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Configure card detect interrupts */

  sam_gpioirq(state->cdcfg);
  irq_attach(state->cdirq, sam_hsmci_cardetect_handler, (void *)state);

  /* Then inform the HSMCI driver if there is or is not a card in the slot. */

  state->cd = sam_cardinserted_internal(state);
  sdio_mediachange(state->hsmci, state->cd);

  /* Enable card detect interrupts */

  sam_gpioirqenable(state->cdirq);
  return OK;
}

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

bool sam_cardinserted(int slotno)
{
  struct sam_hsmci_state_s *state;

  /* Get the HSMI description */

  state = sam_hsmci_state(slotno);
  if (state == NULL)
    {
      ferr("ERROR: No state for slotno %d\n", slotno);
      return false;
    }

  /* Return the state of the PIO pin */

  return sam_cardinserted_internal(state);
}

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

bool sam_writeprotected(int slotno)
{
  /* There are no write protect pins */

  return false;
}
