/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_hsmci.c
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

#include "sam_gpio.h"
#include "sam_hsmci.h"

#include "sam4s-xplained-pro.h"

#ifdef HAVE_HSMCI

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
  bool initialized;           /* TRUE: HSMCI block driver is initialized */
  bool inserted;              /* TRUE: card is inserted */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HSCMI device state */

static struct sam_hsmci_state_s g_hsmci;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hsmci_cardetect_int
 *
 * Description:
 *   Card detect interrupt handler
 *
 * TODO: Any way to automatically moun/unmount filesystem based on card
 * detect status?  Yes... send a message or signal to an application.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
static int sam_hsmci_cardetect_int(int irq, void *regs, void *arg)
{
  bool inserted;

  /* Get the state of the GPIO pin */

  inserted = sam_cardinserted(0);

  /* Has the card detect state changed? */

  if (inserted != g_hsmci.inserted)
    {
      /* Yes... remember that new state and inform the HSMCI driver */

      g_hsmci.inserted = inserted;

      /* Report the new state to the SDIO driver */

      sdio_mediachange(g_hsmci.hsmci, inserted);
    }

  return OK;
}
#endif

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

int sam_hsmci_initialize(void)
{
  int ret;
  finfo("Initializing SDIO\n");

  /* Have we already initialized? */

  if (!g_hsmci.initialized)
    {
      /* Mount the SDIO-based MMC/SD block driver */

      /* First, get an instance of the SDIO interface */

      g_hsmci.hsmci = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
      if (!g_hsmci.hsmci)
        {
          ferr("ERROR: Failed to initialize SDIO\n");
          return -ENODEV;
        }

      /* Now bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_hsmci.hsmci);
      if (ret != OK)
        {
          ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
          return ret;
        }

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
      /* Initialize card-detect GPIO.  There is no write-protection GPIO. */

      sam_configgpio(GPIO_MCI_CD);

      /* Configure card detect interrupts */

      sam_gpioirq(GPIO_MCI_CD);
      irq_attach(MCI_CD_IRQ, sam_hsmci_cardetect_int, NULL);
      g_hsmci.inserted = sam_cardinserted(0);
#else
      g_hsmci.inserted = true; /* An assumption? */
#endif
      /* Then inform the HSMCI driver if there is or is not a card in the slot. */

      sdio_mediachange(g_hsmci.hsmci, g_hsmci.inserted);

      /* Now we are initialized */

      g_hsmci.initialized = true;

      /* Enable card detect interrupts */

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
      sam_gpioirqenable(MCI_CD_IRQ);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
bool sam_cardinserted(int slotno)
{
  bool removed;

  /* Get the state of the GPIO pin */

  removed = sam_gpioread(GPIO_MCI_CD);
  finfo("Slot %d inserted: %s\n", slotno, removed ? "NO" : "YES");

  return !removed;
}
#endif

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

#endif /* HAVE_HSMCI */
