/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_hsmci.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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
static int sam_hsmci_cardetect_int(int irq, void *regs, FAR void *arg)
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
