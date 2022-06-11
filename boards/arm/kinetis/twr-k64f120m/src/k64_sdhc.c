/****************************************************************************
 * boards/arm/kinetis/twr-k64f120m/src/k64_sdhc.c
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

/* A micro Secure Digital (SD) card slot is available on the FRDM-K64F
 * connected to the SD Host Controller (SDHC) signals of the MCU.
 * This slot will accept micro format SD memory cards.
 * The SD card detect pin (PTE6) is an open switch that shorts with VDD when
 * card is inserted.
 *
 *   ------------ ------------- --------
 *    SD Card Slot Board Signal  K64F Pin
 *    ------------ ------------- --------
 *    DAT0         SDHC0_D0      PTE0
 *    DAT1         SDHC0_D1      PTE1
 *    DAT2         SDHC0_D2      PTE5
 *    CD/DAT3      SDHC0_D3      PTE4
 *    CMD          SDHC0_CMD     PTE3
 *    CLK          SDHC0_DCLK    PTE2
 *    SWITCH       D_CARD_DETECT PTE6
 *    ------------ ------------- --------
 *
 * There is no Write Protect pin available to the K64F.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "kinetis.h"

#include "twrk64.h"

#ifdef HAVE_MMCSD

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds static information unique to one SDHC peripheral */

struct k64_sdhc_state_s
{
  struct sdio_dev_s *sdhc;    /* R/W device handle */
  bool inserted;              /* TRUE: card is inserted */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HSCMI device state */

static struct k64_sdhc_state_s g_sdhc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k64_mediachange
 ****************************************************************************/

static void k64_mediachange(void)
{
  bool inserted;

  /* Get the current value of the card detect pin.  This pin is pulled up on
   * board.  So low means that a card is present.
   */

  inserted = !kinetis_gpioread(GPIO_SD_CARDDETECT);
  mcinfo("inserted: %s\n", inserted ? "Yes" : "No");

  /* Has the pin changed state? */

  if (inserted != g_sdhc.inserted)
    {
      mcinfo("Media change: %d->%d\n",  g_sdhc.inserted, inserted);

      /* Yes..
       * perform the appropriate action (this might need some debounce).
       */

      g_sdhc.inserted = inserted;
      sdhc_mediachange(g_sdhc.sdhc, inserted);

#ifdef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT
      /* Let the automounter know about the insertion event */

      k64_automount_event(k64_cardinserted());
#endif
    }
}

/****************************************************************************
 * Name: k64_cdinterrupt
 ****************************************************************************/

static int k64_cdinterrupt(int irq, void *context, void *arg)
{
  /* All of the work is done by k64_mediachange() */

  k64_mediachange();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k64_sdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

int k64_sdhc_initialize(void)
{
  int ret;

  /* Configure GPIO pins */

  kinetis_pinconfig(GPIO_SD_CARDDETECT);

  /* Attached the card detect interrupt (but don't enable it yet) */

  kinetis_pinirqattach(GPIO_SD_CARDDETECT, k64_cdinterrupt, NULL);

  /* Configure the write protect GPIO -- None */

  /* Mount the SDHC-based MMC/SD block driver */

  /* First, get an instance of the SDHC interface */

  mcinfo("Initializing SDHC slot %d\n", MMCSD_SLOTNO);

  g_sdhc.sdhc = sdhc_initialize(MMCSD_SLOTNO);
  if (!g_sdhc.sdhc)
    {
      mcerr("ERROR: Failed to initialize SDHC slot %d\n", MMCSD_SLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDHC interface to the MMC/SD driver */

  mcinfo("Bind SDHC to the MMC/SD driver, minor=%d\n", MMSCD_MINOR);

  ret = mmcsd_slotinitialize(MMSCD_MINOR, g_sdhc.sdhc);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfully bound SDHC to the MMC/SD driver\n");

  /* Handle the initial card state */

  k64_mediachange();

  /* Enable CD interrupts to handle subsequent media changes */

  kinetis_pinirqenable(GPIO_SD_CARDDETECT);

  /* Initialize automount system if configured */

#ifdef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT
  k64_automount_initialize();
#endif

  return OK;
}

/****************************************************************************
 * Name: k64_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool k64_cardinserted(void)
{
  bool inserted;

  /* Get the current value of the card detect pin.  This pin is pulled up on
   * board.  So low means that a card is present.
   */

  inserted = !kinetis_gpioread(GPIO_SD_CARDDETECT);
  mcinfo("inserted: %s\n", inserted ? "Yes" : "No");
  return inserted;
}
#endif

/****************************************************************************
 * Name: k64_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool k64_writeprotected(void)
{
  /* There are no write protect pins */

  return false;
}
#endif

#endif /* HAVE_MMCSD */
