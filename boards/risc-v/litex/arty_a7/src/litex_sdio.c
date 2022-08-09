/****************************************************************************
 * boards/risc-v/litex/arty_a7/src/litex_sdio.c
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

#include "litex.h"
#include "arty_a7.h"
#include "litex_sdio.h"

#ifdef HAVE_SDMMC

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: litex_sdio_card_isr_callback
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

static void litex_sdio_card_isr_callback(void *arg)
{
  bool cd;

  cd = litex_cardinserted(SDIO_SLOTNO);
  finfo("Card detect: %d\n", cd);
  sdio_mediachange(arg, cd);

#ifdef HAVE_AUTOMOUNTER
  /* Let the automounter know about the insertion event */

  litex_automount_event(SDIO_SLOTNO, cd);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: litex_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

bool litex_cardinserted(int slotno)
{
  return litex_sdio_get_card_detect();
}

/****************************************************************************
 * Name: litex_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int litex_sdio_initialize(void)
{
  int ret = 0;
  struct sdio_dev_s *sdio_dev;

  finfo("Initializing SDIO slot %d\n", SDIO_SLOTNO);
  sdio_dev = sdio_initialize(SDIO_SLOTNO);
  if (sdio_dev == NULL)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", SDIO_SLOTNO);
      return -ENODEV;
    }

  finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", SDIO_MINOR);
  ret = mmcsd_slotinitialize(SDIO_MINOR, sdio_dev);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  litex_sdio_set_card_isr(sdio_dev, &litex_sdio_card_isr_callback, sdio_dev);

  finfo("Successfully bound SDIO to the MMC/SD driver\n");

  /* Assume that the SD card is inserted.
   * The Arty A7 board doesnt have the CD pin wired.
   */

  sdio_mediachange(sdio_dev, litex_sdio_get_card_detect());

  return OK;
}

#endif /* HAVE_SDMMC */
