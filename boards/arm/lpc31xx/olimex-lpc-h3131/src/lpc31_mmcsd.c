/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/src/lpc31_mmcsd.c
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

#include "lpc31.h"

#include "lpc_h3131.h"

#ifdef HAVE_MMCSD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_mmcsd_initialize
 *
 * Description:
 *   Create the SDIO-based MMC/SD device
 *
 ****************************************************************************/

int lpc31_mmcsd_initialize(int slot, int minor)
{
  struct sdio_dev_s *sdio;
  int ret;

  /* First, get an instance of the SDIO interface */

  finfo("Initializing SDIO slot %d\n", slot);
  sdio = sdio_initialize(slot);
  if (!sdio)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", slot);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the MMC/SD driver */

  finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", minor);
  ret = mmcsd_slotinitialize(minor, sdio);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  finfo("Successfully bound SDIO to the MMC/SD driver\n");

  /* Then let's guess and say that there is a card in the slot.
   * I need to check to see if the LPC-H3131 board supports a GPIO to detect
   * if there is a card in the slot.
   */

  sdio_mediachange(sdio, true);
  return OK;
}

#endif /* HAVE_MMCSD */
