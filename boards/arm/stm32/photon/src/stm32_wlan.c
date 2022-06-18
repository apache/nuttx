/****************************************************************************
 * boards/arm/stm32/photon/src/stm32_wlan.c
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

#include <debug.h>

#include <nuttx/wireless/ieee80211/bcmf_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32_sdio.h"

#include "photon.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sdio_dev_s *g_sdio_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_board_reset
 ****************************************************************************/

void bcmf_board_reset(int minor, bool reset)
{
  if (minor != SDIO_WLAN0_MINOR)
    {
      return;
    }

  stm32_gpiowrite(GPIO_WLAN0_RESET, !reset);
}

/****************************************************************************
 * Name: bcmf_board_power
 ****************************************************************************/

void bcmf_board_power(int minor, bool power)
{
  /* Power signal is not used on Photon board */
}

/****************************************************************************
 * Name: bcmf_board_initialize
 ****************************************************************************/

void bcmf_board_initialize(int minor)
{
  if (minor != SDIO_WLAN0_MINOR)
    {
      return;
    }

  /* Configure reset pin */

  stm32_configgpio(GPIO_WLAN0_RESET);

  /* Put wlan chip in reset state */

  bcmf_board_reset(minor, true);
}

/****************************************************************************
 * Name: bcmf_board_setup_oob_irq
 ****************************************************************************/

void bcmf_board_setup_oob_irq(int minor, int (*func)(void *), void *arg)
{
  if (minor != SDIO_WLAN0_MINOR)
    {
      return;
    }

  /* Configure SDIO card in-band interrupt callback */

  if (g_sdio_dev != NULL)
    {
      sdio_set_sdio_card_isr(g_sdio_dev, func, arg);
    }
}

/****************************************************************************
 * Name: bcmf_board_etheraddr
 ****************************************************************************/

bool bcmf_board_etheraddr(struct ether_addr *ethaddr)
{
  return false;
}

/****************************************************************************
 * Name: photon_wlan_initialize
 ****************************************************************************/

int photon_wlan_initialize()
{
  int ret;

  /* Initialize sdio interface */

  wlinfo("Initializing SDIO slot %d\n", SDIO_WLAN0_SLOTNO);

  g_sdio_dev = sdio_initialize(SDIO_WLAN0_SLOTNO);

  if (!g_sdio_dev)
    {
      wlerr("ERROR: Failed to initialize SDIO with slot %d\n",
             SDIO_WLAN0_SLOTNO);
      return ERROR;
    }

  /* Bind the SDIO interface to the bcmf driver */

  ret = bcmf_sdio_initialize(SDIO_WLAN0_MINOR, g_sdio_dev);

  if (ret != OK)
    {
      wlerr("ERROR: Failed to bind SDIO to bcmf driver\n");

      /* FIXME deinitialize sdio device */

      return ERROR;
    }

  return OK;
}
