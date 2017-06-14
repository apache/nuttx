/****************************************************************************
 * configs/photon/src/stm32_wlan.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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

#include <debug.h>

#include <nuttx/wireless/ieee80211/bcmf_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32_sdio.h"

#include "photon.h"

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

void bcmf_board_setup_oob_irq(int minor, xcpt_t func, void *arg)
{
  if (minor != SDIO_WLAN0_MINOR)
    {
      return;
    }

  /* Configure interrupt pin */

  stm32_configgpio(GPIO_WLAN0_OOB_INT);

  stm32_gpiosetevent(GPIO_WLAN0_OOB_INT, true, false, false, func, arg);
}

/****************************************************************************
 * Name: photon_wlan_initialize
 ****************************************************************************/

int photon_wlan_initialize()
{
  int ret;
  struct sdio_dev_s *sdio_dev;

  /* Initialize sdio interface */

  wlinfo("Initializing SDIO slot %d\n", SDIO_WLAN0_SLOTNO);

  sdio_dev = sdio_initialize(SDIO_WLAN0_SLOTNO);

  if (!sdio_dev)
    {
      wlerr("ERROR: Failed to initialize SDIO with slot %d\n",
             SDIO_WLAN0_SLOTNO);
      return ERROR;
    }

  /* Bind the SDIO interface to the bcmf driver */

  ret = bcmf_sdio_initialize(SDIO_WLAN0_MINOR, sdio_dev);

  if (ret != OK)
    {
      wlerr("ERROR: Failed to bind SDIO to bcmf driver\n");

      /* FIXME deinitialize sdio device */
      return ERROR;
    }

  return OK;
}
