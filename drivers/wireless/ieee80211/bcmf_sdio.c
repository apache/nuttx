/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_sdio.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sdio.h>
#include <nuttx/arch.h>

#include <nuttx/wireless/ieee80211/bcmf_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCMF_DEVICE_RESET_DELAY_MS 10
#define BCMF_DEVICE_START_DELAY_MS 10
#define BCMF_DEVICE_IDLE_DELAY_MS  50

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is contains the unique state of the Broadcom FullMAC driver */

struct bcmf_dev_s
{
  FAR struct sdio_dev_s *sdio_dev; /* The SDIO device bound to this instance */
  int minor;                       /* Device minor number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  bcmf_sendcmdpoll(FAR struct bcmf_dev_s *priv,
              uint32_t cmd, uint32_t arg);

static int  bcmf_probe(FAR struct bcmf_dev_s *priv);
static int  bcmf_hwinitialize(FAR struct bcmf_dev_s *priv);
static void bcmf_hwuninitialize(FAR struct bcmf_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_sendcmdpoll
 ****************************************************************************/

int bcmf_sendcmdpoll(FAR struct bcmf_dev_s *priv, uint32_t cmd, uint32_t arg)
{
  int ret;

  /* Send the command */

  ret = SDIO_SENDCMD(priv->sdio_dev, cmd, arg);
  if (ret == OK)
    {
      /* Then poll-wait until the response is available */

      ret = SDIO_WAITRESPONSE(priv->sdio_dev, cmd);
      if (ret != OK)
        {
          _err("ERROR: Wait for response to cmd: %08x failed: %d\n",
               cmd, ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_probe
 ****************************************************************************/

int bcmf_probe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t data = 0;

  /* Set device state from reset to idle */

  bcmf_sendcmdpoll(priv, MMCSD_CMD0, 0);
  up_mdelay(BCMF_DEVICE_START_DELAY_MS);

  /* Send IO_SEND_OP_COND command */

  ret = bcmf_sendcmdpoll(priv, SDIO_CMD5, 0);

  if (ret != OK)
    {
      goto exit_error;
    }

  /* Receive R4 response */

  ret = SDIO_RECVR4(priv->sdio_dev, SDIO_CMD5, &data);

  if (ret != OK)
    {
      goto exit_error;
    }

  /* Broadcom chips have 2 additional functions and wide voltage range */

  if ((((data >> 28) & 7) != 2) ||
      (((data >> 8)  & 0xff80) != 0xff80))
    {
      goto exit_error;
    }

  return OK;

exit_error:

  _err("ERROR: failed to probe device %d\n", priv->minor);
  return ret;
}

/****************************************************************************
 * Name: bcmf_hwinitialize
 ****************************************************************************/

int bcmf_hwinitialize(FAR struct bcmf_dev_s *priv)
{
  /* Attach and prepare SDIO interrupts */

  SDIO_ATTACH(priv->sdio_dev);

  /* Set ID mode clocking (<400KHz) */

  SDIO_CLOCK(priv->sdio_dev, CLOCK_IDMODE);

  /* Configure hardware */

  bcmf_board_initialize(priv->minor);

  /* Reset and power device */

  bcmf_board_reset(priv->minor, true);
  bcmf_board_power(priv->minor, true);
  up_mdelay(BCMF_DEVICE_RESET_DELAY_MS);
  bcmf_board_reset(priv->minor, false);

  /* Wait for device to start */

  up_mdelay(BCMF_DEVICE_START_DELAY_MS);

  return OK;
}

/****************************************************************************
 * Name: bcmf_hwuninitialize
 ****************************************************************************/

void bcmf_hwuninitialize(FAR struct bcmf_dev_s *priv)
{
  /*  Shutdown device */

  bcmf_board_power(priv->minor, false);
  bcmf_board_reset(priv->minor, true);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_sdio_initialize
 ****************************************************************************/

int bcmf_sdio_initialize(int minor, FAR struct sdio_dev_s *dev)
{
  FAR struct bcmf_dev_s *priv;
  int ret;

  _info("minor: %d\n", minor);

  /* Allocate a bcmf device structure */

  priv = (FAR struct bcmf_dev_s *)kmm_malloc(sizeof(*priv));

  if (!priv)
    {
      return -ENOMEM;
    }

  /* Initialize bcmf device structure */

  memset(priv, 0, sizeof(*priv));
  priv->sdio_dev = dev;
  priv->minor = minor;

  /* Initialize device hardware */

  ret = bcmf_hwinitialize(priv);

  if (ret != OK)
    {
      goto exit_free_priv;
    }

  /* Probe device */

  ret = bcmf_probe(priv);

  if (ret != OK)
    {
      goto exit_uninit_hw;
    }

  /* TODO Create a wlan device name and register network driver here */

  return OK;

exit_uninit_hw:
  bcmf_hwuninitialize(priv);

exit_free_priv:
  kmm_free(priv);
  return ret;
}
