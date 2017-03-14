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

#include <nuttx/wireless/ieee80211/mmc_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_sdio.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCMF_DEVICE_RESET_DELAY_MS 10
#define BCMF_DEVICE_START_DELAY_MS 10
#define BCMF_CLOCK_SETUP_DELAY_MS  500

#define SDIO_FN1_CHIPCLKCSR     0x1000E /* Clock Control Source Register */
#define SDIO_FN1_PULLUP         0x1000F /* Pull-up Control Register for cmd, D0-2 lines */

#define SDIO_FN1_CHIPCLKCSR_FORCE_ALP           0x01
#define SDIO_FN1_CHIPCLKCSR_FORCE_HT            0x02
#define SDIO_FN1_CHIPCLKCSR_FORCE_ILP           0x04
#define SDIO_FN1_CHIPCLKCSR_ALP_AVAIL_REQ       0x08
#define SDIO_FN1_CHIPCLKCSR_HT_AVAIL_REQ        0x10
#define SDIO_FN1_CHIPCLKCSR_FORCE_HW_CLKREQ_OFF 0x20
#define SDIO_FN1_CHIPCLKCSR_ALP_AVAIL           0x40
#define SDIO_FN1_CHIPCLKCSR_HT_AVAIL            0x80

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

static int  bcmf_transfer_bytes(FAR struct bcmf_dev_s *priv, bool write,
                                uint8_t function, uint32_t address,
                                uint8_t *buf, unsigned int len);

static int  bcmf_read_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                          uint32_t address, uint8_t *reg,
                          unsigned int len);

static int  bcmf_write_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                          uint32_t address, uint32_t reg,
                          unsigned int len);

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
 * Name: bcmf_transfer_bytes
 ****************************************************************************/

int bcmf_transfer_bytes(FAR struct bcmf_dev_s *priv, bool write,
                        uint8_t function, uint32_t address,
                        uint8_t *buf, unsigned int len)
{
  /*  Use rw_io_direct method if len is 1 */

  if (len == 1)
    {
      return sdio_io_rw_direct(priv->sdio_dev, write,
                               function, address, *buf, buf);
    }

    // return sdio_io_rw_extended(priv->sdio_dev, write,
    //                            function, address, *buf, buf);
    return -EINVAL;
}

/****************************************************************************
 * Name: bcmf_read_reg
 ****************************************************************************/

int bcmf_read_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                  uint32_t address, uint8_t *reg, unsigned int len)
{
  return bcmf_transfer_bytes(priv, false, function, address, reg, len);
}

/****************************************************************************
 * Name: bcmf_write_reg
 ****************************************************************************/

int bcmf_write_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                   uint32_t address, uint32_t reg, unsigned int len)
{
  if (len > 4)
    {
      return -EINVAL;
    }

  return bcmf_transfer_bytes(priv, true, function, address,
                             (uint8_t*)&reg, len);
}

/****************************************************************************
 * Name: bcmf_probe
 ****************************************************************************/

int bcmf_probe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint8_t value;
  int loops;

  /* Probe sdio card compatible device */

  ret = sdio_probe(priv->sdio_dev);
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Enable bus FN1 */

  ret = sdio_enable_function(priv->sdio_dev, 1);
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Set FN0 / FN1 / FN2 default block size */

  ret = sdio_set_blocksize(priv->sdio_dev, 0, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  ret = sdio_set_blocksize(priv->sdio_dev, 1, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  ret = sdio_set_blocksize(priv->sdio_dev, 2, 64);
  if (ret != OK)
    {
      goto exit_error;
    }

  /* Enable device interrupts for FN0, FN1 and FN2 */

  ret = sdio_io_rw_direct(priv->sdio_dev, true, 0, SDIO_CCCR_INTEN,
                          (1 << 0) | (1 << 1) | (1 << 2), NULL);
  if (ret != OK)
    {
      goto exit_error;
    }


  /* Default device clock speed is up to 25 Mhz
   * We could set EHS bit to operate at a clock rate up to 50 Mhz */

  SDIO_CLOCK(priv->sdio_dev, CLOCK_SD_TRANSFER_4BIT);
  up_mdelay(BCMF_CLOCK_SETUP_DELAY_MS);

  /* Wait for function 1 to be ready */

  loops = 10;
  while (--loops > 0)
    {
      up_mdelay(1);

      ret = sdio_io_rw_direct(priv->sdio_dev, false, 0, SDIO_CCCR_IORDY, 0, &value);
      if (ret != OK)
        {
          return ret;
        }

      if (value & (1 << 1))
        {
          /* Function 1 is ready */
          break;
        }
    }

  if (loops <= 0)
    {
      return -ETIMEDOUT;
    }

  _info("sdio fn1 ready\n");

  return OK;

exit_error:

  _err("ERROR: failed to probe device %d\n", priv->minor);
  return ret;
}

/****************************************************************************
 * Name: bcmf_businitialize
 ****************************************************************************/

 int bcmf_businitialize(FAR struct bcmf_dev_s *priv)
{
  int ret;
  int loops;

  /* Send Active Low-Power clock request */

  ret = bcmf_write_reg(priv, 1, SDIO_FN1_CHIPCLKCSR,
            SDIO_FN1_CHIPCLKCSR_FORCE_HW_CLKREQ_OFF |
            SDIO_FN1_CHIPCLKCSR_ALP_AVAIL_REQ |
            SDIO_FN1_CHIPCLKCSR_FORCE_ALP, 1);

  if (ret != OK)
    {
      return ret;;
    }

  loops = 10;
  while (--loops > 0)
    {
      uint8_t value;

      up_mdelay(10);
      ret = bcmf_read_reg(priv, 1, SDIO_FN1_CHIPCLKCSR, &value, 1);

      if (ret != OK)
        {
          return ret;
        }

      if (value & SDIO_FN1_CHIPCLKCSR_ALP_AVAIL)
        {
          /* Active Low-Power clock is ready */
          break;
        }
    }

  if (loops <= 0)
    {
      _err("failed to enable ALP\n");
      return -ETIMEDOUT;
    }

  /* Clear Active Low-Power clock request */

  ret = bcmf_write_reg(priv, 1, SDIO_FN1_CHIPCLKCSR, 0, 1);
  if (ret != OK)
    {
      return ret;
    }

  /* Disable pull-ups on SDIO cmd, d0-2 lines */

  ret = bcmf_write_reg(priv, 1, SDIO_FN1_PULLUP, 0, 1);
  if (ret != OK)
    {
      return ret;
    }

  /* Enable oob gpio interrupt */

  // bcmf_board_setup_oob_irq(priv->minor, bcmf_oob_irq, (void*)priv);

  /* Enable F2 interrupt only */

  /* Upload firmware */

  /* Enable function 2 */

  // ret = sdio_enable_function(priv->sdio_dev, 2);
  // if (ret != OK)
  //   {
  //     goto exit_error;
  //   }


  return OK;
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

  /* Initialize device */

  ret = bcmf_businitialize(priv);

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
