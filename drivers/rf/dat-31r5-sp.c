/****************************************************************************
 * drivers/rf/dat-31r5-sp.c
 * Character driver for the Mini-Circuits DAT-31R5-SP+ digital step
 * attenuator.
 *
 *   Copyright (C) 2019, Augusto Fraga Giachero. All rights reserved.
 *   Author: Augusto Fraga Giachero <afg@augustofg.net>
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/rf/attenuator.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SPI) && defined(CONFIG_RF_DAT31R5SP)

#ifndef CONFIG_DAT31R5SP_SPI_FREQUENCY
#  define CONFIG_DAT31R5SP_SPI_FREQUENCY 1000000
#endif

#define DAT31R5SP_SPI_MODE (SPIDEV_MODE0) /* SPI Mode 0: CPOL=0,CPHA=0 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dat31r5sp_dev_s
{
  FAR struct spi_dev_s *spi;    /* Saved SPI driver instance */
  int spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int dat31r5sp_open(FAR struct file *filep);
static int dat31r5sp_close(FAR struct file *filep);
static ssize_t dat31r5sp_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t dat31r5sp_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int dat31r5sp_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dat31r5sp_fops =
{
  dat31r5sp_open,
  dat31r5sp_close,
  dat31r5sp_read,
  dat31r5sp_write,
  NULL,
  dat31r5sp_ioctl,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dat31r5sp_configspi
 *
 * Description:
 *   Configure the SPI instance for to match the DAT-31R5-SP+
 *   specifications
 *
 ****************************************************************************/

static inline void dat31r5sp_configspi(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, DAT31R5SP_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_DAT31R5SP_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: dat31r5sp_set_attenuation
 *
 * Description:
 *   Set the attenuation level in dB (16.16 bits fixed point).
 *
 ****************************************************************************/

static void dat31r5sp_set_attenuation(FAR struct dat31r5sp_dev_s *priv,
                                      b16_t attenuation)
{
  SPI_LOCK(priv->spi, true);

  dat31r5sp_configspi(priv->spi);

  SPI_SELECT(priv->spi, priv->spidev, false);

  /* Convert the attenuation value from 16.16 bits to 5.1 bits. */

  SPI_SEND(priv->spi, (uint8_t)(attenuation >> 15));

  /* Send a pulse to the LE pin */

  SPI_SELECT(priv->spi, priv->spidev, true);
  up_udelay(1);
  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: dat31r5sp_open
 *
 * Description:
 *   This function is called whenever the DAT-31R5-SP+ device is
 *   opened.
 *
 ****************************************************************************/

static int dat31r5sp_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: dat31r5sp_close
 *
 * Description:
 *   This function is called whenever the DAT-31R5-SP+ device is
 *   closed.
 *
 ****************************************************************************/

static int dat31r5sp_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: dat31r5sp_write
 *
 * Description:
 *   Write is not permited, only IOCTLs.
 ****************************************************************************/

static ssize_t dat31r5sp_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: dat31r5sp_read
 *
 * Description:
 *   Read is ignored.
 ****************************************************************************/

static ssize_t dat31r5sp_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: dat31r5sp_ioctl
 *
 * Description:
 *   The only available ICTL is RFIOC_SETATT. It expects a struct
 *   attenuator_control* as the argument to set the attenuation
 *   level. The channel is ignored as the DAT-31R5-SP+ has just a
 *   single attenuator.
 ****************************************************************************/

static int dat31r5sp_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct dat31r5sp_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case RFIOC_SETATT:
        {
          FAR struct attenuator_control *att =
            (FAR struct attenuator_control *)((uintptr_t)arg);
          DEBUGASSERT(att != NULL);
          dat31r5sp_set_attenuation(priv, att->attenuation);
        }
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dat31r5sp_register
 *
 * Description:
 *   Register the dat31r5sp character device as 'devpath'.
 *
 ****************************************************************************/

int dat31r5sp_register(FAR const char *devpath,
                       FAR struct spi_dev_s *spi,
                       int spidev)
{
  FAR struct dat31r5sp_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the DAT-31R5-SP+ device structure */

  priv = (FAR struct dat31r5sp_dev_s *)
      kmm_malloc(sizeof(struct dat31r5sp_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi    = spi;
  priv->spidev = spidev;

  /* Clear the LE pin */

  SPI_SELECT(priv->spi, priv->spidev, false);

  /* Register the character driver */

  ret = register_driver(devpath, &g_dat31r5sp_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif
