/****************************************************************************
 * drivers/analog/ad5410.c
 *
 *   Copyright (C) 2010, 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_DAC_AD5410)

#define AD5410_REG_NOP    0x00
#define AD5410_REG_WR     0x01
#define AD5410_REG_RD     0x02
#define AD5410_REG_CMD    0x55
#define AD5410_REG_RST    0x56

#define AD5410_CMD_REXT       (1 << 13)
#define AD5410_CMD_OUTEN      (1 << 12)
#define AD5410_CMD_SRCLK(x)   ((x) << 8)
#define AD5410_CMD_SRSTEP(x)  ((x) << 5)
#define AD5410_CMD_SREN       (1 << 4)
#define AD5410_CMD_DCEN       (1 << 3)
#define AD5410_CMD_420MA      0x05
#define AD5410_CMD_020MA      0x06
#define AD5410_CMD_024MA      0x07

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  int devno;
  FAR struct spi_dev_s  *spi;      /* Cached SPI device reference */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void dac_lock(FAR struct spi_dev_s *spi);
static void dac_unlock(FAR struct spi_dev_s *spi);

/* DAC methods */

static void dac_reset(FAR struct dac_dev_s *dev);
static int  dac_setup(FAR struct dac_dev_s *dev);
static void dac_shutdown(FAR struct dac_dev_s *dev);
static void dac_txint(FAR struct dac_dev_s *dev, bool enable);
static int  dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);
static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  dac_reset,    /* ao_reset */
  dac_setup,    /* ao_setup */
  dac_shutdown, /* ao_shutdown */
  dac_txint,    /* ao_txint */
  dac_send,     /* ao_send */
  dac_ioctl     /* ao_ioctl */
};

static struct up_dev_s g_dacpriv;

static struct dac_dev_s g_dacdev =
{
  &g_dacops,    /* ad_ops */
  &g_dacpriv    /* ad_priv */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void dac_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 400000);
}

/****************************************************************************
 * Name: dac_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void dac_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void dac_reset(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts. Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int dac_setup(FAR struct dac_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->ad_priv;
  FAR struct spi_dev_s *spi = priv->spi;

  dac_lock(spi);

  SPI_SELECT(spi, priv->devno, true);
  SPI_SEND(spi, AD5410_REG_CMD);
  SPI_SEND(spi, (AD5410_CMD_OUTEN | AD5410_CMD_420MA) >> 8);
  SPI_SEND(spi, AD5410_CMD_OUTEN | AD5410_CMD_420MA);
  SPI_SELECT(spi, priv->devno, false);

  dac_unlock(spi);
  return OK;
}

/****************************************************************************
 * Name: dac_shutdown
 *
 * Description:
 *   Disable the DAC.  This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void dac_shutdown(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: dac_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void dac_txint(FAR struct dac_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: dac_send
 *
 * Description:
 *
 ****************************************************************************/

static int dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->ad_priv;
  FAR struct spi_dev_s *spi = priv->spi;

  dac_lock(spi);

  SPI_SELECT(spi,  priv->devno,  true);
  SPI_SEND(spi, AD5410_REG_WR);
  SPI_SEND(spi, (uint8_t)(msg->am_data >> 24));
  SPI_SEND(spi, (uint8_t)(msg->am_data >> 16));
  SPI_SELECT(spi, priv->devno, false);

  dac_unlock(spi);
  dac_txdone(&g_dacdev);
  return 0;
}

/****************************************************************************
 * Name: dac_ioctl
 *
 * Description:
 *  All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  _err("ERROR: Fix me; Not Implemented\n");
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ad5410cinitialize
 *
 * Description:
 *   Initialize the selected DAC port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple DAC interfaces)
 *
 * Returned Value:
 *   Valid ad5410 device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct dac_dev_s *up_ad5410initialize(FAR struct spi_dev_s *spi,
                                          unsigned int devno)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)g_dacdev.ad_priv;

  priv->spi   = spi;
  priv->devno = devno;
  return &g_dacdev;
}
#endif
