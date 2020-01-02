/****************************************************************************
 * drivers/sensors/apa102.c
 * Character driver to control LED strips with APA102.
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * More info:
 * https://cpldcpu.com/2014/11/30/understanding-the-apa102-superled/
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
#include <nuttx/leds/apa102.h>

#if defined(CONFIG_SPI) && defined(CONFIG_LEDS_APA102)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct apa102_dev_s
{
  FAR struct spi_dev_s *spi; /* SPI interface */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void apa102_configspi(FAR struct spi_dev_s *spi);
static inline void apa102_write32(FAR struct apa102_dev_s *priv,
                                  FAR uint32_t value);

/* Character driver methods */

static int     apa102_open(FAR struct file *filep);
static int     apa102_close(FAR struct file *filep);
static ssize_t apa102_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t apa102_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_apa102fops =
{
  apa102_open,   /* open */
  apa102_close,  /* close */
  apa102_read,   /* read */
  apa102_write,  /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  NULL           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apa102_configspi
 *
 * Description:
 *   Set the SPI bus configuration
 *
 ****************************************************************************/

static inline void apa102_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the APA102 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, APA102_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Name: apa102_write32
 *
 * Description:
 *   Write 32-bit to APA102
 *
 ****************************************************************************/

static inline void apa102_write32(FAR struct apa102_dev_s *priv,
                                  FAR uint32_t value)
{
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);

  apa102_configspi(priv->spi);

  /* Note: APA102 doesn't use chip select */
  /* Send 32 bits (4 bytes) */

  SPI_SEND(priv->spi, (value & 0xff));
  SPI_SEND(priv->spi, ((value & 0xff00) >> 8));
  SPI_SEND(priv->spi, ((value & 0xff0000) >> 16));
  SPI_SEND(priv->spi, ((value & 0xff000000) >> 24));

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: apa102_open
 *
 * Description:
 *   This function is called whenever the APA102 device is opened.
 *
 ****************************************************************************/

static int apa102_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: apa102_close
 *
 * Description:
 *   This routine is called when the LM-75 device is closed.
 *
 ****************************************************************************/

static int apa102_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: apa102_read
 ****************************************************************************/

static ssize_t apa102_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: apa102_write
 ****************************************************************************/

static ssize_t apa102_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct apa102_dev_s *priv = inode->i_private;
  FAR uint32_t *leds = (FAR uint32_t *) buffer;
  uint16_t nleds;
  uint16_t i;

  if (!leds)
    {
      snerr("ERROR: Buffer is null\n");
      return -1;
    }

  /* At least one LED, so 4 bytes */

  if (buflen < 4)
    {
      snerr("ERROR: You need to control at least 1 LED!\n");
      return -1;
    }

  /* It needs to be multiple of 4 */

  if ((buflen % 4) != 0)
    {
      snerr("ERROR: Each LED uses 4 bytes, so (buflen % 4) needs to be 0!\n");
      return -1;
    }

  nleds = buflen / 4;

  /* Send a start of frame */

  apa102_write32(priv, APA102_START_FRAME);

  /* Write all LEDs */

  for (i = 0; i < nleds; i++)
    {
      /* Send the LED color, the LSB is the Bright and Header */

      apa102_write32(priv, (uint32_t) (leds[i] | APA102_HEADER_FRAME));
    }

  /* Send an end of frame */

  apa102_write32(priv, APA102_END_FRAME);

  for (i = 0; i < (1 + nleds/32); i++)
    {
      apa102_write32(priv, 0);
    }

  /* All bytes were written */

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apa102_register
 *
 * Description:
 *   Register the APA102 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APA102
 *   addr    - The I2C address of the LM-75.  The base I2C address of the
 *             APA102 is 0x48.  Bits 0-3 can be controlled to get 8 unique
 *             addresses from 0x48 through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apa102_register(FAR const char *devpath, FAR struct spi_dev_s *spi)
{
  FAR struct apa102_dev_s *priv;
  int ret;

  /* Initialize the APA102 device structure */

  priv = (FAR struct apa102_dev_s *)kmm_malloc(sizeof(struct apa102_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = spi;

  /* Configure SPI for the APA102 */

  apa102_configspi(spi);

  /* Register the character driver */

  ret = register_driver(devpath, &g_apa102fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_APA102 */
