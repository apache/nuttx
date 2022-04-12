/****************************************************************************
 * drivers/leds/apa102.c
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

/* Character driver to control LED strips with APA102.
 *
 * More info:
 * https://cpldcpu.com/2014/11/30/understanding-the-apa102-superled/
 */

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

static ssize_t apa102_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t apa102_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_apa102fops =
{
  NULL,          /* open */
  NULL,          /* close */
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
 * Name: apa102_read
 ****************************************************************************/

static ssize_t apa102_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
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
      snerr("ERROR: Each LED uses 4 bytes, so (buflen % 4)"
            " needs to be 0!\n");
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

  for (i = 0; i < (1 + nleds / 32); i++)
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
 *   spi     - An instance of the SPI interface to use to communicate with
 *             APA102
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
