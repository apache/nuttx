/****************************************************************************
 * drivers/sensors/max6675.c
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

/* * Character driver for the Maxim MAX6675 Thermocouple-to-Digital Converter
 *
 * NOTE: Some Maxim MAX6675 chips have an issue it report value 25% lower
 * of real temperature, for more info read this thread:
 * http://www.eevblog.com/forum/projects/max6675-temperature-error/
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/max6675.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MAX6675)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

#define MAX6675_THREE_STATE   (1 << 0)
#define MAX6675_DEV_ID        (1 << 1)
#define MAX6675_OPEN_CIRCUIT  (1 << 2)
#define MAX6675_TEMP_COUPLE   0x7ff8

struct max6675_dev_s
{
  FAR struct spi_dev_s *spi;           /* Saved SPI driver instance */
  int16_t temp;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    max6675_lock(FAR struct spi_dev_s *spi);
static void    max6675_unlock(FAR struct spi_dev_s *spi);

/* Character driver methods */

static ssize_t max6675_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t max6675_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max6675fops =
{
  NULL,            /* open */
  NULL,            /* close */
  max6675_read,    /* read */
  max6675_write,   /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max6675_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void max6675_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 4000000);
}

/****************************************************************************
 * Name: max6675_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void max6675_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: max6675_read
 ****************************************************************************/

static ssize_t max6675_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct max6675_dev_s *priv  = inode->i_private;
  FAR uint16_t             *temp  = (FAR uint16_t *) buffer;
  int                       ret   = 2;
  int16_t                   regmsb;
  int16_t                   regval;

  /* Check for issues */

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -EINVAL;
    }

  if (buflen != 2)
    {
      snerr("ERROR:");
      snerr(" You can't read something other than 16 bits (2 bytes)\n");
      return -EINVAL;
    }

  /* Enable MAX6675's chip select */

  max6675_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(0), true);

  /* Read temperature */

  SPI_RECVBLOCK(priv->spi, &regmsb, 2);

  /* Disable MAX6675's chip select */

  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(0), false);
  max6675_unlock(priv->spi);

  regval  = (regmsb & 0xff00) >> 8;
  regval |= (regmsb & 0xff) << 8;

  sninfo("Read from MAX6675 = 0x%04X\n", regval);

  /* Verify if the device ID bit is really zero */

  if (regval & MAX6675_DEV_ID)
    {
      snerr("ERROR: The Device ID bit needs to be 0 !\n");
      ret = -EINVAL;
    }

  /* Detect if termocople input is open */

  if (regval & MAX6675_OPEN_CIRCUIT)
    {
      snerr("ERROR: The thermocouple input is not connected!\n");
      ret = -EINVAL;
    }

  /* Feed sensor data to entropy pool */

  add_sensor_randomness(regval);

  /* Get the temperature */

  *temp = (regval & MAX6675_TEMP_COUPLE) >> 3;

  /* Return two bytes, the temperature is fixed point Q10.2, then divide by 4
   * in your application in order to get real temperature in Celsius degrees.
   */

  return ret;
}

/****************************************************************************
 * Name: max6675_write
 ****************************************************************************/

static ssize_t max6675_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max6675_register
 *
 * Description:
 *   Register the MAX6675 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   spi - An instance of the SPU interface to use to communicate with
 *   MAX6675.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max6675_register(FAR const char *devpath, FAR struct spi_dev_s *spi)
{
  FAR struct max6675_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the MAX6675 device structure */

  priv = (FAR struct max6675_dev_s *)
          kmm_malloc(sizeof(struct max6675_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi        = spi;
  priv->temp       = 0;

  /* Register the character driver */

  ret = register_driver(devpath, &g_max6675fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_SENSORS_MAX6675 */
