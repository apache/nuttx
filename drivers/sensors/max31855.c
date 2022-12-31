/****************************************************************************
 * drivers/sensors/max31855.c
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

/* Character driver for the Maxim MAX31855 Thermocouple-to-Digital Converter
 *
 * NOTE: Some Maxim MAX31855 chips have an issue it report value 25% lower
 * of real temperature, for more info read this thread:
 * http://www.eevblog.com/forum/projects/max31855-temperature-error/
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/max31855.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MAX31855)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

#define MAX31855_FAULT         (1 << 16)
#define MAX31855_SHORT_VCC     (1 << 2)
#define MAX31855_SHORT_GND     (1 << 1)
#define MAX31855_OPEN_CIRCUIT  (1 << 0)
#define MAX31855_TEMP_COUPLE   0xffffc000
#define MAX31855_TEMP_JUNCTION 0xfff0

struct max31855_dev_s
{
  FAR struct spi_dev_s *spi;           /* Saved SPI driver instance */
  int16_t temp;
  uint16_t devid;                      /* Select minor device number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void max31855_lock(FAR struct spi_dev_s *spi);
static void max31855_unlock(FAR struct spi_dev_s *spi);

/* Character driver methods */

static ssize_t max31855_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t max31855_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max31855fops =
{
  NULL,            /* open */
  NULL,            /* close */
  max31855_read,   /* read */
  max31855_write,  /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max31855_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void max31855_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 400000);
}

/****************************************************************************
 * Name: max31855_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void max31855_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: max31855_read
 ****************************************************************************/

static ssize_t max31855_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct max31855_dev_s *priv  = inode->i_private;
  FAR uint16_t              *temp  = (FAR uint16_t *) buffer;
  int                       ret    = 2;
  int32_t                   regmsb;
  int32_t                   regval;

  /* Check for issues */

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -EINVAL;
    }

  if (buflen != 2)
    {
      snerr("ERROR: You can't read something other than 16 bits "
            "(2 bytes)\n");
      return -EINVAL;
    }

  /* Enable MAX31855's chip select */

  max31855_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(priv->devid), true);

  /* Read temperature */

  SPI_RECVBLOCK(priv->spi, &regmsb, 4);

  /* Disable MAX31855's chip select */

  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(priv->devid), false);
  max31855_unlock(priv->spi);

  /* Detect any errors during SPI transmission */

  if (!(regmsb) || regmsb == -1)
    {
      snerr("ERROR: Data transmission failed on device %d:\n", priv->devid);
      snerr("  One or more MAX31855 pins are not properly connected!\n\n");
      return -EINVAL;
    }

  regval  = (regmsb & 0xff000000) >> 24;
  regval |= (regmsb & 0xff0000) >> 8;
  regval |= (regmsb & 0xff00) << 8;
  regval |= (regmsb & 0xff) << 24;

  sninfo("Read from MAX31855 = 0x%08" PRIX32 "\n", regval);

  /* Feed sensor data to entropy pool */

  add_sensor_randomness(regval);

  /* If negative, fix signal bits */

  if (regval & 0x80000000)
    {
      *temp = 0xc000 | (regval >> 18);
    }
  else
    {
      *temp = (regval >> 18);
    }

  /* Detect any fault */

  if (regval & MAX31855_FAULT)
    {
      snerr("ERROR: A fault was detected by MAX31855:\n");

      if (regval & MAX31855_SHORT_VCC)
        {
          snerr("  The thermocouple input is shorted to VCC!\n");
        }

      if (regval & MAX31855_SHORT_GND)
        {
          snerr("  The thermocouple input is shorted to GND!\n");
        }

      if (regval & MAX31855_OPEN_CIRCUIT)
        {
          snerr("  The thermocouple input is not connected!\n");
        }

      return -EINVAL;
    }

  /* Return two bytes, the temperature is fixed point Q12.2, then divide by 4
   * in your application in other to get real temperature in Celsius degrees.
   */

  return ret;
}

/****************************************************************************
 * Name: max31855_write
 ****************************************************************************/

static ssize_t max31855_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max31855_register
 *
 * Description:
 *   This function will register the max31855 driver as /dev/tempN
 *   where N is the minor device number
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register.  E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MAX31855
 *   devid   - Minor device number.  E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int max31855_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                      uint16_t devid)
{
  FAR struct max31855_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the MAX31855 device structure */

  priv = (FAR struct max31855_dev_s *)
         kmm_malloc(sizeof(struct max31855_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi   = spi;
  priv->temp  = 0;
  priv->devid = devid;

  /* Register the character driver */

  ret = register_driver(devpath, &g_max31855fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_SENSORS_MAX31855 */
