/****************************************************************************
 * drivers/sensors/max31865.c
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

/* Character driver for the Maxim MAX31865 Thermocouple-to-Digital Converter
 *
 * NOTE: Some Maxim MAX31865 chips have an issue it report value 25% lower
 * of real temperature, for more info read this thread:
 * http://www.eevblog.com/forum/projects/max31865-temperature-error/
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
#include <math.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/max31865.h>
#include <nuttx/random.h>
#include <sys/endian.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MAX31865)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

#define MAX31865_CONFIG_REG             0x00
# define MAX31865_BIAS_ON               (1 << 7)
# define MAX31865_AUTO_CONVERSION       (1 << 6)
# define MAX31865_ONE_SHOT              (1 << 5)
# define MAX31865_3WIRE                 (1 << 4)
# define MAX31865_FAULT_DET_OFFSET      (2)
#  define MAX31865_FAULT_DET_MASK       (3 << MAX31865_FAULT_DET_OFFSET)
#  define MAX31865_FAULT_DET_OFF        (0 << MAX31865_FAULT_DET_OFFSET)
#  define MAX31865_FAULT_DET_AUTO       (1 << MAX31865_FAULT_DET_OFFSET)
#  define MAX31865_FAULT_DET_1CYCLE     (2 << MAX31865_FAULT_DET_OFFSET)
#  define MAX31865_FAULT_DET_2CYCLE     (3 << MAX31865_FAULT_DET_OFFSET)
# define MAX31865_FAULT_STATUS_CLR      (1 << 1)
# define MAX31865_50HZ_FILTER           (1 << 0)

#define MAX31865_RTDMSB_REG             0x01
#define MAX31865_RTDLSB_REG             0x02
#define MAX31865_HFAULTMSB_REG          0x03
#define MAX31865_HFAULTLSB_REG          0x04
#define MAX31865_LFAULTMSB_REG          0x05
#define MAX31865_LFAULTLSB_REG          0x06
#define MAX31865_FAULTSTAT_REG          0x07

#define MAX31865_REF_RESISTOR           430

#define MAX31865_WRITE                  0x80

#define RTD_A                           (3.9083e-3)
#define RTD_B                           (-5.775e-7)

struct max31865_dev_s
{
  FAR struct spi_dev_s *spi; /* Saved SPI driver instance */
  int16_t temp;
  uint16_t devid; /* Select minor device number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void max31865_lock(FAR struct spi_dev_s *spi);
static void max31865_unlock(FAR struct spi_dev_s *spi);

/* Character driver methods */

static int max31865_open(FAR struct file *filep);

static ssize_t max31865_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t max31865_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max31865fops =
{
  max31865_open,  /* open */
  NULL,           /* close */
  max31865_read,  /* read */
  max31865_write, /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max31865_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void max31865_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE1);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 400000);
}

/****************************************************************************
 * Name: max31865_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void max31865_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: max31865_open
 *
 * Description:
 *   This function is called whenever the MAX31865 device is opened.
 *
 ****************************************************************************/

static int max31865_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct max31865_dev_s *priv = inode->i_private;
  unsigned char regval;

  /* Enable MAX31865's chip select */

  max31865_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(priv->devid), true);

  regval = MAX31865_WRITE | MAX31865_CONFIG_REG;
  SPI_SEND(priv->spi, regval);

  regval = MAX31865_50HZ_FILTER | MAX31865_3WIRE | \
  MAX31865_AUTO_CONVERSION | MAX31865_BIAS_ON;
  SPI_SEND(priv->spi, regval);

  /* Disable MAX31865's chip select */

  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(priv->devid), false);
  max31865_unlock(priv->spi);

  return OK;
}

/****************************************************************************
 * Name: max31865_read
 ****************************************************************************/

static ssize_t max31865_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct max31865_dev_s *priv = inode->i_private;
  uint8_t regval[8];
  float temp;
  uint16_t regmsb;
  float rt;

  /* Check for issues */

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -EINVAL;
    }

  if (buflen != sizeof(float))
    {
      snerr("ERROR: You can't read something other than 16 bits "
            "(2 bytes)\n");
      return -EINVAL;
    }

  /* Enable MAX31865's chip select */

  max31865_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(priv->devid), true);

  /* Read temperature */

  memset(regval, 0, sizeof(regval));
  SPI_SEND(priv->spi, MAX31865_CONFIG_REG);

  SPI_RECVBLOCK(priv->spi, regval, sizeof(regval));

  /* Disable MAX31865's chip select */

  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(priv->devid), false);
  max31865_unlock(priv->spi);

  /* Detect any errors during SPI transmission */

  if (*regval != (MAX31865_50HZ_FILTER | MAX31865_3WIRE | \
                  MAX31865_AUTO_CONVERSION | MAX31865_BIAS_ON))
    {
      snerr("ERROR: Data transmission failed on device %d:\n", priv->devid);
      snerr("  One or more MAX31865 pins are not properly connected!\n\n");
      return -EINVAL;
    }

  /* Resistance of the RTD */

  regmsb = ((regval[1] << 8) | regval[2]) >> 1;

  rt = (float)(regmsb * MAX31865_REF_RESISTOR / 32768.0);

  /* T = (-A + sqrt(A^2 - 4*B*(1 - R/R0))) / (2*B) */

  temp = (-RTD_A + sqrt(RTD_A * RTD_A - 4 * RTD_B * (1 - rt / 100.0))) \
          / (2 * RTD_B);

  *(float *)buffer = temp;

  sninfo("MAX31865 ADC: %d, RTD: %.2f, temperature: %.1f\n", \
        regmsb, rt, temp);

  return sizeof(temp);
}

/****************************************************************************
 * Name: max31865_write
 ****************************************************************************/

static ssize_t max31865_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max31865_register
 *
 * Description:
 *   This function will register the max31865 driver as /dev/tempN
 *   where N is the minor device number
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register.  E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MAX31865
 *   devid   - Minor device number.  E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int max31865_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                      uint16_t devid)
{
  FAR struct max31865_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the MAX31865 device structure */

  priv = (FAR struct max31865_dev_s *)
      kmm_malloc(sizeof(struct max31865_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = spi;
  priv->temp = 0;
  priv->devid = devid;

  /* Register the character driver */

  ret = register_driver(devpath, &g_max31865fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_SENSORS_MAX31865 */
