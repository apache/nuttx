/****************************************************************************
 * drivers/sensors/adt7320.c
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
#include <nuttx/sensors/adt7320.h>
#include <nuttx/random.h>

#include "adt7320.h"

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_ADT7320)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ADT7320_SPI_FREQUENCY
#  define CONFIG_ADT7320_SPI_FREQUENCY 1000000
#endif

#define ADT7320_SPI_MODE (SPIDEV_MODE3) /* SPI Mode 3: CPOL=1,CPHA=1 */

/* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

#define B16_9DIV5  (9 * 65536 / 5)
#define B16_32     (32 * 65536)

/****************************************************************************
 * Private
 ****************************************************************************/

struct adt7320_dev_s
{
  FAR struct spi_dev_s *spi;    /* Saved SPI driver instance */
  int spidev;
  bool fahrenheit;              /* true: temperature will be reported in
                                 * fahrenheit */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI Helpers */

static void adt7320_write_reg16(FAR struct adt7320_dev_s *priv,
                                uint8_t reg,
                                uint16_t data);
static uint16_t adt7320_read_reg16(FAR struct adt7320_dev_s *priv,
                                   uint8_t reg);
static void adt7320_write_reg8(FAR struct adt7320_dev_s *priv,
                               uint8_t reg,
                               uint8_t data);
static uint8_t adt7320_read_reg8(FAR struct adt7320_dev_s *priv,
                                 uint8_t reg);
static int adt7320_readtemp(FAR struct adt7320_dev_s *priv, FAR b16_t *temp);

/* Character driver methods */

static int adt7320_open(FAR struct file *filep);
static ssize_t adt7320_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t adt7320_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int adt7320_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_adt7320fops =
{
  adt7320_open,    /* open */
  NULL,            /* close */
  adt7320_read,    /* read */
  adt7320_write,   /* write */
  NULL,            /* seek */
  adt7320_ioctl,   /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adt7320_configspi
 *
 * Description:
 *   Configure the SPI interface to match the serial interface specification
 *   of the ADT7320.
 *
 ****************************************************************************/

static inline void adt7320_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ADT7320 */

  SPI_SETMODE(spi, ADT7320_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ADT7320_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: adt7320_write_reg16
 *
 * Description:
 *   Write to a 16-bit register (ADT7320_TEMP_REG, ADT7320_TCRIT_REG,
 *   ADT7320_THIGH_REG or ADT7320_TLOW_REG).
 *
 ****************************************************************************/

static void adt7320_write_reg16(FAR struct adt7320_dev_s *priv,
                                uint8_t reg,
                                const uint16_t data)
{
  SPI_LOCK(priv->spi, true);

  adt7320_configspi(priv->spi);

  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_SEND(priv->spi, (reg & 0x07) << 3);
  SPI_SEND(priv->spi, data >> 8);
  SPI_SEND(priv->spi, data & 0xff);

  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adt7320_read_reg16
 *
 * Description:
 *   Read a 16-bit register (ADT7320_TEMP_REG, ADT7320_TCRIT_REG,
 *   ADT7320_THIGH_REG or ADT7320_TLOW_REG).
 *
 ****************************************************************************/

static uint16_t adt7320_read_reg16(FAR struct adt7320_dev_s *priv,
                                   uint8_t reg)
{
  uint8_t data[2];

  SPI_LOCK(priv->spi, true);

  adt7320_configspi(priv->spi);

  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_SEND(priv->spi, ((reg & 0x07) << 3) | (1 << 6));
  SPI_RECVBLOCK(priv->spi, data, 2);

  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);

  return (data[0] << 8) | data[1];
}

/****************************************************************************
 * Name: adt7320_write_reg8
 *
 * Description:
 *   Write to a 8-bit register (ADT7320_STAT_REG, ADT7320_TCRIT_REG,
 *   ADT7320_CONF_REG, ADT7320_ID_REG or ADT7320_THYST_REG).
 ****************************************************************************/

static void adt7320_write_reg8(FAR struct adt7320_dev_s *priv,
                               uint8_t reg,
                               const uint8_t data)
{
  SPI_LOCK(priv->spi, true);

  adt7320_configspi(priv->spi);

  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_SEND(priv->spi, (reg & 0x07) << 3);
  SPI_SEND(priv->spi, data);

  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adt7320_read_reg8
 *
 * Description:
 *   Read a 8-bit register (ADT7320_STAT_REG, ADT7320_TCRIT_REG,
 *   ADT7320_CONF_REG, ADT7320_ID_REG or ADT7320_THYST_REG).
 ****************************************************************************/

static uint8_t adt7320_read_reg8(FAR struct adt7320_dev_s *priv,
                             uint8_t reg)
{
  uint8_t data;

  SPI_LOCK(priv->spi, true);

  adt7320_configspi(priv->spi);

  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_SEND(priv->spi, ((reg & 0x07) << 3) | (1 << 6));
  SPI_RECVBLOCK(priv->spi, &data, 1);

  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);

  return data;
}

/****************************************************************************
 * Name: adt7320_reset
 *
 * Description:
 *   Reset the ADT7320, you should wait at least 500us before trying to
 *   access the sensor again.
 *
 ****************************************************************************/

static void adt7320_reset(FAR struct adt7320_dev_s *priv)
{
  SPI_LOCK(priv->spi, true);

  adt7320_configspi(priv->spi);

  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_SEND(priv->spi, 0xff);
  SPI_SEND(priv->spi, 0xff);
  SPI_SEND(priv->spi, 0xff);
  SPI_SEND(priv->spi, 0xff);

  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adt7320_readtemp
 *
 * Description:
 *   Read the temperature register with special scaling (ADT7320_TEMP_REG).
 *
 ****************************************************************************/

static int adt7320_readtemp(FAR struct adt7320_dev_s *priv, FAR b16_t *temp)
{
  b16_t temp16;
  int16_t temp_raw;

  /* Read the raw temperature data (b16_t) */

  temp_raw = (int16_t)adt7320_read_reg16(priv, ADT7320_TEMP_REG) & 0xfff8;

  /* Convert from 9.7 bits to 16.16 bits */

  temp16 = (int32_t)temp_raw << 9;

  add_sensor_randomness(temp16);

  sninfo("Centigrade: %08x\n", temp16);

  /* Was fahrenheit requested? */

  if (priv->fahrenheit)
    {
      /* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

      temp16 =  b16mulb16(temp16, B16_9DIV5) + B16_32;
      sninfo("Fahrenheit: %08x\n", temp16);
    }

  *temp = temp16;
  return OK;
}

/****************************************************************************
 * Name: adt7320_open
 *
 * Description:
 *   This function is called whenever the ADT7320 device is opened.
 *
 ****************************************************************************/

static int adt7320_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adt7320_dev_s *priv = inode->i_private;

  if (adt7320_read_reg8(priv, ADT7320_ID_REG) != ADT7320_ID)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: adt7320_read
 ****************************************************************************/

static ssize_t adt7320_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adt7320_dev_s *priv = inode->i_private;
  FAR b16_t *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(b16_t);
  ptr      = (FAR b16_t *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      b16_t temp = 0;

      /* Read the next b16_t temperature value */

      ret = adt7320_readtemp(priv, &temp);
      if (ret < 0)
        {
          snerr("ERROR: adt7320_readtemp failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = temp;
    }

  return nsamples * sizeof(b16_t);
}

/****************************************************************************
 * Name: adt7320_write
 ****************************************************************************/

static ssize_t adt7320_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: adt7320_ioctl
 *
 * Description:
 *   Supported IOCTLs:
 *
 *     SNIOC_READSTAT: Read the status register, treat arg as an
 *     uint8_t pointer.
 *
 *     SNIOC_READCONF: Read the configuration register, treat arg as
 *     an uint8_t pointer.
 *
 *     SNIOC_WRITECONF: Write to the configuration register, treat arg
 *     as an uint8_t value.
 *
 *     SNIOC_FAHRENHEIT: Report samples in Fahrenheit, ignores arg.
 *
 *     SNIOC_CENTIGRADE: Report samples in Celsius, ignores arg.
 *
 *     SNIOC_READTCRIT: Read the critical temperature register, treat
 *     arg as an b16_t pointer.
 *
 *     SNIOC_WRITETCRIT: Write to the critical temperature register,
 *     treat arg as an b16_t value.
 *
 *     SNIOC_READTHYS: Read the hysteresis temperature register, treat
 *     arg as an b16_t pointer.
 *
 *     SNIOC_WRITETHYS: Write to the hysteresis temperature register,
 *     treat arg as an b16_t value.
 *
 *     SNIOC_READTLOW: Read the low temperature register, treat arg as
 *     an b16_t pointer.
 *
 *     SNIOC_WRITETLOW: Write to the low temperature register, treat
 *     arg as an b16_t value.
 *
 *     SNIOC_READTHIGH: Read the high temperature register, treat arg
 *     as an b16_t pointer.
 *
 *     SNIOC_WRITETHIGH: Write to the high temperature register, treat
 *     arg as an b16_t value.
 *
 ****************************************************************************/

static int adt7320_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adt7320_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Read the status register. Arg: uint8_t* pointer */

      case SNIOC_READSTAT:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = adt7320_read_reg8(priv, ADT7320_STAT_REG);
        }
        break;

      /* Read the configuration register. Arg: uint8_t* pointer */

      case SNIOC_READCONF:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = adt7320_read_reg8(priv, ADT7320_CONF_REG);
        }
        break;

      /* Write to the configuration register. Arg:  uint8_t value */

      case SNIOC_WRITECONF:
        adt7320_write_reg8(priv, ADT7320_CONF_REG, (uint8_t)arg);
        break;

      /* Report samples in Fahrenheit */

      case SNIOC_FAHRENHEIT:
        priv->fahrenheit = true;
        sninfo("Fahrenheit\n");
        break;

      /* Report samples in Celsius */

      case SNIOC_CENTIGRADE:
        priv->fahrenheit = false;
        sninfo("Celsius\n");
        break;

      /* Read the critical temperature register. Arg: b16_t* pointer */

      case SNIOC_READTCRIT:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          int16_t temp_raw;
          DEBUGASSERT(ptr != NULL);
          temp_raw = adt7320_read_reg16(priv, ADT7320_TCRIT_REG);
          *ptr = b8tob16(temp_raw << 1);
        }
        break;

      /* Write to the critical temperature register. Arg: b16_t value */

      case SNIOC_WRITETCRIT:
        adt7320_write_reg16(priv, ADT7320_TCRIT_REG,
                            b16tob8((b16_t)arg) >> 1);
        break;

      /* Read the hysteresis temperature register. Arg: b16_t* */

      case SNIOC_READTHYS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          uint8_t tmp;
          DEBUGASSERT(ptr != NULL);
          tmp = adt7320_read_reg8(priv, ADT7320_THYST_REG);
          *ptr = uitoub16(tmp);
        }
        break;

      /* Write to the hysteresis temperature register. Arg: b16_t value */

      case SNIOC_WRITETHYS:
        adt7320_write_reg8(priv, ADT7320_THYST_REG, ub16toi((b16_t)arg));
        break;

      /* Read the low temperature register. Arg: b16_t* pointer */

      case SNIOC_READTLOW:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          int16_t temp_raw;
          DEBUGASSERT(ptr != NULL);
          temp_raw = adt7320_read_reg16(priv, ADT7320_TLOW_REG);
          *ptr = b8tob16(temp_raw << 1);
        }
        break;

      /* Write to the low temperature register. Arg: b16_t value */

      case SNIOC_WRITETLOW:
        adt7320_write_reg16(priv, ADT7320_TLOW_REG,
                            b16tob8((b16_t)arg) >> 1);
        break;

      /* Read the high temperature register. Arg: b16_t* pointer */

      case SNIOC_READTHIGH:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          int16_t temp_raw;
          DEBUGASSERT(ptr != NULL);
          temp_raw = adt7320_read_reg16(priv, ADT7320_THIGH_REG);
          *ptr = b8tob16(temp_raw << 1);
        }
        break;

      /* Write to the high temperature register. Arg: b16_t value */

      case SNIOC_WRITETHIGH:
        adt7320_write_reg16(priv, ADT7320_THIGH_REG,
                            b16tob8((b16_t)arg) >> 1);
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
 * Name: adt7320_register
 *
 * Description:
 *   Register the ADT7320 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   spi - An instance of the SPI bus to use to communicate with ADT7320
 *   spidev - The SPI device number used to select the correct CS line
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adt7320_register(FAR const char *devpath,
                     FAR struct spi_dev_s *spi, int spidev)
{
  FAR struct adt7320_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the ADT7320 device structure */

  priv = kmm_malloc(sizeof(struct adt7320_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi        = spi;
  priv->spidev     = spidev;
  priv->fahrenheit = false;

  /* Register the character driver */

  ret = register_driver(devpath, &g_adt7320fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  /* Reset the ADT7320 SPI interface */

  adt7320_reset(priv);

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_SENSORS_ADT7320 */
