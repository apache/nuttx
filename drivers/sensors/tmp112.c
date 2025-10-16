/****************************************************************************
 * drivers/sensors/tmp112.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/random.h>

#include <nuttx/sensors/tmp112.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tmp112_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* TMP112 I2C address */
  int freq;                     /* TMP112 Frequency */
  uint16_t tmp112_temp_raw;     /* Temperature as read from TMP112 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t tmp112_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);

static ssize_t tmp112_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_tmp112fops =
{
  NULL,         /* open */
  NULL,         /* close */
  tmp112_read,  /* read */
  tmp112_write, /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t tmp112_getreg16(FAR struct tmp112_dev_s *priv,
                                uint8_t regaddr, uint8_t ms_delay)
{
  struct i2c_config_s config;
  uint16_t msb;
  uint16_t lsb;
  uint16_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %s (%d)\n", strerror(-ret), ret);
      return ret;
    }

  if (ms_delay > 0)
    {
      nxsched_usleep(ms_delay * 1000);
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (FAR uint8_t *)&regval, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* MSB and LSB are inverted */

  msb = (regval & 0xff);
  lsb = (regval & 0xff00) >> 8;

  regval = (msb << 4) | (lsb >> 4);

  return regval;
}

static int tmp112_putreg16(FAR struct tmp112_dev_s *priv, uint8_t regaddr,
                           uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t data[3];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = regaddr;
  data[1] = (regval >> 8) & 0xff;
  data[2] = regval & 0xff;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (FAR uint8_t *)&data, 3);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %s (%d)\n", strerror(-ret), ret);
      return ret;
    }

  return OK;
}

static int tmp112_writeconfig(FAR struct tmp112_dev_s *priv)
{
  /* Datasheet Table 7-10: Configuration and Power-Up/Reset Formats
   * | BYTE | D7  | D6  | D5  | D4  | D3  | D2  | D1  | D0  |
   * |------|-----|-----|-----|-----|-----|-----|-----|-----|
   * | 1    | OS  | R1  | R0  | F1  | F0  | POL | TM  | SD  |
   * |      | 0   | 1   | 1   | 0   | 0   | 0   | 0   | 0   |
   * |------|-----|-----|-----|-----|-----|-----|-----|-----|
   * | 2    | CR1 | CR0 | AL  | EM  | 0   | 0   | 0   | 0   |
   * |      | 1   | 0   | 1   | 0   | 0   | 0   | 0   | 0   |
   * |------|-----|-----|-----|-----|-----|-----|-----|-----|
   */

  /* Byte 1: set 12 bit resolution, Comparator mode, Continuous
   * Conversion mode
   */

  const uint8_t b1 = 0b01100000;

  /* Byte 2: set 4Hz conversion rate, non-Extended mode */

  const uint8_t b2 = 0b10100000;

  return tmp112_putreg16(priv, TMP112_REG_CONFIG, (b1 << 8) | b2);
}

static void tmp112_read_temp(FAR struct tmp112_dev_s *priv)
{
  /* Wait 1.5x typical conversion time (10ms) to ensure that the
   * temperature register is primed and then read it.
   */

  uint16_t value = tmp112_getreg16(priv, TMP112_REG_TEMP, 15);

  if (value >= UINT16_MAX - __ELASTERROR)
    {
      snerr("ERROR: Invalid temperature read\n");
      priv->tmp112_temp_raw = 0;
    }
  else
    {
      priv->tmp112_temp_raw = value;
      sninfo("Temperature (raw) = %" PRIu16 "\n", priv->tmp112_temp_raw);
    }
}

static ssize_t tmp112_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct tmp112_dev_s *priv = inode->i_private;
  FAR float *temperature = (FAR float *)buffer;

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -1;
    }

  if (buflen != 4)
    {
      snerr("ERROR: You can't read something other than 32 bits "
        "(4 bytes)\n");
      return -1;
    }

  /* Refresh the temperature */

  tmp112_read_temp(priv);

  /* Get the temperature */

  *temperature = (float)priv->tmp112_temp_raw * 0.0625f;

  /* Return size of data (float, 4 bytes) */

  return sizeof(float);
}

static ssize_t tmp112_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp112_register
 *
 * Description:
 *   Register the TMP112 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use.
 *   addr    - The I2C address to use.
 *             When multiple sensors are connected to the same bus, each one
 *             must be electrically configured to use a different I2C
 *             address as specified in the datasheet.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tmp112_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr)
{
  FAR struct tmp112_dev_s *priv;
  int ret;

  /* Initialize the TMP112 device structure */

  priv = (FAR struct tmp112_dev_s *)kmm_malloc(sizeof(struct tmp112_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  priv->freq = CONFIG_TMP112_I2C_FREQUENCY;

  /* Write the configuration registers */

  sninfo("Writing configuration\n");
  tmp112_writeconfig(priv);

  /* Register the character driver */

  ret = register_driver(devpath, &g_tmp112fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("Driver loaded successfully\n");
  return ret;
}
