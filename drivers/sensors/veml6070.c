/****************************************************************************
 * drivers/sensors/veml6070.c
 * Character driver for the Vishay UV-A Light Sensor VEML6070
 *
 *   Copyright (C) 2016 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/veml6070.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_VEML6070)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_VEML6070_I2C_FREQUENCY
#  define CONFIG_VEML6070_I2C_FREQUENCY 100000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct veml6070_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     veml6070_read8(FAR struct veml6070_dev_s *priv, int offset,
                 FAR uint8_t *regval);
static int     veml6070_write8(FAR struct veml6070_dev_s *priv,
                 uint8_t regval);

/* Character driver methods */

static int     veml6070_open(FAR struct file *filep);
static int     veml6070_close(FAR struct file *filep);
static ssize_t veml6070_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t veml6070_write(FAR struct file *filep,
                 FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_veml6070_fops =
{
  veml6070_open,   /* open */
  veml6070_close,  /* close */
  veml6070_read,   /* read */
  veml6070_write,  /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: veml6070_read8
 *
 * Description:
 *   Read 8-bit register
 *
 ****************************************************************************/

static int veml6070_read8(FAR struct veml6070_dev_s *priv, int offset,
                            FAR uint8_t *regval)
{
  struct i2c_config_s config;
  uint8_t data[1];
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_VEML6070_I2C_FREQUENCY;
  config.address   = priv->addr + offset;
  config.addrlen   = 7;

  /* Read 8-bits from the device */

  ret = i2c_read(priv->i2c, &config, data, 1);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Copy the content of the buffer to the location of the uint8_t pointer */

  *regval = data[0];

  sninfo("value: %08x ret: %d\n", *regval, ret);
  return OK;
}

/****************************************************************************
 * Name: veml6070_write8
 *
 * Description:
 *   Write from an 8-bit register
 *
 ****************************************************************************/

static int veml6070_write8(FAR struct veml6070_dev_s *priv, uint8_t regval)
{
  struct i2c_config_s config;
  int ret;

  sninfo("value: %02x\n", regval);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_VEML6070_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write 8 bits to device */

  ret = i2c_write(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: veml6070_open
 *
 * Description:
 *   This function is called whenever the VEML6070 device is opened.
 *
 ****************************************************************************/

static int veml6070_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: veml6070_close
 *
 * Description:
 *   This routine is called when the VEML6070 device is closed.
 *
 ****************************************************************************/

static int veml6070_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: veml6070_read
 ****************************************************************************/

static ssize_t veml6070_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  int ret;
  FAR struct inode         *inode;
  FAR struct veml6070_dev_s *priv;
  int msb = 1;
  uint16_t regdata;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct veml6070_dev_s *)inode->i_private;

  /* Check if the user is reading the right size */

  if (buflen != 2)
    {
      snerr("ERROR: You need to read 2 bytes from this sensor!\n");
      return -EINVAL;
    }

  /* Enable the sensor */

  ret = veml6070_write8(priv, VEML6070_CMD_RSV & ~VEML6070_CMD_SD);
  if (ret < 0)
    {
      snerr("ERROR: Failed to enable the VEML6070!\n");
      return -EINVAL;
    }

  /* 1T for Rset 270Kohms is 125ms */

  nxsig_usleep(125000);

  /* Read the MSB first */

  ret = veml6070_read8(priv, msb, (FAR uint8_t *) &regdata);
  if (ret < 0)
    {
      snerr("ERROR: Error reading light sensor!\n");
      return ret;
    }

  buffer[1] = regdata;

  /* Read the LSB */

  msb = 0;
  ret = veml6070_read8(priv, msb, (FAR uint8_t *) &regdata);
  if (ret < 0)
    {
      snerr("ERROR: Error reading light sensor!\n");
      return ret;
    }

  buffer[0] = regdata;

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((buffer[1] << 16) ^ buffer[0]);

  return buflen;
}

/****************************************************************************
 * Name: veml6070_write
 ****************************************************************************/

static ssize_t veml6070_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: veml6070_register
 *
 * Description:
 *   Register the VEML6070 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/uvlight0"
 *   i2c - An instance of the I2C interface to use to communicate with VEML6070
 *   addr - The I2C address of the VEML6070.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int veml6070_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the VEML6070 device structure */

  FAR struct veml6070_dev_s *priv =
    (FAR struct veml6070_dev_s *)kmm_malloc(sizeof(struct veml6070_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Initialize the device (shut it down) */

  ret = veml6070_write8(priv, VEML6070_CMD_RSV | VEML6070_CMD_SD);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the VEML6070!\n");
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_veml6070_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_VEML6070 */
