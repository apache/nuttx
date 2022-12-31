/****************************************************************************
 * drivers/leds/ncp5623c.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/ncp5623c.h>

#if defined(CONFIG_I2C) && defined(CONFIG_NCP5623C)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct ncp5623c_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t i2c_addr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ncp5623c_i2c_write_byte(FAR struct ncp5623c_dev_s *priv,
                 uint8_t const reg_addr, uint8_t const reg_val);

static int ncp5623c_open(FAR struct file *filep);
static int ncp5623c_close(FAR struct file *filep);
static int ncp5623c_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static ssize_t ncp5623c_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t ncp5623c_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ncp5623c_fileops =
{
  ncp5623c_open,               /* open */
  ncp5623c_close,              /* close */
  ncp5623c_read,               /* read */
  ncp5623c_write,              /* write */
  NULL,                        /* seek */
  ncp5623c_ioctl,              /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncp5623c_i2c_write_byte
 *
 * Description:
 *   Write a single byte to one of the NCP5623C configuration registers.
 *
 ****************************************************************************/

static int ncp5623c_i2c_write_byte(FAR struct ncp5623c_dev_s *priv,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_val)
{
  struct i2c_config_s config;
  int ret = OK;

  /* Assemble the 1 byte message comprised of reg_val */

  uint8_t const BUFFER_SIZE = 1;
  uint8_t buffer[BUFFER_SIZE];

  buffer[0] = NCP5623C_SET_REG(reg_addr, reg_val);

  /* Setup up the I2C configuration */

  config.frequency = I2C_BUS_FREQ_HZ;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the data (no RESTART) */

  lcdinfo("i2c addr: 0x%02X value: 0x%02X\n", priv->i2c_addr,
          buffer[0]);

  ret = i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
  if (ret != OK)
    {
      lcderr("ERROR: i2c_write returned error code %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ncp5623c_open
 *
 * Description:
 *   This function is called whenever a NCP5623C device is opened.
 *
 ****************************************************************************/

static int ncp5623c_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ncp5623c_dev_s *priv = inode->i_private;

  int ret = -1;

  /* Shutdown the NCP5623C */

  ret = ncp5623c_i2c_write_byte(priv, NCP5623C_SHUTDOWN, 0x00);
  if (ret != OK)
    {
      lcderr("ERROR: Could not shut down the NCP5623C\n");
      return ret;
    }

  /* Set up Max current */

  ret = ncp5623c_i2c_write_byte(priv, NCP5623C_ILED, 0x1f);
  if (ret != OK)
    {
      lcderr("ERROR: Could not set up max current\n");
      return ret;
    }

  /* Let the chip settle a bit */

  nxsig_usleep(1);
  return OK;
}

/****************************************************************************
 * Name: ncp5623c_close
 *
 * Description:
 *   This function is called whenever a NCP5623C device is closed.
 *
 ****************************************************************************/

static int ncp5623c_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ncp5623c_dev_s *priv = inode->i_private;
  int ret = -1;

  /* Shut down NCP5623C */

  ret = ncp5623c_i2c_write_byte(priv, NCP5623C_SHUTDOWN, 0x00);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ncp5623c_ioctl
 *
 * Description:
 *   This function is called whenever an ioctl call to a NCP5623C is
 *   performed.
 *
 ****************************************************************************/

static int ncp5623c_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ncp5623c_dev_s *priv = inode->i_private;
  int ret = OK;

  lcdinfo("cmd: %d arg: %ld\n", cmd, arg);

  switch (cmd)
    {
    case LEDIOC_SET_REG:
      {
        /* Retrieve the information handed over as argument for this ioctl */

        FAR const struct ncp5623c_set_reg_s *ptr =
          (FAR const struct ncp5623c_set_reg_s *)((uintptr_t)arg);

        DEBUGASSERT(ptr != NULL);
        if (ptr->reg > NCP5623C_MAX_REG)
          {
            lcderr("ERROR: Unrecognized register: %d\n", ptr->reg);
            ret = -EFAULT;
            break;
          }

        ret = ncp5623c_i2c_write_byte(priv, ptr->reg, ptr->val);
      }
      break;

      /* The used ioctl command was invalid */

    default:
      {
        lcderr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
      }
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncp5623c_register
 *
 * Description:
 *   Register the NCP5623C device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/rgbdrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LED driver.
 *   ncp5623c_i2c_addr
 *           - The I2C address of the NCP5623C.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ncp5623c_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t const ncp5623c_i2c_addr)
{
  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the NCP5623C device structure */

  FAR struct ncp5623c_dev_s *priv =
    (FAR struct ncp5623c_dev_s *)kmm_malloc(sizeof(struct ncp5623c_dev_s));

  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate instance of ncp5623c_dev_s\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->i2c_addr = ncp5623c_i2c_addr;

  /* Register the character driver */

  int const ret = register_driver(devpath, &g_ncp5623c_fileops, 0666, priv);
  if (ret != OK)
    {
      lcderr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ncp5623c_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t ncp5623c_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: ncp5623c_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t ncp5623c_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ncp5623c_dev_s *priv = inode->i_private;
  int ret = OK;

  unsigned int red;
  unsigned int green;
  unsigned int blue;
  char color[3];

  lcdinfo("%s\n", buffer);

  /* We need to receive a string #RRGGBB = 7 bytes */

  if (buffer == NULL || buflen < 7)
    {
      /* Well... nothing to do */

      return -EINVAL;
    }

  /* Check if it is a color format */

  if (buffer[0] != '#')
    {
      /* The color code needs to start with # */

      return -EINVAL;
    }

  /* Move buffer to next character */

  buffer++;

  color[0] = buffer[0];
  color[1] = buffer[1];
  color[2] = '\0';

  red = strtol(color, NULL, 16);

  color[0] = buffer[2];
  color[1] = buffer[3];
  color[2] = '\0';

  green = strtol(color, NULL, 16);

  color[0] = buffer[4];
  color[1] = buffer[5];
  color[2] = '\0';

  blue = strtol(color, NULL, 16);

  /* Sane check */

  if (red > NCP5623C_MAX_VALUE)
    {
      red = NCP5623C_MAX_VALUE;
    }

  if (green > NCP5623C_MAX_VALUE)
    {
      green = NCP5623C_MAX_VALUE;
    }

  if (blue > NCP5623C_MAX_VALUE)
    {
      blue = NCP5623C_MAX_VALUE;
    }

  /* Setup LED R */

  ret = ncp5623c_i2c_write_byte(priv, NCP5623C_PWM1,
                                 red);
  if (ret != OK)
    {
      lcderr("ERROR: Could not set red led\n");
      return ret;
    }

  /* Setup LED G */

  ret = ncp5623c_i2c_write_byte(priv, NCP5623C_PWM2,
                                 green);
  if (ret != OK)
    {
      lcderr("ERROR: Could not set green led\n");
      return ret;
    }

  /* Setup LED B */

  ret = ncp5623c_i2c_write_byte(priv, NCP5623C_PWM3,
                                 blue);
  if (ret != OK)
    {
      lcderr("ERROR: Could not set blue led\n");
      return ret;
    }

  return buflen;
}

#endif /* CONFIG_I2C && CONFIG_I2C_NCP5623C */
