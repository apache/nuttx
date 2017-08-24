/****************************************************************************
 * drivers/sensors/mcp9844.c
 * Character driver for the MCP9844 Temperature Sensor
 *
 *   Copyright (C) 2015 DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mcp9844.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MCP9844)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_MCP9844_I2C_FREQUENCY
#  define CONFIG_MCP9844_I2C_FREQUENCY 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcp9844_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C helper functions */

static int     mcp9844_read_u16(FAR struct mcp9844_dev_s *priv,
                  uint8_t const regaddr, FAR uint16_t *value);
static int     mcp9844_write_u16(FAR struct mcp9844_dev_s *priv,
                  uint8_t const regaddr, uint16_t const regval);

/* Character driver methods */

static int     mcp9844_open(FAR struct file *filep);
static int     mcp9844_close(FAR struct file *filep);
static ssize_t mcp9844_read(FAR struct file *filep, FAR char *buffer,
                  size_t buflen);
static ssize_t mcp9844_write(FAR struct file *filep, FAR const char *buffer,
                  size_t buflen);
static int     mcp9844_ioctl(FAR struct file *filep, int cmd,
                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mcp9844_fops =
{
  mcp9844_open,
  mcp9844_close,
  mcp9844_read,
  mcp9844_write,
  NULL,
  mcp9844_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp9844_read_u16
 *
 * Description:
 *  Read a 16 bit valie from the MCP9844 at the address regaddr.
 *
 ****************************************************************************/

static int mcp9844_read_u16(FAR struct mcp9844_dev_s *priv,
                            uint8_t const regaddr, FAR uint16_t *value)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MCP9844_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr ("i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16-bits from the register */

  ret = i2c_read(priv->i2c, &config, buffer, 2);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Copy the content of the buffer to the location of the uint16_t pointer */

  *value = (((uint16_t)(buffer[0]))<<8) + ((uint16_t)(buffer[1]));

  sninfo("addr: %02x value: %08x ret: %d\n", regaddr, *value, ret);
  return OK;
}

/****************************************************************************
 * Name: mcp9844_write_u16
 *
 * Description:
 *   Write to a 16-bit register of the MCP9844.
 *
 ****************************************************************************/

static int mcp9844_write_u16(FAR struct mcp9844_dev_s *priv,
                             uint8_t const regaddr, uint16_t const regval)
{
  struct i2c_config_s config;

  sninfo("addr: %02x value: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  uint8_t const BUFFER_SIZE = 3;
  uint8_t buffer[BUFFER_SIZE];

  buffer[0] = regaddr;
  buffer[1] = (uint8_t)(regval >> 8);
  buffer[2] = (uint8_t)(regval);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MCP9844_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
}

/****************************************************************************
 * Name: mcp9844_open
 *
 * Description:
 *   This function is called whenever the MCP9844 device is opened.
 *
 ****************************************************************************/

static int mcp9844_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mcp9844_close
 *
 * Description:
 *   This routine is called when the MCP9844 device is closed.
 *
 ****************************************************************************/

static int mcp9844_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mcp9844_read
 ****************************************************************************/

static ssize_t mcp9844_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: mcp9844_write
 ****************************************************************************/

static ssize_t mcp9844_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lm75_ioctl
 ****************************************************************************/

static int mcp9844_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mcp9844_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Read from the ambient temperature register. Arg: uint16_t* pointer */

      case SNIOC_READTEMP:
        {
          FAR struct mcp9844_temp_arg_s *temp_result =
            (FAR struct mcp9844_temp_arg_s *)((uintptr_t)arg);

          DEBUGASSERT(temp_result != NULL);

          /* Read the ambient temperature value from the device */

          uint16_t raw_temperature = 0;
          ret = mcp9844_read_u16(priv, MCP9844_TEMP_REG, &raw_temperature);

          /* Convert from the proprietary sensor temperature data representation
           * to a more user friendly version.
           */

          if (ret == OK)
            {
              /* Feed sensor data to entropy pool */

              add_sensor_randomness(raw_temperature);

              /* BIT15 - 13 contain information if preset temperature values
               * have been exceeded or undercut. BIT12 is now not any longer
               * needed since we do have the sign information retrieved.
               * We do not need them for the temperature so those bits
               * need to be masked out.
               */

              raw_temperature &= 0x0FFF; /* 0x0FFF = 0b 0000 1111 1111 1111 */

              /* The post comma temperature value is encoded in BIT3 to BIT0 */

              temp_result->temp_post_comma = (uint8_t)(raw_temperature & 0x000F);

              /* The pre comma temperature value is encoded in BIT11 to BIT4 */

              temp_result->temp_pre_comma = (int8_t)(raw_temperature >> 4);
            }
          else
            {
              snerr("ERROR: ioctl::SNIOC_READTEMP - mcp9844_read_u16 failed - no temperature retrieved\n");
            }
        }
        break;

      case SNIOC_SETRESOLUTION:
        {
          ret = mcp9844_write_u16(priv, MCP9844_RESO_REG, (uint16_t)(arg));
          if (ret != OK)
            {
              snerr("ERROR: ioctl::SNIOC_SETRESOLUTION - mcp9844_write_u16 failed - no resolution set\n");
            }
        }
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp9844_register
 *
 * Description:
 *   Register the MCP9844 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with MCP9844
 *   addr - The I2C address of the MCP9844.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mcp9844_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr)
{
  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the LM-75 device structure */

  FAR struct mcp9844_dev_s *priv =
    (FAR struct mcp9844_dev_s *)kmm_malloc(sizeof(struct mcp9844_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Register the character driver */

  int ret = register_driver(devpath, &g_mcp9844_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_LM75_I2C */
