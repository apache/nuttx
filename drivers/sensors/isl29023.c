/****************************************************************************
 * drivers/sensors/isl29023.c
 *
 *   Copyright (C) 2019 DataVision s.r.o. All rights reserved.
 *   Authors: Matous Pokorny <matous.pokorny@datavision.cz>
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
#include <sys/types.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>

#include <nuttx/sensors/isl29023.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_ISL29023)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ISL29023_I2C_FREQUENCY
#  define CONFIG_ISL29023_I2C_FREQUENCY 400000
#endif

/* Registers definitions */

#define ISL29023_COMMAND_1   0x00
#define ISL29023_COMMAND_2   0x01
#define ISL29023_DATA_LSB    0x02
#define ISL29023_DATA_MSB    0x03
#define ISL29023_INT_LT_LSB  0x04
#define ISL29023_INT_LT_MSB  0x05
#define ISL29023_INT_HT_LSB  0x06
#define ISL29023_INT_HT_MSB  0x07
#define ISL29023_TEST        0x08

/* Registers definitions */

#define ISL29023_RESOLUTION_MASK  0x3
#define ISL29023_RESOLUTION_SHIFT 0x2

#define ISL29023_ALS_RANGE_MASK   0x3
#define ISL29023_ALS_RANGE_SHIFT  0x0

#define ISL29023_OP_MODE_MASK     0x7
#define ISL29023_OP_MODE_SHIFT    0x5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct isl29023_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;                   /* Address on the I2C bus */
  uint8_t op_mode;                /* Defined by isl29023_operational_mode_e */
  uint32_t resolution;            /* Sensor ADC res. 16..65536 */
  uint32_t range;                 /* Sensor range 1000..64000 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int isl29023_i2c_write(FAR struct isl29023_dev_s *dev,
                          FAR const uint8_t *buffer, ssize_t buflen);
static int isl29023_i2c_read(FAR struct isl29023_dev_s *dev,
                         FAR uint8_t *buffer, ssize_t buflen);
static int isl29023_read_reg(FAR struct isl29023_dev_s *dev,
                      const uint8_t regaddr, uint8_t *buffer, size_t buflen);
static int isl29023_read_lux(FAR struct isl29023_dev_s *dev,
                              FAR struct isl29023_data_s *data);
static int isl29023_set_op_mode(FAR struct isl29023_dev_s *dev, uint8_t mode);
static int isl29023_set_resolution(FAR struct isl29023_dev_s *dev,
                                    uint8_t res_mode);
static int isl29023_set_range(FAR struct isl29023_dev_s *dev,
                              uint8_t range_mode);

/* Driver methods */

static int isl29023_open(FAR struct file *filep);
static int isl29023_close(FAR struct file *filep);
static ssize_t isl29023_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t isl29023_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int isl29023_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_isl29023fops =
{
  isl29023_open,   /* open */
  isl29023_close,  /* close */
  isl29023_read,   /* read */
  isl29023_write,  /* write */
  NULL,            /* seek */
  isl29023_ioctl,  /* ioctl */
  NULL             /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: isl29023_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int isl29023_i2c_write(FAR struct isl29023_dev_s *dev,
                          FAR const uint8_t *buffer, ssize_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_ISL29023_I2C_FREQUENCY,
  msg.addr      = dev->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: isl29023_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int isl29023_i2c_read(FAR struct isl29023_dev_s *dev,
                         FAR uint8_t *buffer, ssize_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LM75_I2C_FREQUENCY,
  msg.addr      = dev->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: isl29023_read_reg
 *
 * Description:
 *   Read register from the I2C device.
 *
 ****************************************************************************/

static int isl29023_read_reg(FAR struct isl29023_dev_s *dev,
                        const uint8_t regaddr, uint8_t *buffer, size_t buflen)
{
  int ret;

  ret = isl29023_i2c_write(dev, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c write failed: %d\n", ret);
      return ret;
    }

  ret = isl29023_i2c_read(dev, buffer, buflen);
  if (ret < 0)
    {
      snerr("ERROR: i2c read failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: isl29023_open
 *
 * Description:
 *   This function is called whenever the ISL29023 device is opened.
 *
 ****************************************************************************/

static int isl29023_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: isl29023_close
 *
 * Description:
 *   This routine is called when the ISL29023 device is closed.
 *
 ****************************************************************************/

static int isl29023_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: isl29023_read
 ****************************************************************************/

static ssize_t isl29023_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct isl29023_dev_s *priv = inode->i_private;
  int ret;
  struct isl29023_data_s data;

  ret = isl29023_read_lux(priv, &data);
  if (ret < 0)
    {
      snerr("ERROR: failed to read the sensor: %d\n", ret);
      return (ssize_t)ret;
    }
  else
    {
      if (buflen < sizeof(data))
        {
          return -EIO;
        }

      memcpy(buffer, &data, sizeof(data));
    }

  return buflen;
}

/****************************************************************************
 * Name: isl29023_write
 ****************************************************************************/

static ssize_t isl29023_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: isl29023_read_lux
 ****************************************************************************/

static int isl29023_read_lux(FAR struct isl29023_dev_s *dev,
                              FAR struct isl29023_data_s *data)
{
  int ret;
  uint8_t buffer[2];

  ret = isl29023_read_reg(dev, ISL29023_DATA_LSB, buffer, 2);
  if (ret < 0)
    {
      return ret;
    }

  data->raw = (buffer[1] << 8) | buffer[0];

  add_sensor_randomness(data->raw);

  uint32_t tmp = (data->raw * dev->range) / dev->resolution;
  data->lux = (uint16_t)tmp;

  sninfo("raw value %8x, lux: %5u\n", data->raw, data->lux);

  return OK;
}

/****************************************************************************
 * Name: isl29023_set_op_mode
 ****************************************************************************/

static int isl29023_set_op_mode(FAR struct isl29023_dev_s *dev, uint8_t mode)
{
  uint8_t buffer[2];
  int ret;

  ret = isl29023_read_reg(dev, ISL29023_COMMAND_1, &buffer[1], 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c read reg failed: %d\n", ret);
      return ret;
    }

  /* Clear the mode bits */

  buffer[1] &= ~(ISL29023_OP_MODE_MASK << ISL29023_OP_MODE_SHIFT);
  mode &= ISL29023_OP_MODE_MASK;

  /* Modify mode bits */

  buffer[1] |= mode << ISL29023_OP_MODE_SHIFT;
  buffer[0] = ISL29023_COMMAND_1;

  dev->op_mode = mode;
  sninfo("mode: %x\n", dev->mode);

  return isl29023_i2c_write(dev, buffer, 2);
}

/****************************************************************************
 * Name: isl29023_set_resolution
 ****************************************************************************/

static int isl29023_set_resolution(FAR struct isl29023_dev_s *dev,
                                    uint8_t res_mode)
{
  uint8_t buffer[2];
  int ret;

  ret = isl29023_read_reg(dev, ISL29023_COMMAND_2, &buffer[1], 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c read reg failed: %d\n", ret);
      return ret;
    }

  /* Clear the mode bits */

  buffer[1] &= ~(ISL29023_RESOLUTION_MASK << ISL29023_RESOLUTION_SHIFT);
  res_mode &= ISL29023_RESOLUTION_MASK;

  /* Modify mode bits */

  buffer[1] |= res_mode << ISL29023_RESOLUTION_SHIFT;
  buffer[0] = ISL29023_COMMAND_2;

  dev->resolution = 1u << (16u - res_mode * 4u);
  sninfo("resolution: %d\n", dev->resolution);

  return isl29023_i2c_write(dev, buffer, 2);
}

/****************************************************************************
 * Name: isl29023_set_resolution
 ****************************************************************************/

static int isl29023_set_range(FAR struct isl29023_dev_s *dev,
                              uint8_t range_mode)
{
  uint8_t buffer[2];
  int ret;

  ret = isl29023_read_reg(dev, ISL29023_COMMAND_2, &buffer[1], 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c read reg failed: %d\n", ret);
      return ret;
    }

  /* Clear the mode bits */

  buffer[1] &= ~(ISL29023_ALS_RANGE_MASK << ISL29023_ALS_RANGE_SHIFT);

  /* Modify mode bits */

  range_mode &= ISL29023_ALS_RANGE_MASK;
  buffer[1] |= range_mode << ISL29023_ALS_RANGE_SHIFT;
  buffer[0] = ISL29023_COMMAND_2;

  dev->range = 1000u * (1u << range_mode) * (1u << range_mode);
  sninfo("range: %u\n", dev->range);

  return isl29023_i2c_write(dev, buffer, 2);
}

/****************************************************************************
 * Name: isl29023_ioctl
 ****************************************************************************/

static int isl29023_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode            *inode = filep->f_inode;
  FAR struct isl29023_dev_s   *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Write to the COMMAND1 register. Arg:  uint8_t value */

      case SNIOC_SET_OPERATIONAL_MODE:
        ret = isl29023_set_op_mode(priv, (uint8_t)arg);
        sninfo("Set operation mode %d with result %d\n", (uint8_t)arg, ret);
        break;

      /* Write to the COMMAND2 register. Arg:  uint8_t value */

      case SNIOC_SET_RESOLUTION:
        ret = isl29023_set_resolution(priv, (uint8_t)arg);
        sninfo("Set resolution mode %d with result %d\n", (uint8_t)arg, ret);
        break;

      /* Write to the COMMAND2 register. Arg:  uint8_t value */

      case SNIOC_SET_RANGE:
        ret = isl29023_set_range(priv, (uint8_t)arg);
        sninfo("Set range mode %d with result %d\n", (uint8_t)arg, ret);
        break;

      default:
        sninfo("Unrecognized cmd: 0x%04x\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int isl29023_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr)
{
  FAR struct isl29023_dev_s *priv;
  int ret;

  priv = (FAR struct isl29023_dev_s *)kmm_malloc(sizeof(struct isl29023_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->resolution = 0x10000;
  priv->range      = 64000;
  priv->op_mode    = ISL29023_OP_MODE_POWER_DOWN;

  /* Register the character driver */

  ret = register_driver(devpath, &g_isl29023fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_ISL29023 */
