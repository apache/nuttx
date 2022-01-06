/****************************************************************************
 * drivers/sensors/lm92.c
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
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lm92.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LM92)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LM92_I2C_FREQUENCY
#  define CONFIG_LM92_I2C_FREQUENCY 400000
#endif

/* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

#define B16_9DIV5  (9 * 65536 / 5)
#define B16_32     (32 * 65536)

/****************************************************************************
 * Private
 ****************************************************************************/

struct lm92_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  bool fahrenheit;              /* true: temperature will be reported in Fahrenheit */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     lm92_i2c_write(FAR struct lm92_dev_s *priv,
                              FAR const uint8_t *buffer, int buflen);
static int     lm92_i2c_read(FAR struct lm92_dev_s *priv,
                             FAR uint8_t *buffer, int buflen);
static int     lm92_readb16(FAR struct lm92_dev_s *priv, uint8_t regaddr,
                            FAR b16_t *regvalue);
static int     lm92_writeb16(FAR struct lm92_dev_s *priv, uint8_t regaddr,
                             b16_t regval);
static int     lm92_readtemp(FAR struct lm92_dev_s *priv, FAR b16_t *temp);
static int     lm92_readconf(FAR struct lm92_dev_s *priv, FAR uint8_t *conf);
static int     lm92_writeconf(FAR struct lm92_dev_s *priv, uint8_t conf);

/* Character driver methods */

static int     lm92_open(FAR struct file *filep);
static int     lm92_close(FAR struct file *filep);
static ssize_t lm92_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t lm92_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     lm92_ioctl(FAR struct file *filep,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lm92fops =
{
  lm92_open,
  lm92_close,
  lm92_read,
  lm92_write,
  NULL,
  lm92_ioctl,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm92_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int lm92_i2c_write(FAR struct lm92_dev_s *priv,
                          FAR const uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LM92_I2C_FREQUENCY,
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: lm92_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int lm92_i2c_read(FAR struct lm92_dev_s *priv,
                         FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LM92_I2C_FREQUENCY,
  msg.addr      = priv->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: lm92_readb16
 *
 * Description:
 *   Read a 16-bit register (LM92_TEMP_REG, LM92_THYS_REG, LM92_TCRIT_REG,
 *   LM92_TLOW_REG, LM92_THIGH_REG, or LM92_ID_REG)
 *
 ****************************************************************************/

static int lm92_readb16(FAR struct lm92_dev_s *priv, uint8_t regaddr,
                        FAR b16_t *regvalue)
{
  uint8_t buffer[2];
  int ret;

  /* Write the register address */

  ret = lm92_i2c_write(priv, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16 bits from the register (discarding 3) */

  ret = lm92_i2c_read(priv, buffer, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Data format is:
   *  TTTTTTTT TTTTTxxx where TTTTTTTTTTTTT is a thirteen-bit,
   * signed temperature value with LSB = 0.0625 degrees Centigrade.
   */

  *regvalue = (b16_t)((uint32_t)(buffer[0] & (1 << 7)) << 24 |
                      (uint32_t)(buffer[0] & ~(1 << 7)) << 17 |
                      (uint32_t)(buffer[1] & ~7) << 9);
  sninfo("addr: %02x value: %08x ret: %d\n", regaddr, *regvalue, ret);
  return OK;
}

/****************************************************************************
 * Name: lm92_writeb16
 *
 * Description:
 *   Write to a 16-bit register
 *  (LM92_TEMP_REG, LM92_THYS_REG, LM92_TCRIT_REG,
 *   LM92_TLOW_REG, LM92_THIGH_REG, or LM92_ID_REG)
 *
 ****************************************************************************/

static int lm92_writeb16(FAR struct lm92_dev_s *priv, uint8_t regaddr,
                         b16_t regval)
{
  uint8_t buffer[3];

  sninfo("addr: %02x value: %08x\n", regaddr, regval);

  /* Set up a 3-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = (uint8_t)(((uint32_t)regval & (1 << 31)) >> 24 |
                        ((uint32_t)regval & ~(1 << 31)) >> 17);
  buffer[2] = (uint8_t)(((uint32_t)regval & ~(1 << 31)) >> 9);

  /* Write the register address followed by the data (no RESTART) */

  return lm92_i2c_write(priv, buffer, 3);
}

/****************************************************************************
 * Name: lm92_readtemp
 *
 * Description:
 *   Read the temperature register with special scaling (LM92_TEMP_REG)
 *
 ****************************************************************************/

static int lm92_readtemp(FAR struct lm92_dev_s *priv, FAR b16_t *temp)
{
  b16_t temp16;
  int ret;

  /* Read the raw temperature data (b16_t) */

  ret = lm92_readb16(priv, LM92_TEMP_REG, &temp16);
  if (ret < 0)
    {
      snerr("ERROR: lm92_readb16 failed: %d\n", ret);
      return ret;
    }

  add_sensor_randomness(temp16);

  sninfo("Centigrade: %08x\n", temp16);

  /* Was Fahrenheit requested? */

  if (priv->fahrenheit)
    {
      /* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

      temp16 = b16mulb16(temp16, B16_9DIV5) + B16_32;
      sninfo("Fahrenheit: %08x\n", temp16);
    }

  *temp = temp16;
  return OK;
}

/****************************************************************************
 * Name: lm92_readconf
 *
 * Description:
 *   Read the 8-bit LM92 configuration register
 *
 ****************************************************************************/

static int lm92_readconf(FAR struct lm92_dev_s *priv, FAR uint8_t *conf)
{
  uint8_t buffer;
  int ret;

  /* Write the configuration register address */

  buffer = LM92_CONF_REG;

  ret = lm92_i2c_write(priv, &buffer, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = lm92_i2c_read(priv, conf, 1);
  sninfo("conf: %02x ret: %d\n", *conf, ret);
  return ret;
}

/****************************************************************************
 * Name: lm92_writeconf
 *
 * Description:
 *   Write to a 8-bit LM92 configuration register.
 *
 ****************************************************************************/

static int lm92_writeconf(FAR struct lm92_dev_s *priv, uint8_t conf)
{
  uint8_t buffer[2];

  sninfo("conf: %02x\n", conf);

  /* Set up a 2-byte message to send */

  buffer[0] = LM92_CONF_REG;
  buffer[1] = conf;

  /* Write the register address followed by the data (no RESTART) */

  return lm92_i2c_write(priv, buffer, 2);
}

/****************************************************************************
 * Name: lm92_readid
 *
 * Description:
 *   Read the 16-bit LM92 identification register
 *
 ****************************************************************************/

static int lm92_readid(FAR struct lm92_dev_s *priv, FAR uint16_t *id)
{
  uint8_t buffer[2];
  uint8_t regaddr;
  int ret;

  /* Write the identification register address */

  regaddr = LM92_ID_REG;

  ret = lm92_i2c_write(priv, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16 bits from the register */

  ret = lm92_i2c_read(priv, buffer, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  *id = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
  sninfo("id: %04x ret: %d\n", *id, ret);
  return OK;
}

/****************************************************************************
 * Name: lm92_open
 *
 * Description:
 *   This function is called whenever the LM92 device is opened.
 *
 ****************************************************************************/

static int lm92_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: lm92_close
 *
 * Description:
 *   This function is called whenever the LM92 device is closed.
 *
 ****************************************************************************/

static int lm92_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: lm92_read
 ****************************************************************************/

static ssize_t lm92_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct lm92_dev_s *priv  = inode->i_private;
  FAR b16_t             *ptr;
  ssize_t                nsamples;
  int                    i;
  int                    ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(b16_t);
  ptr      = (FAR b16_t *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      b16_t temp = 0;

      /* Read the next b16_t temperature value */

      ret = lm92_readtemp(priv, &temp);
      if (ret < 0)
        {
          snerr("ERROR: lm92_readtemp failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = temp;
    }

  return nsamples * sizeof(b16_t);
}

/****************************************************************************
 * Name: lm92_write
 ****************************************************************************/

static ssize_t lm92_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lm92_ioctl
 ****************************************************************************/

static int lm92_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct lm92_dev_s *priv  = inode->i_private;
  int                    ret   = OK;

  switch (cmd)
    {
      /* Read from the configuration register.  Arg: uint8_t* pointer */

      case SNIOC_READCONF:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm92_readconf(priv, ptr);
          sninfo("conf: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write to the configuration register.  Arg: uint8_t value */

      case SNIOC_WRITECONF:
        ret = lm92_writeconf(priv, (uint8_t)arg);
        sninfo("conf: %02x ret: %d\n", *(uint8_t *)arg, ret);
        break;

      /* Shutdown the LM92.  Arg:  None */

      case SNIOC_SHUTDOWN:
        {
          uint8_t conf;
          ret = lm92_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = lm92_writeconf(priv, conf | LM92_CONF_SHUTDOWN);
            }

          sninfo("conf: %02x ret: %d\n", conf | LM92_CONF_SHUTDOWN, ret);
        }
        break;

      /* Powerup the LM92.  Arg: None */

      case SNIOC_POWERUP:
        {
          uint8_t conf;
          ret = lm92_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = lm92_writeconf(priv, conf & ~LM92_CONF_SHUTDOWN);
            }

          sninfo("conf: %02x ret: %d\n", conf & ~LM92_CONF_SHUTDOWN, ret);
        }
        break;

      /* Report samples in Fahrenheit.  Arg: None */

      case SNIOC_FAHRENHEIT:
        priv->fahrenheit = true;
        sninfo("Fahrenheit\n");
        break;

      /* Report samples in Centigrade.  Arg: None */

      case SNIOC_CENTIGRADE:
        priv->fahrenheit = false;
        sninfo("Centigrade\n");
        break;

      /* Read THYS temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTHYS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm92_readb16(priv, LM92_THYS_REG, ptr);
          sninfo("THYS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write THYS temperature register.  Arg: b16_t value */

      case SNIOC_WRITETHYS:
        ret = lm92_writeb16(priv, LM92_THYS_REG, (b16_t)arg);
        sninfo("THYS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read TCRIT temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTCRIT:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm92_readb16(priv, LM92_TCRIT_REG, ptr);
          sninfo("TCRIT: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write TCRIT temperature register.  Arg: b16_t value */

      case SNIOC_WRITETCRIT:
        ret = lm92_writeb16(priv, LM92_TCRIT_REG, (b16_t)arg);
        sninfo("TCRIT: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read TLOW temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTLOW:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm92_readb16(priv, LM92_TLOW_REG, ptr);
          sninfo("TLOW: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write TLOW temperature register.  Arg: b16_t value */

      case SNIOC_WRITETLOW:
        ret = lm92_writeb16(priv, LM92_TLOW_REG, (b16_t)arg);
        sninfo("TLOW: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read THIGH temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTHIGH:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm92_readb16(priv, LM92_THIGH_REG, ptr);
          sninfo("THIGH: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write THIGH temperature register.  Arg: b16_t value */

      case SNIOC_WRITETHIGH:
        ret = lm92_writeb16(priv, LM92_THIGH_REG, (b16_t)arg);
        sninfo("THIGH: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read from the identification register.  Arg: uint16_t* pointer */

      case SNIOC_READID:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm92_readid(priv, ptr);
          sninfo("id: %04x ret: %d\n", *ptr, ret);
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
 * Name: lm92_register
 *
 * Description:
 *   Register the LM92 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   addr    - The I2C address of the LM92.  The base I2C address of the LM92
 *             is 0x48.  Bits 0-2 can be controlled to get 4 unique addresses
 *             from 0x48 through 0x4b.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lm92_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                  uint8_t addr)
{
  FAR struct lm92_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_LM92_ADDR0 || addr == CONFIG_LM92_ADDR1 ||
              addr == CONFIG_LM92_ADDR2 || addr == CONFIG_LM92_ADDR3);

  /* Initialize the LM92 device structure */

  priv = (FAR struct lm92_dev_s *)kmm_malloc(sizeof(struct lm92_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->fahrenheit = false;

  /* Register the character driver */

  ret = register_driver(devpath, &g_lm92fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_LM92 */
