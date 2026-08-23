/****************************************************************************
 * drivers/ioexpander/ch422g.c
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

/* The WCH CH422G is an I2C I/O expander offering eight bi-directional
 * pins, IO0-IO7, and four open-drain outputs, OC0-OC3.
 *
 * It does not follow the usual convention of a register address written
 * ahead of the data.  Each register is reached through an I2C address of
 * its own and every access carries a single data byte:
 *
 *   0x24  write  System parameter register
 *   0x23  write  OC0-OC3 output register
 *   0x38  write  IO0-IO7 output register
 *   0x26  read   IO0-IO7 input register
 *
 * None of the write-only registers can be read back, so the driver keeps a
 * shadow copy of each and updates it in step with the device.
 *
 * IO0-IO7 do not have individual direction control.  A single bit of the
 * system parameter register, IO_OE, drives the whole group.  The driver
 * records the direction requested for each pin and puts the group in output
 * mode when at least one of them is an output, which is what a board that
 * mixes the two would expect of the pins it drives.  Reading a pin of a
 * group held in output mode returns the shadowed output value rather than
 * the level on the pin, because the hardware cannot report it.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/ch422g.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C addresses that select a register.  See the comment above. */

#define CH422G_ADDR_WR_SET      0x24  /* System parameter register */
#define CH422G_ADDR_WR_OC       0x23  /* OC0-OC3 output register */
#define CH422G_ADDR_WR_IO       0x38  /* IO0-IO7 output register */
#define CH422G_ADDR_RD_IO       0x26  /* IO0-IO7 input register */

/* System parameter register bits */

#define CH422G_SET_IO_OE        (1 << 0)  /* IO0-IO7 drive their pins */
#define CH422G_SET_A_SCAN       (1 << 1)  /* Digital tube scan enable */
#define CH422G_SET_OD_EN        (1 << 2)  /* OC0-OC3 open-drain enable */
#define CH422G_SET_SLEEP        (1 << 3)  /* Low power mode */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ch422g_dev_s
{
  /* Must appear first so the structure can be cast to the public type */

  struct ioexpander_dev_s dev;

  FAR struct i2c_master_s     *i2c;    /* Saved I2C driver instance */
  FAR struct ch422g_config_s  *config; /* Board configuration data */
  mutex_t                      lock;   /* Mutual exclusion */

  uint8_t sysparam;                    /* Shadow of system parameter reg */
  uint8_t outio;                       /* Shadow of IO0-IO7 output reg */
  uint8_t outoc;                       /* Shadow of OC0-OC3 output reg */
  uint8_t outmask;                     /* IO0-IO7 configured as outputs */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ch422g_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            int dir);
static int ch422g_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         int opt, FAR void *val);
static int ch422g_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           bool value);
static int ch422g_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value);
static int ch422g_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int ch422g_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                FAR const uint8_t *pins,
                                FAR const bool *values, int count);
static int ch422g_multireadpin(FAR struct ioexpander_dev_s *dev,
                               FAR const uint8_t *pins, FAR bool *values,
                               int count);
static int ch422g_multireadbuf(FAR struct ioexpander_dev_s *dev,
                               FAR const uint8_t *pins, FAR bool *values,
                               int count);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ioexpander_ops_s g_ch422g_ops =
{
  .ioe_direction     = ch422g_direction,
  .ioe_option        = ch422g_option,
  .ioe_writepin      = ch422g_writepin,
  .ioe_readpin       = ch422g_readpin,
  .ioe_readbuf       = ch422g_readbuf,
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  .ioe_multiwritepin = ch422g_multiwritepin,
  .ioe_multireadpin  = ch422g_multireadpin,
  .ioe_multireadbuf  = ch422g_multireadbuf,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ch422g_write_reg
 *
 * Description:
 *   Write the single data byte of one register.  The register is selected
 *   by the I2C address the transfer is addressed to.
 *
 ****************************************************************************/

static int ch422g_write_reg(FAR struct ch422g_dev_s *priv, uint8_t addr,
                            uint8_t value)
{
  struct i2c_msg_s msg;

  msg.frequency = priv->config->frequency;
  msg.addr      = addr;
  msg.flags     = 0;
  msg.buffer    = &value;
  msg.length    = 1;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

/****************************************************************************
 * Name: ch422g_read_io
 *
 * Description:
 *   Read the level of IO0-IO7.  Only meaningful while the group is in
 *   input mode.
 *
 ****************************************************************************/

static int ch422g_read_io(FAR struct ch422g_dev_s *priv, FAR uint8_t *value)
{
  struct i2c_msg_s msg;

  msg.frequency = priv->config->frequency;
  msg.addr      = CH422G_ADDR_RD_IO;
  msg.flags     = I2C_M_READ;
  msg.buffer    = value;
  msg.length    = 1;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

/****************************************************************************
 * Name: ch422g_setdir
 *
 * Description:
 *   Bring the IO_OE bit into agreement with the recorded per-pin
 *   directions.  Must be called with the lock held.
 *
 ****************************************************************************/

static int ch422g_setdir(FAR struct ch422g_dev_s *priv)
{
  uint8_t sysparam = priv->sysparam;

  if (priv->outmask != 0)
    {
      sysparam |= CH422G_SET_IO_OE;
    }
  else
    {
      sysparam &= ~CH422G_SET_IO_OE;
    }

  if (sysparam == priv->sysparam)
    {
      return OK;
    }

  priv->sysparam = sysparam;
  return ch422g_write_reg(priv, CH422G_ADDR_WR_SET, sysparam);
}

/****************************************************************************
 * Name: ch422g_direction
 *
 * Description:
 *   Set the direction of an I/O pin.
 *
 ****************************************************************************/

static int ch422g_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            int dir)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  int ret;

  if (pin >= CH422G_NPINS)
    {
      return -ENXIO;
    }

  if (dir != IOEXPANDER_DIRECTION_IN && dir != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  /* OC0-OC3 are open-drain outputs and cannot be inputs */

  if (pin >= CH422G_NIO)
    {
      return dir == IOEXPANDER_DIRECTION_OUT ? OK : -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (dir == IOEXPANDER_DIRECTION_OUT)
    {
      priv->outmask |= (1 << pin);
    }
  else
    {
      priv->outmask &= ~(1 << pin);
    }

  ret = ch422g_setdir(priv);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: ch422g_option
 *
 * Description:
 *   Set pin options.  The CH422G offers none of the options the interface
 *   defines, so only a request that asks for nothing succeeds.
 *
 ****************************************************************************/

static int ch422g_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         int opt, FAR void *val)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  int ret = -ENOSYS;

  if (pin >= CH422G_NPINS)
    {
      return -ENXIO;
    }

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      ret = ((uintptr_t)val == IOEXPANDER_VAL_NORMAL) ? OK : -ENOSYS;
    }
  else if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      ret = ((uintptr_t)val == IOEXPANDER_VAL_DISABLE) ? OK : -ENOSYS;
    }

  UNUSED(priv);
  return ret;
}

/****************************************************************************
 * Name: ch422g_writepin
 *
 * Description:
 *   Set the level of an output pin.
 *
 ****************************************************************************/

static int ch422g_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           bool value)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  FAR uint8_t *shadow;
  uint8_t addr;
  uint8_t bit;
  int ret;

  if (pin >= CH422G_NPINS)
    {
      return -ENXIO;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (pin < CH422G_NIO)
    {
      shadow = &priv->outio;
      addr   = CH422G_ADDR_WR_IO;
      bit    = 1 << pin;
    }
  else
    {
      shadow = &priv->outoc;
      addr   = CH422G_ADDR_WR_OC;
      bit    = 1 << (pin - CH422G_NIO);
    }

  if (value)
    {
      *shadow |= bit;
    }
  else
    {
      *shadow &= ~bit;
    }

  ret = ch422g_write_reg(priv, addr, *shadow);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: ch422g_readpin
 *
 * Description:
 *   Read the level of a pin.  A pin of a group held in output mode, and any
 *   of the write-only open-drain outputs, reports the value last written.
 *
 ****************************************************************************/

static int ch422g_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  uint8_t regval;
  int ret;

  if (pin >= CH422G_NPINS)
    {
      return -ENXIO;
    }

  DEBUGASSERT(value != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (pin >= CH422G_NIO)
    {
      *value = ((priv->outoc >> (pin - CH422G_NIO)) & 1) != 0;
      ret    = OK;
    }
  else if ((priv->sysparam & CH422G_SET_IO_OE) != 0)
    {
      *value = ((priv->outio >> pin) & 1) != 0;
      ret    = OK;
    }
  else
    {
      ret = ch422g_read_io(priv, &regval);
      if (ret >= 0)
        {
          *value = ((regval >> pin) & 1) != 0;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: ch422g_readbuf
 *
 * Description:
 *   Read the last value written to a pin.  The CH422G does not buffer a
 *   separately readable copy, so this is the shadowed output value.
 *
 ****************************************************************************/

static int ch422g_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  int ret;

  if (pin >= CH422G_NPINS)
    {
      return -ENXIO;
    }

  DEBUGASSERT(value != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (pin < CH422G_NIO)
    {
      *value = ((priv->outio >> pin) & 1) != 0;
    }
  else
    {
      *value = ((priv->outoc >> (pin - CH422G_NIO)) & 1) != 0;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: ch422g_multiwritepin
 *
 * Description:
 *   Set the level of several pins.  Pins that share a register are gathered
 *   so that the register is written once.
 *
 ****************************************************************************/

static int ch422g_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                FAR const uint8_t *pins,
                                FAR const bool *values, int count)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  bool touchio = false;
  bool touchoc = false;
  int ret;
  int i;

  DEBUGASSERT(pins != NULL && values != NULL);

  for (i = 0; i < count; i++)
    {
      if (pins[i] >= CH422G_NPINS)
        {
          return -ENXIO;
        }
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < count; i++)
    {
      uint8_t pin = pins[i];

      if (pin < CH422G_NIO)
        {
          if (values[i])
            {
              priv->outio |= (1 << pin);
            }
          else
            {
              priv->outio &= ~(1 << pin);
            }

          touchio = true;
        }
      else
        {
          if (values[i])
            {
              priv->outoc |= (1 << (pin - CH422G_NIO));
            }
          else
            {
              priv->outoc &= ~(1 << (pin - CH422G_NIO));
            }

          touchoc = true;
        }
    }

  ret = OK;

  if (touchio)
    {
      ret = ch422g_write_reg(priv, CH422G_ADDR_WR_IO, priv->outio);
    }

  if (ret >= 0 && touchoc)
    {
      ret = ch422g_write_reg(priv, CH422G_ADDR_WR_OC, priv->outoc);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: ch422g_multireadpin
 *
 * Description:
 *   Read the level of several pins with a single read of the device.
 *
 ****************************************************************************/

static int ch422g_multireadpin(FAR struct ioexpander_dev_s *dev,
                               FAR const uint8_t *pins, FAR bool *values,
                               int count)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  uint8_t regval = 0;
  bool haveinput = false;
  int ret;
  int i;

  DEBUGASSERT(pins != NULL && values != NULL);

  for (i = 0; i < count; i++)
    {
      if (pins[i] >= CH422G_NPINS)
        {
          return -ENXIO;
        }
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if ((priv->sysparam & CH422G_SET_IO_OE) == 0)
    {
      for (i = 0; i < count; i++)
        {
          if (pins[i] < CH422G_NIO)
            {
              haveinput = true;
              break;
            }
        }
    }

  if (haveinput)
    {
      ret = ch422g_read_io(priv, &regval);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return ret;
        }
    }

  for (i = 0; i < count; i++)
    {
      uint8_t pin = pins[i];

      if (pin >= CH422G_NIO)
        {
          values[i] = ((priv->outoc >> (pin - CH422G_NIO)) & 1) != 0;
        }
      else if (haveinput)
        {
          values[i] = ((regval >> pin) & 1) != 0;
        }
      else
        {
          values[i] = ((priv->outio >> pin) & 1) != 0;
        }
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: ch422g_multireadbuf
 *
 * Description:
 *   Read the last value written to several pins.
 *
 ****************************************************************************/

static int ch422g_multireadbuf(FAR struct ioexpander_dev_s *dev,
                               FAR const uint8_t *pins, FAR bool *values,
                               int count)
{
  FAR struct ch422g_dev_s *priv = (FAR struct ch422g_dev_s *)dev;
  int ret;
  int i;

  DEBUGASSERT(pins != NULL && values != NULL);

  for (i = 0; i < count; i++)
    {
      if (pins[i] >= CH422G_NPINS)
        {
          return -ENXIO;
        }
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < count; i++)
    {
      uint8_t pin = pins[i];

      if (pin < CH422G_NIO)
        {
          values[i] = ((priv->outio >> pin) & 1) != 0;
        }
      else
        {
          values[i] = ((priv->outoc >> (pin - CH422G_NIO)) & 1) != 0;
        }
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

#endif /* CONFIG_IOEXPANDER_MULTIPIN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ch422g_initialize
 *
 * Description:
 *   Instantiate and configure the CH422G device driver to use the provided
 *   I2C device instance.
 *
 * Input Parameters:
 *   i2c    - An I2C driver instance
 *   config - Persistent board configuration data
 *
 * Returned Value:
 *   An ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *
ch422g_initialize(FAR struct i2c_master_s *i2c,
                  FAR struct ch422g_config_s *config)
{
  FAR struct ch422g_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL);

  priv = kmm_zalloc(sizeof(struct ch422g_dev_s));
  if (priv == NULL)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }

  priv->dev.ops = &g_ch422g_ops;
  priv->i2c     = i2c;
  priv->config  = config;

  nxmutex_init(&priv->lock);

  /* Leave the device in a known state: the open-drain outputs off, the
   * bi-directional pins low, and the group in input mode until a board
   * asks for an output.  Writing the system parameter register first also
   * confirms that the device is answering.
   */

  ret = ch422g_write_reg(priv, CH422G_ADDR_WR_SET, priv->sysparam);
  if (ret < 0)
    {
      gpioerr("ERROR: CH422G not responding: %d\n", ret);
      goto errout;
    }

  ret = ch422g_write_reg(priv, CH422G_ADDR_WR_OC, priv->outoc);
  if (ret < 0)
    {
      goto errout;
    }

  ret = ch422g_write_reg(priv, CH422G_ADDR_WR_IO, priv->outio);
  if (ret < 0)
    {
      goto errout;
    }

  return &priv->dev;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return NULL;
}

