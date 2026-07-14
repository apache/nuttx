/****************************************************************************
 * drivers/analog/mcp445x.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/analog/pot.h>
#include <nuttx/analog/mcp445x.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* MCP445X commands */

#define MCP445X_CMD_SHIFT    (2)
#define MCP445X_CMD_MASK     (3u << MCP445X_CMD_SHIFT)
#define MCP445X_CMD_WRITE    (0u << MCP445X_CMD_SHIFT)
#define MCP445X_CMD_INCR     (1u << MCP445X_CMD_SHIFT)
#define MCP445X_CMD_DECR     (2u << MCP445X_CMD_SHIFT)
#define MCP445X_CMD_READ     (3u << MCP445X_CMD_SHIFT)

#define MCP445X_REG_SHIFT    (4)
#define MCP445X_DATA_MASK    (0x1ffu)

/* Register map */

#define MCP445X_REG_WIPER0   (0x00)  /* Volatile Wiper 0 */
#define MCP445X_REG_WIPER1   (0x01)  /* Volatile Wiper 1 */
#define MCP445X_REG_TCON0    (0x04)  /* Volatile TCON0 (Wipers 0-1) */
#define MCP445X_REG_STATUS   (0x05)  /* Status */
#define MCP445X_REG_WIPER2   (0x06)  /* Volatile Wiper 2 */
#define MCP445X_REG_WIPER3   (0x07)  /* Volatile Wiper 3 */
#define MCP445X_REG_TCON1    (0x0a)  /* Volatile TCON1 (Wipers 2-3) */

#define MCP445X_NWIPERS      (4)

/* Wiper full scale (8-bit devices, 257 taps) */

#define MCP445X_WIPER_MAX    (0x100)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcp445x_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int  mcp445x_reg_write(FAR struct mcp445x_dev_s *priv, uint8_t reg,
                              uint16_t val);
static int  mcp445x_reg_read(FAR struct mcp445x_dev_s *priv, uint8_t reg,
                             FAR uint16_t *val);

/* Potentiometer methods */

static int  mcp445x_setup(FAR struct pot_dev_s *dev);
static void mcp445x_shutdown(FAR struct pot_dev_s *dev);
static int  mcp445x_setwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                             uint32_t val);
static int  mcp445x_getwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                             FAR uint32_t *val);
static int  mcp445x_move(FAR struct pot_dev_s *dev, uint8_t wiper,
                         int32_t steps);
static int  mcp445x_enable(FAR struct pot_dev_s *dev, uint8_t wiper,
                           bool enable);
static int  mcp445x_settcon(FAR struct pot_dev_s *dev, uint8_t wiper,
                            uint8_t flags);
static int  mcp445x_gettcon(FAR struct pot_dev_s *dev, uint8_t wiper,
                            FAR uint8_t *flags);
static int  mcp445x_ioctl(FAR struct pot_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Volatile wiper register for a given wiper index */

static const uint8_t g_mcp445x_wiper_reg[MCP445X_NWIPERS] =
{
  MCP445X_REG_WIPER0,
  MCP445X_REG_WIPER1,
  MCP445X_REG_WIPER2,
  MCP445X_REG_WIPER3
};

static const struct pot_ops_s g_potops =
{
  mcp445x_setup,        /* po_setup */
  mcp445x_shutdown,     /* po_shutdown */
  mcp445x_setwiper,     /* po_setwiper */
  mcp445x_getwiper,     /* po_getwiper */
  mcp445x_move,         /* po_move */
  mcp445x_enable,       /* po_enable */
  NULL,                 /* po_store */
  NULL,                 /* po_recall */
  mcp445x_ioctl         /* po_ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp445x_reg_write
 *
 * Description:
 *   Write a 9-bit value to a MCP445X register.
 *
 ****************************************************************************/

static int mcp445x_reg_write(FAR struct mcp445x_dev_s *priv, uint8_t reg,
                             uint16_t val)
{
  struct i2c_config_s config;
  uint8_t             buffer[2];

  buffer[0] = (reg << MCP445X_REG_SHIFT) | MCP445X_CMD_WRITE |
              ((val >> 8) & 0x01);
  buffer[1] = val;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_POT_MCP445X_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  return i2c_write(priv->i2c, &config, buffer, 2);
}

/****************************************************************************
 * Name: mcp445x_reg_read
 *
 * Description:
 *   Read a 9-bit value from a MCP445X register.
 *
 ****************************************************************************/

static int mcp445x_reg_read(FAR struct mcp445x_dev_s *priv, uint8_t reg,
                            FAR uint16_t *val)
{
  struct i2c_config_s config;
  uint8_t             cmd;
  uint8_t             buffer[2];
  int                 ret;

  cmd = (reg << MCP445X_REG_SHIFT) | MCP445X_CMD_READ;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_POT_MCP445X_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_writeread(priv->i2c, &config, &cmd, 1, buffer, 2);
  if (ret < 0)
    {
      return ret;
    }

  *val = ((uint16_t)buffer[0] << 8 | buffer[1]) & MCP445X_DATA_MASK;
  return OK;
}

/****************************************************************************
 * Name: mcp445x_setup
 *
 * Description:
 *   Configure the potentiometer. This method is called the first time
 *   that the potentiometer device is opened.
 *
 ****************************************************************************/

static int mcp445x_setup(FAR struct pot_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: mcp445x_shutdown
 *
 * Description:
 *   Disable the potentiometer. This method is called when the
 *   potentiometer device is closed.
 *
 ****************************************************************************/

static void mcp445x_shutdown(FAR struct pot_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp445x_setwiper
 *
 * Description:
 *   Set a wiper value.
 *
 ****************************************************************************/

static int mcp445x_setwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                            uint32_t val)
{
  FAR struct mcp445x_dev_s *priv = dev->pd_priv;

  return mcp445x_reg_write(priv, g_mcp445x_wiper_reg[wiper], val);
}

/****************************************************************************
 * Name: mcp445x_getwiper
 *
 * Description:
 *   Get a wiper value.
 *
 ****************************************************************************/

static int mcp445x_getwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                            FAR uint32_t *val)
{
  FAR struct mcp445x_dev_s *priv = dev->pd_priv;
  uint16_t                  regval;
  int                       ret;

  ret = mcp445x_reg_read(priv, g_mcp445x_wiper_reg[wiper], &regval);
  if (ret < 0)
    {
      return ret;
    }

  *val = regval;
  return OK;
}

/****************************************************************************
 * Name: mcp445x_cmd
 *
 * Description:
 *   Send a single-byte command (INCR/DECR) for a register.
 *
 ****************************************************************************/

static int mcp445x_cmd(FAR struct mcp445x_dev_s *priv, uint8_t reg,
                       uint8_t cmd)
{
  struct i2c_config_s config;
  uint8_t             buffer;

  buffer = (reg << MCP445X_REG_SHIFT) | cmd;

  config.frequency = CONFIG_POT_MCP445X_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  return i2c_write(priv->i2c, &config, &buffer, 1);
}

/****************************************************************************
 * Name: mcp445x_move
 *
 * Description:
 *   Move a wiper using the native INCR/DECR commands. The device stops
 *   at full scale and at zero.
 *
 ****************************************************************************/

static int mcp445x_move(FAR struct pot_dev_s *dev, uint8_t wiper,
                        int32_t steps)
{
  FAR struct mcp445x_dev_s *priv = dev->pd_priv;
  uint8_t                   cmd;
  uint32_t                  n;
  int                       ret = OK;

  cmd = steps < 0 ? MCP445X_CMD_DECR : MCP445X_CMD_INCR;
  n   = steps < 0 ? -(uint32_t)steps : (uint32_t)steps;

  if (n > dev->pd_max)
    {
      n = dev->pd_max;
    }

  while (n-- > 0 && ret == OK)
    {
      ret = mcp445x_cmd(priv, g_mcp445x_wiper_reg[wiper], cmd);
    }

  return ret;
}

/****************************************************************************
 * Name: mcp445x_enable
 *
 * Description:
 *   Enable or force a wiper into shutdown via the TCON HW bit.
 *
 ****************************************************************************/

static int mcp445x_enable(FAR struct pot_dev_s *dev, uint8_t wiper,
                          bool enable)
{
  uint8_t flags;
  int     ret;

  ret = mcp445x_gettcon(dev, wiper, &flags);
  if (ret < 0)
    {
      return ret;
    }

  if (enable)
    {
      flags |= MCP445X_TCON_HW;
    }
  else
    {
      flags &= ~MCP445X_TCON_HW;
    }

  return mcp445x_settcon(dev, wiper, flags);
}

/****************************************************************************
 * Name: mcp445x_settcon
 *
 * Description:
 *   Set wiper terminal control flags. The MCP445X_TCON_* flags map
 *   directly to the MCP445X TCON nibble (D0=B, D1=W, D2=A, D3=HW).
 *
 ****************************************************************************/

static int mcp445x_settcon(FAR struct pot_dev_s *dev, uint8_t wiper,
                           uint8_t flags)
{
  FAR struct mcp445x_dev_s *priv = dev->pd_priv;
  uint8_t                   reg;
  uint8_t                   shift;
  uint16_t                  regval;
  int                       ret;

  reg   = wiper < 2 ? MCP445X_REG_TCON0 : MCP445X_REG_TCON1;
  shift = (wiper & 1) ? 4 : 0;

  ret = mcp445x_reg_read(priv, reg, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~(0xf << shift);
  regval |= (flags & 0xf) << shift;

  return mcp445x_reg_write(priv, reg, regval);
}

/****************************************************************************
 * Name: mcp445x_gettcon
 *
 * Description:
 *   Get wiper terminal control flags.
 *
 ****************************************************************************/

static int mcp445x_gettcon(FAR struct pot_dev_s *dev, uint8_t wiper,
                           FAR uint8_t *flags)
{
  FAR struct mcp445x_dev_s *priv = dev->pd_priv;
  uint8_t                   reg;
  uint8_t                   shift;
  uint16_t                  regval;
  int                       ret;

  reg   = wiper < 2 ? MCP445X_REG_TCON0 : MCP445X_REG_TCON1;
  shift = (wiper & 1) ? 4 : 0;

  ret = mcp445x_reg_read(priv, reg, &regval);
  if (ret < 0)
    {
      return ret;
    }

  *flags = (regval >> shift) & 0xf;
  return OK;
}

/****************************************************************************
 * Name: mcp445x_ioctl
 *
 * Description:
 *   Chip-specific terminal control.
 *
 ****************************************************************************/

static int mcp445x_ioctl(FAR struct pot_dev_s *dev, int cmd,
                         unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_MCP445X_SET_TCON:
        {
          FAR struct mcp445x_tcon_s *tcon =
            (FAR struct mcp445x_tcon_s *)((uintptr_t)arg);

          DEBUGASSERT(tcon != NULL);
          if (tcon->wiper >= MCP445X_NWIPERS)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = mcp445x_settcon(dev, tcon->wiper, tcon->flags);
            }

          break;
        }

      case ANIOC_MCP445X_GET_TCON:
        {
          FAR struct mcp445x_tcon_s *tcon =
            (FAR struct mcp445x_tcon_s *)((uintptr_t)arg);

          DEBUGASSERT(tcon != NULL);
          if (tcon->wiper >= MCP445X_NWIPERS)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = mcp445x_gettcon(dev, tcon->wiper, &tcon->flags);
            }

          break;
        }

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp445x_initialize
 *
 * Description:
 *   Initialize a MCP445X potentiometer.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface to communicate with the device
 *   addr - The I2C address of the MCP445X
 *   rab  - Terminal A-B resistance in ohms, 0 if unknown
 *
 * Returned Value:
 *   Valid MCP445X device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct pot_dev_s *mcp445x_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr, uint32_t rab)
{
  FAR struct mcp445x_dev_s *priv;
  FAR struct pot_dev_s *potdev;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the MCP445X device structure */

  priv = kmm_malloc(sizeof(struct mcp445x_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate mcp445x_dev_s instance\n");
      return NULL;
    }

  potdev = kmm_zalloc(sizeof(struct pot_dev_s));
  if (potdev == NULL)
    {
      aerr("ERROR: Failed to allocate pot_dev_s instance\n");
      kmm_free(priv);
      return NULL;
    }

  potdev->pd_nwipers = MCP445X_NWIPERS;
  potdev->pd_max     = MCP445X_WIPER_MAX;
  potdev->pd_rab     = rab;
  potdev->pd_ops     = &g_potops;
  potdev->pd_priv    = priv;

  priv->i2c  = i2c;
  priv->addr = addr;
  return potdev;
}
