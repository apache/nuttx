/****************************************************************************
 * drivers/analog/mcp45xx.c
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

/* Driver for the Microchip MCP454X/MCP456X/MCP464X/MCP466X family: 7/8-bit,
 * single/dual, I2C digital potentiometers with a nonvolatile wiper
 * (DS22107B).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/analog/pot.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command byte: AD3:AD0 | C1:C0 | D9:D8 */

#define MCP45XX_REG_SHIFT     (4)
#define MCP45XX_CMD_WRITE     (0x0 << 2)
#define MCP45XX_CMD_INCR      (0x1 << 2)
#define MCP45XX_CMD_DECR      (0x2 << 2)
#define MCP45XX_CMD_READ      (0x3 << 2)
#define MCP45XX_DATA_MASK     (0x3ff)

/* Memory map */

#define MCP45XX_REG_VWIPER(n)  (n)       /* Volatile wiper n */
#define MCP45XX_REG_NVWIPER(n) (2 + (n)) /* Nonvolatile wiper n */
#define MCP45XX_REG_TCON       (0x04)    /* Volatile terminal control */
#define MCP45XX_REG_STATUS     (0x05)    /* Status */

/* TCON: R0HW is bit 3 and R1HW is bit 7.  Clearing it forces the wiper
 * into the shutdown configuration.
 */

#define MCP45XX_TCON_HW(n)    (1 << (3 + 4 * (n)))

/* STATUS bits */

#define MCP45XX_STATUS_WP      (1 << 0)         /* EEPROM write protected */
#define MCP45XX_STATUS_WL(n)   (1 << (1 + (n))) /* Wiper n locked */
#define MCP45XX_STATUS_EEWA    (1 << 3)         /* EEPROM write active */

/* Wiper full scale: 128 resistors (7-bit) or 256 resistors (8-bit) */

#define MCP45XX_MAX_7BIT      (0x80)
#define MCP45XX_MAX_8BIT      (0x100)

/* The EEPROM write cycle takes 5 ms typical, 10 ms maximum (tWC) */

#define MCP45XX_TWC_POLL_US   (1000)
#define MCP45XX_TWC_POLLS     (20)

/* Number of INCR/DECR commands sent in one continuous I2C transfer */

#define MCP45XX_MOVE_CHUNK    (16)

/* The only nonvolatile slot is the wiper register restored at power-up */

#define MCP45XX_SLOT_WIPER    (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcp45xx_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  struct i2c_config_s config;    /* I2C frequency and address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int  mcp45xx_reg_write(FAR struct mcp45xx_dev_s *priv, uint8_t reg,
                              uint16_t val);
static int  mcp45xx_reg_read(FAR struct mcp45xx_dev_s *priv, uint8_t reg,
                             FAR uint16_t *val);
static int  mcp45xx_wait_write(FAR struct mcp45xx_dev_s *priv);

/* Potentiometer methods */

static int  mcp45xx_setup(FAR struct pot_dev_s *dev);
static void mcp45xx_shutdown(FAR struct pot_dev_s *dev);
static int  mcp45xx_setwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                             uint32_t val);
static int  mcp45xx_getwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                             FAR uint32_t *val);
static int  mcp45xx_move(FAR struct pot_dev_s *dev, uint8_t wiper,
                         int32_t steps);
static int  mcp45xx_enable(FAR struct pot_dev_s *dev, uint8_t wiper,
                           bool enable);
static int  mcp45xx_store(FAR struct pot_dev_s *dev, uint8_t wiper,
                          uint8_t slot);
static int  mcp45xx_recall(FAR struct pot_dev_s *dev, uint8_t wiper,
                           uint8_t slot);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pot_ops_s g_potops =
{
  mcp45xx_setup,        /* po_setup */
  mcp45xx_shutdown,     /* po_shutdown */
  mcp45xx_setwiper,     /* po_setwiper */
  mcp45xx_getwiper,     /* po_getwiper */
  mcp45xx_move,         /* po_move */
  mcp45xx_enable,       /* po_enable */
  mcp45xx_store,        /* po_store */
  mcp45xx_recall,       /* po_recall */
  NULL                  /* po_ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp45xx_reg_write
 *
 * Description:
 *   Write a 9-bit value to a register.  A write to a nonvolatile location
 *   starts the EEPROM write cycle when the transfer ends.
 *
 ****************************************************************************/

static int mcp45xx_reg_write(FAR struct mcp45xx_dev_s *priv, uint8_t reg,
                             uint16_t val)
{
  uint8_t buffer[2];

  buffer[0] = (reg << MCP45XX_REG_SHIFT) | MCP45XX_CMD_WRITE |
              ((val >> 8) & 0x03);
  buffer[1] = val;

  return i2c_write(priv->i2c, &priv->config, buffer, 2);
}

/****************************************************************************
 * Name: mcp45xx_reg_read
 *
 * Description:
 *   Read a register.  The device answers with 10 data bits in two bytes.
 *
 ****************************************************************************/

static int mcp45xx_reg_read(FAR struct mcp45xx_dev_s *priv, uint8_t reg,
                            FAR uint16_t *val)
{
  uint8_t cmd;
  uint8_t buffer[2];
  int     ret;

  cmd = (reg << MCP45XX_REG_SHIFT) | MCP45XX_CMD_READ;

  ret = i2c_writeread(priv->i2c, &priv->config, &cmd, 1, buffer, 2);
  if (ret < 0)
    {
      return ret;
    }

  *val = ((uint16_t)buffer[0] << 8 | buffer[1]) & MCP45XX_DATA_MASK;
  return OK;
}

/****************************************************************************
 * Name: mcp45xx_wait_write
 *
 * Description:
 *   Wait for the EEPROM write cycle to end by polling STATUS.EEWA.  The
 *   STATUS register is volatile and can be read during the cycle.
 *
 ****************************************************************************/

static int mcp45xx_wait_write(FAR struct mcp45xx_dev_s *priv)
{
  uint16_t status;
  int      i;
  int      ret;

  for (i = 0; i < MCP45XX_TWC_POLLS; i++)
    {
      ret = mcp45xx_reg_read(priv, MCP45XX_REG_STATUS, &status);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & MCP45XX_STATUS_EEWA) == 0)
        {
          return OK;
        }

      nxsig_usleep(MCP45XX_TWC_POLL_US);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: mcp45xx_setup
 *
 * Description:
 *   Configure the potentiometer.  This method is called the first time
 *   that the potentiometer device is opened.
 *
 ****************************************************************************/

static int mcp45xx_setup(FAR struct pot_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: mcp45xx_shutdown
 *
 * Description:
 *   Disable the potentiometer.  This method is called when the
 *   potentiometer device is closed.
 *
 ****************************************************************************/

static void mcp45xx_shutdown(FAR struct pot_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp45xx_setwiper
 *
 * Description:
 *   Set the volatile wiper value.
 *
 ****************************************************************************/

static int mcp45xx_setwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                            uint32_t val)
{
  FAR struct mcp45xx_dev_s *priv = dev->pd_priv;

  return mcp45xx_reg_write(priv, MCP45XX_REG_VWIPER(wiper), val);
}

/****************************************************************************
 * Name: mcp45xx_getwiper
 *
 * Description:
 *   Get the volatile wiper value.
 *
 ****************************************************************************/

static int mcp45xx_getwiper(FAR struct pot_dev_s *dev, uint8_t wiper,
                            FAR uint32_t *val)
{
  FAR struct mcp45xx_dev_s *priv = dev->pd_priv;
  uint16_t                  regval;
  int                       ret;

  ret = mcp45xx_reg_read(priv, MCP45XX_REG_VWIPER(wiper), &regval);
  if (ret < 0)
    {
      return ret;
    }

  *val = regval;
  return OK;
}

/****************************************************************************
 * Name: mcp45xx_move
 *
 * Description:
 *   Move a wiper with the native INCR/DECR commands, sent as continuous
 *   commands in one transfer.  The device stops at full scale and at zero.
 *
 ****************************************************************************/

static int mcp45xx_move(FAR struct pot_dev_s *dev, uint8_t wiper,
                        int32_t steps)
{
  FAR struct mcp45xx_dev_s *priv = dev->pd_priv;
  uint8_t                   buffer[MCP45XX_MOVE_CHUNK];
  uint8_t                   cmd;
  uint32_t                  n;
  uint32_t                  len;
  int                       ret = OK;

  cmd = (MCP45XX_REG_VWIPER(wiper) << MCP45XX_REG_SHIFT) |
        (steps < 0 ? MCP45XX_CMD_DECR : MCP45XX_CMD_INCR);
  n   = steps < 0 ? -(uint32_t)steps : (uint32_t)steps;

  /* More steps than the full scale cannot change the result */

  if (n > dev->pd_max)
    {
      n = dev->pd_max;
    }

  memset(buffer, cmd, sizeof(buffer));

  while (n > 0 && ret == OK)
    {
      len = n > sizeof(buffer) ? sizeof(buffer) : n;
      ret = i2c_write(priv->i2c, &priv->config, buffer, len);
      n  -= len;
    }

  return ret;
}

/****************************************************************************
 * Name: mcp45xx_enable
 *
 * Description:
 *   Enable a wiper, or force it into the shutdown configuration (terminal
 *   A open and the wiper connected to terminal B) with the TCON HW bit.
 *
 ****************************************************************************/

static int mcp45xx_enable(FAR struct pot_dev_s *dev, uint8_t wiper,
                          bool enable)
{
  FAR struct mcp45xx_dev_s *priv = dev->pd_priv;
  uint16_t                  tcon;
  int                       ret;

  ret = mcp45xx_reg_read(priv, MCP45XX_REG_TCON, &tcon);
  if (ret < 0)
    {
      return ret;
    }

  if (enable)
    {
      tcon |= MCP45XX_TCON_HW(wiper);
    }
  else
    {
      tcon &= ~MCP45XX_TCON_HW(wiper);
    }

  return mcp45xx_reg_write(priv, MCP45XX_REG_TCON, tcon);
}

/****************************************************************************
 * Name: mcp45xx_store
 *
 * Description:
 *   Copy the volatile wiper to its nonvolatile register, which is loaded
 *   into the volatile one at power-up.  Slot 0 is the only slot.  The
 *   write is checked, because a write-protected or locked wiper ignores it
 *   without an error.
 *
 ****************************************************************************/

static int mcp45xx_store(FAR struct pot_dev_s *dev, uint8_t wiper,
                         uint8_t slot)
{
  FAR struct mcp45xx_dev_s *priv = dev->pd_priv;
  uint16_t                  status;
  uint16_t                  val;
  uint16_t                  stored;
  int                       ret;

  if (slot != MCP45XX_SLOT_WIPER)
    {
      return -EINVAL;
    }

  ret = mcp45xx_reg_read(priv, MCP45XX_REG_STATUS, &status);
  if (ret < 0)
    {
      return ret;
    }

  if ((status & (MCP45XX_STATUS_WP | MCP45XX_STATUS_WL(wiper))) != 0)
    {
      return -EROFS;
    }

  ret = mcp45xx_reg_read(priv, MCP45XX_REG_VWIPER(wiper), &val);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp45xx_reg_write(priv, MCP45XX_REG_NVWIPER(wiper), val);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp45xx_wait_write(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp45xx_reg_read(priv, MCP45XX_REG_NVWIPER(wiper), &stored);
  if (ret < 0)
    {
      return ret;
    }

  return stored == val ? OK : -EIO;
}

/****************************************************************************
 * Name: mcp45xx_recall
 *
 * Description:
 *   Load the volatile wiper from its nonvolatile register.  Slot 0 is the
 *   only slot.
 *
 ****************************************************************************/

static int mcp45xx_recall(FAR struct pot_dev_s *dev, uint8_t wiper,
                          uint8_t slot)
{
  FAR struct mcp45xx_dev_s *priv = dev->pd_priv;
  uint16_t                  val;
  int                       ret;

  if (slot != MCP45XX_SLOT_WIPER)
    {
      return -EINVAL;
    }

  ret = mcp45xx_reg_read(priv, MCP45XX_REG_NVWIPER(wiper), &val);
  if (ret < 0)
    {
      return ret;
    }

  return mcp45xx_reg_write(priv, MCP45XX_REG_VWIPER(wiper), val);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp45xx_initialize
 *
 * Description:
 *   Initialize a MCP454X/MCP456X/MCP464X/MCP466X potentiometer.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface to communicate with the device
 *   addr - The 7-bit I2C address of the device
 *   chip - The device type
 *   rab  - Terminal A-B resistance in ohms, 0 if unknown
 *
 * Returned Value:
 *   Valid device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct pot_dev_s *mcp45xx_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr,
                                         enum mcp45xx_chip_e chip,
                                         uint32_t rab)
{
  FAR struct mcp45xx_dev_s *priv;
  FAR struct pot_dev_s *potdev;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  priv = kmm_malloc(sizeof(struct mcp45xx_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate mcp45xx_dev_s instance\n");
      return NULL;
    }

  potdev = kmm_zalloc(sizeof(struct pot_dev_s));
  if (potdev == NULL)
    {
      aerr("ERROR: Failed to allocate pot_dev_s instance\n");
      kmm_free(priv);
      return NULL;
    }

  switch (chip)
    {
      case MCP45XX_MCP4541:
        potdev->pd_nwipers = 1;
        potdev->pd_max     = MCP45XX_MAX_7BIT;
        break;

      case MCP45XX_MCP4561:
        potdev->pd_nwipers = 1;
        potdev->pd_max     = MCP45XX_MAX_8BIT;
        break;

      case MCP45XX_MCP4641:
        potdev->pd_nwipers = 2;
        potdev->pd_max     = MCP45XX_MAX_7BIT;
        break;

      case MCP45XX_MCP4661:
        potdev->pd_nwipers = 2;
        potdev->pd_max     = MCP45XX_MAX_8BIT;
        break;

      default:
        aerr("ERROR: Unknown chip %d\n", chip);
        kmm_free(potdev);
        kmm_free(priv);
        return NULL;
    }

  potdev->pd_rab  = rab;
  potdev->pd_ops  = &g_potops;
  potdev->pd_priv = priv;

  priv->i2c              = i2c;
  priv->config.frequency = CONFIG_POT_MCP45XX_I2C_FREQUENCY;
  priv->config.address   = addr;
  priv->config.addrlen   = 7;

  return potdev;
}
