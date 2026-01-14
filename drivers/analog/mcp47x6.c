/****************************************************************************
 * drivers/analog/mcp47x6.c
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/analog/mcp47x6.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error I2C Support Required.
#endif

#if defined(CONFIG_MCP47X6)

#if defined(CONFIG_MCP4706)
#  define MCP47X6_DATA_BITS  8u
#  define MCP47X6_DATA_SHIFT 0u
#elif defined(CONFIG_MCP4716)
#  define MCP47X6_DATA_BITS  10u
#  define MCP47X6_DATA_SHIFT 2u
#elif defined(CONFIG_MCP4726)
#  define MCP47X6_DATA_BITS  12u
#  define MCP47X6_DATA_SHIFT 0u
#else
#  error MCP47x6 variant selection required
#endif

#ifndef CONFIG_MCP47X6_I2C_FREQUENCY
#  define CONFIG_MCP47X6_I2C_FREQUENCY 400000
#endif

#define MCP47X6_GAIN_MASK        (1u << MCP47X6_GAIN_SHIFT)
#define MCP47X6_GAIN_SHIFT       0u
#define MCP47X6_POWER_DOWN_MASK  (3u << MCP47X6_POWER_DOWN_SHIFT)
#define MCP47X6_POWER_DOWN_SHIFT 1u
#define MCP47X6_REFERENCE_MASK   (3u << MCP47X6_REFERENCE_SHIFT)
#define MCP47X6_REFERENCE_SHIFT  3u
#define MCP47X6_COMMAND_MASK     (7u << MCP47X6_COMMAND_SHIFT)
#define MCP47X6_COMMAND_SHIFT    5u

#define MCP47X6_DATA_MASK            ((1u << MCP47X6_DATA_BITS) - 1u)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcp47x6_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  uint8_t cmd;                   /* MCP47x6 current state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* DAC methods */

static void mcp47x6_reset(FAR struct dac_dev_s *dev);
static int  mcp47x6_setup(FAR struct dac_dev_s *dev);
static void mcp47x6_shutdown(FAR struct dac_dev_s *dev);
static void mcp47x6_txint(FAR struct dac_dev_s *dev, bool enable);
static int  mcp47x6_send(FAR struct dac_dev_s *dev,
                         FAR struct dac_msg_s *msg);
static int  mcp47x6_ioctl(FAR struct dac_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  mcp47x6_reset,        /* ao_reset */
  mcp47x6_setup,        /* ao_setup */
  mcp47x6_shutdown,     /* ao_shutdown */
  mcp47x6_txint,        /* ao_txint */
  mcp47x6_send,         /* ao_send */
  mcp47x6_ioctl         /* ao_ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp47x6_i2c_write
 *
 * Description:
 *   Send the raw content of a buffer to the DAC.
 *
 ****************************************************************************/

static int mcp47x6_i2c_write(FAR struct mcp47x6_dev_s *priv,
                             uint8_t const *source, size_t size)
{
  struct i2c_msg_s msg;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv->i2c != NULL);

  /* Setup for the transfer */

  msg.frequency = CONFIG_MCP47X6_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (uint8_t *)source; /* discard const qualifier */
  msg.length    = size;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      aerr("MCP47X6 I2C write transfer failed: %d", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: mcp47x6_i2c_read
 *
 * Description:
 *   Read raw content from the DAC
 *
 * Response bytes:
 * - volatile status and configuration
 * - volatile data byte
 * - volatile data byte (only for MCP4716 and MCP4726)
 * - non-volatile status and configuration
 * - non-volatile data byte
 * - non-volatile data byte (only for MCP4716 and MCP4726)
 *
 ****************************************************************************/

static int mcp47x6_i2c_read(FAR struct mcp47x6_dev_s *priv,
                             uint8_t *destination, size_t size)
{
  struct i2c_msg_s msg;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv->i2c != NULL);

  /* Setup for the transfer */

  msg.frequency = CONFIG_MCP47X6_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = destination;
  msg.length    = size;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      aerr("MCP47X6 I2C read transfer failed: %d", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: mcp47x6_reset
 *
 * Description:
 *   Reset the DAC device.  Called early to initialize the hardware.  This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void mcp47x6_reset(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp47x6_setup
 *
 * Description:
 *   Configure the DAC.  This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.  This
 *   setup includes configuring and attaching DAC interrupts.  Interrupts are
 *   all disabled upon return.
 *
 ****************************************************************************/

static int mcp47x6_setup(FAR struct dac_dev_s *dev)
{
  FAR struct mcp47x6_dev_s *priv = (FAR struct mcp47x6_dev_s *)dev->ad_priv;
  uint8_t response;
  int ret;

  /* Device's default settings after power up. */

  uint8_t default_settings = MCP47X6_REFERENCE_VDD_UNBUFFERED
                             | MCP47X6_POWER_DOWN_DISABLED
                             | MCP47X6_GAIN_1X;

  /* Retrieve the current device setup. */

  ret = mcp47x6_i2c_read(priv, &response, 1);
  if (ret < 0)
    {
      aerr("MCP47X6 I2C reading initial configuration failed: %d", ret);
      priv->cmd = default_settings;
      return ret;
    }

  /* Store the current setup for future configuration operations. */

  priv->cmd = response & (MCP47X6_REFERENCE_MASK
                         | MCP47X6_POWER_DOWN_MASK
                         | MCP47X6_GAIN_MASK);

  return OK;
}

/****************************************************************************
 * Name: mcp47x6_shutdown
 *
 * Description:
 *   Disable the DAC. This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void mcp47x6_shutdown(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp47x6_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void mcp47x6_txint(FAR struct dac_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: mcp47x6_send
 ****************************************************************************/

static int mcp47x6_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
  FAR struct mcp47x6_dev_s *priv = (FAR struct mcp47x6_dev_s *)dev->ad_priv;
  int ret;

  /* Set up message to send */

  ainfo("value: %08x", (unsigned int)msg->am_data);

  uint8_t const BUFFER_SIZE = 2;
  uint8_t buffer[BUFFER_SIZE];

  uint32_t data;
  data = msg->am_data & MCP47X6_DATA_MASK;
  data <<= MCP47X6_DATA_SHIFT;
  buffer[0] = (uint8_t)(data >> 8);
  buffer[1] = (uint8_t)(data);

  ret = mcp47x6_i2c_write(priv, buffer, sizeof(buffer));
  if (ret < 0)
    {
      aerr("ERROR: mcp47x6_send failed: %d", ret);
    }

  dac_txdone(dev);

  return ret;
}

/****************************************************************************
 * Name: mcp47x6_ioctl
 ****************************************************************************/

static int mcp47x6_ioctl(FAR struct dac_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct mcp47x6_dev_s *priv = (FAR struct mcp47x6_dev_s *)dev->ad_priv;
  int ret = OK;
  bool command_prepared = false;

  switch (cmd)
    {
      case ANIOC_MCP47X6_DAC_SET_GAIN:
        {
          switch (arg)
            {
              case MCP47X6_GAIN_1X:
              case MCP47X6_GAIN_2X:
                priv->cmd &= ~MCP47X6_GAIN_MASK;
                priv->cmd |= arg;
                command_prepared = true;
                break;
              default:
                ret = -EINVAL;
                break;
            }
        }
        break;

      case ANIOC_MCP47X6_DAC_SET_POWER_DOWN:
        {
          switch (arg)
            {
              case MCP47X6_POWER_DOWN_DISABLED:
              case MCP47X6_POWER_DOWN_1K:
              case MCP47X6_POWER_DOWN_100K:
              case MCP47X6_POWER_DOWN_500K:
                priv->cmd &= ~MCP47X6_POWER_DOWN_MASK;
                priv->cmd |= arg;
                command_prepared = true;
                break;
              default:
                ret = -EINVAL;
                break;
            }
          }
        break;

      case ANIOC_MCP47X6_DAC_SET_REFERENCE:
        {
          switch (arg)
            {
              case MCP47X6_REFERENCE_VDD_UNBUFFERED:
              case MCP47X6_REFERENCE_VREF_UNBUFFERED:
              case MCP47X6_REFERENCE_VREF_BUFFERED:
                priv->cmd &= ~MCP47X6_REFERENCE_MASK;
                priv->cmd |= arg;
                command_prepared = true;
                break;
              default:
                ret = -EINVAL;
                break;
            }
        }
        break;

    /* Command was not recognized */

      default:
        aerr("MCP47X6 ERROR: Unrecognized cmd: %d", cmd);
        ret = -ENOTTY;
        break;
    }

  if (command_prepared)
    {
      ret = mcp47x6_i2c_write(priv, &priv->cmd, sizeof(priv->cmd));
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp47x6_initialize
 *
 * Description:
 *   Initialize DAC
 *
 * Input Parameters:
 *   i2c - Pointer to a valid I2C master struct.
 *   addr - I2C device address.
 *
 * Returned Value:
 *   Valid MCP47X6 device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct dac_dev_s *mcp47x6_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr)
{
  FAR struct mcp47x6_dev_s *priv;
  FAR struct dac_dev_s *dacdev;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the MCP47X6 device structure */

  priv = kmm_malloc(sizeof(struct mcp47x6_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate mcp47x6_dev_s instance\n");
      free(priv);
      return NULL;
    }

  dacdev = kmm_malloc(sizeof(struct dac_dev_s));
  if (dacdev == NULL)
    {
      aerr("ERROR: Failed to allocate dac_dev_s instance\n");
      return NULL;
    }

  dacdev->ad_ops = &g_dacops;
  dacdev->ad_priv = priv;

  priv->i2c = i2c;
  priv->addr = addr;
  return dacdev;
}

#endif /* CONFIG_MCP47X6 */
