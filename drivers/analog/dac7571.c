/****************************************************************************
 * arch/drivers/analog/dac7571.c
 *
 *   Copyright (C) 2010, 2016, 2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2018 Daniel P. Carvalho. All rights reserved.
 *   Author: Daniel P. Carvalho <danieloak@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error I2C Support Required.
#endif

#if defined(CONFIG_DAC7571)

/* Operating modes - not controllable by user */

#define DAC7571_CONFIG_OPMODE_SHIFT    4
#define DAC7571_CONFIG_OPMODE_MASK     (0x0F << DAC7571_CONFIG_OPMODE_SHIFT)
#define DAC7571_CONFIG_OPMODE_NORMAL   (0 << DAC7571_CONFIG_OPMODE_SHIFT)
#define DAC7571_CONFIG_OPMODE_1K_PWD   (1 << DAC7571_CONFIG_OPMODE_SHIFT)
#define DAC7571_CONFIG_OPMODE_100K_PWD (2 << DAC7571_CONFIG_OPMODE_SHIFT)
#define DAC7571_CONFIG_OPMODE_HZ_PWD   (3 << DAC7571_CONFIG_OPMODE_SHIFT)

#ifndef CONFIG_DAC7571_I2C_FREQUENCY
#  define CONFIG_DAC7571_I2C_FREQUENCY 400000
#endif

#define I2C_NOSTARTSTOP_MSGS              2
#define I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX 0
#define I2C_NOSTARTSTOP_DATA_MSG_INDEX    1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dac7571_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  uint16_t state;                /* DAC7571 current state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

/* DAC methods */

static void dac7571_reset(FAR struct dac_dev_s *dev);
static int  dac7571_setup(FAR struct dac_dev_s *dev);
static void dac7571_shutdown(FAR struct dac_dev_s *dev);
static void dac7571_txint(FAR struct dac_dev_s *dev, bool enable);
static int  dac7571_send(FAR struct dac_dev_s *dev,
              FAR struct dac_msg_s *msg);
static int  dac7571_ioctl(FAR struct dac_dev_s *dev, int cmd,
              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dac7571_dev_s g_dacpriv;

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = dac7571_reset,
  .ao_setup    = dac7571_setup,
  .ao_shutdown = dac7571_shutdown,
  .ao_txint    = dac7571_txint,
  .ao_send     = dac7571_send,
  .ao_ioctl    = dac7571_ioctl,
};

static struct dac_dev_s g_dacdev =
{
  .ad_ops      = &g_dacops,
  .ad_priv     = &g_dacpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac7571_reset
 *
 * Description:
 *   Reset the DAC device. Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void dac7571_reset(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: dac7571_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts. Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int  dac7571_setup(FAR struct dac_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: dac7571_shutdown
 *
 * Description:
 *   Disable the DAC. This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void dac7571_shutdown(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: dac7571_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void dac7571_txint(FAR struct dac_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: dac7571_send
 ****************************************************************************/

static int dac7571_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
  FAR struct dac7571_dev_s *priv = (FAR struct dac7571_dev_s *)dev->ad_priv;
  struct i2c_config_s config;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv->i2c != NULL);

  /* Set up message to send */

  ainfo("value: %08x\n", msg->am_data);

  uint8_t const BUFFER_SIZE = 2;
  uint8_t buffer[BUFFER_SIZE];

  buffer[0] = (uint8_t)(msg->am_data >> 8);
  buffer[0] &= ~(DAC7571_CONFIG_OPMODE_MASK); /* only normal op mode supported */
  buffer[1] = (uint8_t)(msg->am_data);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_DAC7571_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Then perform the transfer. */

  ret = i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      aerr("ERROR: dac7571_send failed code:%d\n", ret);
    }

  dac_txdone(&g_dacdev);
  return ret;
}

/****************************************************************************
 * Name: dac7571_ioctl
 ****************************************************************************/

static int dac7571_ioctl(FAR struct dac_dev_s *dev, int cmd,
                         unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac7571_initialize
 *
 * Description:
 *   Initialize DAC
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple DAC interfaces)
 *
 * Returned Value:
 *   Valid DAC7571 device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct dac_dev_s *dac7571_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr)
{
  FAR struct dac7571_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the DAC7571 device structure */

  priv = (FAR struct dac7571_dev_s *)g_dacdev.ad_priv;
  priv->i2c = i2c;
  priv->addr = addr;
  return &g_dacdev;
}

#endif /* CONFIG_DAC7571 */
