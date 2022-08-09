/****************************************************************************
 * drivers/i2c/tca9548a.c
 * Driver for the TCA9448A i2c multiplexer
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
#include <nuttx/i2c/tca9548a.h>

#ifdef CONFIG_I2CMULTIPLEXER_TCA9548A

#ifndef CONFIG_TCA9548A_I2C_FREQUENCY
#  define CONFIG_TCA9548A_I2C_FREQUENCY    400000
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int tca9548a_write_config(FAR struct tca9548a_dev_s *priv,
                                 uint8_t regvalue);
static int tca9548a_read_config(FAR struct tca9548a_dev_s *priv,
                                FAR uint8_t *regvalue);

/* Other helpers */

static int tca9548a_select_channel(FAR struct tca9548a_dev_s *priv,
                                   uint8_t val);

/* I2C multiplexer vtable */

static int tca9548a_transfer_on_channel (FAR struct i2c_master_s *dev,
                                         FAR struct i2c_msg_s *msgs,
                                         int count);
#ifdef CONFIG_I2C_RESET
static int tca9548a_reset_on_channel (FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2cmux_ops =
{
  tca9548a_transfer_on_channel
#ifdef CONFIG_I2C_RESET
  , tca9548a_reset_on_channel
#endif
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2c_channel_dev_s
{
  FAR struct i2c_master_s vi2c;      /* Nested structure to allow casting as
                                      * public i2c master */
  uint8_t channel;                   /* Associated channel on the mux */
  FAR struct tca9548a_dev_s *dev;    /* Associated device */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca9548a_write_config
 *
 * Description:
 *   Write to the mux register of TCA9548A.
 *
 * Input Parameters:
 *   priv     - A pointer to the TCA9548A device structure.
 *   regvalue - The value that will be written.
 *
 * Returned Value:
 *   OK on success or a negative error.
 *
 ****************************************************************************/

static int tca9548a_write_config(FAR struct tca9548a_dev_s *priv,
                                 uint8_t regvalue)
{
  struct i2c_config_s iconf;
  int ret;

  iconf.frequency = CONFIG_TCA9548A_I2C_FREQUENCY;
  iconf.address   = priv->addr;
  iconf.addrlen   = 7;

  ret = i2c_write(priv->i2c, &iconf, &regvalue, 1);

  i2cinfo("Write to address 0x%02X; register value: 0x%02x ret: %d\n",
          priv->addr, regvalue, ret);
  return ret;
}

/****************************************************************************
 * Name: tca9548a_read_config
 *
 * Description:
 *   Read the mux register from TCA9548A.
 *
 * Input Parameters:
 *   priv     - A pointer to the TCA9548A device structure.
 *   regvalue - A pointer to a buffer into which data will be received.
 *
 * Returned Value:
 *   OK on success or a negative error.
 *
 ****************************************************************************/

static int tca9548a_read_config(FAR struct tca9548a_dev_s *priv,
                                FAR uint8_t *regvalue)
{
  struct i2c_config_s iconf;
  int ret;

  iconf.frequency = CONFIG_TCA9548A_I2C_FREQUENCY;
  iconf.address   = priv->addr;
  iconf.addrlen   = 7;

  ret = i2c_read(priv->i2c, &iconf, regvalue, 1);

  i2cinfo("Read from address: 0x%02X; register value: 0x%02x ret: %d\n",
          priv->addr, *regvalue, ret);
  return ret;
}

/****************************************************************************
 * Name: tca9548a_select_channel
 *
 * Description:
 *   Helper function to select a channel in the TCA9548A. The TCA9548A allows
 *   to select more than 1 channel at same time, but here we are forcing it
 *   to only enable a single channel by time. It will avoid collision in case
 *   where two devices with same address are connected to two channels.
 *
 * Input Parameters:
 *   dev  - Pointer to the (virtual) i2c_master_s.
 *   val  - The channel to be selected.
 *
 * Returned Value:
 *   OK on success or a negative error.
 *
 ****************************************************************************/

static int tca9548a_select_channel(FAR struct tca9548a_dev_s *priv,
                                   uint8_t val)
{
  int ret;

  if (val > TCA9548A_SEL_CH7)
    {
      /* channel not existent/supported */

      return -EINVAL;
    }

  if ((priv->state & (1 << val)) != 0)
    {
      /* channel already selected */

      return OK;
    }

  /* Try to setup the new channel */

  ret = tca9548a_write_config(priv, 1 << val);
  if (ret < 0)
    {
      return -ECOMM;
    }

  /* Modify state. Selecting a channel always enables the device */

  priv->state = 1 << val;

  return OK;
}

/****************************************************************************
 * Name: tca9548a_transfer_on_channel
 *
 * Description:
 *   Transfer an I2C message over the TCA9548A multiplexer. This function is
 *   attached to the virtual i2c_master ".transfer()" pointer. This way every
 *   time an I2C message is sent it is called and it starts sending a command
 *   to TCA9548A setup the right channel defined to this i2c_master. All the
 *   expected messages are then transferred to the I2C device connected the
 *   this selected channel.
 *
 * Input Parameters:
 *   dev   - Pointer to the (virtual) i2c_master_s.
 *   msgs  - the i2c message to be send to a device connected on TCA9548A.
 *   count - Number of messages to be send.
 *
 * Returned Value:
 *   OK on success or a negative error.
 *
 ****************************************************************************/

static int tca9548a_transfer_on_channel(FAR struct i2c_master_s *dev,
                                        FAR struct i2c_msg_s *msgs,
                                        int count)
{
  FAR struct i2c_channel_dev_s *ch_dev = (FAR struct i2c_channel_dev_s *)dev;
  FAR struct tca9548a_dev_s *priv = ch_dev->dev;
  int ret;

  /* select the mux channel */

  if (tca9548a_select_channel(priv, ch_dev->channel) != OK)
    {
      i2cerr("Could not select proper mux channel\n");
      return -ECOMM;  /* Signal error condition */
    }

  /* Resume the i2c transfer to the device connected to the mux channel */

  ret = I2C_TRANSFER(priv->i2c, msgs, count);
  i2cinfo("Selected channel %d and resumed transfer. Result: %d\n",
          ch_dev->channel, ret);
  return ret;
}

/****************************************************************************
 * Name: tca9548a_reset_on_channel
 *
 * Description:
 *   Reset the I2C device connected to the current channel
 *
 * Input Parameters:
 *   dev  - Pointer to the (virtual) i2c_master_s.
 *
 * Returned Value:
 *   OK on success or a negative error.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int tca9548a_reset_on_channel(FAR struct i2c_master_s *dev)
{
  FAR struct i2c_channel_dev_s *ch_dev = (struct i2c_channel_dev_s *)dev;
  FAR struct tca9548a_dev_s *priv = ch_dev->dev;
  int channel = ch_dev->channel;
  int ret;

  /* select the mux channel */

  if (tca9548a_select_channel(priv, channel) != OK)
    {
      i2cerr("Could not select proper mux channel\n");
      return -ECOMM;  /* signal error condition */
    }

  /* resume the i2c reset for the device connected to the mux channel */

  ret = I2C_RESET(priv->i2c);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca9548a_lower_half
 *
 * Description:
 *   Initialize the lower half of the TCA9548A by creating an i2c_master_s
 *   for the virtual i2c master and link it to the associated TCA9548A and
 *   its channel.
 *
 * Input Parameters:
 *   dev  - Pointer to the associated TCA9548A
 *   channel - The channel number as defined in tca9548a.h
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *
  tca9548a_lower_half(FAR struct tca9548a_dev_s *dev, uint8_t channel)
{
  FAR struct i2c_channel_dev_s *channel_dev;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Initialize the i2c_channel_dev_s device structure */

  channel_dev = (FAR struct i2c_channel_dev_s *)
                kmm_malloc(sizeof(struct i2c_channel_dev_s));

  if (channel_dev == NULL)
    {
      i2cerr("ERROR: Failed to allocate i2c channel lower half instance\n");
      return NULL;
    }

  /* set vi2c ops to their implementations */

  channel_dev->vi2c.ops = &g_i2cmux_ops;

  /* save state variables */

  channel_dev->dev  = dev;
  channel_dev->channel = channel;

  return &channel_dev->vi2c;
}

/****************************************************************************
 * Name: tca9548a_initialize
 *
 * Description:
 *   Initialize the TCA9548A device.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface to use to communicate with
 *          TCA9548A
 *   addr - The I2C address of the TCA9548A. The base I2C address of the
 *          TCA9548A is 0x70.
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct tca9548a_dev_s *
  tca9548a_initialize(FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct tca9548a_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the TCA9548A device structure */

  priv = (FAR struct tca9548a_dev_s *)
    kmm_malloc(sizeof(struct tca9548a_dev_s));

  if (priv == NULL)
    {
      i2cerr("ERROR: Failed to allocate instance\n");
      return NULL;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->state      = 0x00;

  if (tca9548a_read_config(priv, &priv->state) != OK)
    {
      i2cerr("Could not read initial state from the device\n");
      kmm_free(priv);
      return NULL;  /* signal error condition */
    }

  i2cinfo("Initial device state: %d\n", priv->state);

  if (tca9548a_select_channel(priv, TCA9548A_SEL_CH0) != OK)
    {
      i2cerr("Could not select mux channel 0\n");
      kmm_free(priv);
      return NULL;  /* signal error condition */
    }

  i2cinfo("TCA9548A (addr=0x%02x) set up with channel %d\n", priv->addr,
          TCA9548A_SEL_CH0);

  return priv;
}

#endif /* CONFIG_I2CMULTIPLEXER_TCA9548A */
