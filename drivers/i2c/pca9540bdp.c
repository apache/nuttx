/****************************************************************************
 * drivers/i2c/pca9540bdp.c
 * Driver for the PCA9540BDP i2c multiplexer
 *
 *   Copyright (C) 2018 Giorgio Groß. All rights reserved.
 *   Author: Giorgio Groß <giorgio.gross@robodev.eu>
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

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include "pca9540bdp.h"
#include <nuttx/i2c/pca9540bdp.h>

#ifdef CONFIG_I2CMULTIPLEXER_PCA9540BDP

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int pca9540bdp_write_config(FAR struct pca9540bdp_dev_s *priv,
                    FAR uint8_t regvalue);
static int pca9540bdp_read_config(FAR struct pca9540bdp_dev_s *priv,
                    FAR uint8_t *regvalue);

/* Other helpers */

static int pca9540bdp_select_port(FAR struct pca9540bdp_dev_s *priv,
                    uint8_t val);

/* I2C multiplexer vtable */

static int pca9540bdp_transfer_on_port (FAR struct i2c_master_s *dev,
                    FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int pca9540bdp_reset_on_port (FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2cmux_ops =
{
  pca9540bdp_transfer_on_port
#ifdef CONFIG_I2C_RESET
  , pca9540bdp_reset_on_port
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pca9540bdp_write_config(FAR struct pca9540bdp_dev_s *priv,
                                   FAR uint8_t regvalue)
{
  struct i2c_config_s iconf;
  int ret;

  iconf.frequency = CONFIG_PCA9549BDP_I2C_FREQUENCY;
  iconf.address   = priv->addr;
  iconf.addrlen   = 7;

  ret = i2c_write(priv->i2c, &iconf, &regvalue, 1);

  i2cinfo("Write to address 0x%02X; register value: 0x%02x ret: %d\n",
          priv->addr, regvalue, ret);
  return ret;
}

static int pca9540bdp_read_config(FAR struct pca9540bdp_dev_s *priv,
                                  FAR uint8_t *regvalue)
{
  struct i2c_config_s iconf;
  int ret;

  iconf.frequency = CONFIG_PCA9549BDP_I2C_FREQUENCY;
  iconf.address   = priv->addr;
  iconf.addrlen   = 7;

  ret = i2c_read(priv->i2c, &iconf, regvalue, 1);

  i2cinfo("Read from address: 0x%02X; register value: 0x%02x ret: %d\n",
          priv->addr, *regvalue, ret);
  return ret;
}

static int pca9540bdp_select_port(FAR struct pca9540bdp_dev_s *priv,
                                  uint8_t val)
{
  if (val != PCA9540BDP_SEL_PORT0 && val != PCA9540BDP_SEL_PORT1)
    {
      /* port not supported */

      return -EINVAL;
    }

  if ((PCA9540BDP_ENABLE | val) == priv->state)
    {
      /* port already selected */

      return OK;
    }

  /* Modify state and write it to the mux */
  /* selecting a port always enables the device */

  priv->state = PCA9540BDP_ENABLE | val;
  return (pca9540bdp_write_config(priv, priv->state) == 0) ? OK : -ECOMM;
}

static int pca9540bdp_transfer_on_port(FAR struct i2c_master_s* dev,
                                       FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct i2c_port_dev_s* port_dev = (FAR struct i2c_port_dev_s*) dev;
  FAR struct pca9540bdp_dev_s* priv = port_dev->dev;
  int ret;

  /* select the mux port */

  if (pca9540bdp_select_port(priv, port_dev->port) != OK)
    {
      i2cerr("Could not select proper mux port\n");
      return -ECOMM;  /* Signal error condition */
    }

  /* Resume the i2c transfer to the device connected to the mux port */

  ret = I2C_TRANSFER(priv->i2c, msgs, count);
  i2cinfo("Selected port %d and resumed transfer. Result: %d\n",
          port_dev->port, ret);
  return ret;
}

#ifdef CONFIG_I2C_RESET
static int pca9540bdp_reset_on_port (FAR struct i2c_master_s *dev)
{
  FAR struct i2c_port_dev_s* port_dev = (struct i2c_port_dev_s*) dev;
  FAR struct pca9540bdp_dev_s* priv = port_dev->dev;
  int port = port_dev->port;
  int ret;

  /* select the mux port */

  if (pca9540bdp_select_port(priv, port) != OK)
    {
      i2cerr("Could not select proper mux port\n");
      return -ECOMM;  /* signal error condition */
    }

  /* resume the i2c reset for the device connected to the mux port */

  ret = I2C_RESET(priv->i2c);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9540bdp_lower_half
 *
 * Description:
 *   Initialize the lower half of the PCA9540BDP by creating a i2c_master_s
 *   for the virtual i2c master and link it to the associated PCA9540BDP and
 *   its port.
 *
 * Input Parameters:
 *   dev  - Pointer to the associated PCA9540BDP
 *   port - The port number as defined in pca9540bdp.h
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *
  pca9540bdp_lower_half(FAR struct pca9540bdp_dev_s *dev, uint8_t port)
{
  FAR struct i2c_port_dev_s *port_dev;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Initialize the i2c_port_dev_s device structure */

  port_dev = (FAR struct i2c_port_dev_s *)
    kmm_malloc(sizeof(struct i2c_port_dev_s));

  if (port_dev == NULL)
    {
      i2cerr("ERROR: Failed to allocate i2c port lower half instance\n");
      return NULL;
    }

  /* set vi2c ops to their implementations */

  port_dev->vi2c.ops = &g_i2cmux_ops;

  /* save state variables */

  port_dev->dev  = dev;
  port_dev->port = port;

  return &port_dev->vi2c;
}

/****************************************************************************
 * Name: pca9540bdp_initialize
 *
 * Description:
 *   Initialize the PCA9540BDP device.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface to use to communicate with
 *          PCA9540BDP
 *   addr - The I2C address of the PCA9540BDP.  The base I2C address of the
 *          PCA9540BDP is 0x70.
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct pca9540bdp_dev_s *
  pca9540bdp_initialize(FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct pca9540bdp_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the PCA9549BDP device structure */

  priv = (FAR struct pca9540bdp_dev_s *)
    kmm_malloc(sizeof(struct pca9540bdp_dev_s));

  if (priv == NULL)
    {
      i2cerr("ERROR: Failed to allocate instance\n");
      return NULL;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->state      = 0x00;

  if (pca9540bdp_read_config(priv, &priv->state) != OK)
    {
      i2cerr("Could not read initial state from the device\n");
      kmm_free(priv);
      return NULL;  /* signal error condition */
    }

  i2cinfo("Initial device state: %d\n", priv->state);

  if (pca9540bdp_select_port(priv, PCA9540BDP_SEL_PORT0) != OK)
    {
      i2cerr("Could not select mux port 0\n");
      kmm_free(priv);
      return NULL;  /* signal error condition */
    }

  i2cinfo("PCA9549BDP (addr=0x%02x) set up with port %d\n", priv->addr, PCA9540BDP_SEL_PORT0);

  return priv;
}

#endif /* CONFIG_I2CMULTIPLEXER_PCA9540BDP */
