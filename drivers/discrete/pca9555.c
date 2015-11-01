/****************************************************************************
 * drivers/discrete/pca9555.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * References:
 *   "16-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/discrete/ioexpander.h>

#include "pca9555.h"

#if defined(CONFIG_IOEXPANDER_PCA9555)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_I2C
#  warning I2C support is required (CONFIG_I2C)
#endif

#ifndef CONFIG_I2C_WRITEREAD
#  warning Support of the I2C writeread() method is required (CONFIG_I2C_WRITEREAD)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pca9555_direction   (FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, int dir);
static int pca9555_option      (FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, int opt, void *val);
static int pca9555_write       (FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, bool value);
static int pca9555_readpin     (FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, FAR bool *value);
static int pca9555_readbuf     (FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pca9555_multiwrite  (FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count);
static int pca9555_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count);
static int pca9555_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_PCA9555_MULTIPLE
/* If only a single PCA9555 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

static struct pca9555_dev_s g_pca9555;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct pca9555_dev_s *g_pca9555list;
#endif

static const struct ioexpander_ops_s g_pca9555_ops =
{
  pca9555_direction,
  pca9555_option,
  pca9555_write,
  pca9555_readpin,
  pca9555_readbuf,
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  pca9555_multiwrite,
  pca9555_multireadpin,
  pca9555_multireadbuf,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9555_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int pca9555_setbit(FAR struct i2c_dev_s *i2c, uint8_t addr,
                          uint8_t pin, int bitval)
{
  int ret;
  uint8_t buf[2];
  buf[0] = addr;

  if (pin > 15)
    {
      return -ENXIO;
    }
  else if (pin > 7)
    {
      addr += 1;
      pin  -= 8;
    }

  ret = I2C_WRITEREAD(i2c, &addr, 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }

  if (bitval)
    {
      buf[1] |= (1 << pin);
    }
  else
    {
      buf[1] &= ~(1 << pin);
    }

  return I2C_WRITE(i2c, buf, 2);
}

/****************************************************************************
 * Name: pca9555_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int pca9555_getbit(FAR struct i2c_dev_s *i2c, uint8_t addr,
                          uint8_t pin, FAR bool *val)
{
  uint8_t buf;
  int ret;

  if (pin > 15)
    {
      return -ENXIO;
    }
  else if (pin > 7)
    {
      addr += 1;
      pin  -= 8;
    }

  ret = I2C_WRITEREAD(i2c, &addr, 1, &buf, 1);
  if (ret < 0)
    {
      return ret;
    }

  *val = (buf >> pin) & 1;
  return OK;
}

/****************************************************************************
 * Name: pca9555_direction
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  return pca9555_setbit(pca->i2c, PCA9555_REG_CONFIG, pin,
                        (direction == IOEXPANDER_DIRECTION_IN));
}

/****************************************************************************
 * Name: pca9555_option
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *val)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ival = (int)val;

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      return pca9555_setbit(pca->i2c, PCA9555_REG_POLINV, pin, ival);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: pca9555_write
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_write(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  return pca9555_setbit(pca->i2c, PCA9555_REG_OUTPUT, pin, value);
}

/****************************************************************************
 * Name: pca9555_readpin
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  return pca9555_getbit(pca->i2c, PCA9555_REG_INPUT, pin, value);
}

/****************************************************************************
 * Name: pca9555_readbuf
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  return pca9555_getbit(pca->i2c, PCA9555_REG_OUTPUT, pin, value);
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: pca9555_getmultibits
 *
 * Description:
 *  Read multiple bits from PCA9555 registers.
 *
 ****************************************************************************/

static int pca9555_getmultibits(FAR struct i2c_dev_s *i2c, uint8_t addr,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  uint8_t buf[2];
  int ret = OK;
  int i;
  int index;
  int pin;

  ret = I2C_WRITEREAD(i2c, &addr, 1, buf, 2);

  if (ret < 0)
    {
      return ret;
    }

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      index = 0;
      pin   = pins[i];
      if (pin > 15)
        {
          return -ENXIO;
        }
      else if(pin > 7)
        {
          index = 1;
          pin  -= 8;
        }

      values[i] = (buf[index] >> pin) & 1;
    }

  return OK;
}

/****************************************************************************
 * Name: pca9555_multiwrite
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_multiwrite(FAR struct ioexpander_dev_s *dev,
                              FAR uint8_t *pins, FAR bool *values,
                              int count)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  uint8_t addr = PCA9555_REG_OUTPUT;
  uint8_t buf[3];
  int ret;
  int i;
  int index;
  int pin;

  /* Start by reading both registers, whatever the pins to change. We could
   * attempt to read one port only if all pins were on the same port, but
   * this would not save much. */

  ret = I2C_WRITEREAD(pca->i2c, &addr, 1, &buf[1], 2);
  if (ret < 0)
    {
      return ret;
    }

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      index = 1;
      pin = pins[i];
      if (pin > 15)
        {
          return -ENXIO;
        }
      else if(pin > 7)
        {
          index = 2;
          pin  -= 8;
        }

      if (values[i])
        {
          buf[index] |= (1 << pin);
        }
      else
        {
          buf[index] &= ~(1 << pin);
        }
    }

  /* Now write back the new pins states */

  buf[0] = addr;
  return I2C_WRITE(pca->i2c, buf, 3);
}

/****************************************************************************
 * Name: pca9555_multireadpin
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  return pca9555_getmultibits(pca->i2c, PCA9555_REG_INPUT,
                              pins, values, count);
}

/****************************************************************************
 * Name: pca9555_multireadbuf
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  return pca9555_getmultibits(pca->i2c, PCA9555_REG_OUTPUT,
                              pins, values, count);
}

#endif

#ifndef CONFIG_PCA9555_INT_DISABLE

/****************************************************************************
 * Name: pca9555_gpioworker
 *
 * Description:
 *  See include/nuttx/discrete/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_attach(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          ioexpander_handler_t handler)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  return 0;
}

/****************************************************************************
 * Name: pca9555_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void pca9555_irqworker(FAR struct pca9555_dev_s *priv)
{
  uint8_t regval;
  uint8_t pinmask;
  int pin;

  /* Get the set of pending GPIO interrupts */

  /* Look at each pin */

  for (pin = 0; pin < PCA9555_GPIO_NPINS; pin++)
    {
          /* Check if we have a handler for this interrupt (there should
           * be one)
           */

              /* Interrupt is pending... dispatch the interrupt to the
               * callback
               */

          /* Clear the pending GPIO interrupt by writing a '1' to the
           * pin position in the status register.
           */

    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9555_initialize
 *
 * Description:
 *   Initialize a PCA9555 I2C device.
 *
 * TODO: Actually support more than one device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *pca9555_initialize(FAR struct i2c_dev_s *i2cdev,
                                                FAR struct pca9555_config_s *config)
{
  FAR struct pca9555_dev_s *pcadev;

  DEBUGASSERT(i2cdev != NULL && config != NULL);

#ifdef CONFIG_PCA9555_MULTIPLE
  /* Allocate the device state structure */

  pcadev = (FAR struct pca9555_dev_s *)kmm_zalloc(sizeof(struct pca9555_dev_s));
  if (!pcadev)
    {
      return NULL;
    }

  /* And save the device structure in the list of PCA9555 so that we can
   * find it later.
   */

  pcadev->flink = g_pca9555list;
  g_pca9555list = pcadev;

#else
  /* Use the one-and-only PCA9555 driver instance */

  pcadev = &g_pca9555;
#endif

  /* Initialize the device state structure */

  pcadev->i2c     = i2cdev;
  pcadev->dev.ops = &g_pca9555_ops;

  /* Set the I2C address and frequency.  REVISIT:  This logic would be
   * insufficient if we share the I2C bus with any other devices that also
   * modify the address and frequency.
   */

  I2C_SETADDRESS(i2cdev, config->address, 7);
  I2C_SETFREQUENCY(i2cdev, config->frequency);

  return &pcadev->dev;
}

#endif /* CONFIG_IOEXPANDER_PCA9555 */

