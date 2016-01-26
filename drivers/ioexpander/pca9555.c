/****************************************************************************
 * drivers/ioexpander/pca9555.c
 *
 *   Copyright (C) 2015, 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "pca9555.h"

#if defined(CONFIG_IOEXPANDER_PCA9555)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_I2C
#  warning I2C support is required (CONFIG_I2C)
#endif

#ifndef CONFIG_I2C_TRANSFER
#  warning Support of the I2C transfer() method is required (CONFIG_I2C_TRANSFER)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pca9555_writeread(FAR struct pca9555_dev_s *pca,
             FAR const uint8_t *wbuffer, int wbuflen, FAR uint8_t *rbuffer,
             int rbuflen);
static int pca9555_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int pca9555_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *val);
static int pca9555_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int pca9555_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
static int pca9555_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pca9555_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int pca9555_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int pca9555_multireadbuf(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
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
  pca9555_writepin,
  pca9555_readpin,
  pca9555_readbuf,
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  pca9555_multiwritepin,
  pca9555_multireadpin,
  pca9555_multireadbuf,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9555_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static int pca9555_writeread(FAR struct pca9555_dev_s *pca,
                             FAR const uint8_t *wbuffer, int wbuflen,
                             FAR uint8_t *rbuffer, int rbuflen)
{
  struct i2c_msg_s msg[2];

  /* Format two messages: The first is a write */

  msg[0].addr   = pca->config->address;
  msg[0].flags  = 0;
  msg[0].buffer = (uint8_t *)wbuffer;  /* Override const */
  msg[0].length = wbuflen;

  /* The second is either a read (rbuflen > 0) or a write (rbuflen < 0) with
   * no restart.
   */

  if (rbuflen > 0)
    {
      msg[1].flags = I2C_M_READ;
    }
  else
    {
      msg[1].flags = I2C_M_NORESTART;
      rbuflen = -rbuflen;
    }

  msg[1].addr   = pca->config->address;
  msg[1].buffer = rbuffer;
  msg[1].length = rbuflen;

  /* Then perform the transfer */

  I2C_SETFREQUENCY(pca->i2c, pca->config->frequency);
  return I2C_TRANSFER(pca->i2c, msg, 2);
}

/****************************************************************************
 * Name: pca9555_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int pca9555_setbit(FAR struct ioexpander_dev_s *dev, uint8_t addr,
                          uint8_t pin, int bitval)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  uint8_t buf[2];
  int ret;

  if (pin > 15)
    {
      return -ENXIO;
    }
  else if (pin > 7)
    {
      addr++;
      pin  -= 8;
    }

  buf[0] = addr;

  ret = pca9555_writeread(pca, &buf[0], 1, &buf[1], 1);
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

  return I2C_WRITE(pca->i2c, buf, 2);
}

/****************************************************************************
 * Name: pca9555_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int pca9555_getbit(FAR struct ioexpander_dev_s *dev, uint8_t addr,
                          uint8_t pin, FAR bool *val)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
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

  ret = pca9555_writeread(pca, &addr, 1, &buf, 1);
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
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  return pca9555_setbit(dev, PCA9555_REG_CONFIG, pin,
                        (direction == IOEXPANDER_DIRECTION_IN));
}

/****************************************************************************
 * Name: pca9555_option
 *
 * Description:
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *val)
{
  int ival = (int)val;

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      return pca9555_setbit(dev, PCA9555_REG_POLINV, pin, ival);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: pca9555_writepin
 *
 * Description:
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  return pca9555_setbit(dev, PCA9555_REG_OUTPUT, pin, value);
}

/****************************************************************************
 * Name: pca9555_readpin
 *
 * Description:
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  return pca9555_getbit(dev, PCA9555_REG_INPUT, pin, value);
}

/****************************************************************************
 * Name: pca9555_readbuf
 *
 * Description:
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  return pca9555_getbit(dev, PCA9555_REG_OUTPUT, pin, value);
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: pca9555_getmultibits
 *
 * Description:
 *  Read multiple bits from PCA9555 registers.
 *
 ****************************************************************************/

static int pca9555_getmultibits(FAR struct ioexpander_dev_s *dev, uint8_t addr,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  uint8_t buf[2];
  int ret = OK;
  int i;
  int index;
  int pin;

  ret = pca9555_writeread(pca, &addr, 1, buf, 2);
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
 * Name: pca9555_multiwritepin
 *
 * Description:
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_multiwritepin(FAR struct ioexpander_dev_s *dev,
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

  ret = pca9555_writeread(pca, &addr, 1, &buf[1], 2);
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
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  return pca9555_getmultibits(dev, PCA9555_REG_INPUT,
                              pins, values, count);
}

/****************************************************************************
 * Name: pca9555_multireadbuf
 *
 * Description:
 *  See include/nuttx/ioexpander/ioexpander.h
 *
 ****************************************************************************/

static int pca9555_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  return pca9555_getmultibits(dev, PCA9555_REG_OUTPUT,
                              pins, values, count);
}

#endif

#ifdef CONFIG_PCA9555_INT_ENABLE

/****************************************************************************
 * Name: pca9555_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void pca9555_irqworker(void *arg)
{
  uint8_t addr = PCA9555_REG_INPUT;
  uint8_t buf[2];
  int ret, bits;
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)arg;

  /* Read inputs */

  ret = pca9555_writeread(pca, &addr, 1, buf, 2);
  if (ret != OK)
    {
      return;
    }

  bits = (buf[0] << 8) | buf[1];

  /* If signal PID is registered, enqueue signal. */

  if (pca->dev.sigpid)
    {
#ifdef CONFIG_CAN_PASS_STRUCTS
      union sigval value;
      value.sival_int = bits;
      ret = sigqueue(pca->dev.sigpid, pca->dev.sigval, value);
#else
      ret = sigqueue(pca->dev.sigpid, pca->dev.sigval, (FAR void *)bits);
#endif
      dbg("pca signal %04X (sig %d to pid %d)\n",
          bits, pca->dev.sigval, pca->dev.sigpid);
    }
  else
    {
      dbg("no handler registered\n");
    }

  /* Re-enable */

  pca->config->enable(pca->config, TRUE);
}

/****************************************************************************
 * Name: pca9555_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

static int pca9555_interrupt(int irq, FAR void *context)
{
#ifdef CONFIG_PCA9555_MULTIPLE
  /* To support multiple devices,
   * retrieve the priv structure using the irq number.
   */

#  warning Missing logic

#else
  register FAR struct pca9555_dev_s *pca = &g_pca9555;
#endif

  /* In complex environments, we cannot do I2C transfers from the interrupt
   * handler because semaphores are probably used to lock the I2C bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&pca->dev.work));

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in pca9555_irqworker() when the work is completed.
   */

  pca->config->enable(pca->config, FALSE);
  return work_queue(HPWORK, &pca->dev.work, pca9555_irqworker, (FAR void *)pca, 0);
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
  pcadev->config  = config;

  /* Set the I2C address and frequency.  REVISIT:  This logic would be
   * insufficient if we share the I2C bus with any other devices that also
   * modify the address and frequency.
   */

  I2C_SETADDRESS(i2cdev, config->address, 7);
  I2C_SETFREQUENCY(i2cdev, config->frequency);

#ifdef CONFIG_PCA9555_INT_ENABLE
  pcadev->config->attach(pcadev->config, pca9555_interrupt);
  pcadev->config->enable(pcadev->config, TRUE);
#endif
  return &pcadev->dev;
}

#endif /* CONFIG_IOEXPANDER_PCA9555 */
