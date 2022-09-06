/****************************************************************************
 * drivers/ioexpander/pca9555.c
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

/* References:
 *   "16-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int pca9555_write(FAR struct pca9555_dev_s *pca,
             FAR const uint8_t *wbuffer, int wbuflen);
static inline int pca9555_writeread(FAR struct pca9555_dev_s *pca,
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
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *pca9555_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int pca9555_detach(FAR struct ioexpander_dev_s *dev,
             FAR void *handle);
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

/* I/O expander vtable */

static const struct ioexpander_ops_s g_pca9555_ops =
{
  pca9555_direction,
  pca9555_option,
  pca9555_writepin,
  pca9555_readpin,
  pca9555_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , pca9555_multiwritepin
  , pca9555_multireadpin
  , pca9555_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , pca9555_attach
  , pca9555_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9555_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int pca9555_write(FAR struct pca9555_dev_s *pca,
                                FAR const uint8_t *wbuffer, int wbuflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = pca->config->frequency;
  msg.addr      = pca->config->address;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)wbuffer;  /* Override const */
  msg.length    = wbuflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(pca->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: pca9555_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static inline int pca9555_writeread(FAR struct pca9555_dev_s *pca,
                                    FAR const uint8_t *wbuffer, int wbuflen,
                                    FAR uint8_t *rbuffer, int rbuflen)
{
  struct i2c_config_s config;

  /* Set up the configuration and perform the write-read operation */

  config.frequency = pca->config->frequency;
  config.address   = pca->config->address;
  config.addrlen   = 7;

  return i2c_writeread(pca->i2c, &config, wbuffer, wbuflen,
                       rbuffer, rbuflen);
}

/****************************************************************************
 * Name: pca9555_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int pca9555_setbit(FAR struct pca9555_dev_s *pca, uint8_t addr,
                          uint8_t pin, bool bitval)
{
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

#ifdef CONFIG_PCA9555_SHADOW_MODE
  /* Get the shadowed register value */

  buf[1] = pca->sreg[addr];

#else
  /* Get the register value from the IO-Expander */

  ret = pca9555_writeread(pca, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }
#endif

  if (bitval)
    {
      buf[1] |= (1 << pin);
    }
  else
    {
      buf[1] &= ~(1 << pin);
    }

#ifdef CONFIG_PCA9555_SHADOW_MODE
  /* Save the new register value in the shadow register */

  pca->sreg[addr] = buf[1];
#endif

  ret = pca9555_write(pca, buf, 2);
#ifdef CONFIG_PCA9555_RETRY
  if (ret != OK)
    {
      /* Try again (only once) */

      ret = pca9555_write(pca, buf, 2);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: pca9555_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int pca9555_getbit(FAR struct pca9555_dev_s *pca, uint8_t addr,
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

  ret = pca9555_writeread(pca, &addr, 1, &buf, 1);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_PCA9555_SHADOW_MODE
  /* Save the new register value in the shadow register */

  pca->sreg[addr] = buf;
#endif

  *val = (buf >> pin) & 1;
  return OK;
}

/****************************************************************************
 * Name: pca9555_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9555_setbit(pca, PCA9555_REG_CONFIG, pin,
                       (direction == IOEXPANDER_DIRECTION_IN));
  nxmutex_unlock(&pca->lock);
  return ret;
}

/****************************************************************************
 * Name: pca9555_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ret = -EINVAL;

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      /* Get exclusive access to the PCA555 */

      ret = nxmutex_lock(&pca->lock);
      if (ret < 0)
        {
          return ret;
        }

      ret = pca9555_setbit(pca, PCA9555_REG_POLINV, pin,
                           ((uintptr_t)value == IOEXPANDER_VAL_INVERT));
      nxmutex_unlock(&pca->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: pca9555_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ret;

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9555_setbit(pca, PCA9555_REG_OUTPUT, pin, value);
  nxmutex_unlock(&pca->lock);
  return ret;
}

/****************************************************************************
 * Name: pca9555_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *      written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on
 *            this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ret;

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9555_getbit(pca, PCA9555_REG_INPUT, pin, value);
  nxmutex_unlock(&pca->lock);
  return ret;
}

/****************************************************************************
 * Name: pca9555_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the level is stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ret;

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9555_getbit(pca, PCA9555_REG_OUTPUT, pin, value);
  nxmutex_unlock(&pca->lock);
  return ret;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: pca9555_getmultibits
 *
 * Description:
 *  Read multiple bits from PCA9555 registers.
 *
 ****************************************************************************/

static int pca9555_getmultibits(FAR struct pca9555_dev_s *pca, uint8_t addr,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
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

#ifdef CONFIG_PCA9555_SHADOW_MODE
  /* Save the new register value in the shadow register */

  pca->sreg[addr]     = buf[0];
  pca->sreg[addr + 1] = buf[1];
#endif

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      index = 0;
      pin   = pins[i];
      if (pin > 15)
        {
          return -ENXIO;
        }
      else if (pin > 7)
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
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
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

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Start by reading both registers, whatever the pins to change. We could
   * attempt to read one port only if all pins were on the same port, but
   * this would not save much.
   */

#ifndef CONFIG_PCA9555_SHADOW_MODE
  ret = pca9555_writeread(pca, &addr, 1, &buf[1], 2);
  if (ret < 0)
    {
      nxmutex_unlock(&pca->lock);
      return ret;
    }
#else
  /* In Shadow-Mode we "read" the pin status from the shadow registers */

  buf[1] = pca->sreg[addr];
  buf[2] = pca->sreg[addr + 1];
#endif

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      index = 1;
      pin = pins[i];
      if (pin > 15)
        {
          nxmutex_unlock(&pca->lock);
          return -ENXIO;
        }
      else if (pin > 7)
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
#ifdef CONFIG_PCA9555_SHADOW_MODE
  /* Save the new register values in the shadow register */

  pca->sreg[addr]     = buf[1];
  pca->sreg[addr + 1] = buf[2];
#endif
  ret = pca9555_write(pca, buf, 3);

  nxmutex_unlock(&pca->lock);
  return ret;
}

/****************************************************************************
 * Name: pca9555_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   valptr - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ret;

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9555_getmultibits(pca, PCA9555_REG_INPUT,
                             pins, values, count);
  nxmutex_unlock(&pca->lock);
  return ret;
}

/****************************************************************************
 * Name: pca9555_multireadbuf
 *
 * Description:
 *   Read the buffered level of multiple pins. This routine may be faster
 *   than individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the buffered levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  int ret;

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9555_getmultibits(pca, PCA9555_REG_OUTPUT,
                             pins, values, count);
  nxmutex_unlock(&pca->lock);
  return ret;
}

#endif

#ifdef CONFIG_PCA9555_INT_ENABLE

/****************************************************************************
 * Name: pca9555_attach
 *
 * Description:
 *   Attach and enable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback.
 *   arg      - User-provided callback argument
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  This handle may be
 *   used later to detach and disable the pin interrupt.
 *
 ****************************************************************************/

static FAR void *pca9555_attach(FAR struct ioexpander_dev_s *dev,
                                ioe_pinset_t pinset, ioe_callback_t callback,
                                FAR void *arg)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  FAR void *handle = NULL;
  int i;
  int ret;

  /* Get exclusive access to the PCA555 */

  ret = nxmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_PCA9555_INT_NCALLBACKS; i++)
    {
      /* Is this entry available (i.e., no callback attached) */

      if (pca->cb[i].cbfunc == NULL)
        {
          /* Yes.. use this entry */

          pca->cb[i].pinset = pinset;
          pca->cb[i].cbfunc = callback;
          pca->cb[i].cbarg  = arg;
          handle            = &pca->cb[i];
          break;
        }
    }

  /* Add this callback to the table */

  nxmutex_unlock(&pca->lock);
  return handle;
}

/****************************************************************************
 * Name: pca9555_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by pca9555_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9555_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)dev;
  FAR struct pca9555_callback_s *cb =
    (FAR struct pca9555_callback_s *)handle;

  DEBUGASSERT(pca != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&pca->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&pca->cb[CONFIG_PCA9555_INT_NCALLBACKS - 1]);
  UNUSED(pca);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}

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
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)arg;
  uint8_t addr = PCA9555_REG_INPUT;
  uint8_t buf[2];
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read inputs */

  ret = pca9555_writeread(pca, &addr, 1, buf, 2);
  if (ret == OK)
    {
#ifdef CONFIG_PCA9555_SHADOW_MODE
      /* Don't forget to update the shadow registers at this point */

      pca->sreg[addr]     = buf[0];
      pca->sreg[addr + 1] = buf[1];
#endif
      /* Create a 16-bit pinset */

      pinset = ((unsigned int)buf[0] << 8) | buf[1];

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_PCA9555_INT_NCALLBACKS; i++)
        {
          /* Is this entry valid (i.e., callback attached)?  If so, did
           * any of the requested pin interrupts occur?
           */

          if (pca->cb[i].cbfunc != NULL)
            {
              /* Did any of the requested pin interrupts occur? */

              ioe_pinset_t match = pinset & pca->cb[i].pinset;
              if (match != 0)
                {
                  /* Yes.. perform the callback */

                  pca->cb[i].cbfunc(&pca->dev, match,
                                    pca->cb[i].cbarg);
                }
            }
        }
    }

  /* Re-enable interrupts */

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

static int pca9555_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct pca9555_dev_s *pca = (FAR struct pca9555_dev_s *)arg;

  /* In complex environments, we cannot do I2C transfers from the interrupt
   * handler because semaphores are probably used to lock the I2C bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in pca9555_irqworker() when the work is
   * completed.
   */

  if (work_available(&pca->work))
    {
      pca->config->enable(pca->config, FALSE);
      work_queue(HPWORK, &pca->work, pca9555_irqworker,
                 (FAR void *)pca, 0);
    }

  return OK;
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

FAR struct ioexpander_dev_s *pca9555_initialize(
                              FAR struct i2c_master_s *i2cdev,
                              FAR struct pca9555_config_s *config)
{
  FAR struct pca9555_dev_s *pcadev;

  DEBUGASSERT(i2cdev != NULL && config != NULL);

#ifdef CONFIG_PCA9555_MULTIPLE
  /* Allocate the device state structure */

  pcadev = (FAR struct pca9555_dev_s *)
    kmm_zalloc(sizeof(struct pca9555_dev_s));
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

#ifdef CONFIG_PCA9555_INT_ENABLE
  pcadev->config->attach(pcadev->config, pca9555_interrupt, pcadev);
  pcadev->config->enable(pcadev->config, TRUE);
#endif

  nxmutex_init(&pcadev->lock);
  return &pcadev->dev;
}

#endif /* CONFIG_IOEXPANDER_PCA9555 */
