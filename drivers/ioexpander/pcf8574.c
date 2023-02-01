/****************************************************************************
 * drivers/ioexpander/pcf8574.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pcf8574.h>

#include "pcf8574.h"

#ifdef CONFIG_IOEXPANDER_PCF8574

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PCF8574xx Helpers */

static int pcf8574_lock(FAR struct pcf8574_dev_s *priv);
static int pcf8574_read(FAR struct pcf8574_dev_s *priv,
                        FAR uint8_t *portval);
static int pcf8574_write(struct pcf8574_dev_s *priv, uint8_t portval);

/* I/O Expander Methods */

static int pcf8574_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int pcf8574_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *regval);
static int pcf8574_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int pcf8574_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pcf8574_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int pcf8574_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *pcf8574_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int pcf8574_detach(FAR struct ioexpander_dev_s *dev,
                          FAR void *handle);
#endif

#ifdef CONFIG_PCF8574_INT_ENABLE
static void pcf8574_int_update(void *handle, uint8_t input);
static void pcf8574_register_update(FAR struct pcf8574_dev_s *priv);
static void pcf8574_irqworker(void *arg);
static void pcf8574_interrupt(FAR void *arg);
#ifdef CONFIG_PCF8574_INT_POLL
static void pcf8574_poll_expiry(wdparm_t arg);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_PCF8574_MULTIPLE
/* If only a single device is supported, then the driver state structure may
 * as well be pre-allocated.
 */

static struct pcf8574_dev_s g_pcf8574;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_pcf8574_ops =
{
  pcf8574_direction,
  pcf8574_option,
  pcf8574_writepin,
  pcf8574_readpin,
  pcf8574_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , pcf8574_multiwritepin
  , pcf8574_multireadpin
  , pcf8574_multireadpin
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , pcf8574_attach
  , pcf8574_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8574_read
 *
 * Description:
 *  Read the PCF8574 8-bit value from a PCF8574xx port
 *
 *  Primitive I2C read operation for the PCA8574. The PCF8574 is
 *  'interesting' in that it doesn't really have a data direction register,
 *  but instead the outputs are current-limited when high, so by setting an
 *  IO line high, you are also making it an input.  Consequently, before
 *  using this method, you'll need to perform a pca8574_write() setting the
 *  bits you are interested in reading to 1's, then call this method.
 *
 ****************************************************************************/

static int pcf8574_read(FAR struct pcf8574_dev_s *priv, FAR uint8_t *portval)
{
  struct i2c_msg_s msg;
  int ret;

  DEBUGASSERT(priv != NULL && priv->i2c != NULL && priv->config != NULL);

  /* Setup for the transfer */

  msg.frequency = priv->config->frequency,
  msg.addr      = priv->config->address,
  msg.flags     = I2C_M_READ;
  msg.buffer    = portval;
  msg.length    = 1;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: pcf8574_write
 *
 * Description:
 *  Write an 8-bit value to a PCF8574xx port
 *
 *  Primitive I2C write operation for the PCA8574.  The I2C interface
 *  simply sets the state of the 8 IO lines in the PCA8574 port.
 *
 ****************************************************************************/

static int pcf8574_write(struct pcf8574_dev_s *priv, uint8_t portval)
{
  struct i2c_msg_s msg;
  int ret;

  DEBUGASSERT(priv != NULL && priv->i2c != NULL && priv->config != NULL);

  /* Setup for the transfer */

  msg.frequency = priv->config->frequency,
  msg.addr      = priv->config->address;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)&portval;
  msg.length    = 1;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: pcf8574_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 *   The PCF8574 is 'interesting' in that it doesn't really have a data
 *   direction register, but instead the outputs are current-limited when
 *   high, so by setting an IO line high, you are also making it an input.
 *   Consequently, before using this method, you'll need to perform a
 *   pca8574_write() setting the bits you are interested in reading to 1's,
 *   before calling pca8574_read().
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

static int pcf8574_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int direction)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL && pin < 8);

  gpioinfo("I2C addr=%02x pin=%u direction=%s\n",
           priv->config->address, pin,
           (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set a bit in inpins if the pin is an input.  Clear the bit in
   * inpins if the pin is an output.
   */

  if (direction == IOEXPANDER_DIRECTION_IN)
    {
      priv->inpins   |= (1 << pin);
      priv->outstate &= ~(1 << pin);
    }
  else
    {
      priv->inpins &= ~(1 << pin);
    }

  /* Write the OR of the set of input pins and the set of output pins.
   * In order to read input pins, we have to write a '1' to putt he
   * pin in the current limiting state.
   */

  ret = pcf8574_write(priv, priv->inpins | priv->outstate);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pcf8574_option
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

static int pcf8574_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  int ret = -ENOSYS;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("I2C addr=%02x pin=%u option=%u\n",
           priv->config->address, pin, opt);

#ifdef CONFIG_PCF8574_INT_ENABLE
  /* Interrupt configuration */

  if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      unsigned int ival = (unsigned int)((uintptr_t)value);
      ioe_pinset_t bit = ((ioe_pinset_t)1 << pin);

      ret = OK;
      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      switch (ival)
        {
          case IOEXPANDER_VAL_HIGH:    /* Interrupt on high level */
            priv->trigger  &= ~bit;
            priv->level[0] |= bit;
            priv->level[1] &= ~bit;
            break;

          case IOEXPANDER_VAL_LOW:     /* Interrupt on low level */
            priv->trigger  &= ~bit;
            priv->level[0] &= ~bit;
            priv->level[1] |= bit;
            break;

          case IOEXPANDER_VAL_RISING:  /* Interrupt on rising edge */
            priv->trigger  |= bit;
            priv->level[0] |= bit;
            priv->level[1] &= ~bit;
            break;

          case IOEXPANDER_VAL_FALLING: /* Interrupt on falling edge */
            priv->trigger  |= bit;
            priv->level[0] &= ~bit;
            priv->level[1] |= bit;
            break;

          case IOEXPANDER_VAL_BOTH:    /* Interrupt on both edges */
            priv->trigger  |= bit;
            priv->level[0] |= bit;
            priv->level[1] |= bit;
            break;

          case IOEXPANDER_VAL_DISABLE:
            break;

          default:
            ret = -EINVAL;
        }

      nxmutex_unlock(&priv->lock);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: pcf8574_writepin
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

static int pcf8574_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL && pin < 8);

  gpioinfo("I2C addr=%02x pin=%u value=%u\n",
           priv->config->address, pin, value);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Make sure that this is an output pin */

  if ((priv->inpins & (1 << pin)) != 0)
    {
      gpioerr("ERROR: pin%u is an input\n", pin);
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  /* Set/clear a bit in outstate. */

  if (value)
    {
      priv->outstate |= (1 << pin);
    }
  else
    {
      priv->outstate &= ~(1 << pin);
    }

  /* Write the OR of the set of input pins and the set of output pins.
   * In order to set the new output value.
   */

  ret = pcf8574_write(priv, priv->inpins | priv->outstate);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pcf8574_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 *   The PCF8574 is 'interesting' in that it doesn't really have a data
 *   direction register, but instead the outputs are current-limited when
 *   high, so by setting an IO line high, you are also making it an input.
 *   Consequently, before using this method, you'll need to perform a
 *   pca8574_write() setting the bits you are interested in reading to 1's,
 *   before calling pca8574_read().
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

static int pcf8574_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  uint8_t regval;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pin < 8 && value != NULL);

  gpioinfo("I2C addr=%02x, pin=%u\n", priv->config->address, pin);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Is the pin an output? */

  if ((priv->inpins & (1 << pin)) == 0)
    {
      /* We cannot read the value on pin directly.  Just Return the last
       * value that we wrote to the pin.
       */

      *value = ((priv->outstate & (1 << pin)) != 0);
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* It is an input pin. Read the input register for this pin
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ret = pcf8574_read(priv, &regval);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read port register: %d\n", ret);

      goto errout_with_lock;
    }

#ifdef CONFIG_PCF8574_INT_ENABLE
  /* Update the input status with the 8 bits read from the expander */

  pcf8574_int_update(priv, regval);
#endif

  /* Return 0 or 1 to indicate the state of pin */

  *value = (bool)((regval >> (pin & 7)) & 1);
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pcf8574_multiwritepin
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

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pcf8574_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR uint8_t *pins, FAR bool *values,
                                 int count)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  uint8_t pin;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pins != NULL && values != NULL);

  gpioinfo("I2C addr=%02x count=%d\n", priv->config->address, count);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Process each pin setting */

  for (i = 0; i < count; i++)
    {
      /* Make sure that this is an output pin */

      pin = pins[i];
      DEBUGASSERT(pin < 8);

      gpioinfo("%d. pin=%u value=%u\n", pin, values[i]);

      if ((priv->inpins & (1 << pin)) != 0)
        {
          gpioerr("ERROR: pin%u is an input\n", pin);
          continue;
        }

      /* Set/clear a bit in outstate. */

      if (values[i])
        {
          priv->outstate |= (1 << pin);
        }
      else
        {
          priv->outstate &= ~(1 << pin);
        }
    }

  /* Write the OR of the set of input pins and the set of output pins.
   * In order to set the new output value.
   */

  ret = pcf8574_write(priv, priv->inpins | priv->outstate);

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: pcf8574_multireadpin
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

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pcf8574_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  uint8_t regval;
  uint8_t pin;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pins != NULL && values != NULL);

  gpioinfo("I2C addr=%02x, count=%d\n", priv->config->address, count);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the input register for this pin
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ret = pcf8574_read(priv, &regval);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read port register: %d\n", ret);
      goto errout_with_lock;
    }

#ifdef CONFIG_PCF8574_INT_ENABLE
  /* Update the input status with the 8 bits read from the expander */

  pcf8574_int_update(priv, regval);
#endif

  /* Return the requested pin values */

  for (i = 0; i < count; i++)
    {
      /* Make sure that this is an output pin */

      pin = pins[i];
      DEBUGASSERT(pin < 8);

      /* Is the pin an output? */

      if ((priv->inpins & (1 << pin)) == 0)
        {
          /* We cannot read the value on pin directly.  Just Return the last
           * value that we wrote to the pin.
           */

          values[i] = ((priv->outstate & (1 << pin)) != 0);
        }
      else
        {
          values[i] = ((regval & (1 << pin)) != 0);
        }

      gpioinfo("%d. pin=%u value=%u\n", pin, values[i]);
    }

  ret = OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: pcf8574_attach
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

#ifdef CONFIG_PCF8574_INT_ENABLE
static FAR void *pcf8574_attach(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, ioe_callback_t callback,
                              FAR void *arg)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  FAR void *handle = NULL;
  int i;
  int ret;

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_PCF8574_INT_NCALLBACKS; i++)
    {
      /* Is this entry available (i.e., no callback attached) */

      if (priv->cb[i].cbfunc == NULL)
        {
          /* Yes.. use this entry */

          priv->cb[i].pinset = pinset;
          priv->cb[i].cbfunc = callback;
          priv->cb[i].cbarg  = arg;
          handle             = &priv->cb[i];
          break;
        }
    }

  nxmutex_unlock(&priv->lock);
  return handle;
}
#endif

/****************************************************************************
 * Name: pcf8574_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by pcf8574_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_PCF8574_INT_ENABLE
static int pcf8574_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)dev;
  FAR struct pcf8574_callback_s *cb =
    (FAR struct pcf8574_callback_s *)handle;

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&priv->cb[CONFIG_PCF8574_INT_NCALLBACKS - 1]);
  UNUSED(priv);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}
#endif

/****************************************************************************
 * Name: pcf8574_int_update
 *
 * Description:
 *   Check for pending interrupts.
 *
 ****************************************************************************/

#ifdef CONFIG_PCF8574_INT_ENABLE
static void pcf8574_int_update(void *handle, uint8_t input)
{
  struct pcf8574_dev_s *priv = handle;
  irqstate_t flags;
  uint8_t diff;
  int pin;

  flags = enter_critical_section();

  /* Check the changed bits from last read */

  diff        = priv->input ^ input;
  priv->input = input;

  /* PCF8574 doesn't support irq trigger, we have to do this in software. */

  for (pin = 0; pin < 8; pin++)
    {
      if (PCF8574_EDGE_SENSITIVE(priv, pin))
        {
          /* Edge triggered. Was there a change in the level? */

          if ((diff & 1) != 0)
            {
              /* Set interrupt as a function of edge type */

              if (((input & 1) == 0 && PCF8574_EDGE_FALLING(priv, pin)) ||
                  ((input & 1) != 0 && PCF8574_EDGE_RISING(priv, pin)))
                {
                  priv->intstat |= 1 << pin;
                }
            }
        }
      else /* if (PCF8574_LEVEL_SENSITIVE(priv, pin)) */
        {
          /* Level triggered. Set intstat if match in level type. */

          if (((input & 1) != 0 && PCF8574_LEVEL_HIGH(priv, pin)) ||
              ((input & 1) == 0 && PCF8574_LEVEL_LOW(priv, pin)))
            {
              priv->intstat |= 1 << pin;
            }
        }

      diff  >>= 1;
      input >>= 1;
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: tc64_update_registers
 *
 * Description:
 *   Read all pin states and update pending interrupts.
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

#ifdef CONFIG_PCF8574_INT_ENABLE
static void pcf8574_register_update(FAR struct pcf8574_dev_s *priv)
{
  uint8_t regval;
  int ret;

  /* Read from the PCF8574 port.
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ret = pcf8574_read(priv, &regval);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read port register: %d\n", ret);
    }
  else
    {
      /* Update the input status with the 8 bits read from the expander */

      pcf8574_int_update(priv, regval);
    }
}
#endif

/****************************************************************************
 * Name: pcf8574_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

#ifdef CONFIG_PCF8574_INT_ENABLE
static void pcf8574_irqworker(void *arg)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)arg;
  uint8_t pinset;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Check for pending interrupts */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  pcf8574_register_update(priv);

  /* Sample and clear the pending interrupts.  */

  pinset        = priv->intstat;
  priv->intstat = 0;
  nxmutex_unlock(&priv->lock);

  /* Perform pin interrupt callbacks */

  for (i = 0; i < CONFIG_PCF8574_INT_NCALLBACKS; i++)
    {
      /* Is this entry valid (i.e., callback attached)?  */

      if (priv->cb[i].cbfunc != NULL)
        {
          /* Did any of the requested pin interrupts occur? */

          ioe_pinset_t match = pinset & priv->cb[i].pinset;
          if (match != 0)
            {
              /* Yes.. perform the callback */

              priv->cb[i].cbfunc(&priv->dev, match,
                                 priv->cb[i].cbarg);
            }
        }
    }

#ifdef CONFIG_PCF8574_INT_POLL
  /* Check for pending interrupts */

  pcf8574_register_update(priv);

  /* Re-start the poll timer */

  sched_lock();
  ret = wd_start(&priv->wdog, PCF8574_POLLDELAY,
                 pcf8574_poll_expiry, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }
#endif

  /* Re-enable interrupts */

  priv->config->enable(priv->config, true);

#ifdef CONFIG_PCF8574_INT_POLL
  sched_unlock();
#endif
}
#endif

/****************************************************************************
 * Name: pcf8574_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

#ifdef CONFIG_PCF8574_INT_ENABLE
static void pcf8574_interrupt(FAR void *arg)
{
  FAR struct pcf8574_dev_s *priv = (FAR struct pcf8574_dev_s *)arg;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in pcf8574_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
#ifdef CONFIG_PCF8574_INT_POLL
      /* Cancel the poll timer */

      wd_cancel(&priv->wdog);
#endif

      /* Disable interrupts */

      priv->config->enable(priv->config, false);

      /* Schedule interrupt related work on the high priority worker
       * thread.
       */

      work_queue(HPWORK, &priv->work, pcf8574_irqworker,
                 (FAR void *)priv, 0);
    }
}
#endif

/****************************************************************************
 * Name: pcf8574_poll_expiry
 *
 * Description:
 *   The poll timer has expired; check for missed interrupts
 *
 * Input Parameters:
 *   Standard wdog expiration arguments.
 *
 ****************************************************************************/

#if defined(CONFIG_PCF8574_INT_ENABLE) && defined(CONFIG_PCF8574_INT_POLL)
static void pcf8574_poll_expiry(wdparm_t arg)
{
  FAR struct pcf8574_dev_s *priv;

  priv = (FAR struct pcf8574_dev_s *)arg;
  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in pcf8574_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      /* Disable interrupts */

      priv->config->enable(priv->config, false);

      /* Schedule interrupt related work on the high priority worker
       * thread.
       */

      work_queue(HPWORK, &priv->work, pcf8574_irqworker, priv, 0);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8574_initialize
 *
 * Description:
 *   Instantiate and configure the PCF8574xx device driver to use the
 *   provided I2C device instance.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   minor   - The device i2c address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *pcf8574_initialize(FAR struct i2c_master_s *i2c,
                              FAR struct pcf8574_config_s *config)
{
  FAR struct pcf8574_dev_s *priv;
  int ret;

#ifdef CONFIG_PCF8574_MULTIPLE
  /* Allocate the device state structure */

  priv = (FAR struct pcf8574_dev_s *)
    kmm_zalloc(sizeof(struct pcf8574_dev_s));
  if (!priv)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }
#else
  /* Use the one-and-only I/O Expander driver instance */

  priv = &g_pcf8574;
#endif

  /* Initialize the device state structure */

  priv->dev.ops  = &g_pcf8574_ops;
  priv->i2c      = i2c;
  priv->config   = config;

#ifdef CONFIG_PCF8574_INT_ENABLE
  /* Initial interrupt state:  Edge triggered on both edges */

  priv->trigger  = 0xff;  /* All edge triggered */
  priv->level[0] = 0xff;  /* All rising edge */
  priv->level[1] = 0xff;  /* All falling edge */

#ifdef CONFIG_PCF8574_INT_POLL
  /* Set up a timer to poll for missed interrupts */

  ret = wd_start(&priv->wdog, PCF8574_POLLDELAY,
                 pcf8574_poll_expiry, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }
#endif

  /* Attach the I/O expander interrupt handler and enable interrupts */

  priv->config->attach(config, pcf8574_interrupt, priv);
  priv->config->enable(config, true);
#endif

  nxmutex_init(&priv->lock);
  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_PCF8574 */
