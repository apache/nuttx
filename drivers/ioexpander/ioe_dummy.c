/****************************************************************************
 * drivers/ioexpander/ioe_dummy.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioe_dummy.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IOE_DUMMY_POLLDELAY \
  (CONFIG_IOEXPANDER_DUMMY_INT_POLLDELAY / USEC_PER_TICK)

#define IOE_DUMMY_INT_ENABLED(d,p) \
  (((d)->intenab  & ((ioe_pinset_t)1 << (p))) != 0)
#define IOE_DUMMY_INT_DISABLED(d,p) \
  (((d)->intenab  & ((ioe_pinset_t)1 << (p))) == 0)

#define IOE_DUMMY_LEVEL_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) == 0)
#define IOE_DUMMY_LEVEL_HIGH(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define IOE_DUMMY_LEVEL_LOW(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)

#define IOE_DUMMY_EDGE_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) != 0)
#define IOE_DUMMY_EDGE_RISING(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define IOE_DUMMY_EDGE_FALLING(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)
#define IOE_DUMMY_EDGE_BOTH(d,p) \
  (IOE_DUMMY_LEVEL_RISING(d,p) && IOE_DUMMY_LEVEL_FALLING(d,p))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents on registered pin interrupt callback */

struct ioe_dummy_callback_s
{
  ioe_pinset_t pinset;              /* Set of pin interrupts that will generate
                                     * the callback. */
  ioe_callback_t cbfunc;            /* The saved callback function pointer */
  FAR void *cbarg;                  /* Callback argument */
};

/* This structure represents the state of the I/O Expander driver */

struct ioe_dummy_dev_s
{
  struct ioexpander_dev_s dev;       /* Nested structure to allow casting as
                                      * public GPIO expander. */
  ioe_pinset_t inpins;               /* Pins select as inputs */
  ioe_pinset_t invert;               /* Pin value inversion */
  ioe_pinset_t outval;               /* Value of output pins */
  ioe_pinset_t inval;                /* Simulated input register */
  ioe_pinset_t intenab;              /* Interrupt enable */
  ioe_pinset_t last;                 /* Last pin inputs (for detection of
                                      * changes) */
  ioe_pinset_t trigger;              /* Bit encoded: 0=level 1=edge */
  ioe_pinset_t level[2];             /* Bit encoded: 01=high/rising,
                                      * 10 low/falling, 11 both */

  struct wdog_s wdog;                /* Timer used to poll for interrupt
                                      * simulation */
  struct work_s work;                /* Supports the interrupt handling
                                      * "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct ioe_dummy_callback_s cb[CONFIG_IOEXPANDER_DUMMY_INT_NCALLBACKS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I/O Expander Methods */

static int ioe_dummy_direction(FAR struct ioexpander_dev_s *dev,
                               uint8_t pin, int dir);
static int ioe_dummy_option(FAR struct ioexpander_dev_s *dev,
                            uint8_t pin, int opt, void *regval);
static int ioe_dummy_writepin(FAR struct ioexpander_dev_s *dev,
                              uint8_t pin, bool value);
static int ioe_dummy_readpin(FAR struct ioexpander_dev_s *dev,
                             uint8_t pin, FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int ioe_dummy_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                   FAR uint8_t *pins, FAR bool *values,
                                   int count);
static int ioe_dummy_multireadpin(FAR struct ioexpander_dev_s *dev,
                                  FAR uint8_t *pins, FAR bool *values,
                                  int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *ioe_dummy_attach(FAR struct ioexpander_dev_s *dev,
                                  ioe_pinset_t pinset,
                                  ioe_callback_t callback, FAR void *arg);
static int ioe_dummy_detach(FAR struct ioexpander_dev_s *dev,
                            FAR void *handle);
#endif

static ioe_pinset_t ioe_dummy_int_update(FAR struct ioe_dummy_dev_s *priv);
static void ioe_dummy_interrupt_work(void *arg);
static void ioe_dummy_interrupt(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since only single device is supported, the driver state structure may as
 * well be pre-allocated.
 */

static struct ioe_dummy_dev_s g_ioexpander;

/* I/O expander vtable */

static const struct ioexpander_ops_s g_ioe_dummy_ops =
{
  ioe_dummy_direction,
  ioe_dummy_option,
  ioe_dummy_writepin,
  ioe_dummy_readpin,
  ioe_dummy_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , ioe_dummy_multiwritepin
  , ioe_dummy_multireadpin
  , ioe_dummy_multireadpin
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , ioe_dummy_attach
  , ioe_dummy_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioe_dummy_direction
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

static int ioe_dummy_direction(FAR struct ioexpander_dev_s *dev,
                               uint8_t pin, int direction)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  gpioinfo("pin=%u direction=%s\n",
           pin, (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  /* Set the pin direction */

  if (direction == IOEXPANDER_DIRECTION_IN)
    {
      /* Configure pin as input. */

      priv->inpins |= ((ioe_pinset_t)1 << pin);
    }
  else /* if (direction == IOEXPANDER_DIRECTION_OUT) */
    {
      /* Configure pin as output.  If a bit in this register is cleared to
       * 0, the corresponding port pin is enabled as an output.
       *
       * REVISIT: The value of output has not been selected!  This might
       * put a glitch on the output.
       */

      priv->inpins &= ~((ioe_pinset_t)1 << pin);
    }

  return OK;
}

/****************************************************************************
 * Name: ioe_dummy_option
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

static int ioe_dummy_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            int opt, FAR void *value)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;
  int ret = -ENOSYS;

  DEBUGASSERT(priv != NULL);

  gpioinfo("pin=%u option=%u\n", pin, opt);

  /* Check for pin polarity inversion.  The Polarity Inversion Register
   * allows polarity inversion of pins defined as inputs by the
   * Configuration Register. If a bit in this register is set, the
   * corresponding port pin's polarity is inverted. If a bit in this
   * register is cleared, the corresponding port pin's original polarity
   * is retained.
   */

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      if ((uintptr_t)value == IOEXPANDER_VAL_INVERT)
        {
          priv->invert |= ((ioe_pinset_t)1 << pin);
        }
      else
        {
          priv->invert &= ~((ioe_pinset_t)1 << pin);
        }
    }

  /* Interrupt configuration */

  else if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      ioe_pinset_t bit = ((ioe_pinset_t)1 << pin);

      ret = OK;
      switch ((uintptr_t)value)
        {
          case IOEXPANDER_VAL_HIGH:    /* Interrupt on high level */
            priv->intenab  |= bit;
            priv->trigger  &= ~bit;
            priv->level[0] |= bit;
            priv->level[1] &= ~bit;
            break;

          case IOEXPANDER_VAL_LOW:     /* Interrupt on low level */
            priv->intenab  |= bit;
            priv->trigger  &= ~bit;
            priv->level[0] &= ~bit;
            priv->level[1] |= bit;
            break;

          case IOEXPANDER_VAL_RISING:  /* Interrupt on rising edge */
            priv->intenab  |= bit;
            priv->trigger  |= bit;
            priv->level[0] |= bit;
            priv->level[1] &= ~bit;
            break;

          case IOEXPANDER_VAL_FALLING: /* Interrupt on falling edge */
            priv->intenab  |= bit;
            priv->trigger  |= bit;
            priv->level[0] &= ~bit;
            priv->level[1] |= bit;
            break;

          case IOEXPANDER_VAL_BOTH:    /* Interrupt on both edges */
            priv->intenab  |= bit;
            priv->trigger  |= bit;
            priv->level[0] |= bit;
            priv->level[1] |= bit;
            break;

          case IOEXPANDER_VAL_DISABLE:
            priv->trigger  &= ~bit;
            break;

          default:
            ret = -EINVAL;
            break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ioe_dummy_writepin
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

static int ioe_dummy_writepin(FAR struct ioexpander_dev_s *dev,
                              uint8_t pin, bool value)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  gpioinfo("pin=%u value=%u\n", pin, (unsigned int)value);

  /* Set output pins default value (before configuring it as output) The
   * Output Port Register shows the outgoing logic levels of the pins
   * defined as outputs by the Configuration Register.
   */

  if (value == (((priv->invert >> pin) & 1) == 0))
    {
      priv->outval |= ((ioe_pinset_t)1 << pin);
    }
  else
    {
      priv->outval &= ~((ioe_pinset_t)1 << pin);
    }

  return OK;
}

/****************************************************************************
 * Name: ioe_dummy_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
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

static int ioe_dummy_readpin(FAR struct ioexpander_dev_s *dev,
                             uint8_t pin, FAR bool *value)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;
  ioe_pinset_t inval;

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS &&
              value != NULL);

  gpioinfo("pin=%u\n", pin);

  /* Is this an output pin? */

  if (((priv->inpins >> pin) & 1) != 0)
    {
      inval = priv->inval;
    }
  else
    {
      inval = priv->outval;
    }

  /* Return 0 or 1 to indicate the state of pin */

  *value = ((((inval ^ priv->invert) >> pin) & 1) != 0);

  return OK;
}

/****************************************************************************
 * Name: ioe_dummy_multiwritepin
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
static int ioe_dummy_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                   FAR uint8_t *pins, FAR bool *values,
                                   int count)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;
  uint8_t pin;
  int i;

  gpioinfo("count=%d\n", count);
  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      DEBUGASSERT(pin < CONFIG_IOEXPANDER_NPINS);

      if (values[i] == (((priv->invert >> pin) & 1) == 0))
        {
          priv->outval |= ((ioe_pinset_t)1 << pin);
        }
      else
        {
          priv->outval &= ~((ioe_pinset_t)1 << pin);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ioe_dummy_multireadpin
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
static int ioe_dummy_multireadpin(FAR struct ioexpander_dev_s *dev,
                                  FAR uint8_t *pins, FAR bool *values,
                                  int count)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;
  ioe_pinset_t inval;
  uint8_t pin;
  int i;

  gpioinfo("count=%d\n", count);
  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Update the input status with the 8 bits read from the expander */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      DEBUGASSERT(pin < CONFIG_IOEXPANDER_NPINS);

      /* Is this an output pin? */

      if (((priv->inpins >> pin) & 1) != 0)
        {
          inval = priv->inval;
        }
      else
        {
          inval = priv->outval;
        }

      values[i] = ((((inval ^ priv->invert) >> pin) & 1) != 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ioe_dummy_attach
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

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *ioe_dummy_attach(FAR struct ioexpander_dev_s *dev,
                                  ioe_pinset_t pinset,
                                  ioe_callback_t callback, FAR void *arg)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;
  FAR void *handle = NULL;
  int i;

  gpioinfo("pinset=%lx callback=%p arg=%p\n",
           (unsigned long)pinset, callback, arg);

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_IOEXPANDER_DUMMY_INT_NCALLBACKS; i++)
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

  return handle;
}
#endif

/****************************************************************************
 * Name: ioe_dummy_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by ioe_dummy_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int ioe_dummy_detach(FAR struct ioexpander_dev_s *dev,
                            FAR void *handle)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)dev;
  FAR struct ioe_dummy_callback_s *cb =
    (FAR struct ioe_dummy_callback_s *)handle;

  gpioinfo("handle=%p\n", handle);

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] && (uintptr_t)cb <=
           (uintptr_t)&priv->cb[CONFIG_IOEXPANDER_DUMMY_INT_NCALLBACKS - 1]);
  UNUSED(priv);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}
#endif

/****************************************************************************
 * Name: ioe_dummy_int_update
 *
 * Description:
 *   Check for pending interrupts.
 *
 ****************************************************************************/

static ioe_pinset_t ioe_dummy_int_update(FAR struct ioe_dummy_dev_s *priv)
{
  ioe_pinset_t toggles;
  ioe_pinset_t diff;
  ioe_pinset_t input;
  ioe_pinset_t intstat;
  bool pinval;
  int pin;
  int i;

  /* First, toggle all input bits that have associated, attached interrupt
   * handler.  This is a crude simulation for toggle interrupt inputs.
   */

  toggles = 0;
  for (i = 0; i < CONFIG_IOEXPANDER_DUMMY_INT_NCALLBACKS; i++)
    {
      /* Is there a callback attached?  */

      if (priv->cb[i].cbfunc != NULL)
        {
          /* Yes, add the input pins to set of pins to toggle */

          toggles |= (priv->cb[i].pinset & priv->inpins);
        }
    }

  priv->inval = (priv->inval & ~toggles) | (~priv->inval & toggles);

  /* Check the changed bits from last read (Only applies to input pins) */

  input = priv->inval;
  diff  = priv->last ^ input;
  if (diff != 0)
    {
      gpioinfo("toggles=%lx inval=%lx last=%lx diff=%lx\n",
               (unsigned long)toggles, (unsigned long)priv->inval,
               (unsigned long)priv->last, (unsigned long)diff);
    }

  priv->last = input;
  intstat    = 0;

  /* Check for changes in pins that could generate an interrupt. */

  for (pin = 0; pin < CONFIG_IOEXPANDER_NPINS; pin++)
    {
      /* Get the value of the pin (accounting for inversion) */

      pinval = ((((input ^ priv->invert) >> pin) & 1) != 0);

      if (IOE_DUMMY_INT_DISABLED(priv, pin))
        {
          /* Interrupts disabled on this pin.  Do nothing.. just skip to the
           * next pin.
           */
        }
      else if (IOE_DUMMY_EDGE_SENSITIVE(priv, pin))
        {
          /* Edge triggered. Was there a change in the level? */

          if ((diff & 1) != 0)
            {
              /* Set interrupt as a function of edge type */

              if ((!pinval && IOE_DUMMY_EDGE_FALLING(priv, pin)) ||
                  (pinval && IOE_DUMMY_EDGE_RISING(priv, pin)))
                {
                  intstat |= ((ioe_pinset_t)1 << pin);
                }
            }
        }
      else /* if (IOE_DUMMY_LEVEL_SENSITIVE(priv, pin)) */
        {
          /* Level triggered. Set intstat if match in level type. */

          if ((pinval  && IOE_DUMMY_LEVEL_HIGH(priv, pin)) ||
              (!pinval && IOE_DUMMY_LEVEL_LOW(priv, pin)))
            {
              intstat |= ((ioe_pinset_t)1 << pin);
            }
        }

      diff  >>= 1;
      input >>= 1;
    }

  return intstat;
}

/****************************************************************************
 * Name: ioe_dummy_interrupt_work
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void ioe_dummy_interrupt_work(void *arg)
{
  FAR struct ioe_dummy_dev_s *priv = (FAR struct ioe_dummy_dev_s *)arg;
  ioe_pinset_t intstat;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL);

  /* Update the input status with the 32 bits read from the expander */

  intstat = ioe_dummy_int_update(priv);
  if (intstat != 0)
    {
      gpioinfo("intstat=%lx\n", (unsigned long)intstat);

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_IOEXPANDER_DUMMY_INT_NCALLBACKS; i++)
        {
          /* Is this entry valid (i.e., callback attached)?  */

          if (priv->cb[i].cbfunc != NULL)
            {
              /* Did any of the requested pin interrupts occur? */

              ioe_pinset_t match = intstat & priv->cb[i].pinset;
              if (match != 0)
                {
                  /* Yes.. perform the callback */

                  priv->cb[i].cbfunc(&priv->dev, match, priv->cb[i].cbarg);
                }
            }
        }
    }

  /* Re-start the poll timer */

  ret = wd_start(&priv->wdog, IOE_DUMMY_POLLDELAY,
                 ioe_dummy_interrupt, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }
}

/****************************************************************************
 * Name: ioe_dummy_interrupt
 *
 * Description:
 *   The poll timer has expired; check for missed interrupts
 *
 * Input Parameters:
 *   Standard wdog expiration arguments.
 *
 ****************************************************************************/

static void ioe_dummy_interrupt(wdparm_t arg)
{
  FAR struct ioe_dummy_dev_s *priv;

  priv = (FAR struct ioe_dummy_dev_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in ioe_dummy_interrupt_work() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      /* Schedule interrupt related work on the high priority worker
       * thread.
       */

      work_queue(HPWORK, &priv->work, ioe_dummy_interrupt_work,
                 (FAR void *)priv, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioe_dummy_initialize
 *
 * Description:
 *   Instantiate and configure the I/O Expander device driver to use the
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

FAR struct ioexpander_dev_s *ioe_dummy_initialize(void)
{
  FAR struct ioe_dummy_dev_s *priv = &g_ioexpander;
  int ret;

  /* Initialize the device state structure */

  priv->dev.ops = &g_ioe_dummy_ops;

  /* Initial interrupt state:  Edge triggered on both edges */

  priv->trigger  = PINSET_ALL;  /* All edge triggered */
  priv->level[0] = PINSET_ALL;  /* All rising edge */
  priv->level[1] = PINSET_ALL;  /* All falling edge */

  ret = wd_start(&priv->wdog, IOE_DUMMY_POLLDELAY,
                 ioe_dummy_interrupt, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }

  return &priv->dev;
}
