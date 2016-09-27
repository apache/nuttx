/****************************************************************************
 * arch/sim/src/up_ioexpander.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "up_internal.h"

#ifdef CONFIG_SIM_IOEXPANDER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_POLLDELAY   (CONFIG_SIM_INT_POLLDELAY / USEC_PER_TICK)

#define SIM_INT_ENABLED(d,p) \
  (((d)->intenab  & ((ioe_pinset_t)1 << (p))) != 0)
#define SIM_INT_DISABLED(d,p) \
  (((d)->intenab  & ((ioe_pinset_t)1 << (p))) == 0)

#define SIM_LEVEL_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) == 0)
#define SIM_LEVEL_HIGH(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define SIM_LEVEL_LOW(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)

#define SIM_EDGE_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) != 0)
#define SIM_EDGE_RISING(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define SIM_EDGE_FALLING(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)
#define SIM_EDGE_BOTH(d,p) \
  (SIM_LEVEL_RISING(d,p) && SIM_LEVEL_FALLING(d,p))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents on registered pin interrupt callback */

struct sim_callback_s
{
   ioe_pinset_t pinset;              /* Set of pin interrupts that will generate
                                      * the callback. */
   ioe_callback_t cbfunc;            /* The saved callback function pointer */
   FAR void *cbarg;                  /* Callback argument */
};

/* This structure represents the state of the I/O Expander driver */

struct sim_dev_s
{
  struct ioexpander_dev_s dev;       /* Nested structure to allow casting as public gpio
                                      * expander. */
  ioe_pinset_t inpins;               /* Pins select as inputs */
  ioe_pinset_t invert;               /* Pin value inversion */
  ioe_pinset_t outval;               /* Value of output pins */
  ioe_pinset_t inval;                /* Simulated input register */
  ioe_pinset_t intenab;              /* Interrupt enable */
  ioe_pinset_t last;                 /* Last pin inputs (for detection of changes) */
  ioe_pinset_t trigger;              /* Bit encoded: 0=level 1=edge */
  ioe_pinset_t level[2];             /* Bit encoded: 01=high/rising, 10 low/falling, 11 both */

  WDOG_ID wdog;                      /* Timer used to poll for interrupt simulation */
  struct work_s work;                /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct sim_callback_s cb[CONFIG_SIM_INT_NCALLBACKS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I/O Expander Methods */

static int sim_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int sim_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *regval);
static int sim_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int sim_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int sim_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int sim_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
#endif
static FAR void *sim_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int sim_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle);

static ioe_pinset_t sim_int_update(FAR struct sim_dev_s *priv);
static void sim_interrupt_work(void *arg);
static void sim_interrupt(int argc, wdparm_t arg1, ...);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since only single device is supported, the driver state structure may as
 * well be pre-allocated.
 */

static struct sim_dev_s g_ioexpander;

/* I/O expander vtable */

static const struct ioexpander_ops_s g_sim_ops =
{
  sim_direction,
  sim_option,
  sim_writepin,
  sim_readpin,
  sim_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , sim_multiwritepin
  , sim_multireadpin
  , sim_multireadpin
#endif
  , sim_attach
  , sim_detach
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_direction
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

static int sim_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int direction)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS &&
              (direction == IOEXPANDER_DIRECTION_IN ||
               direction == IOEXPANDER_DIRECTION_OUT));

  gpioinfo("pin=%u direction=%s\n",
           pin, (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  /* Set the pin direction */

  if (direction == IOEXPANDER_DIRECTION_IN)
    {
      /* Configure pin as input. */

      priv->inpins |= (1 << pin);
    }
  else /* if (direction == IOEXPANDER_DIRECTION_OUT) */
    {
      /* Configure pin as output.  If a bit in this register is cleared to
       * 0, the corresponding port pin is enabled as an output.
       *
       * REVISIT: The value of output has not been selected!  This might
       * put a glitch on the output.
       */

      priv->inpins &= ~(1 << pin);
    }

  return OK;
}

/****************************************************************************
 * Name: sim_option
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

static int sim_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        int opt, FAR void *value)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;
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
      if ((uintptr_t)value == IOEXPANDER_OPTION_INVERT)
        {
          priv->invert |= (1 << pin);
        }
      else
        {
          priv->invert &= ~(1 << pin);
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
 * Name: sim_writepin
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

static int sim_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  gpioinfo("pin=%u value=%u\n", pin, (unsigned int)value);

  /* Set output pins default value (before configuring it as output) The
   * Output Port Register shows the outgoing logic levels of the pins
   * defined as outputs by the Configuration Register.
   */

  if (value && (priv->invert & (1 << pin)) == 0)
    {
      priv->outval |= (1 << pin);
    }
  else
    {
      priv->outval &= ~(1 << pin);
    }

  return OK;
}

/****************************************************************************
 * Name: sim_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value written
 *   to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sim_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;
  ioe_pinset_t inval;
  bool retval;

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS && value != NULL);

  gpioinfo("pin=%u\n", pin);

  /* Is this an output pin? */

  if ((priv->inpins & (1 << pin)) != 0)
    {
      inval = priv->inval;
    }
  else
    {
      inval = priv->outval;
    }

  /* Return 0 or 1 to indicate the state of pin */

  retval = (((inval >> pin) & 1) != 0);
  *value = ((priv->invert & (1 << pin)) != 0) ? !retval : retval;
  return OK;
}

/****************************************************************************
 * Name: sim_multiwritepin
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
static int sim_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR uint8_t *pins, FAR bool *values,
                                 int count)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;
  uint8_t pin;
  int i;

  gpioinfo("count=%d\n", count);
  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      DEBUGASSERT(pin < CONFIG_IOEXPANDER_NPINS);

      if (values[i] && (priv->invert & (1 << pin)) == 0)
        {
          priv->outval |= (1 << pin);
        }
      else
        {
          priv->outval &= ~(1 << pin);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sim_multireadpin
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
static int sim_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;
  ioe_pinset_t inval;
  uint8_t pin;
  bool pinval;
  int i;

  gpioinfo("count=%d\n", count);
  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Update the input status with the 8 bits read from the expander */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      DEBUGASSERT(pin < CONFIG_IOEXPANDER_NPINS);

      /* Is this an output pin? */

      if ((priv->inpins & (1 << pin)) != 0)
        {
          inval = priv->inval;
        }
      else
        {
          inval = priv->outval;
        }

      pinval    = ((inval & (1 << pin)) != 0);
      values[i] = ((priv->invert & (1 << pin)) != 0) ? !pinval : pinval;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sim_attach
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

static FAR void *sim_attach(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, ioe_callback_t callback,
                              FAR void *arg)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;
  FAR void *handle = NULL;
  int i;

  gpioinfo("pinset=%lx callback=%p arg=%p\n",
           (unsigned long)pinset, callback, arg);

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_SIM_INT_NCALLBACKS; i++)
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

/****************************************************************************
 * Name: sim_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by sim_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sim_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)dev;
  FAR struct sim_callback_s *cb = (FAR struct sim_callback_s *)handle;

  gpioinfo("handle=%p\n", handle);

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <= (uintptr_t)&priv->cb[CONFIG_SIM_INT_NCALLBACKS-1]);
  UNUSED(priv);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}

/****************************************************************************
 * Name: sim_int_update
 *
 * Description:
 *   Check for pending interrupts.
 *
 ****************************************************************************/

static ioe_pinset_t sim_int_update(FAR struct sim_dev_s *priv)
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
  for (i = 0; i < CONFIG_SIM_INT_NCALLBACKS; i++)
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

      pinval = ((input & 1) != 0);
      if ((priv->invert & (1 << pin)) != 0)
        {
          pinval = !pinval;
        }

      if (SIM_INT_DISABLED(priv, pin))
        {
          /* Interrupts disabled on this pin.  Do nothing.. just skip to the
           * next pin.
           */
        }
      else if (SIM_EDGE_SENSITIVE(priv, pin))
        {
          /* Edge triggered. Was there a change in the level? */

          if ((diff & 1) != 0)
            {
              /* Set interrupt as a function of edge type */

              if ((!pinval && SIM_EDGE_FALLING(priv, pin)) ||
                  ( pinval && SIM_EDGE_RISING(priv, pin)))
                {
                  intstat |= 1 << pin;
                }
            }
        }
      else /* if (SIM_LEVEL_SENSITIVE(priv, pin)) */
        {
          /* Level triggered. Set intstat if match in level type. */

          if ((pinval  && SIM_LEVEL_HIGH(priv, pin)) ||
              (!pinval && SIM_LEVEL_LOW(priv, pin)))
            {
              intstat |= 1 << pin;
            }
        }

      diff  >>= 1;
      input >>= 1;
    }

  return intstat;
}

/****************************************************************************
 * Name: sim_interrupt_work
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void sim_interrupt_work(void *arg)
{
  FAR struct sim_dev_s *priv = (FAR struct sim_dev_s *)arg;
  ioe_pinset_t intstat;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL);

  /* Update the input status with the 32 bits read from the expander */

  intstat = sim_int_update(priv);
  if (intstat != 0)
    {
      gpioinfo("intstat=%lx\n", (unsigned long)intstat);

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_SIM_INT_NCALLBACKS; i++)
        {
          /* Is this entry valid (i.e., callback attached)?  */

          if (priv->cb[i].cbfunc != NULL)
            {
              /* Did any of the requested pin interrupts occur? */

              ioe_pinset_t match = intstat & priv->cb[i].pinset;
              if (match != 0)
                {
                  /* Yes.. perform the callback */

                  (void)priv->cb[i].cbfunc(&priv->dev, match,
                                           priv->cb[i].cbarg);
                }
            }
        }
    }

  /* Re-start the poll timer */

  ret = wd_start(priv->wdog, SIM_POLLDELAY, (wdentry_t)sim_interrupt,
                 1, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }
}

/****************************************************************************
 * Name: sim_interrupt
 *
 * Description:
 *   The poll timer has expired; check for missed interrupts
 *
 * Input Parameters:
 *   Standard wdog expiration arguments.
 *
 ****************************************************************************/

static void sim_interrupt(int argc, wdparm_t arg1, ...)
{
  FAR struct sim_dev_s *priv;

  DEBUGASSERT(argc == 1);
  priv = (FAR struct sim_dev_s *)arg1;
  DEBUGASSERT(priv != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in sim_interrupt_work() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      /* Schedule interrupt related work on the high priority worker thread. */

      work_queue(HPWORK, &priv->work, sim_interrupt_work,
                 (FAR void *)priv, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_ioexpander_initialize
 *
 * Description:
 *   Instantiate and configure the I/O Expander device driver to use the provided
 *   I2C device instance.
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

FAR struct ioexpander_dev_s *sim_ioexpander_initialize(void)
{
  FAR struct sim_dev_s *priv = &g_ioexpander;
  int ret;

  /* Initialize the device state structure */

  priv->dev.ops = &g_sim_ops;

  /* Initial interrupt state:  Edge triggered on both edges */

  priv->trigger  = PINSET_ALL;  /* All edge triggered */
  priv->level[0] = PINSET_ALL;  /* All rising edge */
  priv->level[1] = PINSET_ALL;  /* All falling edge */

  /* Set up a timer to poll for simulated interrupts */

  priv->wdog = wd_create();
  DEBUGASSERT(priv->wdog != NULL);

  ret = wd_start(priv->wdog, SIM_POLLDELAY, (wdentry_t)sim_interrupt,
                 1, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }

  return &priv->dev;
}

#endif /* CONFIG_SIM_IOEXPANDER */
