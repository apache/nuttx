/****************************************************************************
 * drivers/input/stmpe811_gpio.c
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
 *   "STMPE811 S-Touch advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/input/stmpe811.h>

#include "stmpe811.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811) && !defined(CONFIG_STMPE811_GPIO_DISABLE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_gpioinit
 *
 * Description:
 *  Initialize the GPIO interrupt subsystem
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static void stmpe811_gpioinit(FAR struct stmpe811_dev_s *priv)
{
  uint8_t regval;

  if ((priv->flags & STMPE811_FLAGS_GPIO_INITIALIZED) == 0)
    {
      /* Enable Clocking for GPIO */

      regval = stmpe811_getreg8(priv, STMPE811_SYS_CTRL2);
      regval &= ~SYS_CTRL2_GPIO_OFF;
      stmpe811_putreg8(priv, STMPE811_SYS_CTRL2, regval);

      /* Disable all GPIO interrupts */

      stmpe811_putreg8(priv, STMPE811_GPIO_EN, 0);

      /* Enable global GPIO interrupts */

#ifndef CONFIG_STMPE811_GPIOINT_DISABLE
      regval = stmpe811_getreg8(priv, STMPE811_INT_EN);
      regval |= INT_GPIO;
      stmpe811_putreg8(priv, STMPE811_INT_EN, regval);
#endif

      priv->flags |= STMPE811_FLAGS_GPIO_INITIALIZED;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_gpioconfig
 *
 * Description:
 *  Configure an STMPE811 GPIO pin
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_gpioconfig(STMPE811_HANDLE handle, uint8_t pinconfig)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  uint8_t pinmask = (1 << pin);
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the pin is not already in use */

  if ((priv->inuse & pinmask) != 0)
    {
      ierr("ERROR: PIN%d is already in-use\n", pin);
      nxmutex_unlock(&priv->lock);
      return -EBUSY;
    }

  /* Make sure that the GPIO block has been initialized */

  stmpe811_gpioinit(priv);

  /* Set the alternate function bit for the pin, making it a GPIO */

  regval  = stmpe811_getreg8(priv, STMPE811_GPIO_AF);
  regval |= pinmask;
  stmpe811_putreg8(priv, STMPE811_GPIO_AF, regval);

  /* Is the pin an input or an output? */

  if ((pinconfig & STMPE811_GPIO_DIR) == STMPE811_GPIO_OUTPUT)
    {
      /* The pin is an output */

      regval  = stmpe811_getreg8(priv, STMPE811_GPIO_DIR_REG);
      regval |= pinmask;
      stmpe811_putreg8(priv, STMPE811_GPIO_DIR_REG, regval);

      /* Set its initial output value */

      if ((pinconfig & STMPE811_GPIO_VALUE) != STMPE811_GPIO_ZERO)
        {
          /* Set the output value(s)e by writing to the SET register */

          stmpe811_putreg8(priv, STMPE811_GPIO_SETPIN, (1 << pin));
        }
      else
        {
          /* Clear the output value(s) by writing to the CLR register */

          stmpe811_putreg8(priv, STMPE811_GPIO_CLRPIN, (1 << pin));
        }
    }
  else
    {
      /* It is an input */

      regval  = stmpe811_getreg8(priv, STMPE811_GPIO_DIR_REG);
      regval &= ~pinmask;
      stmpe811_putreg8(priv, STMPE811_GPIO_DIR_REG, regval);

      /* Set up the falling edge detection */

      regval = stmpe811_getreg8(priv, STMPE811_GPIO_FE);
      if ((pinconfig & STMPE811_GPIO_FALLING) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= ~pinmask;
        }

      stmpe811_putreg8(priv, STMPE811_GPIO_FE, regval);

      /* Set up the rising edge detection */

     regval = stmpe811_getreg8(priv, STMPE811_GPIO_RE);
      if ((pinconfig & STMPE811_GPIO_RISING) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= ~pinmask;
        }

      stmpe811_putreg8(priv, STMPE811_GPIO_RE, regval);

      /* Disable interrupts for now */

      regval = stmpe811_getreg8(priv, STMPE811_GPIO_EN);
      regval &= ~pinmask;
      stmpe811_putreg8(priv, STMPE811_GPIO_EN, regval);
    }

  /* Mark the pin as 'in use' */

  priv->inuse |= pinmask;
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: stmpe811_gpiowrite
 *
 * Description:
 *  Set or clear the GPIO output
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   value     = true: write logic '1'; false: write logic '0;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stmpe811_gpiowrite(STMPE811_HANDLE handle, uint8_t pinconfig,
                        bool value)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return;
    }

  /* Are we setting or clearing outputs? */

  if (value)
    {
      /* Set the output value(s)e by writing to the SET register */

      stmpe811_putreg8(priv, STMPE811_GPIO_SETPIN, (1 << pin));
    }
  else
    {
      /* Clear the output value(s) by writing to the CLR register */

      stmpe811_putreg8(priv, STMPE811_GPIO_CLRPIN, (1 << pin));
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: stmpe811_gpioread
 *
 * Description:
 *  Set or clear the GPIO output
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   value     - The location to return the state of the GPIO pin
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_gpioread(STMPE811_HANDLE handle, uint8_t pinconfig, bool *value)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  regval  = stmpe811_getreg8(priv, STMPE811_GPIO_MPSTA);
  *value = ((regval & STMPE811_GPIO_PIN(pin)) != 0);
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: stmpe811_gpioattach
 *
 * Description:
 *  Attach to a GPIO interrupt input pin and enable interrupts on the pin.
 *  Using the value NULL for the handler address will disable interrupts
 *  from the pin anddetach the handler.
 *
 *  NOTE:  Callbacks do not occur from an interrupt handler but rather
 *  from the context of the worker thread.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   handler   - The handler that will be called when the interrupt occurs.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifndef CONFIG_STMPE811_GPIOINT_DISABLE
int stmpe811_gpioattach(STMPE811_HANDLE handle, uint8_t pinconfig,
                       stmpe811_handler_t handler)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the GPIO interrupt system has been gpioinitialized */

  stmpe811_gpioinit(priv);

  /* Set/clear the handler */

  priv->handlers[pin] = handler;

  /* If an handler has provided, then we are enabling interrupts */

  regval = stmpe811_getreg8(priv, STMPE811_GPIO_EN);
  if (handler)
    {
      /* Enable interrupts for this GPIO */

      regval |= STMPE811_GPIO_PIN(pin);
    }
  else
    {
      /* Disable interrupts for this GPIO */

      regval &= ~STMPE811_GPIO_PIN(pin);
    }

  stmpe811_putreg8(priv, STMPE811_GPIO_EN, regval);

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: stmpe811_gpioworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

#ifndef CONFIG_STMPE811_GPIOINT_DISABLE
void stmpe811_gpioworker(FAR struct stmpe811_dev_s *priv)
{
  uint8_t regval;
  uint8_t pinmask;
  int pin;

  /* Get the set of pending GPIO interrupts */

  regval = stmpe811_getreg8(priv, STMPE811_GPIO_INTSTA);

  /* Look at each pin */

  for (pin = 0; pin < STMPE811_GPIO_NPINS; pin++)
    {
      pinmask = GPIO_INT(pin);
      if ((regval & pinmask) != 0)
        {
          /* Check if we have a handler for this interrupt (there should
           * be one)
           */

          if (priv->handlers[pin])
            {
              /* Interrupt is pending... dispatch the interrupt to the
               * callback
               */

              priv->handlers[pin](pin);
            }
          else
            {
              ierr("ERROR: No handler for PIN%d, GPIO_INTSTA: %02x\n",
                   pin, regval);
            }

          /* Clear the pending GPIO interrupt by writing a '1' to the
           * pin position in the status register.
           */

          stmpe811_putreg8(priv, STMPE811_GPIO_INTSTA, pinmask);

          /* Must also clear the edge detection status bit
           * this is _not documented_ as being required but is used in
           * the SDK and without it a second interrupt will never trigger.
           * Yep you have to "clear" _both_ the edge detection status and
           * GPIO interrupt status register even in level mode.
           */

          stmpe811_putreg8(priv, STMPE811_GPIO_ED, pinmask);
        }
    }
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 && !CONFIG_STMPE811_GPIO_DISABLE */
