/****************************************************************************
 * drivers/ioexpander/tca64xx.c
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
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/tca64xx.h>

#include "tca64xx.h"

#ifdef CONFIG_IOEXPANDER_TCA64XX

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TCA64xx Helpers */

static FAR const struct tca64_part_s *tca64_getpart(
                          FAR struct tca64_dev_s *priv);
static uint8_t tca64_ngpios(FAR struct tca64_dev_s *priv);
static uint8_t tca64_input_reg(FAR struct tca64_dev_s *priv, uint8_t pin);
static uint8_t tca64_output_reg(FAR struct tca64_dev_s *priv, uint8_t pin);
static uint8_t tca64_polarity_reg(FAR struct tca64_dev_s *priv, uint8_t pin);
static uint8_t tca64_config_reg(FAR struct tca64_dev_s *priv, uint8_t pin);
static int tca64_getreg(FAR struct tca64_dev_s *priv, uint8_t regaddr,
             FAR uint8_t *regval, unsigned int count);
static int tca64_putreg(struct tca64_dev_s *priv, uint8_t regaddr,
             FAR uint8_t *regval, unsigned int count);

/* I/O Expander Methods */

static int tca64_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int tca64_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *regval);
static int tca64_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int tca64_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int tca64_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int tca64_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *tca64_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int tca64_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle);
#endif

#ifdef CONFIG_TCA64XX_INT_ENABLE
static void tca64_int_update(FAR struct tca64_dev_s *priv,
             ioe_pinset_t input, ioe_pinset_t mask);
static void tca64_register_update(FAR struct tca64_dev_s *priv);
static void tca64_irqworker(void *arg);
static void tca64_interrupt(FAR void *arg);
#ifdef CONFIG_TCA64XX_INT_POLL
static void tca64_poll_expiry(wdparm_t arg);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_TCA64XX_MULTIPLE
/* If only a single device is supported, then the driver state structure may
 * as well be pre-allocated.
 */

static struct tca64_dev_s g_tca64;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_tca64_ops =
{
  tca64_direction,
  tca64_option,
  tca64_writepin,
  tca64_readpin,
  tca64_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , tca64_multiwritepin
  , tca64_multireadpin
  , tca64_multireadpin
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , tca64_attach
  , tca64_detach
#endif
};

/* TCA64 part data */

static const struct tca64_part_s g_tca64_parts[TCA64_NPARTS] =
{
  {
    TCA6408_PART,
    MIN(TCA6408_NR_GPIOS, CONFIG_IOEXPANDER_NPINS),
    TCA6408_INPUT_REG,
    TCA6408_OUTPUT_REG,
    TCA6408_POLARITY_REG,
    TCA6408_CONFIG_REG,
  },
  {
    TCA6416_PART,
    MIN(TCA6416_NR_GPIOS, CONFIG_IOEXPANDER_NPINS),
    TCA6416_INPUT0_REG,
    TCA6416_OUTPUT0_REG,
    TCA6416_POLARITY0_REG,
    TCA6416_CONFIG0_REG,
  },
  {
    TCA6424_PART,
    MIN(TCA6424_NR_GPIOS, CONFIG_IOEXPANDER_NPINS),
    TCA6424_INPUT0_REG,
    TCA6424_OUTPUT0_REG,
    TCA6424_POLARITY0_REG,
    TCA6424_CONFIG0_REG,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca64_getpart
 *
 * Description:
 *  Look up information for the selected part
 *
 ****************************************************************************/

static FAR const struct tca64_part_s *tca64_getpart(
                          FAR struct tca64_dev_s *priv)
{
  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              priv->config->part < TCA64_NPARTS);

  return &g_tca64_parts[priv->config->part];
}

/****************************************************************************
 * Name: tca64_ngpios
 *
 * Description:
 *  Return the number of GPIOs supported by the selected part
 *
 ****************************************************************************/

static uint8_t tca64_ngpios(FAR struct tca64_dev_s *priv)
{
  FAR const struct tca64_part_s *part = tca64_getpart(priv);
  return part->tp_ngpios;
}

/****************************************************************************
 * Name: tca64_input_reg
 *
 * Description:
 *  Return the address of the input register for the specified pin.
 *
 ****************************************************************************/

static uint8_t tca64_input_reg(FAR struct tca64_dev_s *priv, uint8_t pin)
{
  FAR const struct tca64_part_s *part = tca64_getpart(priv);
  uint8_t reg = part->tp_input;

  DEBUGASSERT(pin <= part->tp_ngpios);
  return reg + (pin >> 3);
}

/****************************************************************************
 * Name: tca64_output_reg
 *
 * Description:
 *  Return the address of the output register for the specified pin.
 *
 ****************************************************************************/

static uint8_t tca64_output_reg(FAR struct tca64_dev_s *priv, uint8_t pin)
{
  FAR const struct tca64_part_s *part = tca64_getpart(priv);
  uint8_t reg = part->tp_output;

  DEBUGASSERT(pin <= part->tp_ngpios);
  return reg + (pin >> 3);
}

/****************************************************************************
 * Name: tca64_polarity_reg
 *
 * Description:
 *  Return the address of the polarity register for the specified pin.
 *
 ****************************************************************************/

static uint8_t tca64_polarity_reg(FAR struct tca64_dev_s *priv, uint8_t pin)
{
  FAR const struct tca64_part_s *part = tca64_getpart(priv);
  uint8_t reg = part->tp_output;

  DEBUGASSERT(pin <= part->tp_ngpios);
  return reg + (pin >> 3);
}

/****************************************************************************
 * Name: tca64_config_reg
 *
 * Description:
 *  Return the address of the configuration register for the specified pin.
 *
 ****************************************************************************/

static uint8_t tca64_config_reg(FAR struct tca64_dev_s *priv, uint8_t pin)
{
  FAR const struct tca64_part_s *part = tca64_getpart(priv);
  uint8_t reg = part->tp_config;

  DEBUGASSERT(pin <= part->tp_ngpios);
  return reg + (pin >> 3);
}

/****************************************************************************
 * Name: tca64_getreg
 *
 * Description:
 *  Read an 8-bit value from a TCA64xx register
 *
 ****************************************************************************/

static int tca64_getreg(FAR struct tca64_dev_s *priv, uint8_t regaddr,
                        FAR uint8_t *regval, unsigned int count)
{
  struct i2c_msg_s msg[2];
  int ret;

  DEBUGASSERT(priv != NULL && priv->i2c != NULL && priv->config != NULL);

  /* Set up for the transfer */

  msg[0].frequency = TCA64XX_I2C_MAXFREQUENCY,
  msg[0].addr      = priv->config->address,
  msg[0].flags     = 0,
  msg[0].buffer    = &regaddr,
  msg[0].length    = 1,

  msg[1].frequency = TCA64XX_I2C_MAXFREQUENCY,
  msg[1].addr      = priv->config->address,
  msg[1].flags     = I2C_M_READ,
  msg[1].buffer    = regval,
  msg[1].length    = count,

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      gpioerr("ERROR: I2C addr=%02x regaddr=%02x: failed, ret=%d!\n",
              priv->config->address, regaddr, ret);
      return ret;
    }
  else
    {
      gpioinfo("I2C addr=%02x regaddr=%02x: read %02x\n",
               priv->config->address, regaddr, *regval);
      return OK;
    }
}

/****************************************************************************
 * Name: tca64_putreg
 *
 * Description:
 *  Write an 8-bit value to a TCA64xx register
 *
 ****************************************************************************/

static int tca64_putreg(struct tca64_dev_s *priv, uint8_t regaddr,
                        FAR uint8_t *regval, unsigned int count)
{
  struct i2c_msg_s msg[1];
  uint8_t cmd[2];
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->i2c != NULL && priv->config != NULL);

  /* Set up for the transfer */

  cmd[0] = regaddr;

  for (i = 0; i < count; i++)
    {
      cmd[i + 1] = regval[i];
    }

  msg[0].frequency = TCA64XX_I2C_MAXFREQUENCY,
  msg[0].addr      = priv->config->address,
  msg[0].flags     = 0,
  msg[0].buffer    = cmd,
  msg[0].length    = count + 1,

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      gpioerr("ERROR: claddr=%02x, regaddr=%02x: failed, ret=%d!\n",
              priv->config->address, regaddr, ret);
      return ret;
    }
  else
    {
      gpioinfo("claddr=%02x, regaddr=%02x, regval=%02x\n",
               priv->config->address, regaddr, regval);
      return OK;
    }
}

/****************************************************************************
 * Name: tca64_direction
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

static int tca64_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int direction)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
  uint8_t regaddr;
  uint8_t regval;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pin < CONFIG_IOEXPANDER_NPINS);

  gpioinfo("I2C addr=%02x pin=%u direction=%s\n",
           priv->config->address, pin,
           (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the Configuration Register associated with this pin.  The
   * Configuration Register configures the direction of the I/O pins.
   */

  regaddr = tca64_config_reg(priv, pin);
  ret = tca64_getreg(priv, regaddr, &regval, 1);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read config register at %u: %d\n",
              regaddr, ret);
      goto errout_with_lock;
    }

  /* Set the pin direction in the I/O Expander */

  if (direction == IOEXPANDER_DIRECTION_IN)
    {
      /* Configure pin as input. If a bit in the configuration register is
       * set to 1, the corresponding port pin is enabled as an input with a
       * high-impedance output driver.
       */

      regval |= (1 << (pin & 7));
    }
  else /* if (direction == IOEXPANDER_DIRECTION_OUT) */
    {
      /* Configure pin as output.  If a bit in this register is cleared to
       * 0, the corresponding port pin is enabled as an output.
       *
       * REVISIT: The value of output has not been selected!  This might
       * put a glitch on the output.
       */

      regval &= ~(1 << (pin & 7));
    }

  /* Write back the modified register content */

  ret = tca64_putreg(priv, regaddr, &regval, 1);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to write config register at %u: %d\n",
              regaddr, ret);
    }

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: tca64_option
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

static int tca64_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        int opt, FAR void *value)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
  int ret = -ENOSYS;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("I2C addr=%02x pin=%u option=%u\n",
           priv->config->address, pin, opt);

  /* Check for pin polarity inversion.  The Polarity Inversion Register
   * allows polarity inversion of pins defined as inputs by the
   * Configuration Register. If a bit in this register is set, the
   * corresponding port pin's polarity is inverted. If a bit in this
   * register is cleared, the corresponding port pin's original polarity
   * is retained.
   */

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      uint8_t regaddr;
      uint8_t polarity;

      /* Get exclusive access to the I/O Expander */

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      /* Read the polarity register */

      regaddr = tca64_polarity_reg(priv, pin);
      ret = tca64_getreg(priv, regaddr, &polarity, 1);
      if (ret < 0)
        {
          gpioerr("ERROR: Failed to read polarity register at %u: %d\n",
                  regaddr, ret);
          nxmutex_unlock(&priv->lock);
          return ret;
        }

      /* Set/clear the pin option */

      if ((uintptr_t)value == IOEXPANDER_VAL_INVERT)
        {
          polarity |= (1 << (pin & 7));
        }
      else
        {
          polarity &= ~(1 << (pin & 7));
        }

      /* Write back the modified register */

      ret = tca64_putreg(priv, regaddr, &polarity, 1);
      if (ret < 0)
        {
          gpioerr("ERROR: Failed to read polarity register at %u: %d\n",
                  regaddr, ret);
        }

      nxmutex_unlock(&priv->lock);
    }

#ifdef CONFIG_TCA64XX_INT_ENABLE
  /* Interrupt configuration */

  else if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      unsigned int ival = (unsigned int)((uintptr_t)value);
      ioe_pinset_t bit = ((ioe_pinset_t)1 << pin);

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
 * Name: tca64_writepin
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

static int tca64_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
  uint8_t regaddr;
  uint8_t regval;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pin < CONFIG_IOEXPANDER_NPINS);

  gpioinfo("I2C addr=%02x pin=%u value=%u\n",
           priv->config->address, pin, value);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the output register. */

  regaddr = tca64_output_reg(priv, pin);
  ret = tca64_getreg(priv, regaddr, &regval, 1);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read output register at %u: %d\n",
              regaddr, ret);
      goto errout_with_lock;
    }

  /* Set output pins default value (before configuring it as output) The
   * Output Port Register shows the outgoing logic levels of the pins
   * defined as outputs by the Configuration Register.
   */

  if (value != 0)
    {
      regval |= (1 << (pin & 7));
    }
  else
    {
      regval &= ~(1 << (pin & 7));
    }

  /* Write the modified output register value */

  ret = tca64_putreg(priv, regaddr, &regval, 1);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to write output register at %u: %d\n",
              regaddr, ret);
    }

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: tca64_readpin
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

static int tca64_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
  uint8_t regaddr;
  uint8_t regval;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pin < CONFIG_IOEXPANDER_NPINS && value != NULL);

  gpioinfo("I2C addr=%02x, pin=%u\n", priv->config->address, pin);

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

  regaddr = tca64_input_reg(priv, pin);
  ret = tca64_getreg(priv, regaddr, &regval, 1);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read input register at %u: %d\n",
              regaddr, ret);
      goto errout_with_lock;
    }

#ifdef CONFIG_TCA64XX_INT_ENABLE
  /* Update the input status with the 8 bits read from the expander */

  tca64_int_update(priv, (ioe_pinset_t)regval << (pin & ~7),
                   (ioe_pinset_t)0xff << (pin & ~7));
#endif

  /* Return 0 or 1 to indicate the state of pin */

  *value = (bool)((regval >> (pin & 7)) & 1);
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: tca64_multiwritepin
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
static int tca64_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR uint8_t *pins, FAR bool *values,
                                 int count)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
  ioe_pinset_t pinset;
  uint8_t regaddr;
  uint8_t ngpios;
  uint8_t nregs;
  uint8_t pin;
  int ret;
  int i;

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the output registers for pin 0 through the number of supported
   * pins.
   */

  ngpios  = tca64_ngpios(priv);
  nregs   = (ngpios + 7) >> 3;
  pinset  = 0;
  regaddr = tca64_output_reg(priv, 0);

  ret = tca64_getreg(priv, regaddr, (FAR uint8_t *)&pinset, nregs);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read %u output registers at %u: %d\n",
              nregs, regaddr, ret);
      goto errout_with_lock;
    }

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      DEBUGASSERT(pin < CONFIG_IOEXPANDER_NPINS);

      if (values[i])
        {
          pinset |= ((ioe_pinset_t)1 << pin);
        }
      else
        {
          pinset &= ~((ioe_pinset_t)1 << pin);
        }
    }

  /* Now write back the new pins states */

  ret = tca64_putreg(priv, regaddr, (FAR uint8_t *)&pinset, nregs);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to write %u output registers at %u: %d\n",
              nregs, regaddr, ret);
    }

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: tca64_multireadpin
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
static int tca64_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
  ioe_pinset_t pinset;
  uint8_t regaddr;
  uint8_t ngpios;
  uint8_t nregs;
  uint8_t pin;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->config != NULL && pins != NULL &&
              values != NULL && count > 0);

  gpioinfo("I2C addr=%02x, count=%u\n", priv->config->address, count);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the input register for pin 0 through the number of supported pins.
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ngpios  = tca64_ngpios(priv);
  nregs   = (ngpios + 7) >> 3;
  pinset  = 0;
  regaddr = tca64_input_reg(priv, 0);

  ret = tca64_getreg(priv, regaddr, (FAR uint8_t *)&pinset, nregs);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read input %u registers at %u: %d\n",
              nregs, regaddr, ret);
      goto errout_with_lock;
    }

  /* Update the input status with the 8 bits read from the expander */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      DEBUGASSERT(pin < CONFIG_IOEXPANDER_NPINS);

      values[i] = (((pinset >> pin) & 1) != 0);
    }

#ifdef CONFIG_TCA64XX_INT_ENABLE
  /* Update the input status with the 32 bits read from the expander */

  tca64_int_update(priv, pinset, PINSET_ALL);
#endif

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: tca64_attach
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

#ifdef CONFIG_TCA64XX_INT_ENABLE
static FAR void *tca64_attach(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, ioe_callback_t callback,
                              FAR void *arg)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
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

  for (i = 0; i < CONFIG_TCA64XX_INT_NCALLBACKS; i++)
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

/****************************************************************************
 * Name: tca64_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by tca64_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca64_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)dev;
  FAR struct tca64_callback_s *cb = (FAR struct tca64_callback_s *)handle;

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&priv->cb[CONFIG_TCA64XX_INT_NCALLBACKS - 1]);
  UNUSED(priv);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}
#endif

/****************************************************************************
 * Name: tca64_int_update
 *
 * Description:
 *   Check for pending interrupts.
 *
 ****************************************************************************/

#ifdef CONFIG_TCA64XX_INT_ENABLE
static void tca64_int_update(FAR struct tca64_dev_s *priv,
                             ioe_pinset_t input,
                             ioe_pinset_t mask)
{
  ioe_pinset_t diff;
  irqstate_t flags;
  int ngios = tca64_ngpios(priv);
  int pin;

  flags = enter_critical_section();

  /* Check the changed bits from last read */

  input       = (priv->input & ~mask) | (input & mask);
  diff        = priv->input ^ input;
  priv->input = input;

  /* TCA64XX doesn't support irq trigger, we have to do this in software. */

  for (pin = 0; pin < ngios; pin++)
    {
      if (TCA64_EDGE_SENSITIVE(priv, pin))
        {
          /* Edge triggered. Was there a change in the level? */

          if ((diff & 1) != 0)
            {
              /* Set interrupt as a function of edge type */

              if (((input & 1) == 0 && TCA64_EDGE_FALLING(priv, pin)) ||
                  ((input & 1) != 0 && TCA64_EDGE_RISING(priv, pin)))
                {
                  priv->intstat |= ((ioe_pinset_t)1 << pin);
                }
            }
        }
      else /* if (TCA64_LEVEL_SENSITIVE(priv, pin)) */
        {
          /* Level triggered. Set intstat bit if match in level type. */

          if (((input & 1) != 0 && TCA64_LEVEL_HIGH(priv, pin)) ||
              ((input & 1) == 0 && TCA64_LEVEL_LOW(priv, pin)))
            {
              priv->intstat |= ((ioe_pinset_t)1 << pin);
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

#ifdef CONFIG_TCA64XX_INT_ENABLE
static void tca64_register_update(FAR struct tca64_dev_s *priv)
{
  ioe_pinset_t pinset;
  uint8_t regaddr;
  uint8_t ngpios;
  uint8_t nregs;
  int ret;

  /* Read the input register for pin 0 through the number of supported pins.
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ngpios  = tca64_ngpios(priv);
  nregs   = (ngpios + 7) >> 3;
  pinset  = 0;
  regaddr = tca64_input_reg(priv, 0);

  ret = tca64_getreg(priv, regaddr, (FAR uint8_t *)&pinset, nregs);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read input %u registers at %u: %d\n",
              nregs, regaddr, ret);
      return;
    }

  /* Update the input status with the 32 bits read from the expander */

  tca64_int_update(priv, pinset, PINSET_ALL);
}
#endif

/****************************************************************************
 * Name: tca64_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

#ifdef CONFIG_TCA64XX_INT_ENABLE
static void tca64_irqworker(void *arg)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)arg;
  ioe_pinset_t pinset;
  uint8_t regaddr;
  uint8_t ngpios;
  uint8_t nregs;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Get exclusive access to read inputs and assess pending interrupts. */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the input register for pin 0 through the number of supported pins.
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ngpios  = tca64_ngpios(priv);
  nregs   = (ngpios + 7) >> 3;
  pinset  = 0;
  regaddr = tca64_input_reg(priv, 0);

  ret = tca64_getreg(priv, regaddr, (FAR uint8_t *)&pinset, nregs);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read input %u registers at %u: %d\n",
              nregs, regaddr, ret);
      nxmutex_unlock(&priv->lock);
      goto errout_with_restart;
    }

  /* Update the input status with the 32 bits read from the expander */

  tca64_int_update(priv, pinset, PINSET_ALL);

  /* Sample and clear the pending interrupts.  */

  pinset        = priv->intstat;
  priv->intstat = 0;
  nxmutex_unlock(&priv->lock);

  /* Perform pin interrupt callbacks */

  for (i = 0; i < CONFIG_TCA64XX_INT_NCALLBACKS; i++)
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

errout_with_restart:
#ifdef CONFIG_TCA64XX_INT_POLL
  /* Check for pending interrupts */

  tca64_register_update(priv);

  /* Re-start the poll timer */

  sched_lock();
  ret = wd_start(&priv->wdog, TCA64XX_POLLDELAY,
                 tca64_poll_expiry, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }
#endif

  /* Re-enable interrupts */

  priv->config->enable(priv->config, true);

#ifdef CONFIG_TCA64XX_INT_POLL
  sched_unlock();
#endif
}
#endif

/****************************************************************************
 * Name: tca64_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

#ifdef CONFIG_TCA64XX_INT_ENABLE
static void tca64_interrupt(FAR void *arg)
{
  FAR struct tca64_dev_s *priv = (FAR struct tca64_dev_s *)arg;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in tca64_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
#ifdef CONFIG_TCA64XX_INT_POLL
      /* Cancel the poll timer */

      wd_cancel(&priv->wdog);
#endif

      /* Disable interrupts */

      priv->config->enable(priv->config, false);

      /* Schedule interrupt related work on the high priority worker
       * thread.
       */

      work_queue(HPWORK, &priv->work, tca64_irqworker,
                 (FAR void *)priv, 0);
    }
}
#endif

/****************************************************************************
 * Name: tca64_poll_expiry
 *
 * Description:
 *   The poll timer has expired; check for missed interrupts
 *
 * Input Parameters:
 *   Standard wdog expiration arguments.
 *
 ****************************************************************************/

#if defined(CONFIG_TCA64XX_INT_ENABLE) && defined(CONFIG_TCA64XX_INT_POLL)
static void tca64_poll_expiry(wdparm_t arg)
{
  FAR struct tca64_dev_s *priv;

  priv = (FAR struct tca64_dev_s *)arg;
  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in tca64_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      /* Disable interrupts */

      priv->config->enable(priv->config, false);

      /* Schedule interrupt related work on the high priority worker
       * thread.
       */

      work_queue(HPWORK, &priv->work, tca64_irqworker, priv, 0);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca64_initialize
 *
 * Description:
 *   Instantiate and configure the TCA64xx device driver to use the provided
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

FAR struct ioexpander_dev_s *tca64_initialize(FAR struct i2c_master_s *i2c,
                               FAR struct tca64_config_s *config)
{
  FAR struct tca64_dev_s *priv;
  int ret;

#ifdef CONFIG_TCA64XX_MULTIPLE
  /* Allocate the device state structure */

  priv = (FAR struct tca64_dev_s *)kmm_zalloc(sizeof(struct tca64_dev_s));
  if (!priv)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }
#else
  /* Use the one-and-only I/O Expander driver instance */

  priv = &g_tca64;
#endif

  /* Initialize the device state structure */

  priv->dev.ops = &g_tca64_ops;
  priv->i2c     = i2c;
  priv->config  = config;

#ifdef CONFIG_TCA64XX_INT_ENABLE
  /* Initial interrupt state:  Edge triggered on both edges */

  priv->trigger  = PINSET_ALL;  /* All edge triggered */
  priv->level[0] = PINSET_ALL;  /* All rising edge */
  priv->level[1] = PINSET_ALL;  /* All falling edge */

#ifdef CONFIG_TCA64XX_INT_POLL
  /* Set up a timer to poll for missed interrupts */

  ret = wd_start(&priv->wdog, TCA64XX_POLLDELAY,
                 tca64_poll_expiry, (wdparm_t)priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to start poll timer\n");
    }
#endif

  /* Attach the I/O expander interrupt handler and enable interrupts */

  priv->config->attach(config, tca64_interrupt, priv);
  priv->config->enable(config, true);
#endif

  nxmutex_init(&priv->lock);
  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_TCA64XX */
