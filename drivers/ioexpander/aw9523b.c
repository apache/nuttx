/****************************************************************************
 * drivers/ioexpander/aw9523b.c
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

/* The AW9523B is a 16-channel I/O expander with an I2C interface.
 * It provides GPIO and LED control functionality, including adjustable LED
 * dimming.
 *
 * It also has some quirks - pins 0-7 default to open-drain rather than
 * push-pull and the default direction and state of pins is determined by
 * the hardware-set I2C device address.
 *
 * For more details check the comment at aw9523b_initialize().
 *
 * References:
 *   "16 Multi-function LED Driver and GPIO Controller with I2C Interface",
 *   May 2016, v1.1.1, Shanghai Awinic Technology Co., Ltd.
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

#include "aw9523b.h"

#if defined(CONFIG_IOEXPANDER_AW9523B)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_I2C
#  warning I2C support is required (CONFIG_I2C)
#endif

/* Pins and ports on the AW9523B. */
#define AW9523B_PINS_PER_PORT_BITS 3
#define AW9523B_PINS_PER_PORT      (1 << AW9523B_PINS_PER_PORT_BITS)
#define AW9523B_PORT_PIN_MASK      (AW9523B_PINS_PER_PORT - 1)
#define AW9523B_NUM_PORTS          2

/****************************************************************************
 * Macros to handle pinsets
 ****************************************************************************/

#define GET_BIT(pinset, pin) \
          (((pinset) & (1 << (pin))) != 0)
#define SET_BIT(pinset, pin, value) \
          ((value) ? ((pinset) |= (1 << (pin))) : ((pinset) &= ~(1 << (pin))))

#define AW9523B_GET_INVERT_PIN(aw, pin) \
          (GET_BIT((aw)->invert_pin, pin))
#define AW9523B_SET_INVERT_PIN(aw, pin, v) \
          (SET_BIT((aw)->invert_pin, pin, v))

#ifdef CONFIG_AW9523B_LED_ENABLE
#define AW9523B_GET_IS_LED(aw, b) \
          (GET_BIT((aw)->is_led_bitset, b))
#define AW9523B_SET_IS_LED(aw, b, v) \
          (SET_BIT((aw)->is_led_bitset, b, v))
#define AW9523B_GET_OUTPUT_IS_ON(aw, b) \
          (GET_BIT((aw)->output_is_on_bitset, b))
#define AW9523B_SET_OUTPUT_IS_ON(aw, b, v) \
          (SET_BIT((aw)->output_is_on_bitset, b, v))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int aw9523b_write(FAR struct aw9523b_dev_s *aw,
             FAR const uint8_t *wbuffer, int wbuflen);
static inline int aw9523b_writeread(FAR struct aw9523b_dev_s *aw,
             FAR const uint8_t *wbuffer, int wbuflen, FAR uint8_t *rbuffer,
             int rbuflen);
static int aw9523b_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int aw9523b_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *val);
static int aw9523b_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int aw9523b_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
static int aw9523b_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int aw9523b_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR const uint8_t *pins, FAR const bool *values, int count);
static int aw9523b_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR const uint8_t *pins, FAR bool *values, int count);
static int aw9523b_multireadbuf(FAR struct ioexpander_dev_s *dev,
             FAR const uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_AW9523B_INT_ENABLE
static FAR void *aw9523b_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int aw9523b_detach(FAR struct ioexpander_dev_s *dev,
             FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_AW9523B_MULTIPLE
/* If only a single AW9523B device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

static struct aw9523b_dev_s g_aw9523b;

/* Otherwise, we will need to maintain allocated driver instances in a
 * list.
 */

#else
static struct aw9523b_dev_s *g_aw9523blist;
#endif

/* I/O expander vtable. */

static const struct ioexpander_ops_s g_aw9523b_ops =
{
  aw9523b_direction,
  aw9523b_option,
  aw9523b_writepin,
  aw9523b_readpin,
  aw9523b_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , aw9523b_multiwritepin
  , aw9523b_multireadpin
  , aw9523b_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#ifdef CONFIG_AW9523B_INT_ENABLE
  , aw9523b_attach
  , aw9523b_detach
#else
  , NULL
  , NULL
#endif
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aw9523b_lock
 *
 * Description:
 *   Get exclusive access to the AW9523B
 *
 ****************************************************************************/

static int aw9523b_lock(FAR struct aw9523b_dev_s *aw)
{
  return nxsem_wait_uninterruptible(&aw->exclsem);
}

#define aw9523b_unlock(p) nxsem_post(&(p)->exclsem)

/****************************************************************************
 * Name: aw9523b_port_from_pin
 *
 * Description:
 *   Get the port offset and port pin index for a given pin.
 *
 ****************************************************************************/

static inline int aw9523b_port_from_pin(uint8_t pin,
                                        FAR int *port,
                                        FAR int *port_pin)
{
  if (pin >= AW9523B_GPIO_NPINS)
    {
      return -EINVAL;
    }

  /* Get the register address and port pin index for the given pin. */

  *port = (pin >> AW9523B_PINS_PER_PORT_BITS);
  *port_pin = (pin & AW9523B_PORT_PIN_MASK);

  return OK;
}

/****************************************************************************
 * Name: aw9523b_dimming_reg_from_pin
 *
 * Description:
 *   Get the dimming register address for a given pin.
 *
 ****************************************************************************/

static inline int aw9523b_dimming_reg_from_pin(uint8_t pin)
{
  if (pin >= AW9523B_GPIO_NPINS)
    {
      return -EINVAL;
    }

  /* Get the register address and port pin index for the given pin. */

  if (pin < 8)
    {
      return AW9523B_REG_DIM_P0_0 + pin;
    }
  else if (pin < 12)
    {
      return AW9523B_REG_DIM_P1_0 + (pin - 8);
    }
  else
    {
      return AW9523B_REG_DIM_P1_4 + (pin - 12);
    }
}

/****************************************************************************
 * Name: aw9523b_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int aw9523b_write(FAR struct aw9523b_dev_s *aw,
                                FAR const uint8_t *wbuffer, int wbuflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer. */

  msg.frequency = aw->config->frequency;
  msg.addr      = AW9523B_I2C_ADDR_BASE + aw->config->sub_address;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)wbuffer;  /* Override const. */
  msg.length    = wbuflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(aw->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: aw9523b_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static inline int aw9523b_writeread(FAR struct aw9523b_dev_s *aw,
                                    FAR const uint8_t *wbuffer, int wbuflen,
                                    FAR uint8_t *rbuffer, int rbuflen)
{
  struct i2c_config_s config;

  /* Set up the configuration and perform the write-read operation */

  config.frequency = aw->config->frequency;
  config.address   = AW9523B_I2C_ADDR_BASE + aw->config->sub_address;
  config.addrlen   = 7;

  return i2c_writeread(aw->i2c, &config, wbuffer, wbuflen,
                       rbuffer, rbuflen);
}

/****************************************************************************
 * Name: aw9523b_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int aw9523b_setbit(FAR struct aw9523b_dev_s *aw, uint8_t addr,
                          uint8_t pin, bool bitval)
{
  uint8_t buf[2];
  int ret;
  int port;
  int port_pin;
  uint8_t reg16addr;

  ret = aw9523b_port_from_pin(pin, &port, &port_pin);
  if (ret < 0)
    {
      return ret;
    }

  reg16addr = addr + port;
  buf[0] = reg16addr;

#ifdef CONFIG_AW9523B_SHADOW_MODE
  /* Get the shadowed register value. */

  buf[1] = aw->sreg[reg16addr];

#else
  /* Get the register value from the IO-Expander. */

  ret = aw9523b_writeread(aw, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }
#endif

  SET_BIT(buf[1], port_pin, bitval);

  ret = aw9523b_write(aw, buf, 2);

#ifdef CONFIG_AW9523B_RETRY
  if (ret != OK)
    {
      /* Try again (only once). */

      ret = aw9523b_write(aw, buf, 2);
    }
#endif

#ifdef CONFIG_AW9523B_SHADOW_MODE
  if (ret == OK && addr < AW9523B_NUM_SHADOW_REGS)
    {
      /* Save the new register value in the shadow register. */

      aw->sreg[reg16addr] = buf[1];
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: aw9523b_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int aw9523b_getbit(FAR struct aw9523b_dev_s *aw, uint8_t addr,
                          uint8_t pin, FAR bool *val)
{
  uint8_t buf[2];
  int ret;
  int port;
  int port_pin;

  ret = aw9523b_port_from_pin(pin, &port, &port_pin);
  if (ret < 0)
    {
      return ret;
    }

  buf[0] = addr + port;

  ret = aw9523b_writeread(aw, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_AW9523B_SHADOW_MODE
  if (addr < AW9523B_NUM_SHADOW_REGS)
    {
      /* Save the new register value in the shadow register. */

      aw->sreg[addr] = buf[1];
    }
#endif

  *val = (buf[1] >> port_pin) & 0x1;
  return OK;
}

/****************************************************************************
 * Name: aw9523b_setbitsmasked
 *
 * Description:
 *  Write masked bits in a single register
 *  Doesn't use shadow mode.
 *
 ****************************************************************************/

static int aw9523b_setbitsmasked(FAR struct aw9523b_dev_s *aw, uint8_t addr,
                                  uint8_t mask, uint8_t value)
{
  uint8_t buf[2];
  int ret;

  buf[0] = addr;

  /* Get the register value from the IO-Expander. */

  ret = aw9523b_writeread(aw, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Apply the mask and value. */

  buf[1] = (buf[1] & ~mask) | (value & mask);

  ret = aw9523b_write(aw, buf, 2);

#ifdef CONFIG_AW9523B_RETRY
  if (ret != OK)
    {
      /* Try again (only once). */

      ret = aw9523b_write(aw, buf, 2);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: aw9523b_write_reg
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int aw9523b_write_reg(FAR struct aw9523b_dev_s *aw,
                                    int reg, uint8_t value)
{
  uint8_t buf[2] = {
    reg,
    value
  };

  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer. */

  msg.frequency = aw->config->frequency;
  msg.addr      = AW9523B_I2C_ADDR_BASE + aw->config->sub_address;
  msg.flags     = 0;
  msg.buffer    = &buf[0];  /* Override const. */
  msg.length    = sizeof(buf);

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(aw->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: aw9523b_direction
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

static int aw9523b_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  int ret;
#ifdef CONFIG_AW9523B_LED_ENABLE
  bool is_led = false;
#endif

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT &&
      direction != IOEXPANDER_DIRECTION_OUT_LED)
    {
      return -EINVAL;
    }

#ifndef CONFIG_AW9523B_LED_ENABLE
  if (direction == IOEXPANDER_DIRECTION_OUT_LED)
    {
      return -EINVAL;
    }
#endif

  /* Get exclusive access to the AW9523B. */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_AW9523B_LED_ENABLE
  /* Set the LED mode first. */

  if (direction == IOEXPANDER_DIRECTION_OUT_LED)
    {
      is_led = true;
    }

  AW9523B_SET_IS_LED(aw, pin, is_led);

  ret = aw9523b_setbit(aw, AW9523B_REG_LEDMODE0, pin, !is_led);
  if (ret < 0)
    {
      goto errout;
    }

  if (is_led)
    {
      /* Set the LED current. */

      uint8_t dim_reg = aw9523b_dimming_reg_from_pin(pin);
      uint8_t led_current = AW9523B_GET_OUTPUT_IS_ON(aw, pin) ?
                            aw->led_current[pin] : 0;
      ret = aw9523b_write_reg(aw, dim_reg, led_current);
      if (ret < 0)
        {
          goto errout;
        }
    }
#endif

  /* Set the pin's GPIO direction. */

  ret = aw9523b_setbit(aw, AW9523B_REG_CONFIG0, pin,
                      (direction == IOEXPANDER_DIRECTION_IN));
  if (ret < 0)
    {
      goto errout;
    }

errout:
  aw9523b_unlock(aw);
  return ret;
}

/****************************************************************************
 * Name: aw9523b_option
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

static int aw9523b_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  int ret = -EINVAL;

  if (pin >= AW9523B_GPIO_NPINS)
    {
      return -EINVAL;
    }

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      /* Set the pin as inverted or not. */

      if (value == NULL)
        {
          return -EINVAL;
        }

      /* Get exclusive access to the AW9523B. */

      ret = aw9523b_lock(aw);
      if (ret < 0)
        {
          return ret;
        }

      /* Update the invert pin set. */

      AW9523B_SET_INVERT_PIN(aw, pin,
        (uintptr_t)value == IOEXPANDER_VAL_INVERT);

      aw9523b_unlock(aw);
      ret = OK;
    }

  else if (opt == IOEXPANDER_OPTION_NONGENERIC)
    {
      struct aw9523b_nongeneric_option_s *optval =
        (struct aw9523b_nongeneric_option_s *)value;

      /* Get exclusive access to the AW9523B. */

      ret = aw9523b_lock(aw);
      if (ret < 0)
        {
          return ret;
        }

      switch (optval->command)
        {
          case AW9523B_OPTION_SOFTWARE_RESET:

            /* Reset the device by writing zero to the reset register. */

            ret = aw9523b_write_reg(aw, AW9523B_REG_RESET, 0);
            break;

          case AW9523B_OPTION_PORT0_PUSH_PULL:
            /* Set port 0 as pull-pull or open-drain.
             * This is a global setting, not per-pin.
             * Sets bit 4 of the AW9523B_REG_GCR register.
             */

            ret = aw9523b_setbit(aw, AW9523B_REG_GCR, 4,
                                 optval->value.port0_push_pull !=
                                   AW9523B_OPTION_P0_PP_OPEN_DRAIN);
            break;

          case AW9523B_OPTION_READ_ID:
            {
              uint8_t reg = AW9523B_REG_ID;
              ret = aw9523b_writeread(aw, &reg, 1, &optval->value.id, 1);
              break;
            }

#ifdef CONFIG_AW9523B_LED_ENABLE
          case AW9523B_OPTION_DIMMING:
            /* Set the dimming range for LEDs. 0-3.
             * 0 = max. 1 = 0.75x. 2 = 0.5x. 3 = 0.25x.
             * This is a global setting, not per-pin.
             * Sets bits 0-1 of the AW9523B_REG_GCR register.
             */

            if (optval->value.dimming > AW9523B_OPTION_DIMMING_25)
              {
                ret = -EINVAL;
                break;
              }

            ret = aw9523b_setbitsmasked(aw, AW9523B_REG_GCR,
                                        AW9523B_REG_GCR_DIMMING_MASK,
                                        optval->value.dimming);
            break;

          case AW9523B_OPTION_LED_CURRENT:
            {
              uint8_t dim_reg = aw9523b_dimming_reg_from_pin(pin);
              uint8_t led_current = AW9523B_GET_OUTPUT_IS_ON(aw, pin) ?
                                    optval->value.led_current : 0;

              if (led_current > AW9523B_OPTION_LED_CURRENT_MAX)
                {
                  ret = -EINVAL;
                  break;
                }

              ret = aw9523b_write_reg(aw, dim_reg, led_current);
              break;
            }
#endif

          default:
            ret = -ENOSYS;
            break;
        }

      aw9523b_unlock(aw);
    }
#ifdef CONFIG_AW9523B_INT_ENABLE
  else if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      uintptr_t v = (uintptr_t)value;
      if (v != IOEXPANDER_VAL_BOTH &&
          v != IOEXPANDER_VAL_DISABLE)
        {
          return -EINVAL;
        }

      /* Get exclusive access to the AW9523B. */

      ret = aw9523b_lock(aw);
      if (ret < 0)
        {
          return ret;
        }

      /* Turn on the interrupt. */

      ret = aw9523b_setbit(aw, AW9523B_REG_INT0, pin,
                           v == IOEXPANDER_VAL_BOTH);
      aw9523b_unlock(aw);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: aw9523b_writepin
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

static int aw9523b_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  int ret;

  if (pin >= AW9523B_GPIO_NPINS)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the AW9523B. */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      return ret;
    }

  /* If the pin is inverted, invert the value. */

  if (AW9523B_GET_INVERT_PIN(aw, pin))
    {
      value = !value;
    }

  /* Set the pin level. */

  ret = aw9523b_setbit(aw, AW9523B_REG_OUTPUT0, pin, value);
  if (ret < 0)
    {
      goto errout;
    }

  /* If the pin is an LED, set the LED current. */
#ifdef CONFIG_AW9523B_LED_ENABLE
  if (AW9523B_GET_IS_LED(aw, pin))
    {
      /* Set the LED current. */

      uint8_t dim_reg = aw9523b_dimming_reg_from_pin(pin);
      uint8_t led_current = value ? aw->led_current[pin] : 0;

      ret = aw9523b_write_reg(aw, dim_reg, led_current);
      if (ret < 0)
        {
          goto errout;
        }

      AW9523B_SET_OUTPUT_IS_ON(aw, pin, value);
    }
#endif

errout:
  aw9523b_unlock(aw);
  return ret;
}

/****************************************************************************
 * Name: aw9523b_readpin
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

static int aw9523b_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  int ret;
  bool pin_value = false;

  /* Get exclusive access to the AW9523B. */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw9523b_getbit(aw, AW9523B_REG_INPUT0, pin, &pin_value);
  aw9523b_unlock(aw);

  /* If the pin is inverted, invert the value. */

  if (AW9523B_GET_INVERT_PIN(aw, pin))
    {
      pin_value = !pin_value;
    }

  *value = pin_value;
  return ret;
}

/****************************************************************************
 * Name: aw9523b_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *   NOTE: Does not read a buffer full of pins, reads a single buffered pin.
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

static int aw9523b_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  int ret;
  int port;
  int port_pin;
  bool pin_value;

  ret = aw9523b_port_from_pin(pin, &port, &port_pin);
  if (ret < 0)
    {
      return ret;
    }

  /* Get exclusive access to the AW9523B. */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_AW9523B_SHADOW_MODE
  /* Get the value from a shadow register if it's in use. */

  pin_value = (aw->sreg[AW9523B_REG_INPUT0 + port] >> port_pin) & 0x1;
#else
  /* Read the register value from the IO-Expander. */

  ret = aw9523b_getbit(aw, AW9523B_REG_INPUT0, pin, &pin_value);
  *value = pin_value;
#endif

  aw9523b_unlock(aw);

  /* If the pin is inverted, invert the value. */

  if (AW9523B_GET_INVERT_PIN(aw, pin))
    {
      pin_value = !pin_value;
    }

  *value = pin_value;
  return ret;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: aw9523b_getmultibits
 *
 * Description:
 *  Read multiple bits from AW9523B registers.
 *
 ****************************************************************************/

static int aw9523b_getmultibits(FAR struct aw9523b_dev_s *aw, uint8_t addr,
                                FAR const uint8_t *pins, FAR bool *values,
                                int count)
{
  uint8_t buf[2];
  int ret = OK;
  int i;
  int pin;
  int port;
  int port_pin;

  ret = aw9523b_writeread(aw, &addr, 1, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_AW9523B_SHADOW_MODE
  /* Save the new register value in the shadow register. */

  aw->sreg[addr]     = buf[0];
  aw->sreg[addr + 1] = buf[1];
#endif

  /* Read the requested bits. */

  for (i = 0; i < count; i++)
    {
      pin   = pins[i];

      ret = aw9523b_port_from_pin(pin, &port, &port_pin);
      if (ret < 0)
        {
          break;
        }

      values[i] = (buf[port] >> port_pin) & 1;

      if (AW9523B_GET_INVERT_PIN(aw, pin))
        {
          /* If the pin is inverted, invert the value. */

          values[i] = !values[i];
        }
    }

  return ret;
}

/****************************************************************************
 * Name: aw9523b_multiwritepin
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

static int aw9523b_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR const uint8_t *pins,
                                 FAR const bool *values,
                                 int count)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  uint8_t addr = AW9523B_REG_OUTPUT0;
  uint8_t buf[AW9523B_NUM_PORTS + 1];
  int ret;
  int i;
  int pin;
  int port;
  int port_pin;
  bool pin_value;

  /* Get exclusive access to the AW9523B. */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      return ret;
    }

  /* Start by reading both registers, whatever the pins to change. We could
   * attempt to read one port only if all pins were on the same port, but
   * this would not save much.
   */

#ifndef CONFIG_AW9523B_SHADOW_MODE
  ret = aw9523b_writeread(aw, &addr, 1, &buf[1], 2);
  if (ret < 0)
    {
      aw9523b_unlock(aw);
      return ret;
    }
#else
  /* In Shadow-Mode we "read" the pin status from the shadow registers. */

  buf[1] = aw->sreg[addr];
  buf[2] = aw->sreg[addr + 1];
#endif

  /* Apply the user defined changes. */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];

      ret = aw9523b_port_from_pin(pin, &port, &port_pin);
      if (ret < 0)
        {
          aw9523b_unlock(aw);
          return ret;
        }

      pin_value = values[i];
      if (AW9523B_GET_INVERT_PIN(aw, pin))
        {
          /* If the pin is inverted, invert the value. */

          pin_value = !pin_value;
        }

      SET_BIT(buf[port + 1], port_pin, pin_value);
    }

  /* Now write back the new pins states. */

  buf[0] = addr;
  ret = aw9523b_write(aw, buf, sizeof(buf));
  if (ret < 0)
    {
      goto errout;
    }

#ifdef CONFIG_AW9523B_SHADOW_MODE
  /* Save the new register values in the shadow register. */

  aw->sreg[addr]     = buf[1];
  aw->sreg[addr + 1] = buf[2];
#endif

#ifdef CONFIG_AW9523B_LED_ENABLE
  /* If the pin is an LED, set the LED current. */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];

      if (AW9523B_GET_IS_LED(aw, pin))
        {
          /* Set the LED current. */

          uint8_t dim_reg = aw9523b_dimming_reg_from_pin(pin);
          uint8_t led_current = values[i] ? aw->led_current[pin] : 0;

          ret = aw9523b_write_reg(aw, dim_reg, led_current);
          if (ret < 0)
            {
              goto errout;
            }

          AW9523B_SET_OUTPUT_IS_ON(aw, pin, values[i]);
        }
    }
#endif

errout:
  aw9523b_unlock(aw);
  return ret;
}

/****************************************************************************
 * Name: aw9523b_multireadpin
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

static int aw9523b_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR const uint8_t *pins,
                                FAR bool *values,
                                int count)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  int ret;

  /* Get exclusive access to the AW9523B. */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw9523b_getmultibits(aw, AW9523B_REG_INPUT0,
                             pins, values, count);
  aw9523b_unlock(aw);
  return ret;
}

/****************************************************************************
 * Name: aw9523b_multireadbuf
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

static int aw9523b_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR const uint8_t *pins,
                                FAR bool *values,
                                int count)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  int ret;

  /* Get exclusive access to the AW9523B */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw9523b_getmultibits(aw, AW9523B_REG_INPUT0,
                             pins, values, count);
  aw9523b_unlock(aw);
  return ret;
}

#endif

#ifdef CONFIG_AW9523B_INT_ENABLE

/****************************************************************************
 * Name: aw9523b_attach
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

static FAR void *aw9523b_attach(FAR struct ioexpander_dev_s *dev,
                                ioe_pinset_t pinset, ioe_callback_t callback,
                                FAR void *arg)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  FAR void *handle = NULL;
  int i;
  int ret;

  /* Get exclusive access to the AW9523B. */

  ret = aw9523b_lock(aw);
  if (ret < 0)
    {
      set_errno(-ret);
      return NULL;
    }

  /* Find an available in entry in the callback table. */

  for (i = 0; i < CONFIG_AW9523B_INT_NCALLBACKS; i++)
    {
      /* Is this entry available (i.e., no callback attached). */

      if (aw->cb[i].cbfunc == NULL)
        {
          /* Yes.. use this entry. */

          aw->cb[i].pinset = pinset;
          aw->cb[i].cbfunc = callback;
          aw->cb[i].cbarg  = arg;
          handle            = &aw->cb[i];
          break;
        }
    }

  aw9523b_unlock(aw);

  if (i >= CONFIG_AW9523B_INT_NCALLBACKS)
    {
      /* No available entry found. */

      set_errno(ENOSPC);
      return NULL;
    }

  return handle;
}

/****************************************************************************
 * Name: aw9523b_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value returned by aw9523b_attach()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int aw9523b_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)dev;
  FAR struct aw9523b_callback_s *cb =
    (FAR struct aw9523b_callback_s *)handle;

  DEBUGASSERT(aw != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&aw->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&aw->cb[CONFIG_AW9523B_INT_NCALLBACKS - 1]);
  UNUSED(aw);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}

/****************************************************************************
 * Name: aw9523b_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void aw9523b_irqworker(void *arg)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)arg;
  uint8_t addr = AW9523B_REG_INPUT0;
  uint8_t buf[2];
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read inputs. */

  ret = aw9523b_writeread(aw, &addr, 1, buf, 2);
  if (ret == OK)
    {
#ifdef CONFIG_AW9523B_SHADOW_MODE
      /* Don't forget to update the shadow registers at this point. */

      aw->sreg[addr]     = buf[0];
      aw->sreg[addr + 1] = buf[1];
#endif
      /* Create a 16-bit pinset */

      pinset = ((unsigned int)buf[1] << 8) | buf[0];

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_AW9523B_INT_NCALLBACKS; i++)
        {
          /* Is this entry valid (i.e., callback attached)?  If so, did
           * any of the requested pin interrupts occur?
           */

          if (aw->cb[i].cbfunc != NULL)
            {
              /* Did any of the requested pin interrupts occur? */

              ioe_pinset_t match = pinset & aw->cb[i].pinset;
              if (match != 0)
                {
                  /* Yes.. perform the callback. */

                  aw->cb[i].cbfunc(&aw->dev, match,
                                    aw->cb[i].cbarg);
                }
            }
        }
    }

  /* Re-enable interrupts. */

  aw->config->enable(aw->config, TRUE);
}

/****************************************************************************
 * Name: aw9523b_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

static int aw9523b_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct aw9523b_dev_s *aw = (FAR struct aw9523b_dev_s *)arg;

  /* In complex environments, we cannot do I2C transfers from the interrupt
   * handler because semaphores are probably used to lock the I2C bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in aw9523b_irqworker() when the work is
   * completed.
   */

  if (work_available(&aw->work))
    {
      aw->config->enable(aw->config, FALSE);
      work_queue(HPWORK, &aw->work, aw9523b_irqworker,
                 (FAR void *)aw, 0);
    }

  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aw9523b_initialize
 *
 * Description:
 *   Initialize a AW9523B I2C device.
 *
 *   Note that the AW9523B has hardware gotchas that need to be understood
 *   when you design around it.
 *
 *    1) Port 0 (pins 0-7) defaults to open drain on reset, so if you want
 *       to use those pins as push-pull outputs, you need to set the option
 *       IOEXPANDER_OPTION_NONGENERIC with AW9523B_OPTION_PORT0_PUSH_PULL=1
 *
 *    2) Port 1 (pins 8-15) is push-pull only, if configured as outputs.
 *
 *    3) The default direction and state of pins on reset is dependent on the
 *       hardware sub-address, according to the following table:
 *
 *       +-----------------------------------------------------------------+
 *       | AD1 | AD0 | subaddr | I2C addr | P0-3  | P4-7  | P8-11 | P12-15 |
 *       +-----------------------------------------------------------------+
 *       |  0  |  0  |    0    |   0x58   | out 0 | out 0 | out 0 | out 0  |
 *       |  0  |  1  |    1    |   0x59   | in    | out 0 | out 1 | out 0  |
 *       |  1  |  0  |    2    |   0x5a   | out 0 | in    | out 0 | out 1  |
 *       |  1  |  1  |    3    |   0x5b   | in    | in    | out 1 | out 1  |
 *       +-----------------------------------------------------------------+
 *
 *       This annoying default can't be turned off, and your hardware has
 *       to be designed around it.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *aw9523b_initialize(
                              FAR struct i2c_master_s *i2cdev,
                              FAR struct aw9523b_config_s *config)
{
  FAR struct aw9523b_dev_s *aw;
  int i;

  DEBUGASSERT(i2cdev != NULL && config != NULL);

#ifdef CONFIG_AW9523B_MULTIPLE
  /* Allocate the device state structure. */

  aw = (FAR struct aw9523b_dev_s *)
    kmm_zalloc(sizeof(struct aw9523b_dev_s));
  if (!aw)
    {
      return NULL;
    }

  /* And save the device structure in the list of AW9523B so that we can
   * find it later.
   */

  aw->flink = g_aw9523blist;
  g_aw9523blist = aw;

#else
  /* Use the one-and-only AW9523B driver instance. */

  aw = &g_aw9523b;
#endif

  /* Initialize the device state structure. */

  aw->i2c     = i2cdev;
  aw->dev.ops = &g_aw9523b_ops;
  aw->config  = config;

#ifdef CONFIG_AW9523B_INT_ENABLE
  aw->config->attach(aw->config, aw9523b_interrupt, aw);
  aw->config->enable(aw->config, TRUE);
#endif

#ifdef CONFIG_AW9523B_SHADOW_MODE
  /* Initialize the shadow registers. */

  for (i = 0; i < AW9523B_NUM_SHADOW_REGS; i++)
    {
      aw->sreg[i] = 0;
    }

  /* All pins are GPIOs by default, not LEDs. */

  aw->sreg[AW9523B_REG_LEDMODE0] = AW9523B_LEDMODE_ALL_GPIO;
  aw->sreg[AW9523B_REG_LEDMODE1] = AW9523B_LEDMODE_ALL_GPIO;
#endif

#ifdef CONFIG_AW9523B_LED_ENABLE
  /* LED output configuration. */

    {
      static uint16_t default_outputs[4] =
        {
          AW9523B_DEFAULT_OUT_0,
          AW9523B_DEFAULT_OUT_1,
          AW9523B_DEFAULT_OUT_2,
          AW9523B_DEFAULT_OUT_3
        };

      if (aw->config->sub_address > 3)
        {
          aw->config->sub_address = 0;
        }

      /* Initialize the LED dimming values. */

      for (i = 0; i < AW9523B_GPIO_NPINS; i++)
        {
          aw->led_current[i] = AW9523B_LED_DEFAULT_DIMMING;
        }

      aw->is_led_bitset = 0;
      aw->output_is_on_bitset = default_outputs[aw->config->sub_address];
    }
#endif

  nxsem_init(&aw->exclsem, 0, 1);
  return &aw->dev;
}

#endif /* CONFIG_IOEXPANDER_AW9523B */
