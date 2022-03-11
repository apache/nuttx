/****************************************************************************
 * arch/arm/src/xmc4/xmc4_gpio.c
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
#include <arch/board/board.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/xmc4_ports.h"
#include "xmc4_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_gpio_getreg
 *
 * Description:
 *   Return the pin number for this pin configuration
 *
 ****************************************************************************/

static inline uint32_t xmc4_gpio_getreg(uintptr_t portbase,
                                        unsigned int offset)
{
  return getreg32(portbase + offset);
}

/****************************************************************************
 * Name: xmc4_gpio_putreg
 *
 * Description:
 *   Return the pin number for this pin configuration
 *
 ****************************************************************************/

static inline void xmc4_gpio_putreg(uintptr_t portbase, unsigned int offset,
                                    uint32_t regval)
{
  putreg32(regval, portbase + offset);
}

/****************************************************************************
 * Name: xmc4_gpio_port
 *
 * Description:
 *   Return the port number for this pin configuration
 *
 ****************************************************************************/

static inline int xmc4_gpio_port(gpioconfig_t pinconfig)
{
  return ((pinconfig & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

/****************************************************************************
 * Name: xmc4_gpio_portbase
 *
 * Description:
 *   Return the base address of the port register for this pin configuration.
 *
 ****************************************************************************/

static uintptr_t xmc4_gpio_portbase(gpioconfig_t pinconfig)
{
  return XMC4_PORT_BASE(xmc4_gpio_port(pinconfig));
}

/****************************************************************************
 * Name: xmc4_gpio_pin
 *
 * Description:
 *   Return the pin number for this pin configuration
 *
 ****************************************************************************/

static unsigned int xmc4_gpio_pin(gpioconfig_t pinconfig)
{
  return ((pinconfig & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: xmc4_gpio_pintype
 *
 * Description:
 *   Return the pintype for this pin configuration
 *
 ****************************************************************************/

static inline unsigned int xmc4_gpio_pintype(gpioconfig_t pinconfig)
{
  return ((pinconfig & GPIO_PINTYPE_MASK) >> GPIO_PINTYPE_SHIFT);
}

/****************************************************************************
 * Name: xmc4_gpio_pinctrl
 *
 * Description:
 *   Return the pintype for this pin configuration
 *
 ****************************************************************************/

static inline unsigned int xmc4_gpio_pinctrl(gpioconfig_t pinconfig)
{
  return ((pinconfig & GPIO_PINCTRL_MASK) >> GPIO_PINCTRL_SHIFT);
}

/****************************************************************************
 * Name: xmc4_gpio_padtype
 *
 * Description:
 *   Return the padtype for this pin configuration
 *
 ****************************************************************************/

static inline unsigned int xmc4_gpio_padtype(gpioconfig_t pinconfig)
{
  return ((pinconfig & GPIO_PADTYPE_MASK) >> GPIO_PADTYPE_SHIFT);
}

/****************************************************************************
 * Name: xmc4_gpio_iocr
 *
 * Description:
 *   Update the IOCR register
 *
 ****************************************************************************/

static void xmc4_gpio_iocr(uintptr_t portbase, unsigned int pin,
                           unsigned int value)
{
  uint32_t regval;
  uint32_t mask;
  unsigned int offset;
  unsigned int shift;

  /* Read the IOCR register */

  offset = XMC4_PORT_IOCR_OFFSET(pin);
  regval = xmc4_gpio_getreg(portbase, offset);

  /* Set the new value for this field */

  pin &= 3;
  shift = PORT_IOCR0_PC_SHIFT(pin);
  mask  = PORT_IOCR0_PC_MASK(pin);

  regval &= ~mask;
  regval |= (uint32_t)value << shift;

  xmc4_gpio_putreg(portbase, offset, regval);
}

/****************************************************************************
 * Name: xmc4_gpio_hwsel
 *
 * Description:
 *   Update the HWSEL register
 *
 ****************************************************************************/

static inline void xmc4_gpio_hwsel(uintptr_t portbase, unsigned int pin,
                                   unsigned int value)
{
  uint32_t regval;
  uint32_t mask;
  unsigned int shift;

  /* Read the HWSEL register */

  regval = xmc4_gpio_getreg(portbase, XMC4_PORT_HWSEL_OFFSET);

  /* Set the new value for this field */

  shift = PORT_HWSEL_HW_SHIFT(pin);
  mask  = PORT_HWSEL_HW_MASK(pin);

  regval &= ~mask;
  regval |= (uint32_t)value << shift;

  xmc4_gpio_putreg(portbase, XMC4_PORT_HWSEL_OFFSET, regval);
}

/****************************************************************************
 * Name: xmc4_gpio_pdisc
 *
 * Description:
 *   Update the PDISC register
 *
 ****************************************************************************/

static inline void xmc4_gpio_pdisc(uintptr_t portbase, unsigned int pin,
                                   bool enable)
{
  uint32_t regval;
  uint32_t mask;

  /* Read the PDISC register */

  regval = xmc4_gpio_getreg(portbase, XMC4_PORT_PDISC_OFFSET);

  /* Set or clear the pin field in the PDISC register.
   *
   * Disable = set
   * Analog  = set
   * Enable  = clear
   */

  mask = PORT_PIN(pin);
  if (enable)
    {
      regval &= ~mask;
    }
  else
    {
      regval |= mask;
    }

  xmc4_gpio_putreg(portbase, XMC4_PORT_PDISC_OFFSET, regval);
}

/****************************************************************************
 * Name: xmc4_gpio_pps
 *
 * Description:
 *   Update the PPS register
 *
 ****************************************************************************/

static inline void xmc4_gpio_pps(uintptr_t portbase, unsigned int pin,
                                 bool powersave)
{
  uint32_t regval;
  uint32_t mask;

  /* Read the PPS register */

  regval = xmc4_gpio_getreg(portbase, XMC4_PORT_PPS_OFFSET);

  /* Set/clear the enable/disable power save value for this field */

  mask = PORT_PIN(pin);
  if (powersave)
    {
      regval |= mask;
    }
  else
    {
      regval &= ~mask;
    }

  xmc4_gpio_putreg(portbase, XMC4_PORT_PPS_OFFSET, regval);
}

/****************************************************************************
 * Name: xmc4_gpio_pdr
 *
 * Description:
 *   Update the IOCR register
 *
 ****************************************************************************/

static void xmc4_gpio_pdr(uintptr_t portbase, unsigned int pin,
                          unsigned int value)
{
  uint32_t regval;
  uint32_t mask;
  unsigned int offset;
  unsigned int shift;

  /* Read the PDR register */

  offset = XMC4_PORT_PDR_OFFSET(pin);
  regval = xmc4_gpio_getreg(portbase, offset);

  /* Set the new value for this field */

  pin &= 7;
  shift = PORT_PDR0_PD_SHIFT(pin);
  mask  = PORT_PDR0_PD_MASK(pin);

  regval &= ~mask;
  regval |= (uint32_t)value << shift;

  xmc4_gpio_putreg(portbase, offset, regval);
}

/****************************************************************************
 * Name: xmc4_gpio_inverted
 *
 * Description:
 *   Check if the input is inverted
 *
 ****************************************************************************/

static inline bool xmc4_gpio_inverted(gpioconfig_t pinconfig)
{
  return ((pinconfig & GPIO_INPUT_INVERT) != 0);
}

/****************************************************************************
 * Name: xmc4_gpio_opendrain
 *
 * Description:
 *   Check if the output is opendram
 *
 ****************************************************************************/

static inline bool xmc4_gpio_opendrain(gpioconfig_t pinconfig)
{
  return ((pinconfig & GPIO_OUTPUT_OPENDRAIN) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_gpio_config
 *
 * Description:
 *   Configure a PIN based on bit-encoded description of the pin,
 *   'pincconfig'.
 *
 ****************************************************************************/

int xmc4_gpio_config(gpioconfig_t pinconfig)
{
  uintptr_t portbase = xmc4_gpio_portbase(pinconfig);
  unsigned int pin   = xmc4_gpio_pin(pinconfig);
  unsigned int value;
  irqstate_t flags;

  flags = enter_critical_section();
  if (GPIO_ISINPUT(pinconfig))
    {
      /* Get input pin type (IOCR) */

      value = xmc4_gpio_pintype(pinconfig);

      /* Check if the input is inverted */

      if (xmc4_gpio_inverted(pinconfig))
        {
          value |= IOCR_INPUT_INVERT;
        }
    }
  else
    {
      /* Force input while we configure */

      xmc4_gpio_iocr(portbase, pin, IOCR_INPUT_NOPULL);

      /* Set output value before enabling output */

      xmc4_gpio_write(pinconfig, ((pinconfig & GPIO_OUTPUT_SET) != 0));

      /* Get output pin type (IOCR) */

      value = xmc4_gpio_pintype(pinconfig);

      /* Get if the output is opendrain */

      if (xmc4_gpio_opendrain(pinconfig))
        {
          value |= IOCR_OUTPUT_OPENDRAIN;
        }
    }

  /* Update the IOCR register to instantiate the pin type */

  xmc4_gpio_iocr(portbase, pin, value);

  /* Select pin control (HWSEL) */

  value = xmc4_gpio_pinctrl(pinconfig);
  xmc4_gpio_hwsel(portbase, pin, value);

  /* Select drive strength (PDR) */

  value = xmc4_gpio_padtype(pinconfig);
  xmc4_gpio_pdr(portbase, pin, value);

  /* Enable/enable pad or Analog only (PDISC) */

  xmc4_gpio_pdisc(portbase, pin, ((pinconfig & GPIO_PAD_DISABLE) == 0));

  /* Make sure pin is not in power save mode (PPS) */

  xmc4_gpio_pps(portbase, pin, false);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: xmc4_gpio_write
 *
 * Description:
 *   Write one or zero to the PORT pin selected by 'pinconfig'
 *
 ****************************************************************************/

void xmc4_gpio_write(gpioconfig_t pinconfig, bool value)
{
  uintptr_t portbase = xmc4_gpio_portbase(pinconfig);
  unsigned int pin   = xmc4_gpio_pin(pinconfig);
  uint32_t regval;

  /* Setup OMR value for this pin:
   *
   *   PRx PSx Function
   *   0   0   Bit Pn_OUT.Px is not changed.
   *   0   1   Bit Pn_OUT.Px is set.
   *   1   0   Bit Pn_OUT.Px is reset.
   *   1   1   Bit Pn_OUT.Px is toggled.
   */

  if (value)
    {
      /* PRx==0; PSx==1 -> Set output */

      regval = OMR_PS(pin);
    }
  else
    {
      /* PRx==1; PSx==0 -> Reset output */

      regval = OMR_PR(pin);
    }

  /* Set/clear the OUTPUT.  This is an atomoc operation so no critical
   * section is needed.
   */

  xmc4_gpio_putreg(portbase, XMC4_PORT_OMR_OFFSET, regval);
}

/****************************************************************************
 * Name: xmc4_gpio_read
 *
 * Description:
 *   Read one or zero from the PORT pin selected by 'pinconfig'
 *
 ****************************************************************************/

bool xmc4_gpio_read(gpioconfig_t pinconfig)
{
  uintptr_t portbase = xmc4_gpio_portbase(pinconfig);
  unsigned int pin   = xmc4_gpio_pin(pinconfig);
  uint32_t regval;

  /* Read the OUT register.  This is an atomoc operation so no critical
   * section is needed.
   */

  regval = xmc4_gpio_getreg(portbase, XMC4_PORT_IN_OFFSET);

  /* Return in the input state for this pin at the time is was read */

  return ((regval & PORT_PIN(pin)) != 0);
}
