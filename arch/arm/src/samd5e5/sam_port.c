/****************************************************************************
 * arch/arm/src/samd5e5/sam_port.c
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

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_port.h"
#include "sam_eic.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
static const char g_portchar[SAM_NPORTS]   =
{
  'A', 'B', 'C', 'D'
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_portbase
 *
 * Description:
 *   Return the base address of the PORT register set
 *
 ****************************************************************************/

static inline uintptr_t sam_portbase(port_pinset_t pinset)
{
  int port = (pinset & PORT_MASK) >> PORT_SHIFT;
  return SAM_PORTN_BASE(port);
}

/****************************************************************************
 * Name: sam_portpin
 *
 * Description:
 *   Return the bit associated with the pin
 *
 ****************************************************************************/

static inline int sam_portpin(port_pinset_t pinset)
{
  return 1 << ((pinset & PORT_PIN_MASK) >> PORT_PIN_SHIFT);
}

/****************************************************************************
 * Name: sam_configinput
 *
 * Description:
 *   Configure a PORT input pin based on bit-encoded description of the pin.
 *   This function serves the dual role of putting all pins into a known,
 *   initial state.  Hence, it is overkill for what really needs to be done.
 *
 * Assumption:
 *   sam_configreset has been called to put the pin into the default reset
 *   state.
 *
 ****************************************************************************/

static inline void sam_configinput(uintptr_t base, port_pinset_t pinset)
{
  uint32_t regval;
  uint32_t bit;
  int pin;

  /* Decode pin information */

  pin = (pinset & PORT_PIN_MASK) >> PORT_PIN_SHIFT;
  bit = (1 << pin);

  /* Direction bit is already zero (input) */

  /* Enable the I/O synchronizer? */

  if ((pinset & PORT_SYNCHRONIZER_MASK) == PORT_SYNCHRONIZER_ON)
    {
      regval  = getreg32(base + SAM_PORT_CTRL_OFFSET);
      regval |= bit;
      putreg32(regval, base + SAM_PORT_CTRL_OFFSET);
    }

  /* Set the pin configuration */

  regval = (PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX |
            PORT_WRCONFIG_INEN);
  if (pin >= 16)
    {
       /* Select the upper half word and adjust the bit setting */

       regval |= PORT_WRCONFIG_HWSEL;
       pin    -= 16;
    }

  regval |= PORT_WRCONFIG_PINMASK(pin);

  /* Check for pull-up/down selection */

  switch (pinset & PORT_PULL_MASK)
    {
      case PORT_PULL_UP:
        {
          /* Select pull-up by setting the corresponding bit in OUT
           * register.
           */

          putreg32(bit, base + SAM_PORT_OUTSET_OFFSET);
        }

        /* Fall through */

      case PORT_PULL_DOWN:
        {
          regval |= PORT_WRCONFIG_PULLEN;
        }
        break;

      default:
      case PORT_PULL_NONE:
        break;
    }

  /* Configure the pin as an input */

  putreg32(regval, base + SAM_PORT_WRCONFIG_OFFSET);
}

/****************************************************************************
 * Name: sam_configinterrupt
 *
 * Description:
 *   Configure a PORT interrupt pin based on bit-encoded description of the
 *   pin.
 *
 * Assumption:
 *   sam_configreset has been called to put the pin into the default reset
 *   state:
 *
 ****************************************************************************/

static inline void sam_configinterrupt(uintptr_t base, port_pinset_t pinset)
{
#ifdef CONFIG_SAMD5E5_EIC
  uint32_t func;
  uint32_t regval;
  int pin;

  pin     = (pinset & PORT_PIN_MASK) >> PORT_PIN_SHIFT;

  regval  = (PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX |
             PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_INEN);
  regval |= PORT_WRCONFIG_PINMASK(pin);

  func    = (pinset & PORT_FUNC_MASK) >> PORT_FUNC_SHIFT;
  regval |= (func << PORT_WRCONFIG_PMUX_SHIFT);

  putreg32(regval, base + SAM_PORT_WRCONFIG_OFFSET);

  /* Configure the interrupt edge sensitivity in CONFIGn register of
   * the EIC
   */

  sam_eic_configure(pin, pinset);

#ifdef CONFIG_DEBUG_GPIO_INFO
  sam_dumpport(pinset, "extint");
#endif
#endif /* CONFIG_SAMD5E5_EIC */
}

/****************************************************************************
 * Name: sam_configoutput
 *
 * Description:
 *   Configure a PORT output pin based on bit-encoded description of the pin.
 *
 * Assumption:
 *   sam_configreset has been called to put the pin into the default reset
 *   state.
 *
 ****************************************************************************/

static inline void sam_configoutput(uintptr_t base, port_pinset_t pinset)
{
  uint32_t regval;
  uint32_t bit;
  int pin;

  /* Decode pin information */

  pin = (pinset & PORT_PIN_MASK) >> PORT_PIN_SHIFT;
  bit = (1 << pin);

  /* Set the direction bit to configure the pin as an input */

  putreg32(bit, base + SAM_PORT_DIRSET_OFFSET);

  /* Set the initial output value to high? */

  if ((pinset & PORT_OUTVALUE_MASK) == PORT_OUTPUT_SET)
    {
      putreg32(bit, base + SAM_PORT_OUTSET_OFFSET);
    }

  /* Set the pin configuration.  This will be an output with the input
   * buffer enabled.
   */

  regval = (PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX |
            PORT_WRCONFIG_INEN);
  if (pin > 16)
    {
       /* Select the upper half word and adjust the bit setting */

       regval |= PORT_WRCONFIG_HWSEL;
       pin    -= 16;
    }

  regval |= PORT_WRCONFIG_PINMASK(pin);

  /* Check for pull-up/down selection */

  switch (pinset & PORT_PULL_MASK)
    {
      case PORT_PULL_UP:
        {
          /* Select pull-up by setting the corresponding bit in OUT
           * register.
           */

          putreg32(bit, base + SAM_PORT_OUTSET_OFFSET);
        }

        /* Fall through */

      case PORT_PULL_DOWN:
        {
          regval |= PORT_WRCONFIG_PULLEN;
        }
        break;

      default:
      case PORT_PULL_NONE:
        break;
    }

  /* Select higher strength drive? */

  if ((pinset & PORT_DRIVE_MASK) == PORT_DRIVE_HIGH)
    {
      regval |= PORT_WRCONFIG_DRVSTR;
    }

  /* Configure the pin as an output */

  putreg32(regval, base + SAM_PORT_WRCONFIG_OFFSET);
}

/****************************************************************************
 * Name: sam_configperiph
 *
 * Description:
 *   Configure a PORT pin driven by a peripheral based on bit-encoded
 *   description of the pin.
 *
 * Assumption:
 *   sam_configreset has been called to put the pin into the default reset
 *   state.
 *
 ****************************************************************************/

static inline void sam_configperiph(uintptr_t base, port_pinset_t pinset)
{
  uint32_t regval;
  uint32_t bit;
  uint32_t func;
  int pin;

  /* Decode pin information */

  pin = (pinset & PORT_PIN_MASK) >> PORT_PIN_SHIFT;
  bit = (1 << pin);

  /* If pin is output with readback then set the pin as output */

  if ((pinset & PORT_OUTREADBACK_MASK) == PORT_OUTREADBACK_ENABLE)
    {
      putreg32(bit, base + SAM_PORT_DIRSET_OFFSET);
    }

  /* Set the pin configuration.  This will be an peripheral with the
   * selected function.
   */

  regval = (PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX |
            PORT_WRCONFIG_PMUXEN);

  /* If pin is output with readback then enable the input buffer */

  if ((pinset & PORT_OUTREADBACK_MASK) == PORT_OUTREADBACK_ENABLE)
    {
      regval |= PORT_WRCONFIG_INEN;
    }

  if (pin >= 16)
    {
       /* Select the upper half word and adjust the bit setting */

       regval |= PORT_WRCONFIG_HWSEL;
       pin    -= 16;
    }

  regval |= PORT_WRCONFIG_PINMASK(pin);

  /* Set the pin function */

  func    = (pinset & PORT_FUNC_MASK) >> PORT_FUNC_SHIFT;
  regval |= (func << PORT_WRCONFIG_PMUX_SHIFT);

  /* Check for pull-up/down selection */

  switch (pinset & PORT_PULL_MASK)
    {
      case PORT_PULL_UP:
        {
          /* Select pull-up by setting the corresponding bit in OUT
           * register.
           */

          putreg32(bit, base + SAM_PORT_OUTSET_OFFSET);
        }

        /* Fall through */

      case PORT_PULL_DOWN:
        {
          regval |= PORT_WRCONFIG_PULLEN;
        }
        break;

      default:
      case PORT_PULL_NONE:
        break;
    }

  /* Configure the pin for the peripheral function */

  putreg32(regval, base + SAM_PORT_WRCONFIG_OFFSET);
}

/****************************************************************************
 * Name: sam_configreset
 *
 * Description:
 *   Configure a PORT pin in the default, reset state.
 *
 ****************************************************************************/

static inline void sam_configreset(uintptr_t base, port_pinset_t pinset)
{
  uint32_t regval;
  uint32_t bit;
  int pin;

  /* Decode pin information */

  pin = (pinset & PORT_PIN_MASK) >> PORT_PIN_SHIFT;
  bit = (1 << pin);

  /* Set the direction bit to zero (input) */

  putreg32(bit, base + SAM_PORT_DIRCLR_OFFSET);

  /* Disable the I/O synchronizer */

  regval  = getreg32(base + SAM_PORT_CTRL_OFFSET);
  regval &= ~bit;
  putreg32(regval, base + SAM_PORT_CTRL_OFFSET);

  /* Assume input pull-down or output value low */

  putreg32(bit, base + SAM_PORT_OUTCLR_OFFSET);

  /* Set the pin configuration */

  regval = (PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX);
  if (pin > 16)
    {
       /* Select the upper half word and adjust the bit setting */

       regval |= PORT_WRCONFIG_HWSEL;
       pin    -= 16;
    }

  regval |= PORT_WRCONFIG_PINMASK(pin);

  /* Disable the peripheral multiplexor, disable the input, disable
   * pull-up/down, reset driver strength, etc.
   */

  putreg32(regval, base + SAM_PORT_WRCONFIG_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_portconfig
 *
 * Description:
 *   Configure a PORT pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int sam_portconfig(port_pinset_t pinset)
{
  uintptr_t base = sam_portbase(pinset);
  irqstate_t flags;

  /* Make sure that all operations on the port are atomic */

  flags = enter_critical_section();

  /* Put the PORT in the known, reset state.  */

  sam_configreset(base, pinset);

  /* Then put the PORT into the requested state */

  switch (pinset & PORT_MODE_MASK)
    {
      case PORT_INPUT:
        sam_configinput(base, pinset);
        break;

      case PORT_OUTPUT:
        sam_configoutput(base, pinset);
        break;

      case PORT_PERIPHERAL:
        sam_configperiph(base, pinset);
        break;

      case PORT_INTERRUPT:
        sam_configinterrupt(base, pinset);
        break;

      default:
        break;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_portwrite
 *
 * Description:
 *   Write one or zero to the selected PORT pin
 *
 ****************************************************************************/

void sam_portwrite(port_pinset_t pinset, bool value)
{
  uintptr_t base = sam_portbase(pinset);
  uint32_t  pin  = sam_portpin(pinset);

  if (value)
    {
      putreg32(pin, base + SAM_PORT_OUTSET_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PORT_OUTCLR_OFFSET);
    }
}

/****************************************************************************
 * Name: sam_portread
 *
 * Description:
 *   Read one or zero from the selected PORT pin
 *
 ****************************************************************************/

bool sam_portread(port_pinset_t pinset)
{
  uintptr_t base = sam_portbase(pinset);
  uint32_t  pin  = sam_portpin(pinset);

  return (getreg32(base + SAM_PORT_IN_OFFSET) & pin) != 0;
}

/****************************************************************************
 * Function:  sam_dumpport
 *
 * Description:
 *   Dump all PORT registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int sam_dumpport(uint32_t pinset, const char *msg)
{
  irqstate_t    flags;
  uintptr_t     base;
  unsigned int  pin;
  unsigned int  port;

  /* Get the base address associated with the PIO port */

  pin  = (pinset & PORT_PIN_MASK) >> PORT_PIN_SHIFT;
  port = (pinset & PORT_MASK) >> PORT_SHIFT;
  base = SAM_PORTN_BASE(port);

  /* The following requires exclusive access to the PORT registers */

  flags = enter_critical_section();
  gpioinfo("PORT%c pin: %d pinset: %08x base: %08x -- %s\n",
           g_portchar[port], pin, pinset, base, msg);
  gpioinfo("  DIR: %08x OUT: %08x IN: %08x\n",
           getreg32(base + SAM_PORT_DIR_OFFSET),
           getreg32(base + SAM_PORT_OUT_OFFSET),
           getreg32(base + SAM_PORT_IN_OFFSET));
  gpioinfo("  CTRL: %08x WRCONFIG: %08x\n",
           getreg32(base + SAM_PORT_CTRL_OFFSET),
           getreg32(base + SAM_PORT_WRCONFIG_OFFSET));
  gpioinfo("  PMUX[%08x]: %02x PINCFG[%08x]: %02x\n",
           base + SAM_PORT_PMUX_OFFSET(pin),
           getreg8(base + SAM_PORT_PMUX_OFFSET(pin)),
           base + SAM_PORT_PINCFG_OFFSET(pin),
           getreg8(base + SAM_PORT_PINCFG_OFFSET(pin)));

  leave_critical_section(flags);
  return OK;
}
#endif
