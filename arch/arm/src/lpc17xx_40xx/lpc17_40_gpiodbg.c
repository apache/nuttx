/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_gpiodbg.c
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

/* Output debug info even if debug output is not selected. */

#undef  CONFIG_DEBUG_INFO
#define CONFIG_DEBUG_INFO 1

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc17_40_gpio.h"

#ifdef CONFIG_DEBUG_GPIO_INFO

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_pinsel
 *
 * Description:
 *   Get the address of the PINSEL register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

#ifdef LPC176x
static uint32_t lpc17_40_pinsel(unsigned int port, unsigned int pin)
{
  if (pin < 16)
    {
      return g_lopinsel[port];
    }
  else
    {
      return g_hipinsel[port];
    }
}
#endif /* LPC176x */

/****************************************************************************
 * Name: lpc17_40_pinmode
 *
 * Description:
 *   Get the address of the PINMODE register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

#ifdef LPC176x
static uint32_t lpc17_40_pinmode(unsigned int port, unsigned int pin)
{
  if (pin < 16)
    {
      return g_lopinmode[port];
    }
  else
    {
      return g_hipinmode[port];
    }
}
#endif /* LPC176x */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  lpc17_40_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int lpc17_40_dumpgpio(lpc17_40_pinset_t pinset, const char *msg)
{
  irqstate_t   flags;
  uint32_t     base;
#if defined(LPC176x)
  uint32_t     pinsel;
  uint32_t     pinmode;
#elif defined(LPC178x_40xx)
  uint32_t     iocon;
#endif /* LPC176x */
  unsigned int port;
  unsigned int pin;

  /* Get the base address associated with the GPIO port */

  port    = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin     = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

#if defined(LPC176x)
  pinsel  = lpc17_40_pinsel(port, pin);
  pinmode = lpc17_40_pinmode(port, pin);
#elif defined(LPC178x_40xx)
  iocon   = LPC17_40_IOCON_P(port, pin);
#endif /* LPC176x */

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  gpioinfo("GPIO%c pin%d (pinset: %08x) -- %s\n",
           port + '0', pin, pinset, msg);

#if defined(LPC176x)
  gpioinfo("PINSEL[%08x]: %08x PINMODE[%08x]: %08x ODMODE[%08x]: %08x\n",
           pinsel,  pinsel  ? getreg32(pinsel) : 0,
           pinmode, pinmode ? getreg32(pinmode) : 0,
           g_odmode[port],    getreg32(g_odmode[port]));
#elif defined(LPC178x_40xx)
  gpioinfo("  IOCON[%08x]: %08x\n", iocon, getreg32(iocon));
#endif

  base = g_fiobase[port];
  gpioinfo("FIODIR[%08x]: %08x FIOMASK[%08x]: %08x FIOPIN[%08x]: %08x\n",
           base + LPC17_40_FIO_DIR_OFFSET,
           getreg32(base + LPC17_40_FIO_DIR_OFFSET),
           base + LPC17_40_FIO_MASK_OFFSET,
           getreg32(base + LPC17_40_FIO_MASK_OFFSET),
           base + LPC17_40_FIO_PIN_OFFSET,
           getreg32(base + LPC17_40_FIO_PIN_OFFSET));

  base = g_intbase[port];
  gpioinfo(
          "IOINTSTAT[%08x]: %08x INTSTATR[%08x]: %08x INSTATF[%08x]: %08x\n",
           LPC17_40_GPIOINT_IOINTSTATUS,
           getreg32(LPC17_40_GPIOINT_IOINTSTATUS),
           base + LPC17_40_GPIOINT_INTSTATR_OFFSET,
           getreg32(base + LPC17_40_GPIOINT_INTSTATR_OFFSET),
           base + LPC17_40_GPIOINT_INTSTATF_OFFSET,
           getreg32(base + LPC17_40_GPIOINT_INTSTATF_OFFSET));
  gpioinfo("INTENR[%08x]: %08x INTENF[%08x]: %08x\n",
           base + LPC17_40_GPIOINT_INTENR_OFFSET,
           getreg32(base + LPC17_40_GPIOINT_INTENR_OFFSET),
           base + LPC17_40_GPIOINT_INTENF_OFFSET,
           getreg32(base + LPC17_40_GPIOINT_INTENF_OFFSET));

  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_DEBUG_GPIO_INFO */
