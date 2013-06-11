/****************************************************************************
 * arch/arm/src/sam34/sam3u_gpio.c
 * General Purpose Input/Output (GPIO) logic for the SAM3U and SAM4S
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "sam_gpio.h"

#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  include "chip/sam3u_pio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4S)
#  include "chip/sam4s_pio.h"
#else
#  error Unrecognized SAM architecture
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
static const char g_portchar[4]   = { 'A', 'B', 'C', 'D' };
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: sam_gpiobase
 *
 * Description:
 *   Return the base address of the GPIO register set
 *
 ****************************************************************************/

static inline uintptr_t sam_gpiobase(gpio_pinset_t cfgset)
{
  int port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  return SAM_PION_BASE(port);
}

/****************************************************************************
 * Name: sam_gpiopin
 *
 * Description:
 *   Returun the base address of the GPIO register set
 *
 ****************************************************************************/

static inline int sam_gpiopin(gpio_pinset_t cfgset)
{
  return 1 << ((cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: sam_configinput
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configinput(uintptr_t base, uint32_t pin,
                                  gpio_pinset_t cfgset)
{
#ifdef GPIO_HAVE_SCHMITT
  uint32_t regval;
#endif

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & GPIO_CFG_PULLUP) != 0)
    {
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef GPIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Check if filtering should be enabled */

  if ((cfgset & GPIO_CFG_DEGLITCH) != 0)
    {
      putreg32(pin, base + SAM_PIO_IFER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_IFDR_OFFSET);
    }

#ifdef GPIO_HAVE_SCHMITT
  /* Enable/disable the Schmitt trigger */

  regval = getreg32(base + SAM_PIO_SCHMITT_OFFSET);
  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      regval |= pin;
    }
  else
    {
      regval &= ~pin;
    }
  putreg32(regval, base + SAM_PIO_SCHMITT_OFFSET);
#endif

  /* Configure the pin as an input and enable the GPIO function */

  putreg32(pin, base + SAM_PIO_ODR_OFFSET);
  putreg32(pin, base + SAM_PIO_PER_OFFSET);

  /* To-Do:  If DEGLITCH is selected, need to configure DIFSR, SCIFSR, and
   *         IFDGSR registers.  This would probably best be done with
   *         another, new API... perhaps sam_configfilter()
   */

 return OK;
}

/****************************************************************************
 * Name: sam_configoutput
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configoutput(uintptr_t base, uint32_t pin,
                                   gpio_pinset_t cfgset)
{
  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & GPIO_CFG_PULLUP) != 0)
    {
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef GPIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Enable the open drain driver if requrested */

  if ((cfgset & GPIO_CFG_OPENDRAIN) != 0)
    {
      putreg32(pin, base + SAM_PIO_MDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_MDDR_OFFSET);
    }

  /* Set default value */

  if ((cfgset & GPIO_OUTPUT_SET) != 0)
    {
      putreg32(pin, base + SAM_PIO_SODR_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_CODR_OFFSET);
    }

  /* Configure the pin as an input and enable the GPIO function */

  putreg32(pin, base + SAM_PIO_OER_OFFSET);
  putreg32(pin, base + SAM_PIO_PER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_configperiph
 *
 * Description:
 *   Configure a GPIO pin driven by a peripheral A or B signal based on
 *   bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configperiph(uintptr_t base, uint32_t pin,
                                   gpio_pinset_t cfgset)
{
  uint32_t regval;

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & GPIO_CFG_PULLUP) != 0)
    {
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef GPIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

#ifdef GPIO_HAVE_PERIPHCD
  /* Configure pin, depending upon the peripheral A, B, C or D
   *
   *   PERIPHA: ABCDSR1[n] = 0 ABCDSR2[n] = 0
   *   PERIPHB: ABCDSR1[n] = 1 ABCDSR2[n] = 0
   *   PERIPHC: ABCDSR1[n] = 0 ABCDSR2[n] = 1
   *   PERIPHD: ABCDSR1[n] = 1 ABCDSR2[n] = 1
   */

  regval = getreg32(base + SAM_PIO_ABCDSR1_OFFSET);
  if ((cfgset & GPIO_MODE_MASK) == GPIO_PERIPHA ||
      (cfgset & GPIO_MODE_MASK) == GPIO_PERIPHC)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
    }
  putreg32(regval, base + SAM_PIO_ABCDSR1_OFFSET);

  regval = getreg32(base + SAM_PIO_ABCDSR2_OFFSET);
  if ((cfgset & GPIO_MODE_MASK) == GPIO_PERIPHA ||
      (cfgset & GPIO_MODE_MASK) == GPIO_PERIPHB)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
    }
  putreg32(regval, base + SAM_PIO_ABCDSR2_OFFSET);

#else
  /* Configure pin, depending upon the peripheral A or B:
   *
   *   PERIPHA: ABSR[n] = 0
   *   PERIPHB: ABSR[n] = 1
   */

  regval = getreg32(base + SAM_PIO_ABSR_OFFSET);
  if ((cfgset & GPIO_MODE_MASK) == GPIO_PERIPHA)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
    }
  putreg32(regval, base + SAM_PIO_ABSR_OFFSET);
#endif

  /* Disable PIO functionality */

  putreg32(pin, base + SAM_PIO_PDR_OFFSET);
  return OK;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int sam_configgpio(gpio_pinset_t cfgset)
{
  uintptr_t base = sam_gpiobase(cfgset);
  uint32_t  pin  = sam_gpiopin(cfgset);
  int       ret;

  switch (cfgset & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        ret = sam_configinput(base, pin, cfgset);
        break;

      case GPIO_OUTPUT:
        ret = sam_configoutput(base, pin, cfgset);
        break;

      case GPIO_PERIPHA:
      case GPIO_PERIPHB:
        ret = sam_configperiph(base, pin, cfgset);
        break;

      default:
        ret = -EINVAL;
        break;
    }
  return ret;
}

/****************************************************************************
 * Name: sam_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void sam_gpiowrite(gpio_pinset_t pinset, bool value)
{
  uintptr_t base = sam_gpiobase(pinset);
  uint32_t  pin  = sam_gpiopin(pinset);

  if (value)
    {
      putreg32(pin, base + SAM_PIO_SODR_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_CODR_OFFSET);
    }
}

/****************************************************************************
 * Name: sam_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool sam_gpioread(gpio_pinset_t pinset)
{
  uintptr_t base = sam_gpiobase(pinset);
  uint32_t  pin  = sam_gpiopin(pinset);
  uint32_t  regval;

  if ((pinset & GPIO_MODE_MASK) == GPIO_OUTPUT)
    {
      regval = getreg32(base + SAM_PIO_ODSR_OFFSET);
    }
  else
    {
      regval = getreg32(base + SAM_PIO_PDSR_OFFSET);
    }

  return (regval & pin) != 0;
}

/************************************************************************************
 * Function:  sam_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
int sam_dumpgpio(uint32_t pinset, const char *msg)
{
  irqstate_t    flags;
  uintptr_t     base;
  unsigned int  pin;
  unsigned int  port;

  /* Get the base address associated with the PIO port */

  pin  = sam_gpiopin(pinset);
  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  base = SAM_PION_BASE(port);

  /* The following requires exclusive access to the GPIO registers */

  flags = irqsave();
  lldbg("PIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);
  lldbg("    PSR: %08x    OSR: %08x   IFSR: %08x   ODSR: %08x\n",
        getreg32(base + SAM_PIO_PSR_OFFSET), getreg32(base + SAM_PIO_OSR_OFFSET),
        getreg32(base + SAM_PIO_IFSR_OFFSET), getreg32(base + SAM_PIO_ODSR_OFFSET));
  lldbg("   PDSR: %08x    IMR: %08x    ISR: %08x   MDSR: %08x\n",
        getreg32(base + SAM_PIO_PDSR_OFFSET), getreg32(base + SAM_PIO_IMR_OFFSET),
        getreg32(base + SAM_PIO_ISR_OFFSET), getreg32(base + SAM_PIO_MDSR_OFFSET));
#if defined(CONFIG_ARCH_CHIP_ATSAM3U)
  lldbg("   ABSR: %08x SCIFSR: %08x  DIFSR: %08x IFDGSR: %08x\n",
        getreg32(base + SAM_PIO_ABSR_OFFSET), getreg32(base + SAM_PIO_SCIFSR_OFFSET),
        getreg32(base + SAM_PIO_DIFSR_OFFSET), getreg32(base + SAM_PIO_IFDGSR_OFFSET));
#elif defined(CONFIG_ARCH_CHIP_ATSAM4S)
  lldbg(" ABCDSR: %08x %08x         IFSCSR: %08x  PPDSR: %08x\n",
        getreg32(base + SAM_PIO_ABCDSR1_OFFSET), getreg32(base + SAM_PIO_ABCDSR2_OFFSET),
        getreg32(base + SAM_PIO_IFSCSR_OFFSET), getreg32(base + SAM_PIOC_PPDSR));
#endif
  lldbg("   PUSR: %08x   SCDR: %08x   OWSR: %08x  AIMMR: %08x\n",
        getreg32(base + SAM_PIO_PUSR_OFFSET), getreg32(base + SAM_PIO_SCDR_OFFSET),
        getreg32(base + SAM_PIO_OWSR_OFFSET), getreg32(base + SAM_PIO_AIMMR_OFFSET));
  lldbg("    ESR: %08x    LSR: %08x   ELSR: %08x FELLSR: %08x\n",
        getreg32(base + SAM_PIO_ESR_OFFSET), getreg32(base + SAM_PIO_LSR_OFFSET),
        getreg32(base + SAM_PIO_ELSR_OFFSET), getreg32(base + SAM_PIO_FELLSR_OFFSET));
  lldbg(" FRLHSR: %08x LOCKSR: %08x   WPMR: %08x   WPSR: %08x\n",
        getreg32(base + SAM_PIO_FRLHSR_OFFSET), getreg32(base + SAM_PIO_LOCKSR_OFFSET),
        getreg32(base + SAM_PIO_WPMR_OFFSET), getreg32(base + SAM_PIO_WPSR_OFFSET));
#if defined(CONFIG_ARCH_CHIP_ATSAM4S)
  lldbg("   PCMR: %08x PCIMR: %08x   PCISR: %08x   PCRHR: %08x\n",
        getreg32(base + SAM_PIO_PCMR_OFFSET), getreg32(base + SAM_PIO_PCIMR_OFFSET),
        getreg32(base + SAM_PIO_PCISR_OFFSET), getreg32(base + SAM_PIO_PCRHR_OFFSET));
  lldbg("SCHMITT: %08x\n",
        getreg32(base + SAM_PIO_SCHMITT_OFFSET));
#endif
  irqrestore(flags);
  return OK;
}
#endif

