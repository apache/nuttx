/****************************************************************************
 * arch/arm/src/sama5/sam_pio.c
 * General Purpose Input/Output (PIO) logic for the SAMA5
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include "sam_pio.h"
#include "chip/sam_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
  static const char g_portchar[SAM_NPIO] =
  {
    'A', 'B', 'C', 'D', 'E'
  };
#endif

/* SAM_PION_VBASE will only be defined if the PIO register blocks are
 * contiguous.  If not defined, then we need to do a table lookup.
 */

#ifndef SAM_PION_VBASE
  static const uintptr_t g_piobase[SAM_NPIO] =
  {
    SAM_PIOA_VBASE, SAM_PIOB_VBASE, SAM_PIOC_VBASE, SAM_PIOD_VBASE,
    SAM_PIOE_VBASE
  };

# define SAM_PION_VBASE(n) (g_piobase[(n)])
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: sam_piobase
 *
 * Description:
 *   Return the base address of the PIO register set
 *
 ****************************************************************************/

static inline uintptr_t sam_piobase(pio_pinset_t cfgset)
{
  int port = (cfgset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;

  if (port < SAM_NPIO)
    {
      return SAM_PION_VBASE(port);
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: sam_piopin
 *
 * Description:
 *   Returun the base address of the PIO register set
 *
 ****************************************************************************/

static inline int sam_piopin(pio_pinset_t cfgset)
{
  return 1 << ((cfgset & PIO_PIN_MASK) >> PIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: sam_configinput
 *
 * Description:
 *   Configure a PIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configinput(uintptr_t base, uint32_t pin,
                                  pio_pinset_t cfgset)
{
#if defined(PIO_HAVE_SCHMITT) || defined(PIO_HAVE_DRIVE)
  uint32_t regval;
#endif
#if defined(PIO_HAVE_DRIVE)
  uint32_t offset;
  uint32_t mask;
  uint32_t drive;
  int shift;
#endif

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & PIO_CFG_PULLUP) != 0)
    {
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef PIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & PIO_CFG_PULLDOWN) != 0)
    {
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Check if filtering should be enabled */

  if ((cfgset & PIO_CFG_DEGLITCH) != 0)
    {
      putreg32(pin, base + SAM_PIO_IFER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_IFDR_OFFSET);
    }

#ifdef PIO_HAVE_SCHMITT
  /* Enable/disable the Schmitt trigger */

  regval = getreg32(base + SAM_PIO_SCHMITT_OFFSET);
  if ((cfgset & PIO_CFG_PULLDOWN) != 0)
    {
      regval |= pin;
    }
  else
    {
      regval &= ~pin;
    }
  putreg32(regval, base + SAM_PIO_SCHMITT_OFFSET);
#endif

#ifdef PIO_HAVE_DRIVE
  /* Configure drive strength */

  drive = (cfgset & PIO_DRIVE_MASK) >> PIO_DRIVE_SHIFT;
  if (pin < 32)
    {
      offset = SAM_PIO_DRIVER1_OFFSET;
      mask   = PIO_DRIVER1_LINE_MASK(pin);
      shift  = PIO_DRIVER1_LINE_SHIFT(pin);
    }
  else
    {
      offset = SAM_PIO_DRIVER2_OFFSET;
      mask   = PIO_DRIVER2_LINE_MASK(pin);
      shift  = PIO_DRIVER2_LINE_SHIFT(pin);
    }

  regval = getreg32(base + offset);
  regval &= ~mask;
  regval |= drive << shift;
  putreg32(regval, base + offset);
#endif

  /* Configure the pin as an input and enable the PIO function */

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
 *   Configure a PIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configoutput(uintptr_t base, uint32_t pin,
                                   pio_pinset_t cfgset)
{
  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & PIO_CFG_PULLUP) != 0)
    {
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef PIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & PIO_CFG_PULLDOWN) != 0)
    {
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Enable the open drain driver if requrested */

  if ((cfgset & PIO_CFG_OPENDRAIN) != 0)
    {
      putreg32(pin, base + SAM_PIO_MDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_MDDR_OFFSET);
    }

  /* Set default value */

  if ((cfgset & PIO_OUTPUT_SET) != 0)
    {
      putreg32(pin, base + SAM_PIO_SODR_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_CODR_OFFSET);
    }

  /* Configure the pin as an output and enable the PIO function */

  putreg32(pin, base + SAM_PIO_OER_OFFSET);
  putreg32(pin, base + SAM_PIO_PER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_configperiph
 *
 * Description:
 *   Configure a PIO pin driven by a peripheral A or B signal based on
 *   bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configperiph(uintptr_t base, uint32_t pin,
                                   pio_pinset_t cfgset)
{
  uint32_t regval;

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & PIO_CFG_PULLUP) != 0)
    {
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef PIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & PIO_CFG_PULLDOWN) != 0)
    {
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

#ifdef PIO_HAVE_PERIPHCD
  /* Configure pin, depending upon the peripheral A, B, C or D
   *
   *   PERIPHA: ABCDSR1[n] = 0 ABCDSR2[n] = 0
   *   PERIPHB: ABCDSR1[n] = 1 ABCDSR2[n] = 0
   *   PERIPHC: ABCDSR1[n] = 0 ABCDSR2[n] = 1
   *   PERIPHD: ABCDSR1[n] = 1 ABCDSR2[n] = 1
   */

  regval = getreg32(base + SAM_PIO_ABCDSR1_OFFSET);
  if ((cfgset & PIO_MODE_MASK) == PIO_PERIPHA ||
      (cfgset & PIO_MODE_MASK) == PIO_PERIPHC)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
    }
  putreg32(regval, base + SAM_PIO_ABCDSR1_OFFSET);

  regval = getreg32(base + SAM_PIO_ABCDSR2_OFFSET);
  if ((cfgset & PIO_MODE_MASK) == PIO_PERIPHA ||
      (cfgset & PIO_MODE_MASK) == PIO_PERIPHB)
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
  if ((cfgset & PIO_MODE_MASK) == PIO_PERIPHA)
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
 * Name: sam_configpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int sam_configpio(pio_pinset_t cfgset)
{
  uintptr_t base;
  uint32_t pin;
  irqstate_t flags;
  int ret;

  /* Sanity check */

  base = sam_piobase(cfgset);
  if (base == 0)
    {
      return -EINVAL;
    }

  pin = sam_piopin(cfgset);

  /* Disable interrupts to prohibit re-entrance. */

  flags = irqsave();

  /* Enable writing to PIO registers */

  putreg32(PIO_WPMR_WPKEY, base + SAM_PIO_WPMR_OFFSET);

  /* Handle the pin configuration according to pin type */

  switch (cfgset & PIO_MODE_MASK)
    {
      case PIO_INPUT:
        ret = sam_configinput(base, pin, cfgset);
        break;

      case PIO_OUTPUT:
        ret = sam_configoutput(base, pin, cfgset);
        break;

      case PIO_PERIPHA:
      case PIO_PERIPHB:
#ifdef PIO_HAVE_PERIPHCD
      case PIO_PERIPHC:
      case PIO_PERIPHD:
#endif
        ret = sam_configperiph(base, pin, cfgset);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  /* Disable writing to PIO registers */

  putreg32(PIO_WPMR_WPEN | PIO_WPMR_WPKEY, base + SAM_PIO_WPMR_OFFSET);
  irqrestore(flags);

  return ret;
}

/****************************************************************************
 * Name: sam_piowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ****************************************************************************/

void sam_piowrite(pio_pinset_t pinset, bool value)
{
  uintptr_t base = sam_piobase(pinset);
  uint32_t  pin  = sam_piopin(pinset);

  if (base != 0)
    {
      if (value)
        {
          putreg32(pin, base + SAM_PIO_SODR_OFFSET);
        }
      else
        {
          putreg32(pin, base + SAM_PIO_CODR_OFFSET);
        }
    }
}

/****************************************************************************
 * Name: sam_pioread
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ****************************************************************************/

bool sam_pioread(pio_pinset_t pinset)
{
  uintptr_t base = sam_piobase(pinset);
  uint32_t  pin;
  uint32_t  regval;

  if (base != 0)
    {
      pin  = sam_piopin(pinset);

      if ((pinset & PIO_MODE_MASK) == PIO_OUTPUT)
        {
          regval = getreg32(base + SAM_PIO_ODSR_OFFSET);
        }
      else
        {
          regval = getreg32(base + SAM_PIO_PDSR_OFFSET);
        }

      return (regval & pin) != 0;
    }

  return 0;
}

/************************************************************************************
 * Function:  sam_dumppio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
int sam_dumppio(uint32_t pinset, const char *msg)
{
  irqstate_t    flags;
  uintptr_t     base;
  unsigned int  pin;
  unsigned int  port;

  /* Get the base address associated with the PIO port */

  pin  = sam_piopin(pinset);
  port = (pinset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;
  base = SAM_PION_BASE(port);

  /* The following requires exclusive access to the PIO registers */

  flags = irqsave();
  lldbg("PIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);
  lldbg("    PSR: %08x    OSR: %08x   IFSR: %08x   ODSR: %08x\n",
        getreg32(base + SAM_PIO_PSR_OFFSET), getreg32(base + SAM_PIO_OSR_OFFSET),
        getreg32(base + SAM_PIO_IFSR_OFFSET), getreg32(base + SAM_PIO_ODSR_OFFSET));
  lldbg("   PDSR: %08x    IMR: %08x    ISR: %08x   MDSR: %08x\n",
        getreg32(base + SAM_PIO_PDSR_OFFSET), getreg32(base + SAM_PIO_IMR_OFFSET),
        getreg32(base + SAM_PIO_ISR_OFFSET), getreg32(base + SAM_PIO_MDSR_OFFSET));
  lldbg(" ABCDSR: %08x %08x         IFSCSR: %08x  PPDSR: %08x\n",
        getreg32(base + SAM_PIO_ABCDSR1_OFFSET), getreg32(base + SAM_PIO_ABCDSR2_OFFSET),
        getreg32(base + SAM_PIO_IFSCSR_OFFSET), getreg32(base + SAM_PIOC_PPDSR));
  lldbg("   PUSR: %08x   SCDR: %08x   OWSR: %08x  AIMMR: %08x\n",
        getreg32(base + SAM_PIO_PUSR_OFFSET), getreg32(base + SAM_PIO_SCDR_OFFSET),
        getreg32(base + SAM_PIO_OWSR_OFFSET), getreg32(base + SAM_PIO_AIMMR_OFFSET));
  lldbg("    ESR: %08x    LSR: %08x   ELSR: %08x FELLSR: %08x\n",
        getreg32(base + SAM_PIO_ESR_OFFSET), getreg32(base + SAM_PIO_LSR_OFFSET),
        getreg32(base + SAM_PIO_ELSR_OFFSET), getreg32(base + SAM_PIO_FELLSR_OFFSET));
  lldbg(" FRLHSR: %08x LOCKSR: %08x   WPMR: %08x   WPSR: %08x\n",
        getreg32(base + SAM_PIO_FRLHSR_OFFSET), getreg32(base + SAM_PIO_LOCKSR_OFFSET),
        getreg32(base + SAM_PIO_WPMR_OFFSET), getreg32(base + SAM_PIO_WPSR_OFFSET));
  lldbg("   PCMR: %08x PCIMR: %08x   PCISR: %08x   PCRHR: %08x\n",
        getreg32(base + SAM_PIO_PCMR_OFFSET), getreg32(base + SAM_PIO_PCIMR_OFFSET),
        getreg32(base + SAM_PIO_PCISR_OFFSET), getreg32(base + SAM_PIO_PCRHR_OFFSET));
  lldbg("SCHMITT: %08x\n",
        getreg32(base + SAM_PIO_SCHMITT_OFFSET));
  irqrestore(flags);
  return OK;
}
#endif

