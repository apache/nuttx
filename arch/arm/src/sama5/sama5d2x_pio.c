/****************************************************************************
 * arch/arm/src/sama5/sama5d2x_pio.c
 * General Purpose Input/Output (PIO) logic for the SAMA5D2x
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include "chip/sama5d2x_pio.h"

#include "chip.h"
#include "sam_periphclks.h"
#include "sam_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to convert a pin to a vanilla input */

#define PIO_INPUT_BITS (PIO_INPUT | PIO_CFG_DEFAULT)
#define MK_INPUT(p)    (((p) & (PIO_PORT_MASK | PIO_PIN_MASK)) | PIO_INPUT_BITS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Lookup for non-secure PIOs */

const uintptr_t g_piobase[SAM_NPIO] =
{
  SAM_PIO_IOGROUPA_VBASE
#if SAM_NPIO > 1
  , SAM_PIO_IOGROUPB_VBASE
#endif
#if SAM_NPIO > 2
  , SAM_PIO_IOGROUPC_VBASE
#endif
#if SAM_NPIO > 3
  , SAM_PIO_IOGROUPD_VBASE
#endif
#if SAM_NPIO > 4
  , SAM_PIO_IOGROUPE_VBASE
#endif
};

/* Lookup for non-secure PIOs */

const uintptr_t g_spiobase[SAM_NPIO] =
{
  SAM_SPIO_IOGROUPA_VBASE
#if SAM_NPIO > 1
  , SAM_SPIO_IOGROUPB_VBASE
#endif
#if SAM_NPIO > 2
  , SAM_SPIO_IOGROUPC_VBASE
#endif
#if SAM_NPIO > 3
  , SAM_SPIO_IOGROUPD_VBASE
#endif
#if SAM_NPIO > 4
  , SAM_SPIO_IOGROUPE_VBASE
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Maps a port number to the standard port character */

#if defined(CONFIG_DEBUG_GPIO) && SAM_NPIO > 0
static const char g_portchar[SAM_NPIO] =
{
  'A'
#if SAM_NPIO > 1
  , 'B'
#endif
#if SAM_NPIO > 2
  , 'C'
#endif
#if SAM_NPIO > 3
  , 'D'
#endif
#if SAM_NPIO > 4
  , 'E'
#endif
};
#endif

/* Used to determine if a PIO port is configured to support interrupts */

#if SAM_NPIO > 0
static const bool g_piointerrupt[SAM_NPIO] =
{
#ifdef CONFIG_SAMA5_PIOA_IRQ
  true
#else
  false
#endif

#if SAM_NPIO > 1
#ifdef CONFIG_SAMA5_PIOB_IRQ
  , true
#else
  , false
#endif
#endif

#if SAM_NPIO > 2
#ifdef CONFIG_SAMA5_PIOC_IRQ
  , true
#else
  , false
#endif
#endif

#if SAM_NPIO > 3
#ifdef CONFIG_SAMA5_PIOD_IRQ
  , true
#else
  , false
#endif
#endif

#if SAM_NPIO > 4
#ifdef CONFIG_SAMA5_PIOE_IRQ
  , true
#else
  , false
#endif
#endif
};
#endif

/* This is an array of ports that PIO enable forced on */

static uint32_t g_forced[SAM_NPIO];

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

static inline bool sam_issecure(pio_pinset_t cfgset)
{
  return ((cfgset & PIO_INT_SECURE) != 0);
}

/****************************************************************************
 * Name: sam_piobase
 *
 * Description:
 *   Return the base address of the PIO register set
 *
 ****************************************************************************/

static uintptr_t sam_piobase(pio_pinset_t cfgset)
{
  int port = (cfgset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;

  /* Verify that the port number is within range */

  if (port < SAM_NPIO)
    {
      /* Is this a secure or an un-secured PIO? */

      if (sam_issecure(cfgset))
        {
          return sam_spion_vbase(port);
        }
      else
        {
          return sam_pion_vbase(port);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: sam_piopin
 *
 * Description:
 *   Return a bitmask corresponding to the bit position in a PIO register
 *
 ****************************************************************************/

static inline uint32_t sam_piopin(pio_pinset_t cfgset)
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
  uint32_t regval;

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Select GPIO input */

  regval = (PIO_CFGR_FUNC_GPIO | PIO_CFGR_DIR_INPUT);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & PIO_CFG_PULLUP) != 0)
    {
      regval |= PIO_CFGR_PUEN;
    }

  /* Enable/disable the pull-down as requested */

  if ((cfgset & PIO_CFG_PULLDOWN) != 0)
    {
      regval |= PIO_CFGR_PDEN;
    }

  /* Check if filtering should be enabled */

  if ((cfgset & PIO_CFG_DEGLITCH) != 0)
    {
      if ((cfgset & PIO_CFG_DEGLITCH) != 0)
        {
          regval |= (PIO_CFGR_IFEN | PIO_CFGR_IFSCEN);
        }
      else
        {
          regval |= PIO_CFGR_IFEN;
        }
    }

  /* Enable/disable the Schmitt trigger inputs */

  if ((cfgset & PIO_CFG_SCHMITT) != 0)
    {
      regval |= PIO_CFGR_SCHMITT;
    }

  /* Select I/O drive.
   * REVISIT: Don't open drain and drive strength apply only to
   * output and peripheral pins.
   */

  switch (cfgset & PIO_DRIVE_MASK)
    {
    default:
    case PIO_DRIVE_LOW:
      regval |= PIO_CFGR_DRVSTR_LOW;
      break;

    case PIO_DRIVE_MEDIUM:
      regval |= PIO_CFGR_DRVSTR_MED;
      break;

    case PIO_DRIVE_HIGH:
      regval |= PIO_CFGR_DRVSTR_HIGH;
      break;
    }

  /* Clear some output only bits.  Mostly this just simplifies debug. */

  putreg32(pin, base + SAM_PIO_CODR_OFFSET);

  /* Configure the pin as an input and enable the PIO function */

  putreg32(regval, base + SAM_PIO_CFGR_OFFSET);
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
  uint32_t regval;

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Select GPIO output */

  regval = (PIO_CFGR_FUNC_GPIO | PIO_CFGR_DIR_OUTPUT);

  /* Enable/disable the pull-up as requested
   * NOTE: Control of the pull-up resistor is possible regardless of the
   * configuration of the I/O line (Input, Output, Open-drain).
   */

  if ((cfgset & PIO_CFG_PULLUP) != 0)
    {
      regval |= PIO_CFGR_PUEN;
    }

  /* Enable/disable the pull-down as requested */

  if ((cfgset & PIO_CFG_PULLDOWN) != 0)
    {
      regval |= PIO_CFGR_PDEN;
    }

  /* Input filtering and Schmitt triggering apply only to inputs */

  /* Enable the open drain driver if requested */

  if ((cfgset & PIO_CFG_OPENDRAIN) != 0)
    {
      regval |= PIO_CFGR_OPD;
    }

  /* Select I/O drive */

  switch (cfgset & PIO_DRIVE_MASK)
    {
    default:
    case PIO_DRIVE_LOW:
      regval |= PIO_CFGR_DRVSTR_LOW;
      break;

    case PIO_DRIVE_MEDIUM:
      regval |= PIO_CFGR_DRVSTR_MED;
      break;

    case PIO_DRIVE_HIGH:
      regval |= PIO_CFGR_DRVSTR_HIGH;
      break;
    }

  /* Set default value. This is to be done before the pin is configured as
   * an output in order to avoid any glitches at the time of the
   * configuration.
   */

  if ((cfgset & PIO_OUTPUT_SET) != 0)
    {
      putreg32(pin, base + SAM_PIO_SODR_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_CODR_OFFSET);
    }

  /* Configure the pin as an output and enable the PIO function */

  putreg32(regval, base + SAM_PIO_CFGR_OFFSET);
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
  unsigned int periph;

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Select the peripheral function.  The Direction bit does not apply to
   * peripherals.
   */

  periph = ((cfgset & PIO_CFGR_FUNC_MASK) - PIO_CFGR_FUNC_PERIPHA) >> PIO_CFGR_FUNC_SHIFT;
  regval = PIO_CFGR_FUNC_PERIPH(periph);

  /* Enable/disable the pull-up as requested
   * NOTE: Control of the pull-up resistor is possible regardless of the
   * configuration of the I/O line (Input, Output, Open-drain).
   */

  if ((cfgset & PIO_CFG_PULLUP) != 0)
    {
      regval |= PIO_CFGR_PUEN;
    }

  /* Enable/disable the pull-down as requested */

  if ((cfgset & PIO_CFG_PULLDOWN) != 0)
    {
      regval |= PIO_CFGR_PDEN;
    }

  /* REVIT: Input filtering and Schmitt triggering apply only to inputs */

  /* Enable the open drain driver if requested */

  if ((cfgset & PIO_CFG_OPENDRAIN) != 0)
    {
      regval |= PIO_CFGR_OPD;
    }

  /* Select I/O drive.
   * REVISIT: Does this apply to peripherals?
   */

  switch (cfgset & PIO_DRIVE_MASK)
    {
    default:
    case PIO_DRIVE_LOW:
      regval |= PIO_CFGR_DRVSTR_LOW;
      break;

    case PIO_DRIVE_MEDIUM:
      regval |= PIO_CFGR_DRVSTR_MED;
      break;

    case PIO_DRIVE_HIGH:
      regval |= PIO_CFGR_DRVSTR_HIGH;
      break;
    }

  /* Clear some output only bits.  Mostly this just simplifies debug. */

  putreg32(pin, base + SAM_PIO_CODR_OFFSET);


  /* Configure the pin as a peripheral */

  putreg32(regval, base + SAM_PIO_CFGR_OFFSET);
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

  /* Get the base address and pin mask associated with this pin configuration */

  base = sam_piobase(cfgset);
  if (base == 0)
    {
      return -EINVAL;
    }

  pin = sam_piopin(cfgset);

  /* Disable interrupts to prohibit re-entrance. */

  flags = irqsave();

  /* Enable writing to PIO registers.
   *
   *   The following registers are write-protected when WPEN is set in
   *   PIO_WPMR:
   *     - PIO Mask Register
   *     - PIO Configuration Register
   *   The following registers are write-protected when WPEN is set in
   *   S_PIO_WPMR:
   *     - Secure PIO Mask Register
   *     - Secure PIO Configuration Register
   *     - Secure PIO Slow Clock Divider Debouncing Register
   *   The following registers are write-protected when WPITEN is set in
   *   PIO_WPMR:
   *     - PIO Interrupt Enable Register
   *     - PIO Interrupt Disable Register
   *   The following registers are write-protected when WPITEN is set in
   *   S_PIO_WPMR:
   *     - Secure PIO Interrupt Enable Register
   *     - Secure PIO Interrupt Disable Register
   *
   * I suspect that the default state is the WPMR is unprotected, so these
   * operations could probably all be avoided.
   */

  putreg32(PIO_WPMR_WPKEY, SAM_PIO_WPMR);
  putreg32(PIO_WPMR_WPKEY, SAM_SPIO_WPMR);

  /* Select the secure or un-secured PIO operation */

  if (sam_issecure(cfgset))
    {
      putreg32(pin, base + SAM_SPIO_SIOSR_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_SPIO_SIONR_OFFSET);
    }

  /* Set the mask register to modify only the specific pin being configured. */

  putreg32(pin, base + SAM_PIO_MSKR_OFFSET);

  /* Put the pin in an initial state -- a vanilla input pin */

  (void)sam_configinput(base, pin, MK_INPUT(cfgset));

  /* Then handle the real pin configuration according to pin type */

  switch (cfgset & PIO_MODE_MASK)
    {
      case PIO_INPUT:
        ret = sam_configinput(base, pin, cfgset);
        break;

      case PIO_OUTPUT:
        ret = sam_configoutput(base, pin, cfgset);
        break;

      case PIO_ANALOG:
        /* REVISIT */
        ret = OK;
        break;

      case PIO_PERIPHA:
      case PIO_PERIPHB:
      case PIO_PERIPHC:
      case PIO_PERIPHD:
      case PIO_PERIPHE:
      case PIO_PERIPHF:
      case PIO_PERIPHG:
        ret = sam_configperiph(base, pin, cfgset);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  /* Disable writing to PIO registers */

  putreg32(PIO_WPMR_WPEN | PIO_WPMR_WPITEN | PIO_WPMR_WPKEY, SAM_PIO_WPMR);
  putreg32(PIO_WPMR_WPEN | PIO_WPMR_WPITEN | PIO_WPMR_WPKEY, SAM_SPIO_WPMR);
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
      /* Set or clear the output as requested.  NOTE: that there is no
       * check if the pin is actually configured as an output so this could,
       * potentially, do nothing.
       */

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
      pin = sam_piopin(pinset);

      /* For output PIOs, the ODSR register provides the output value to
       * drive the pin.  The PDSR register, on the the other hand, provides
       * the current sensed value on a pin, whether the pin is configured
       * as an input, an output or as a peripheral.
       *
       * There is small delay between the setting in ODSR and PDSR but
       * otherwise the they should be the same unless something external
       * is driving the pin.
       *
       * Let's assume that PDSR is what the caller wants.
       */

      regval = getreg32(base + SAM_PIO_PDSR_OFFSET);
      return (regval & pin) != 0;
    }

  return 0;
}

/************************************************************************************
 * Name: sam_pio_forceclk
 *
 * Description:
 *   Enable PIO clocking.  This logic is overly conservative and does not enable PIO
 *   clocking unless necessary (PIO input selected, glitch/filtering enable, or PIO
 *   interrupts enabled).  There are, however, certain conditions were we may want
 *   for force the PIO clock to be enabled.  An example is reading the input value
 *   from an open drain output.
 *
 *   The PIO automatic enable/disable logic is not smart enough enough to know about
 *   these cases.  For those cases, sam_pio_forceclk() is provided.
 *
 ************************************************************************************/

void sam_pio_forceclk(pio_pinset_t pinset, bool enable)
{
  unsigned int port;
  uint32_t pin;
  irqstate_t flags;

  /* Extract the port number */

  port = (pinset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;
  pin  = sam_piopin(pinset);

  /* The remainder of this operation must be atomic */

  flags = irqsave();

  /* Are we enabling or disabling clocking */

  if (enable)
    {
      /* Indicate that clocking is forced and enable the clock */

      g_forced[port] |= pin;
    }
  else
    {
      /* Clocking is no longer forced for this pin */

      g_forced[port] &= ~pin;
    }

  irqrestore(flags);
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
  unsigned int  port;
  bool          secure;

  /* Get the base address associated with the PIO port */

  port   = (pinset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;
  base   = sam_piobase(pinset);
  secure = sam_issecure(pinset);

  /* The following requires exclusive access to the PIO registers */

  flags = irqsave();

  if (secure)
    {
      lldbg("SPIO%c pinset: %08x base: %08x -- %s\n",
             g_portchar[port], pinset, base, msg);
    }
  else
    {
      lldbg("PIO%c pinset: %08x base: %08x -- %s\n",
             g_portchar[port], pinset, base, msg);
    }

  lldbg("   MSKR: %08x   CFGR: %08x   PDSR: %08x LOCKSR: %08x\n",
        getreg32(base + SAM_PIO_MSKR_OFFSET), getreg32(base + SAM_PIO_CFGR_OFFSET),
        getreg32(base + SAM_PIO_PDSR_OFFSET), getreg32(base + SAM_PIO_LOCKSR_OFFSET));
  lldbg("   ODSR: %08x    IMR: %08x    ISR: %08x\n",
        getreg32(base + SAM_PIO_ODSR_OFFSET), getreg32(base + SAM_PIO_IMR_OFFSET),
        getreg32(base + SAM_PIO_ISR_OFFSET));

  if (secure)
    {
      lldbg("   SCDR: %08x   WPMR: %08x   WPSR: %08x  IOSSR: %08x\n",
            getreg32(SAM_SPIO_SCDR), getreg32(SAM_SPIO_WPMR),
            getreg32(SAM_SPIO_WPSR), getreg32(base + SAM_SPIO_IOSSR_OFFSET),
            );
    }
  else
    {
      lldbg("   WPMR: %08x   WPSR: %08x\n",
            getreg32(SAM_PIO_WPMR), getreg32(SAM_PIO_WPSR));
    }

  irqrestore(flags);
  return OK;
}
#endif
