/****************************************************************************
 * arch/arm/src/sama5/sama5d3x4x_pio.c
 * General Purpose Input/Output (PIO) logic for the SAMA5D3x and SAMA5D4x
 *
 *   Copyright (C) 2013-2014, 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/_sama5d3x4x_pio.h"

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
 * Public Data
 ****************************************************************************/

/* Lookup for (non-secure) PIOs */

const uintptr_t g_piobase[SAM_NPIO] =
{
  SAM_PIOA_VBASE
#if SAM_NPIO > 1
  , SAM_PIOB_VBASE
#endif
#if SAM_NPIO > 2
  , SAM_PIOC_VBASE
#endif
#if SAM_NPIO > 3
  , SAM_PIOD_VBASE
#endif
#if SAM_NPIO > 4
  , SAM_PIOE_VBASE
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Maps a port number to the standard port character */

#if defined(CONFIG_DEBUG_GPIO_INFO) && SAM_NPIO > 0
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

/* Map a PIO number to the PIO peripheral identifier (PID) */

#if SAM_NPIO > 0
static const uint8_t g_piopid[SAM_NPIO] =
{
  SAM_PID_PIOA
#if SAM_NPIO > 1
  , SAM_PID_PIOB
#endif
#if SAM_NPIO > 2
  , SAM_PID_PIOC
#endif
#if SAM_NPIO > 3
  , SAM_PID_PIOD
#endif
#if SAM_NPIO > 4
  , SAM_PID_PIOE
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

static inline uintptr_t sam_piobase(pio_pinset_t cfgset)
{
  int port = (cfgset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;

  if (port < SAM_NPIO)
    {
      return sam_pion_vbase(port);
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
 *   Return a bitmask corresponding to the bit position in a PIO register
 *
 ****************************************************************************/

static inline uint32_t sam_piopin(pio_pinset_t cfgset)
{
  return 1 << ((cfgset & PIO_PIN_MASK) >> PIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: sam_pio_enableclk
 *
 * Description:
 *   Enable clocking on the selected PIO
 *
 ****************************************************************************/

static void sam_pio_enableclk(pio_pinset_t cfgset)
{
  int port = (cfgset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;
  int pid;

  if (port < SAM_NPIO)
    {
      /* Get the peripheral ID associated with the PIO port and enable
       * clocking to the PIO block.
       */

      pid = g_piopid[port];
      if (pid < 32)
        {
          sam_enableperiph0(pid);
        }
      else
        {
          sam_enableperiph1(pid);
        }
    }
}

/****************************************************************************
 * Name: sam_pio_disableclk
 *
 * Description:
 *   Disable clocking on the selected PIO if we can.  We can that if:
 *
 *   1) No pins are configured as PIO inputs (peripheral inputs don't need
 *      clocking, and
 *   2) Glitch and debounce filtering are not enabled.  Currently, this can
 *      only happen if the pin is a PIO input, but we may need to
 *      implement glitch filtering on peripheral inputs as well in the
 *      future???
 *   3) The port is not configured for PIO interrupts.  At present, the logic
 *      always keeps clocking on to ports that are configured for interrupts,
 *      but that could be dynamically controlled as well be keeping track
 *      of which PIOs have interrupts enabled.
 *
 * My!  Wouldn't is be much easier to just keep all of the PIO clocks
 * enabled?  Is there a power management downside?
 *
 ****************************************************************************/

static void sam_pio_disableclk(pio_pinset_t cfgset)
{
  int port = (cfgset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;
  uintptr_t base;
  int pid;

  /* Leave clocking enabled for configured interrupt ports or for ports that
   * have forced enabling of PIO clocking.
   */

  if (port < SAM_NPIO && !g_piointerrupt[port] && g_forced[port] == 0)
    {
      /* Get the base address of the PIO port */

      base = sam_pion_vbase(port);

      /* Are any pins configured as PIO inputs?
       *
       * PSR - A bit set to "1" means that the corresponding pin is a PIO
       * OSR - A bit set to "1" means that the corresponding pin is an output
       */

      if ((getreg32(base + SAM_PIO_PSR_OFFSET) &
           ~getreg32(base + SAM_PIO_PSR_OFFSET)) == 0)
        {
          /* Any remaining configured pins are either not PIOs or all not
           * PIO inputs.  Disable clocking to this PIO block.
           *
           * Get the peripheral ID associated with the PIO port and disable
           * clocking to the PIO block.
           */

          pid = g_piopid[port];
          if (pid < 32)
            {
              sam_disableperiph0(pid);
            }
          else
            {
              sam_disableperiph1(pid);
            }
        }
    }
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
#ifdef PIO_HAVE_PULLDOWN
      /* The pull-up on a pin can not be enabled if its pull-down is still
       * active. Therefore, we need to disable the pull-down first before
       * enabling the pull-up.
       */

      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
#endif
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
      /* The pull-down on a pin can not be enabled if its pull-up is still
       * active. Therefore, we need to disable the pull-up first before
       * enabling the pull-down.
       */

      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
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
  /* Enable/disable the Schmitt trigger:  Zero enables.  Schmitt triggered
   * inputs are enabled by default.
   */

  regval = getreg32(base + SAM_PIO_SCHMITT_OFFSET);
  if ((cfgset & PIO_CFG_SCHMITT) != 0)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
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

  /* Clear some output only bits.  Mostly this just simplifies debug. */

  putreg32(pin, base + SAM_PIO_MDDR_OFFSET);
  putreg32(pin, base + SAM_PIO_CODR_OFFSET);

  /* Configure the pin as an input and enable the PIO function */

  putreg32(pin, base + SAM_PIO_ODR_OFFSET);
  putreg32(pin, base + SAM_PIO_PER_OFFSET);

  /* To-Do:  If DEGLITCH is selected, need to configure DIFSR, SCIFSR, and
   *         IFDGSR registers.  This would probably best be done with
   *         another, new API... perhaps sam_configfilter()
   */

  /* "Reading the I/O line levels requires the clock of the PIO Controller
   * to be enabled, otherwise PIO_PDSR reads the levels present on the I/O
   * line at the time the clock was disabled."
   */

  sam_pio_enableclk(cfgset);
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
#ifdef PIO_HAVE_PULLDOWN
      /* The pull-up on a pin can not be enabled if its pull-down is still
       * active. Therefore, we need to disable the pull-down first before
       * enabling the pull-up.
       */

      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
#endif
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
      /* The pull-down on a pin can not be enabled if its pull-up is still
       * active. Therefore, we need to disable the pull-up first before
       * enabling the pull-down.
       */

      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Disable glitch filtering */

  putreg32(pin, base + SAM_PIO_IFDR_OFFSET);

  /* Enable the open drain driver if requested */

  if ((cfgset & PIO_CFG_OPENDRAIN) != 0)
    {
      putreg32(pin, base + SAM_PIO_MDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_MDDR_OFFSET);
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

  putreg32(pin, base + SAM_PIO_OER_OFFSET);
  putreg32(pin, base + SAM_PIO_PER_OFFSET);

  /* Clocking to the PIO block may no longer be necessary. */

  sam_pio_disableclk(cfgset);
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
#ifdef PIO_HAVE_PULLDOWN
      /* The pull-up on a pin can not be enabled if its pull-down is still
       * active. Therefore, we need to disable the pull-down first before
       * enabling the pull-up.
       */

      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
#endif
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
      /* The pull-down on a pin can not be enabled if its pull-up is still
       * active. Therefore, we need to disable the pull-up first before
       * enabling the pull-down.
       */

      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Disable glitch filtering */

  putreg32(pin, base + SAM_PIO_IFDR_OFFSET);

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

  /* Clocking to the PIO block may no longer be necessary. */

  sam_pio_disableclk(cfgset);
  return OK;
}

/****************************************************************************
 * Public Functions
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

  flags = enter_critical_section();

  /* Enable writing to PIO registers.  The following registers are protected:
   *
   *  - PIO Enable/Disable Registers (PER/PDR)
   *  - PIO Output Enable/Disable Registers (OER/ODR)
   *  - PIO Interrupt Security Level Register (ISLR)
   *  - PIO Input Filter Enable/Disable Registers (IFER/IFDR)
   *  - PIO Multi-driver Enable/Disable Registers (MDER/MDDR)
   *  - PIO Pull-Up Enable/Disable Registers (PUER/PUDR)
   *  - PIO Peripheral ABCD Select Register 1/2 (ABCDSR1/2)
   *  - PIO Output Write Enable/Disable Registers
   *  - PIO Pad Pull-Down Enable/Disable Registers (PPER/PPDR)
   *
   * I suspect that the default state is the WPMR is unprotected, so these
   * operations could probably all be avoided.
   */

  putreg32(PIO_WPMR_WPKEY, base + SAM_PIO_WPMR_OFFSET);

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
  leave_critical_section(flags);

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
       * drive the pin.  The PDSR register, on the other hand, provides
       * the current sensed value on a pin, whether the pin is configured
       * as an input, an output or as a peripheral.
       *
       * There is small delay between the setting in ODSR and PDSR but
       * otherwise they should be the same unless something external is
       * driving the pin.
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

  flags = enter_critical_section();

  /* Are we enabling or disabling clocking */

  if (enable)
    {
      /* Indicate that clocking is forced and enable the clock */

      g_forced[port] |= pin;
      sam_pio_enableclk(pinset);
    }
  else
    {
      /* Clocking is no longer forced for this pin */

      g_forced[port] &= ~pin;
      sam_pio_disableclk(pinset);
    }

  leave_critical_section(flags);
}

/************************************************************************************
 * Function:  sam_dumppio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int sam_dumppio(uint32_t pinset, const char *msg)
{
  irqstate_t    flags;
  uintptr_t     base;
  unsigned int  port;

  /* Get the base address associated with the PIO port */

  port = (pinset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;
  base = sam_pion_vbase(port);

  /* The following requires exclusive access to the PIO registers */

  flags = enter_critical_section();

  gpioinfo("PIO%c pinset: %08x base: %08x -- %s\n",
         g_portchar[port], pinset, base, msg);

#ifdef SAM_PIO_ISLR_OFFSET
  gpioinfo("    PSR: %08x   ISLR: %08x    OSR: %08x   IFSR: %08x\n",
           getreg32(base + SAM_PIO_PSR_OFFSET), getreg32(base + SAM_PIO_ISLR_OFFSET),
           getreg32(base + SAM_PIO_OSR_OFFSET), getreg32(base + SAM_PIO_IFSR_OFFSET));
#else
  gpioinfo("    PSR: %08x    OSR: %08x   IFSR: %08x\n",
           getreg32(base + SAM_PIO_PSR_OFFSET), getreg32(base + SAM_PIO_OSR_OFFSET),
           getreg32(base + SAM_PIO_IFSR_OFFSET));
#endif
  gpioinfo("   ODSR: %08x   PDSR: %08x    IMR: %08x    ISR: %08x\n",
           getreg32(base + SAM_PIO_ODSR_OFFSET), getreg32(base + SAM_PIO_PDSR_OFFSET),
           getreg32(base + SAM_PIO_IMR_OFFSET), getreg32(base + SAM_PIO_ISR_OFFSET));
  gpioinfo("   MDSR: %08x   PUSR: %08x ABDCSR: %08x %08x\n",
           getreg32(base + SAM_PIO_MDSR_OFFSET), getreg32(base + SAM_PIO_PUSR_OFFSET),
           getreg32(base + SAM_PIO_ABCDSR1_OFFSET), getreg32(base + SAM_PIO_ABCDSR2_OFFSET));
  gpioinfo(" IFSCSR: %08x   SCDR: %08x  PPDSR: %08x   OWSR: %08x\n",
           getreg32(base + SAM_PIO_IFSCSR_OFFSET), getreg32(base + SAM_PIO_SCDR_OFFSET),
           getreg32(base + SAM_PIO_PPDSR_OFFSET), getreg32(base + SAM_PIO_OWSR_OFFSET));
#ifdef SAM_PIO_LOCKSR_OFFSET
  gpioinfo("  AIMMR: %08x   ELSR: %08x FRLHSR: %08x LOCKSR: %08x\n",
           getreg32(base + SAM_PIO_AIMMR_OFFSET), getreg32(base + SAM_PIO_ELSR_OFFSET),
           getreg32(base + SAM_PIO_FRLHSR_OFFSET), getreg32(base + SAM_PIO_LOCKSR_OFFSET));
#else
  gpioinfo("  AIMMR: %08x   ELSR: %08x FRLHSR: %08x\n",
           getreg32(base + SAM_PIO_AIMMR_OFFSET), getreg32(base + SAM_PIO_ELSR_OFFSET),
           getreg32(base + SAM_PIO_FRLHSR_OFFSET));
#endif
  gpioinfo("SCHMITT: %08x DRIVER: %08x %08x\n",
           getreg32(base + SAM_PIO_SCHMITT_OFFSET), getreg32(base + SAM_PIO_DRIVER1_OFFSET),
           getreg32(base + SAM_PIO_DRIVER2_OFFSET));
  gpioinfo("   WPMR: %08x   WPSR: %08x\n",
           getreg32(base + SAM_PIO_WPMR_OFFSET), getreg32(base + SAM_PIO_WPSR_OFFSET));

  leave_critical_section(flags);
  return OK;
}
#endif
