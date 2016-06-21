/****************************************************************************
 * arch/arm/src/tms570/tms570_gio.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

#include "chip/tms570_gio.h"
#include "tms570_gio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
static const char g_portchar[TMS570_NPORTS] =
{
  'A'
#if TMS570_NPORTS > 1
  , 'B'
#endif
#if TMS570_NPORTS > 2
  , 'C'
#endif
#if TMS570_NPORTS > 3
  , 'D'
#endif
#if TMS570_NPORTS > 4
  , 'E'
#endif
#if TMS570_NPORTS > 5
  , 'F'
#endif
#if TMS570_NPORTS > 6
  , 'G'
#endif
#if TMS570_NPORTS > 7
  , 'H'
#endif
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_gio_initialize
 *
 * Description:
 *   Take the GIO block out of reset and assure that it is ready for use.
 *
 ****************************************************************************/

int tms570_gio_initialize(void)
{
  /* Take the GIO block out of reset */

  putreg32(GIO_GCR0_RESET, TMS570_GIO_GCR0);

  /* Disable all pin interrupts on the pin.  Make sure they are all level 0. */

  putreg32(0xffffffff, TMS570_GIO_ENACLR);
  putreg32(0xffffffff, TMS570_GIO_LVLCLR);
  return OK;
}

/****************************************************************************
 * Name: tms570_configgio
 *
 * Description:
 *   Configure a GIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int tms570_configgio(gio_pinset_t cfgset)
{
  uint32_t port = tms570_gio_port(cfgset);
  uintptr_t base = tms570_gio_base(cfgset);
  uint32_t pin = tms570_gio_pin(cfgset);
  uint32_t pinmask = tms570_gio_pinmask(cfgset);
  uint32_t regval;
  irqstate_t flags;

  /* Disable interrupts to prohibit re-entrance. */

  flags = enter_critical_section();

  /* Force the pin to be an input for now */

  regval  = getreg32(base + TMS570_GIO_DIR_OFFSET);
  regval &= ~pinmask;
  putreg32(regval, base + TMS570_GIO_DIR_OFFSET);

  /* Disable interrupts on the pin.  Make sure this is a level 0 pin. */

  putreg32(GIO_ENACLR_PORT_PIN(port, pin), TMS570_GIO_ENACLR);
  putreg32(GIO_LVLCLR_PORT_PIN(port, pin), TMS570_GIO_LVLCLR);

  /* Setup settings common to both input and output pins */
  /* Enable/disable the pull-up/down as requested */

  switch (cfgset & GIO_CFG_MASK)
    {
      case GIO_CFG_DEFAULT: /* Default, no attribute */
      default:
        {
          /* Disable pull functionality */

          regval  = getreg32(base + TMS570_GIO_PULDIS_OFFSET);
          regval &= ~pinmask;
          putreg32(regval, base + TMS570_GIO_PULDIS_OFFSET);
        }
        break;

      case GIO_CFG_PULLUP: /* Internal pull-up */
        {
          /* Select pull-up */

          regval  = getreg32(base + TMS570_GIO_PSL_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + TMS570_GIO_PSL_OFFSET);

          /* Enable pull functionality */

          regval  = getreg32(base + TMS570_GIO_PULDIS_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + TMS570_GIO_PULDIS_OFFSET);
        }
        break;

      case GIO_CFG_PULLDOWN: /* Internal pull-down */
        {
          /* Select pull-down */

          regval  = getreg32(base + TMS570_GIO_PSL_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + TMS570_GIO_PSL_OFFSET);

          /* Enable pull functionality */

          regval  = getreg32(base + TMS570_GIO_DIR_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + TMS570_GIO_DIR_OFFSET);
        }
        break;
    }

  /* Then do unique operations for an output pin */

  if ((cfgset & GIO_MODE_MASK) == GIO_OUTPUT)
    {
      /* Enable the open drain driver if requested */

      regval = getreg32(base + TMS570_GIO_PDR_OFFSET);
      if ((cfgset & GIO_OPENDRAIN) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= ~pinmask;
        }

      putreg32(regval, base + TMS570_GIO_PDR_OFFSET);

      /* Set default output value */

      if ((cfgset & GIO_OUTPUT_SET) != 0)
       {
          putreg32(pinmask, base + TMS570_GIO_DSET_OFFSET);
        }
      else
        {
          putreg32(pinmask, base + TMS570_GIO_DCLR_OFFSET);
        }

      /* Finally, configure the pin as an output */

      regval  = getreg32(base + TMS570_GIO_DIR_OFFSET);
      regval |= pinmask;
      putreg32(regval, base + TMS570_GIO_DIR_OFFSET);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: tms570_giowrite
 *
 * Description:
 *   Write one or zero to the selected GIO pin
 *
 ****************************************************************************/

void tms570_giowrite(gio_pinset_t pinset, bool value)
{
  uintptr_t base = tms570_gio_base(pinset);
  uint32_t pinmask = tms570_gio_pinmask(pinset);

  if (value)
    {
      putreg32(pinmask, base + TMS570_GIO_DSET_OFFSET);
    }
  else
    {
      putreg32(pinmask, base + TMS570_GIO_DCLR_OFFSET);
    }
}

/****************************************************************************
 * Name: tms570_gioread
 *
 * Description:
 *   Read one or zero from the selected GIO pin
 *
 ****************************************************************************/

bool tms570_gioread(gio_pinset_t pinset)
{
  uintptr_t base = tms570_gio_base(pinset);
  uint32_t pinmask = tms570_gio_pinmask(pinset);
  uint32_t regval;

  if ((pinset & GIO_MODE_MASK) == GIO_OUTPUT)
    {
      regval = getreg32(base + TMS570_GIO_DOUT_OFFSET);
    }
  else
    {
      regval = getreg32(base + TMS570_GIO_DIN_OFFSET);
    }

  return (regval & pinmask) != 0;
}

/************************************************************************************
 * Function:  tms570_dumpgio
 *
 * Description:
 *   Dump all GIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int tms570_dumpgio(uint32_t pinset, const char *msg)
{
  irqstate_t    flags;
  uintptr_t     base;
  unsigned int  port;

  _info("GIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);

  /* Get the base address associated with the GIO port */

  port = (pinset & GIO_PORT_MASK) >> GIO_PORT_SHIFT;
  base = TMS570_GIO_PORTBASE(port);

  /* The following requires exclusive access to the GIO registers */

  flags = enter_critical_section();

  /* Show global GIO registers */

  _info("   GCR0: %08x INTDET: %08x    POL: %08x   ENA: %08x\n",
        getreg32(TMS570_GIO_GCR0), getreg32(TMS570_GIO_INTDET),
        getreg32(TMS570_GIO_POL), getreg32(TMS570_GIO_ENASET));
  _info("    LVL: %08x    FLG: %08x   EMU1: %08x   EMU2: %08x\n",
        getreg32(TMS570_GIO_LVLSET), getreg32(TMS570_GIO_FLG),
        getreg32(TMS570_GIO_EMU1), getreg32(TMS570_GIO_EMU2));

  /* Port specific registers */

  _info("    DIR: %08x    DIN: %08x   DOUT: %08x    PDR: %08x\n",
        getreg32(base + TMS570_GIO_DIR_OFFSET), getreg32(base + TMS570_GIO_DIN_OFFSET),
        getreg32(base + TMS570_GIO_DOUT_OFFSET), getreg32(base + TMS570_GIO_PDR_OFFSET));
  _info(" PULDIS: %08x    PSL: %08x\n",
        getreg32(base + TMS570_GIO_PULDIS_OFFSET), getreg32(base + TMS570_GIO_PSL_OFFSET));

  leave_critical_section(flags);
  return OK;
}
#endif
