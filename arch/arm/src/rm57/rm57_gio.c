/****************************************************************************
 * arch/arm/src/rm57/rm57_gio.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Adapted from tms570_gio.c, using RM57's 2-port GIO register layout. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_gio.h"
#include "rm57_gio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_gio_initialize
 *
 * Description:
 *   Take the GIO block out of reset and assure that it is ready for use.
 *
 ****************************************************************************/

int rm57_gio_initialize(void)
{
  /* Take the GIO block out of reset */

  putreg32(GIO_GCR0_RESET, RM57_GIO_GCR0);

  /* Disable all pin interrupts. Make sure they are all level 0. */

  putreg32(0xffffffff, RM57_GIO_ENACLR);
  putreg32(0xffffffff, RM57_GIO_LVLCLR);
  return OK;
}

/****************************************************************************
 * Name: rm57_configgio
 *
 * Description:
 *   Configure a GIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int rm57_configgio(gio_pinset_t cfgset)
{
  uint32_t port = rm57_gio_port(cfgset);
  uintptr_t base = rm57_gio_base(cfgset);
  uint32_t pin = rm57_gio_pin(cfgset);
  uint32_t pinmask = rm57_gio_pinmask(cfgset);
  uint32_t regval;
  irqstate_t flags;

  /* Disable interrupts to prohibit re-entrance. */

  flags = enter_critical_section();

  /* Force the pin to be an input for now */

  regval  = getreg32(base + RM57_GIO_DIR_OFFSET);
  regval &= ~pinmask;
  putreg32(regval, base + RM57_GIO_DIR_OFFSET);

  /* Disable interrupts on the pin. Make sure this is a level 0 pin. */

  putreg32(GIO_ENACLR_PORT_PIN(port, pin), RM57_GIO_ENACLR);
  putreg32(GIO_LVLCLR_PORT_PIN(port, pin), RM57_GIO_LVLCLR);

  /* Enable/disable the pull-up/down as requested */

  switch (cfgset & GIO_CFG_MASK)
    {
      case GIO_CFG_DEFAULT: /* Default, no attribute */
      default:
        {
          /* Disable pull functionality */

          regval  = getreg32(base + RM57_GIO_PULDIS_OFFSET);
          regval &= ~pinmask;
          putreg32(regval, base + RM57_GIO_PULDIS_OFFSET);
        }
        break;

      case GIO_CFG_PULLUP: /* Internal pull-up */
        {
          /* Select pull-up */

          regval  = getreg32(base + RM57_GIO_PSL_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + RM57_GIO_PSL_OFFSET);

          /* Enable pull functionality */

          regval  = getreg32(base + RM57_GIO_PULDIS_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + RM57_GIO_PULDIS_OFFSET);
        }
        break;

      case GIO_CFG_PULLDOWN: /* Internal pull-down */
        {
          /* Select pull-down */

          regval  = getreg32(base + RM57_GIO_PSL_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + RM57_GIO_PSL_OFFSET);

          /* Enable pull functionality */

          regval  = getreg32(base + RM57_GIO_PULDIS_OFFSET);
          regval |= pinmask;
          putreg32(regval, base + RM57_GIO_PULDIS_OFFSET);
        }
        break;
    }

  /* Then do unique operations for an output pin */

  if ((cfgset & GIO_MODE_MASK) == GIO_OUTPUT)
    {
      /* Enable the open drain driver if requested */

      regval = getreg32(base + RM57_GIO_PDR_OFFSET);
      if ((cfgset & GIO_OPENDRAIN) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= ~pinmask;
        }

      putreg32(regval, base + RM57_GIO_PDR_OFFSET);

      /* Set default output value */

      if ((cfgset & GIO_OUTPUT_SET) != 0)
        {
          putreg32(pinmask, base + RM57_GIO_DSET_OFFSET);
        }
      else
        {
          putreg32(pinmask, base + RM57_GIO_DCLR_OFFSET);
        }

      /* Finally, configure the pin as an output */

      regval  = getreg32(base + RM57_GIO_DIR_OFFSET);
      regval |= pinmask;
      putreg32(regval, base + RM57_GIO_DIR_OFFSET);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rm57_giowrite
 *
 * Description:
 *   Write one or zero to the selected GIO pin
 *
 ****************************************************************************/

void rm57_giowrite(gio_pinset_t pinset, bool value)
{
  uintptr_t base = rm57_gio_base(pinset);
  uint32_t pinmask = rm57_gio_pinmask(pinset);

  if (value)
    {
      putreg32(pinmask, base + RM57_GIO_DSET_OFFSET);
    }
  else
    {
      putreg32(pinmask, base + RM57_GIO_DCLR_OFFSET);
    }
}

/****************************************************************************
 * Name: rm57_gioread
 *
 * Description:
 *   Read one or zero from the selected GIO pin
 *
 ****************************************************************************/

bool rm57_gioread(gio_pinset_t pinset)
{
  uintptr_t base = rm57_gio_base(pinset);
  uint32_t pinmask = rm57_gio_pinmask(pinset);
  uint32_t regval;

  if ((pinset & GIO_MODE_MASK) == GIO_OUTPUT)
    {
      regval = getreg32(base + RM57_GIO_DOUT_OFFSET);
    }
  else
    {
      regval = getreg32(base + RM57_GIO_DIN_OFFSET);
    }

  return (regval & pinmask) != 0;
}
