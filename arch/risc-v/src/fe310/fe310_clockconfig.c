/****************************************************************************
 * arch/risc-v/src/fe310/fe310_clockconfig.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "fe310_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EXT_OSC 16000000

#define PLL_64M  (1 + (31 << 4) + (3 << 10)) /* R:2 F:64 Q:8 8M/512M/64M  */
#define PLL_128M (1 + (31 << 4) + (2 << 10)) /* R:2 F:64 Q:4 8M/512M/128M */
#define PLL_256M (1 + (31 << 4) + (1 << 10)) /* R:2 F:64 Q:2 8M/512M/256M */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fe310_get_hfclk
 ****************************************************************************/

uint32_t fe310_get_hfclk(void)
{
  uint32_t val;
  uint32_t freq = 0;

  /* Check pllbypass */

  if (0 != (getreg32(FE310_PLLCFG) & PLLCFG_PLLBYPASS))
    {
      freq = EXT_OSC;
      goto out;
    }

  /* Check pllsel */

  if (0 != ((val = getreg32(FE310_PLLCFG)) & PLLCFG_PLLSEL))
    {
      freq = EXT_OSC;
      freq /= ((val & 0x3) + 1);               /* R: 2bit */
      freq *= ((((val >> 4) & 0x3f) + 1) * 2); /* F: 6bit */
      freq /= (1 << ((val >> 10) & 0x3));      /* Q: 2bit */
      goto out;
    }

  /* TODO: HFROSC */

  PANIC();

out:
  return freq;
}

/****************************************************************************
 * Name: fe310_clockconfig
 ****************************************************************************/

void fe310_clockconfig(void)
{
  uint32_t val;
  uint32_t pllsel;

  /* NOTE: These are workarounds to avoid a bug with debugger */

  pllsel   = 0x1;
  pllsel <<= 16;

  /* Disable PLL by setting pllbypass and clear pllsel */

  modifyreg32(FE310_PLLCFG, pllsel, PLLCFG_PLLBYPASS);

  /* Enable HFXOSC (external Xtal OSC 16MHz) and wait */

  putreg32(HFXOSCCFG_HFXOSCEN, FE310_HFXOSCCFG);

  while ((getreg32(FE310_HFXOSCCFG) & HFXOSCCFG_HFXOSCRDY)
         != HFXOSCCFG_HFXOSCRDY)
    {
    }

  val  = PLL_256M;
  val |= (PLLCFG_PLLREFSEL);   /* Set PLLREFSEL (XOSCOUT) */
  val |= PLLCFG_PLLBYPASS;     /* But Still disable PLL */

  putreg32(val, FE310_PLLCFG); /* Set PLL config */

  /* Set plloutdiv to pass-through */

  putreg32(0x1 << 8, FE310_PLLOUTDIV);

  /* Enable PLL by clearing pllbypass and wait */

  modifyreg32(FE310_PLLCFG, PLLCFG_PLLBYPASS, 0);

  while ((getreg32(FE310_PLLCFG) & PLLCFG_PLLLOCK) == 0x0)
    {
    }

  /* TODO: Set QSPI divider if needed */

  /* Select PLL as hfclk */

  modifyreg32(FE310_PLLCFG, 0, pllsel);
}
