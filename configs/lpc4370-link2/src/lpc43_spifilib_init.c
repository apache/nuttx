/****************************************************************************
 * configs/lpc4370-link2/src/lpc43_spifilib_init.c
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lpc43_spifi.h"
#include "lpc43_cgu.h"

#include "lpc4370-link2.h"

#include "spifi/inc/spifilib_api.h"

#ifdef CONFIG_SPIFI_LIBRARY

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Local memory, 32-bit aligned that will be used for driver context
 * (handle).
 */

static uint32_t lmem[21];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_spifi_initialize(void)
{
  irqstate_t flags = enter_critical_section();
  uint32_t regval;

  flags = enter_critical_section();

  /* Initial frequency is set by boot ROM in IDIVE */

  /* Pin configuration */

  lpc43_pin_config(PINCONF_SPIFI_CS);
  lpc43_pin_config(PINCONF_SPIFI_MISO);
  lpc43_pin_config(PINCONF_SPIFI_MOSI);
  lpc43_pin_config(PINCONF_SPIFI_SCK);
  lpc43_pin_config(PINCONF_SPIFI_SIO2);
  lpc43_pin_config(PINCONF_SPIFI_SIO3);

  /* Initialize LPCSPIFILIB library, reset the interface */

  spifiInit(LPC43_SPIFI_CTRL, true);

  /* Register the family for the device */

  spifiRegisterFamily(spifi_REG_FAMILY_CommonCommandSet);

  /* Initialize and detect a device and get device context */

  SPIFI_HANDLE_T* pSpifi = spifiInitDevice(&lmem, sizeof(lmem),
                                           LPC43_SPIFI_CTRL,
                                           LPC43_LOCSRAM_SPIFI_BASE);

  /* Enable quad.  If not supported it will be ignored */

  spifiDevSetOpts(pSpifi, SPIFI_OPT_USE_QUAD, true);

  /* Enter memMode */

  spifiDevSetMemMode(pSpifi, true);

  /* Configure divider as the input to the SPIFI */

  regval  = getreg32(LPC43_BASE_SPIFI_CLK);
  regval &= ~BASE_SPIFI_CLK_CLKSEL_MASK;
  regval |= BASE_SPIFI_CLKSEL_IDIVE;
  putreg32(regval, LPC43_BASE_SPIFI_CLK);

  regval  = getreg32(LPC43_IDIVE_CTRL);
  regval &= ~(IDIVE_CTRL_CLKSEL_MASK | IDIVE_CTRL_IDIV_MASK);
  regval |= BASE_SPIFI_CLKSEL_PLL1 | IDIVE_CTRL_AUTOBLOCK |
            IDIVE_CTRL_IDIV(SPIFI_DEVICE_REQUENCY_DIVIDER);
  putreg32(regval, LPC43_IDIVE_CTRL);

  leave_critical_section(flags);
}

#endif /* CONFIG_SPIFI_LIBRARY */
