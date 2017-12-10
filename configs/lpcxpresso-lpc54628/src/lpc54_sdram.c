/****************************************************************************
 * configs/lpcxpresso-lpc54628/src/lpc54_bringup.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "lpc54_emc.h.h"
#include "lpcxpresso-lpc54628.h"

#include <arch/board/board.h>

#ifdef CONFIG_LPC54_EMC

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* EMC basic configuration. */

static const struct emc_config_s g_emc_config =
{
  .bigendian = false,           /* Little endian */
  .clksrc    = EMC_INTLOOPBACK; /* Internal loop back from EMC_CLK output */
#ifdef BOARD_220MHz
  .clkdiv    = 3;               /* EMC Clock = CPU FREQ/3 */
#else /* if BOARD_180MHz */
  .clkdiv    = 2;               /* EMC Clock = CPU FREQ/2 */
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_sdram_initialize
 *
 * Description:
 *   Initialize external SDRAM
 *
 ****************************************************************************/

void lpc54_sdram_initialize(void)
{
  /* Dynamic memory timing configuration. */
#warning Missing logic

  /* Dynamic memory chip specific configuration: Chip 0 - MTL48LC8M16A2B4-6A */
#warning Missing logic

  /* EMC Basic configuration. */

  lpc54_emc_initialize(EMC, &g_emc_config);

  /* EMC Dynamc memory configuration. */
#warning Missing logic
}

#endif /* CONFIG_LPC54_EMC */
