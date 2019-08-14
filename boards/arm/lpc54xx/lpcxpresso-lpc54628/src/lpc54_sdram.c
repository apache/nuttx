/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_bringup.c
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

#include "hardware/lpc54_pinmux.h"
#include "lpc54_gpio.h"
#include "lpc54_emc.h"
#include "lpcxpresso-lpc54628.h"

#include <arch/board/board.h>

#if defined(CONFIG_LPC54_EMC) && defined(CONFIG_LPC54_EMC_DYNAMIC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EMC_CLOCK_PERIOD_NS  (1000000000 / BOARD_EMC_FREQUENCY)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* EMC basic configuration. */

static const struct emc_config_s g_emc_config =
{
  .bigendian = false,           /* Little endian */
  .clksrc    = EMC_INTLOOPBACK, /* Internal loop back from EMC_CLK output */
#ifdef BOARD_220MHz
  .clkdiv    = 3,               /* EMC Clock = CPU FREQ/3 */
#else /* if BOARD_180MHz */
  .clkdiv    = 2,               /* EMC Clock = CPU FREQ/2 */
#endif
};

/* Pin configuration */

static const lpc54_pinset_t g_emc_pinset[] =
{
  /* Control signals */

  GPIO_EMC_CASN, GPIO_EMC_RASN,   GPIO_EMC_WEN,  GPIO_EMC_CLK0,
  GPIO_EMC_CKE0, GPIO_EMC_DYCSN0, GPIO_EMC_DQM0, GPIO_EMC_DQM1,

  /* Address lines */

  GPIO_EMC_A0,  GPIO_EMC_A1,  GPIO_EMC_A2,  GPIO_EMC_A3,  GPIO_EMC_A4,
  GPIO_EMC_A5,  GPIO_EMC_A6,  GPIO_EMC_A7,  GPIO_EMC_A8,  GPIO_EMC_A9,
  GPIO_EMC_A10, GPIO_EMC_A11, GPIO_EMC_A12, GPIO_EMC_A13, GPIO_EMC_A14,

  /* Data lines */

  GPIO_EMC_D0,  GPIO_EMC_D1,  GPIO_EMC_D2,  GPIO_EMC_D3,  GPIO_EMC_D4,
  GPIO_EMC_D5,  GPIO_EMC_D6,  GPIO_EMC_D7,  GPIO_EMC_D8,  GPIO_EMC_D9,
  GPIO_EMC_D10, GPIO_EMC_D11, GPIO_EMC_D12, GPIO_EMC_D13, GPIO_EMC_D14,
  GPIO_EMC_D15
};

#define EMC_NPINS (sizeof(g_emc_pinset) / sizeof(lpc54_pinset_t))

/* Dynamic memory timing configuration. */

static const struct emc_dynamic_timing_config_s g_emc_dynconfig =
{
  .rdconfig = EMC_CMDDELAY,
  .refresh  = (64 * 1000000 / 4096),   /* 4096 rows/ 64ms */
  .rp       = 18,
  .ras      = 42,
  .srex     = 67,
  .apr      = 18,
  .wr       = EMC_CLOCK_PERIOD_NS + 6, /* one clk + 6ns */
  .dal      = EMC_CLOCK_PERIOD_NS + 24,
  .rc       = 60,
  .rfc      = 60,
  .xsr      = 67,
  .rrd      = 12,
  .mrd      = 2,
};

/* Dynamic memory chip specific configuration: Chip 0 - MTL48LC8M16A2B4-6A */

static const struct emc_dynamic_chip_config_s g_emc_dynchipconfig =
{
  .chndx    = 0,
  .dyndev   = EMC_SDRAM,
  .rasnclk  = 2,
  .mode     = 0x23,
  .extmode  = 0,    /* SDRAM only */
  .addrmap  = 0x09, /* 128Mbits (8M*16, 4banks, 12 rows, 9 columns) */
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
  int i;

  /* EMC Basic configuration. */

  lpc54_emc_initialize(&g_emc_config);

  /* Configured pins used on the board */

  for (i = 0; i < EMC_NPINS; i++)
    {
      lpc54_gpio_config(g_emc_pinset[i]);
    }

  /* EMC Dynamc memory configuration. */

  lpc54_emc_sdram_initialize(&g_emc_dynconfig, &g_emc_dynchipconfig, 1);
}

#endif /* CONFIG_LPC54_EMC && CONFIG_LPC54_EMC_DYNAMIC */
