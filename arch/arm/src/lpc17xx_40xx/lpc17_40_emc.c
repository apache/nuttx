/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_emc.c
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

#include <arch/irq.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_emc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const lpc17_40_pinset_t g_emcctrl[] =
{
  GPIO_EMC_OE,    GPIO_EMC_WE,
  GPIO_EMC_BLS0,  GPIO_EMC_BLS1,  GPIO_EMC_BLS2,  GPIO_EMC_BLS3,
  GPIO_EMC_CS0,   GPIO_EMC_CS1,   GPIO_EMC_CS2,   GPIO_EMC_CS3,
  GPIO_EMC_CAS,   GPIO_EMC_RAS,
  GPIO_EMC_CLK0,  GPIO_EMC_CLK1,
  GPIO_EMC_DYCS0, GPIO_EMC_DYCS1, GPIO_EMC_DYCS2, GPIO_EMC_DYCS3,
  GPIO_EMC_CKE0,  GPIO_EMC_CKE1,  GPIO_EMC_CKE2,  GPIO_EMC_CKE3,
  GPIO_EMC_DQM0,  GPIO_EMC_DQM1,  GPIO_EMC_DQM2,  GPIO_EMC_DQM3,
};
#define EMC_NCTRL (sizeof(g_emcctrl) / sizeof(lpc17_40_pinset_t))

static const lpc17_40_pinset_t g_emcdata[] =
{
  GPIO_EMC_D0,    GPIO_EMC_D1,    GPIO_EMC_D2,    GPIO_EMC_D3,
  GPIO_EMC_D4,    GPIO_EMC_D5,    GPIO_EMC_D6,    GPIO_EMC_D7,
  GPIO_EMC_D8,    GPIO_EMC_D9,    GPIO_EMC_D10,   GPIO_EMC_D11,
  GPIO_EMC_D12,   GPIO_EMC_D13,   GPIO_EMC_D14,   GPIO_EMC_D15,
  GPIO_EMC_D16,   GPIO_EMC_D17,   GPIO_EMC_D18,   GPIO_EMC_D19,
  GPIO_EMC_D20,   GPIO_EMC_D21,   GPIO_EMC_D22,   GPIO_EMC_D23,
  GPIO_EMC_D24,   GPIO_EMC_D25,   GPIO_EMC_D26,   GPIO_EMC_D27,
  GPIO_EMC_D28,   GPIO_EMC_D29,   GPIO_EMC_D30,   GPIO_EMC_D31,
};

/* You can limit the number of data lines configured by defining
 * BOARD_NDATA in your board.h header file.
 */

#ifdef BOARD_NDATA
#  define EMC_NDATA BOARD_NDATA
#else
#  define EMC_NDATA (sizeof(g_emcdata) / sizeof(lpc17_40_pinset_t))
#endif

static const lpc17_40_pinset_t g_emcaddr[] =
{
  GPIO_EMC_A0,    GPIO_EMC_A1,    GPIO_EMC_A2,    GPIO_EMC_A3,
  GPIO_EMC_A4,    GPIO_EMC_A5,    GPIO_EMC_A6,    GPIO_EMC_A7,
  GPIO_EMC_A8,    GPIO_EMC_A9,    GPIO_EMC_A10,   GPIO_EMC_A11,
  GPIO_EMC_A12,   GPIO_EMC_A13,   GPIO_EMC_A14,   GPIO_EMC_A15,
  GPIO_EMC_A16,   GPIO_EMC_A17,   GPIO_EMC_A18,   GPIO_EMC_A19,
  GPIO_EMC_A20,   GPIO_EMC_A21,   GPIO_EMC_A22,   GPIO_EMC_A23,
  GPIO_EMC_A24,   GPIO_EMC_A25
};

/* You can limit the number of address lines configured by defining
 * BOARD_NADDR in your board.h header file.
 */

#ifdef BOARD_NADDR
#  define EMC_NADDR BOARD_NADDR
#else
#  define EMC_NADDR (sizeof(g_emcaddr) / sizeof(lpc17_40_pinset_t))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:
 *   lpc17_40_running_from_sdram
 *
 * Descriptions:
 *   Check whether currently execution from SDRAM.
 *
 * Returned value:
 *   1 running from SDRAM, otherwise 0
 *
 ****************************************************************************/

static int lpc17_40_running_from_sdram(void)
{
  uint32_t extdram_bank_size = LPC17_40_EXTDRAM_CS3 - LPC17_40_EXTDRAM_CS2;
  uint32_t extdram_end       = LPC17_40_EXTDRAM_CS3 + extdram_bank_size;

  if (((uint32_t)lpc17_40_running_from_sdram >= LPC17_40_EXTDRAM_CS0) &&
      ((uint32_t)lpc17_40_running_from_sdram < extdram_end))
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_emcinitialize
 *
 * Description:
 *   Initialize EMC clocking and pin configuration.  This function should be
 *   called once when the system first boots in order to make the EMC
 *   operational.
 *
 ****************************************************************************/

void lpc17_40_emcinitialize(void)
{
  uint32_t regval;
  int i;

  /* Enable clocking for the EMC */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCEMC;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Set EMC delay values:
   *
   * CMDDLY: Programmable delay value for EMC outputs in command delayed
   *   mode.  The delay amount is roughly CMDDLY * 250 picoseconds.
   * FBCLKDLY: Programmable delay value for the feedback clock that controls
   *   input data sampling.  The delay amount is roughly (FBCLKDLY+1) * 250
   *   picoseconds.
   * CLKOUT0DLY: Programmable delay value for the CLKOUT0 output. This would
   *   typically be used in clock delayed mode.  The delay amount is roughly
   *  (CLKOUT0DLY+1) * 250 picoseconds.
   * CLKOUT1DLY: Programmable delay value for the CLKOUT1 output. This would
   *  typically be used in clock delayed mode.  The delay amount is roughly
   *  (CLKOUT1DLY+1) * 250 picoseconds.
   */

  if (lpc17_40_running_from_sdram())
    {
      return;
    }

  regval = SYSCON_EMCDLYCTL_CMDDLY(BOARD_CMDDLY) |
           SYSCON_EMCDLYCTL_FBCLKDLY(BOARD_FBCLKDLY) |
           SYSCON_EMCDLYCTL_CLKOUT0DLY(BOARD_CLKOUT0DLY) |
           SYSCON_EMCDLYCTL_CLKOUT1DLY(BOARD_CLKOUT1DLY);
  putreg32(regval, LPC17_40_SYSCON_EMCDLYCTL);

  /* Enable the EMC */

  putreg32(EMC_CONTROL_E, LPC17_40_EMC_CONTROL);
  putreg32(0, LPC17_40_EMC_CONFIG);

  /* Configure EMC pins */

  /* Control signals */

  for (i = 0; i < EMC_NCTRL; i++)
    {
      lpc17_40_configgpio(g_emcctrl[i]);
    }

  /* Data lines */

  for (i = 0; i < EMC_NDATA; i++)
    {
      lpc17_40_configgpio(g_emcdata[i]);
    }

  /* Address lines */

  for (i = 0; i < EMC_NADDR; i++)
    {
      lpc17_40_configgpio(g_emcaddr[i]);
    }
}
