/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/src/lpc31_mem.c
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

/* Based on the EA3131 SDRAM initialization logic with adjustments to the
 * timing parameters taken from Olimex LPC-H3131 sample code.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lpc31_syscreg.h"
#include "lpc31_cgudrvr.h"
#include "lpc31_mpmc.h"
#include "lpc_h3131.h"

#ifdef CONFIG_LPC31_EXTDRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* My LPC-H3131 is fitted with a Samsung K4S561632J-UC/L75 256Mbit DRAM.
 * The FLASH organization is 16Mbit x 16
 */

/* Command, address, and data delay (DEL2) */

#define H3131_MPMC_DELAY          ((0x00 << SYSCREG_MPMC_DELAYMODES_DEL1_SHIFT) | \
                                   (0x20 << SYSCREG_MPMC_DELAYMODES_DEL2_SHIFT) | \
                                   (0x24 << SYSCREG_MPMC_DELAYMODES_DEL3_SHIFT))

/* Delay constants in nanosecondss for K4S561632J-UC/L75 SDRAM on board */

/* 90MHz SDRAM Clock */

#define H3131_SDRAM_REFRESH        (15625)

#define H3131_SDRAM_TRP            (20)  /* ns */
#define H3131_SDRAM_TRAS           (48)  /* ns */
#define H3131_SDRAM_TAPR           (2)   /* clocks */
#define H3131_SDRAM_TWR            (15)  /* ns */
#define H3131_SDRAM_TRC            (72)  /* ns */
#define H3131_SDRAM_TRFC           (80)  /* ns */
#define H3131_SDRAM_TREX           (80)  /* ns */
#define H3131_SDRAM_TXSR           (80)  /* ns */
#define H3131_SDRAM_TDAL           (5)   /* clocks */
#define H3131_SDRAM_TRRD           (2)   /* clocks */
#define H3131_SDRAM_TMRD           (2)   /* clocks */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_ns2clk
 *
 * Description:
 *   Convert nanoseconds to units of HCLK clocks
 *
 ****************************************************************************/

static uint32_t lpc31_ns2clk(uint32_t ns, uint32_t hclk2)
{
  /* delay (ns) * hclk (Hz) / scale (ns/sec) = cycles
   *
   * Example: ns=80, hclk2=90000000
   * clocks = 80 * 90000000 / 1000000000 + 1 = 8 (actual 7.2 cycles)
   */

  uint64_t tmp = (uint64_t)ns * (uint64_t)hclk2 / 1000000000ull;
  if (tmp > 0)
    {
      tmp++;
    }

  return (uint32_t)tmp;
}

/****************************************************************************
 * Name: lpc31_sdraminitialize
 *
 * Description:
 *   Configure SDRAM on the Olimex LPC-H3131 board
 *
 *   My LPC-H3131 is fitted with a Samsung K4S561632J-UC/L75 256Mbit DRAM.
 *   The FLASH organization is 16Mbit x 16
 *
 ****************************************************************************/

static inline void lpc31_sdraminitialize(void)
{
  uint32_t regval;

  /* These run-time calculations can be reduced dramatically if hclk is
   * replaced with an apriori value.
   */

#ifdef CONFIG_LPC31_SDRAMHCLK
#  define HCLK CONFIG_LPC31_SDRAMHCLK
#else
  uint32_t hclk = lpc31_clkfreq(CLKID_MPMCCFGCLK2, DOMAINID_SYS);
#  define HCLK hclk
#endif

  /* Check RTL for divide by 2 possible.
   * If so change then enable the following logic
   */

#if 0
  uint32_t hclk2 = hclk;

  if (((getreg32(LPC31_MPMC_CONFIG) & MPMC_CONFIG_CLK)) != 0)
    {
      hclk2 >>= 1;
    }
#  define HCLK2 hclk2
#else
#  define HCLK2 hclk
#endif
  up_udelay(100);

  /* Set command delay strategy */

  putreg32(MPMC_DYNREADCONFIG_CMDDEL, LPC31_MPMC_DYNREADCONFIG);

  /* Configure device config register nSDCE0 for proper width SDRAM:
   * Type: 256Mb (16Mx16), 4 banks, row length=13, column length=9
   * Buffer disabled, writes not protected.
   */

  putreg32((MPMC_DYNCONFIG0_MDSDRAM | MPMC_DYNCONFIG_HP16_16MX16),
           LPC31_MPMC_DYNCONFIG0);

  /* Disable buffers + writes not protected */

  regval  = getreg32(LPC31_MPMC_DYNCONFIG0);
  regval &= ~(MPMC_DYNCONFIG0_B | MPMC_DYNCONFIG0_P);
  putreg32(regval, LPC31_MPMC_DYNCONFIG0);

  /* Set RAS/CAS delays */

  putreg32((MPMC_DYNRASCAS0_RAS2CLK | MPMC_DYNRASCAS0_CAS2CLK),
           LPC31_MPMC_DYNRASCAS0);

  /* Configure SDRAM timing */

  putreg32(lpc31_ns2clk(H3131_SDRAM_TRP,  HCLK2), LPC31_MPMC_DYNTRP);
  putreg32(lpc31_ns2clk(H3131_SDRAM_TRAS, HCLK2), LPC31_MPMC_DYNTRAS);
  putreg32(lpc31_ns2clk(H3131_SDRAM_TREX, HCLK2), LPC31_MPMC_DYNTSREX);
  putreg32(H3131_SDRAM_TAPR, LPC31_MPMC_DYNTAPR);
  putreg32(H3131_SDRAM_TDAL + lpc31_ns2clk(H3131_SDRAM_TRP, HCLK2),
           LPC31_MPMC_DYNTDAL);
  putreg32(lpc31_ns2clk(H3131_SDRAM_TWR,  HCLK2), LPC31_MPMC_DYNTWR);
  putreg32(lpc31_ns2clk(H3131_SDRAM_TRC,  HCLK2), LPC31_MPMC_DYNTRC);
  putreg32(lpc31_ns2clk(H3131_SDRAM_TRFC, HCLK2), LPC31_MPMC_DYNTRFC);
  putreg32(lpc31_ns2clk(H3131_SDRAM_TXSR, HCLK2), LPC31_MPMC_DYNTXSR);
  putreg32(H3131_SDRAM_TRRD, LPC31_MPMC_DYNTRRD);
  putreg32(H3131_SDRAM_TMRD, LPC31_MPMC_DYNTMRD);

  /* JEDEC General SDRAM Initialization Sequence DELAY to allow power and
   * clocks to stabilize ~100 us NOP
   */

  up_udelay(100);

  /* Issue continuous NOP commands  */

  putreg32((MPMC_DYNCONTROL_CE | MPMC_DYNCONTROL_CS |
            MPMC_DYNCONTROL_INOP), LPC31_MPMC_DYNCONTROL);

  /* Wait ~200us */

  up_udelay(200);

  /* Issue a "pre-charge all" command */

  putreg32((MPMC_DYNCONTROL_CE | MPMC_DYNCONTROL_CS |
            MPMC_DYNCONTROL_IPALL), LPC31_MPMC_DYNCONTROL);

  /* Minimum refresh pulse interval (tRFC) for MT48LC32M16A2=80nsec,
   * 100nsec provides more than adequate interval.
   */

  putreg32(1, LPC31_MPMC_DYNREFRESH);

  /* Wait ~250us */

  up_udelay(250);

  /* Recommended refresh interval for normal operation of the Micron
   * MT48LC16LFFG = 7.8125usec (128KHz rate). ((HCLK / 128000) - 1) =
   * refresh counter interval rate, (subtract one for safety margin).
   *
   * REVISIT:  Is this okay for the Samsung part?
   */

  putreg32(lpc31_ns2clk(H3131_SDRAM_REFRESH, HCLK) >> 4,
           LPC31_MPMC_DYNREFRESH);

  /* Select mode register update mode */

  putreg32((MPMC_DYNCONTROL_CE | MPMC_DYNCONTROL_CS |
            MPMC_DYNCONTROL_IMODE), LPC31_MPMC_DYNCONTROL);

  /* Program the SDRAM internal mode registers on bank nSDCE0 and reconfigure
   * the SDRAM chips.
   * Bus speeds up to 90MHz requires use of a CAS latency = 2.
   * To get correct value on address bus CAS cycle, requires a shift by 12
   * for 16bit mode
   */

  getreg32(LPC31_EXTSDRAM0_VSECTION | (0x23 << 12));

  /* Select normal operating mode */

  putreg32(MPMC_DYNCONTROL_INORMAL, LPC31_MPMC_DYNCONTROL);

  /* Enable buffers */

  regval  = getreg32(LPC31_MPMC_DYNCONFIG0);
  regval |= MPMC_DYNCONFIG0_B;
  putreg32(regval, LPC31_MPMC_DYNCONFIG0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_meminitialize
 *
 * Description:
 *   Initialize external memory resources (sram, sdram, nand, nor, etc.)
 *
 ****************************************************************************/

void lpc31_meminitialize(void)
{
  /* Configure the LCD pins in external bus interface (EBI/MPMC) memory mode.
   *
   * LCD_CSB   -> MPMC_NSTCS_0
   * LCD_DB_1  -> MPMC_NSTCS_1
   * LCD_DB_0  -> MPMC_CLKOUT
   * LCD_E_RD  -> MPMC_CKE
   * LCD_RS    -> MPMC_NDYCS
   * LCD_RW_WR -> MPMC_DQM_1
   * LCD_DB_2  -> EBI_A_2
   * LCD_DB_3  -> EBI_A_3 l
   * LCD_DB_4  -> EBI_A_4 l
   * LCD_DB_5  -> EBI_A_5 l
   * LCD_DB_6  -> EBI_A_6
   * LCD_DB_7  -> EBI_A_7
   * LCD_DB_8  -> EBI_A_8
   * LCD_DB_9  -> EBI_A_9
   * LCD_DB_10 -> EBI_A_10
   * LCD_DB_11 -> EBI_A_11
   * LCD_DB_12 -> EBI_A_12
   * LCD_DB_13 -> EBI_A_13
   * LCD_DB_14 -> EBI_A_14
   * LCD_DB_15 -> EBI_A_15
   */

  putreg32(SYSCREG_MUX_LCDEBISEL_EBIMPMC, LPC31_SYSCREG_MUX_LCDEBISEL);

  /* Enable EBI clock */

  lpc31_enableclock(CLKID_EBICLK);

  /* Enable MPMC controller clocks */

  lpc31_enableclock(CLKID_MPMCCFGCLK);
  lpc31_enableclock(CLKID_MPMCCFGCLK2);
  lpc31_enableclock(CLKID_MPMCCFGCLK3);

  /* Enable the external memory controller */

  putreg32(MPMC_CONTROL_E, LPC31_MPMC_CONTROL);

  /* Force HCLK to MPMC_CLK to 1:1 ratio, little-endian mode */

  putreg32(0, LPC31_MPMC_CONFIG);

  /* Set MPMC delay based on trace lengths between SDRAM and the chip
   * and on the delay strategy used for SDRAM.
   */

  putreg32(H3131_MPMC_DELAY, LPC31_SYSCREG_MPMC_DELAYMODES);

  /* Configure Samsung K4S561632J-UC/L75 DRAM on the H3131 board */

  lpc31_sdraminitialize();
}
#endif /* CONFIG_LPC31_EXTDRAM */
