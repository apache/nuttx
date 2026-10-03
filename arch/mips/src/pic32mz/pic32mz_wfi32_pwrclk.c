/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_wfi32_pwrclk.c
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

/* PIC32MZ-W1 (WFI32E01) regulator and clock bring-up.
 *
 * Unlike PIC32MZ EC/EF, the W1 family does not configure its PLLs from the
 * configuration words: software must start the primary oscillator, program
 * the PLLs and switch SYSCLK itself.
 *
 * Sources are tagged the same way as hardware/pic32mzw1_pmuclk.h:
 *
 *   [DS]  PIC32MZ W1 and WFI32E01 Family Data Sheet, DS70005425P.
 *   [DFP] Microchip PIC32MZ-W_DFP 1.12.356 (Apache-2.0).
 *   [EX]  Not in [DS] or [DFP]; observed in Microchip's
 *         WFI32_Ethernet_Wi-Fi_Bridge_OOB example firmware (pmu_init.c,
 *         plib_clk.c).  Only hardware facts (register addresses, values
 *         and the order the hardware needs them in) were taken from it;
 *         the code below is an independent implementation.
 *
 * Sections whose values or ordering come from [EX] are marked "[EX]" in
 * their comments.  Everything else is derived from [DS]/[DFP].
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_CHIP_WFI32E01

#include <stdint.h>
#include <stdbool.h>
#include <sys/param.h>

#include "mips_internal.h"
#include "hardware/pic32mz_memorymap.h"
#include "hardware/pic32mz_osc.h"
#include "hardware/pic32mz_uart.h"
#include "hardware/pic32mzw1_features.h"
#include "hardware/pic32mzw1_pmuclk.h"
#include "pic32mz_wfi32_pwrclk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Busy-wait delays use the CP0 Count register, which ticks at SYSCLK/2.
 * The rate is assumed to be the final 100 MHz; before the switch to the
 * PLL the CPU runs slower, so delays only ever get longer, never shorter.
 */

#define WFI32_COUNT_PER_US      100

/* SPLL: 40 MHz POSC / 5 * 150 / 6 = 200 MHz SYSCLK.  These are the values
 * the data sheet recommends for a 200 MHz system clock [DS Reg 11-3].
 */

#define WFI32_SPLLCON \
  (PLLCON_REFDIV(5) | PLLCON_FBDIV(150) | PLLCON_POSTDIV1(6) | \
   PLLCON_BSWSEL(1))

/* EWPLL: 40 MHz POSC / 4 * 160 = 1600 MHz VCO; / 32 = 50 MHz RMII
 * reference on ETH_CLK_OUT, / 10 (CFGCON3.ETHPLLPOSTDIV2) = 160 MHz for
 * Wi-Fi.  Field layout [DS Reg 11-6]; the field values, and starting it
 * with RST and PWDN set, are [EX].  ETH_CLK_OUT is only enabled when the
 * Ethernet MAC uses it as its RMII reference clock [DS Reg 11-6, 38-2].
 */

#if defined(CONFIG_PIC32MZ_ETHERNET) && \
    !defined(CONFIG_PIC32MZ_W1_ETH_EXTREFCLK)
#  define WFI32_EWPLL_CLKOUTEN  PLLCON_CLKOUTEN
#else
#  define WFI32_EWPLL_CLKOUTEN  0
#endif

#define WFI32_EWPLLCON \
  (WFI32_EWPLL_CLKOUTEN | PLLCON_REFDIV(4) | PLLCON_FBDIV(160) | \
   PLLCON_RST | PLLCON_POSTDIV1(32) | PLLCON_PWDN | PLLCON_BSWSEL(2))

#define WFI32_CFGCON3           CFGCON3_ETHPLLPOSTDIV2(10) /* [EX] */

/* PMU interface clocks: SPI = source 2 / 5, buck = source 1 / 8, BACWD
 * cleared (field layout [DFP], values [EX]).
 */

#define WFI32_PMUCLKCTRL \
  (PMUCLKCTRL_SPICLKDIV(5) | PMUCLKCTRL_SPISRC(2) | \
   PMUCLKCTRL_BUCKCLKDIV(8) | PMUCLKCTRL_BUCKSRC(1))

/* Fallback regulator settings when the factory trim word is blank [EX] */

#define WFI32_DEF_BUCKCFG1      0x5480
#define WFI32_DEF_BUCKCFG2      0x8c28
#define WFI32_DEF_BUCKCFG3      0x00c8
#define WFI32_DEF_MLDOCFG1      0x0287
#define WFI32_DEF_MLDOCFG2      0x0280
#define WFI32_DEF_VREGTRIM      0x16161616

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Silicon variants that need different bring-up [EX: DEVID values] */

enum wfi32_silicon_e
{
  WFI32_SILICON_UNKNOWN = 0,
  WFI32_SILICON_A1,
  WFI32_SILICON_B0,
  WFI32_SILICON_G
};

struct wfi32_rfreg_s
{
  uint8_t  addr;
  uint16_t data;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Crystal oscillator analog setup written over the RF serial bridge before
 * POSC is enabled.  Two variants, selected by silicon revision [EX].
 */

static const struct wfi32_rfreg_s g_xosc_cfg_a[] =
{
  { 0x85, 0x00f2 },
  { 0x84, 0x0001 },
  { 0x1e, 0x0510 },
  { 0x82, 0x6000 },
};

static const struct wfi32_rfreg_s g_xosc_cfg_b[] =
{
  { 0x85, 0x00f0 },
  { 0x84, 0x0001 },
  { 0x1e, 0x0510 },
  { 0x82, 0x6400 },
};

static const struct wfi32_rfreg_s g_xosc_cfg_a1[] =
{
  { 0x85, 0x00f0 },
  { 0x84, 0x0001 },
  { 0x1e, 0x0510 },
  { 0x82, 0x6000 },
};

/* Applied once SYSCLK runs from the PLL: disables an input Schmitt trigger
 * in the oscillator for better noise immunity [EX].
 */

static const struct wfi32_rfreg_s g_xosc_post_a =
{
  0x85, 0x00f4
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
/* UART1 baud clock = FRC (CLK_SEL = 10b, DS Reg 24-1), BRGH = 1:
 * 8 MHz / 4 / (16 + 1) = 117647 baud (+2.1% from 115200).
 */

#  define UART_MODE_CLKSEL_MASK (3 << 17)
#  define UART_MODE_CLKSEL_FRC  (2 << 17)
#  define WFI32_TRACE_BRG       16
#  define TRACE(c)              pic32mz_wfi32_trace(c)
#  define TRACEREG(c, a) \
  do \
    { \
      pic32mz_wfi32_trace(' '); \
      pic32mz_wfi32_trace(c); \
      pic32mz_wfi32_trace('='); \
      pic32mz_wfi32_trace_hex(getreg32(a)); \
    } \
  while (0)
#else
#  define TRACE(c)
#  define TRACEREG(c, a)
#endif

static inline uint32_t wfi32_cp0_count(void)
{
  uint32_t count;

  __asm__ __volatile__("mfc0 %0, $9" : "=r"(count));
  return count;
}

static void wfi32_udelay(uint32_t usec)
{
  uint32_t start = wfi32_cp0_count();

  while (wfi32_cp0_count() - start < usec * WFI32_COUNT_PER_US)
    {
    }
}

static enum wfi32_silicon_e wfi32_silicon(void)
{
  uint32_t devid = getreg32(PIC32MZ_DEVID);

  switch ((devid & DEVID_PARTNUM_MASK) >> DEVID_PARTNUM_SHIFT)
    {
      case DEVID_PARTNUM_PIC32MZW1_A1:
        return WFI32_SILICON_A1;

      case DEVID_PARTNUM_PIC32MZW1_B0:
        return WFI32_SILICON_B0;

      case DEVID_PARTNUM_PIC32MZW1_G:
        return WFI32_SILICON_G;

      default:
        return WFI32_SILICON_UNKNOWN;
    }
}

static uint32_t wfi32_silicon_ver(void)
{
  return (getreg32(PIC32MZ_DEVID) & DEVID_VER_MASK) >> DEVID_VER_SHIFT;
}

/* A factory trim word is valid unless blank (all zeros or erased) */

static bool wfi32_trim_valid(uint32_t word)
{
  return word != 0 && word != UINT32_MAX;
}

static uint32_t wfi32_trim(uintptr_t addr, uint32_t fallback)
{
  uint32_t word = getreg32(addr);

  return wfi32_trim_valid(word) ? word : fallback;
}

/* Pack four 5-bit VREG trims (VREG1 in the top byte) into the
 * PMUMODECTRL/PMUOVERCTRL field positions; the trim word already uses the
 * same byte layout.
 */

static uint32_t wfi32_vreg_fields(uint32_t trim)
{
  return trim & PMUMODE_VREGALL_MASK;
}

/* PMU regulator-internal register access through PMUSPICTRL/PMUSPISTAT
 * [DFP fields].  The example waits 5 ms before polling SPIRDY [EX]; 5 us
 * (longer while the CPU still runs from the FRC) was verified on B0
 * silicon and avoids adding most of a second to the boot.
 */

static uint32_t wfi32_pmu_xfer(uint32_t ctrl)
{
  uint32_t status;

  putreg32(ctrl, PIC32MZ_PMUSPICTRL);
  wfi32_udelay(5);

  do
    {
      status = getreg32(PIC32MZ_PMUSPISTAT);
    }
  while ((status & PMUSPISTAT_SPIRDY) == 0);

  return status;
}

static uint16_t wfi32_pmu_read(uint8_t reg)
{
  uint32_t status;

  status = wfi32_pmu_xfer(PMUSPICTRL_CMD |
                          ((uint32_t)reg << PMUSPICTRL_ADDR_SHIFT));

  return (status & PMUSPISTAT_RDATA_MASK) >> PMUSPISTAT_RDATA_SHIFT;
}

static void wfi32_pmu_write(uint8_t reg, uint32_t value)
{
  wfi32_pmu_xfer(((uint32_t)reg << PMUSPICTRL_ADDR_SHIFT) |
                 (value & PMUSPICTRL_WDATA_MASK));
}

static void wfi32_pmu_setbits(uint8_t reg, uint16_t bits)
{
  wfi32_pmu_write(reg, wfi32_pmu_read(reg) | bits);
}

/* RF serial bridge, bit-banged through RFSPICTL [EX: register, framing] */

static void wfi32_rfspi_clock(uint32_t lines)
{
  putreg32(RFSPICTL_EN | lines, PIC32MZ_RFSPICTL);
  putreg32(RFSPICTL_EN | lines | RFSPICTL_SCK, PIC32MZ_RFSPICTL);
}

static void wfi32_rfspi_shift(uint32_t value, unsigned int nbits)
{
  while (nbits-- > 0)
    {
      wfi32_rfspi_clock(((value >> nbits) & 1) << RFSPICTL_SDO_SHIFT);
    }
}

static void wfi32_rfspi_write(const struct wfi32_rfreg_s *reg)
{
  wfi32_rfspi_clock(RFSPICTL_CS);  /* Idle cycle, then select */
  wfi32_rfspi_clock(0);

  wfi32_rfspi_shift(reg->addr, 8);
  wfi32_rfspi_shift(reg->data, 16);

  putreg32(RFSPICTL_EN | RFSPICTL_CS | RFSPICTL_SCK, PIC32MZ_RFSPICTL);
  putreg32(0, PIC32MZ_RFSPICTL);
}

static void wfi32_rfspi_write_table(const struct wfi32_rfreg_s *tab,
                                    unsigned int n)
{
  while (n-- > 0)
    {
      wfi32_rfspi_write(tab++);
    }
}

static void wfi32_posc_enable(bool enable)
{
  uint32_t regval = getreg32(PIC32MZ_CFGCON2) & ~CFGCON2_POSCMOD_MASK;

  regval |= enable ? CFGCON2_POSCMOD_HS : CFGCON2_POSCMOD_OFF;
  putreg32(regval, PIC32MZ_CFGCON2);
}

static void wfi32_wait_clkstat(uint32_t bits)
{
  while ((getreg32(PIC32MZ_CLKSTAT) & bits) == 0)
    {
    }
}

/* Wait up to timeout_us for a CLKSTAT ready bit; false on timeout.  Used
 * for clocks the console does not depend on, so a fault there cannot
 * hang the boot.
 */

static bool wfi32_wait_clkstat_timeout(uint32_t bits, uint32_t timeout_us)
{
  uint32_t start = wfi32_cp0_count();

  while ((getreg32(PIC32MZ_CLKSTAT) & bits) == 0)
    {
      if (wfi32_cp0_count() - start > timeout_us * WFI32_COUNT_PER_US)
        {
          return false;
        }
    }

  return true;
}

static void wfi32_wait_plldbg(uint32_t bits)
{
  while ((getreg32(PIC32MZ_PLLDBG) & bits) == 0)
    {
    }
}

/* Start the system PLL from POSC.  SPLLHWMD hands PLL power-down/reset to
 * hardware [DS Reg 38-1]; SPLLCON values [DS Reg 11-3].
 */

static void wfi32_spll_start(void)
{
  putreg32(WFI32_CFGCON3, PIC32MZ_CFGCON3);
  modifyreg32(PIC32MZ_CFGCON0, 0, CFGCON0_SPLLHWMD);
  putreg32(WFI32_SPLLCON, PIC32MZ_SPLLCON);
}

/* Request SYSCLK = SPLL (FRCDIV = 1) and wait for the switch to complete
 * [DS Reg 11-1].
 */

static void wfi32_sysclk_to_spll(void)
{
  putreg32(OSCCON_NOSC_SPLL, PIC32MZ_OSCCON);
  putreg32(OSCCON_OSWEN, PIC32MZ_OSCCONSET);

  while ((getreg32(PIC32MZ_OSCCON) & OSCCON_OSWEN) != 0)
    {
    }
}

static void wfi32_pll_powerdown(uintptr_t pllcon)
{
  modifyreg32(pllcon, 0, PLLCON_PWDN);
}

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
static void wfi32_trace_pmuregs(void)
{
  uint8_t reg;

  for (reg = PMU_BUCKCFG1; reg <= PMU_MLDOCFG2; reg++)
    {
      pic32mz_wfi32_trace(' ');
      pic32mz_wfi32_trace_hex(wfi32_pmu_read(reg));
    }
}

#  define TRACEPMU()            wfi32_trace_pmuregs()
#else
#  define TRACEPMU()
#endif

/* B0: switch the regulator from MLDO to buck (PWM in run, PSM in sleep)
 * using the hardware auto-switch flow of [DS Table 35-1].  Which internal
 * registers to load, the fallback values, the post-switch tweaks for
 * untrimmed parts and doing it only after a power-on or brown-out reset
 * are [EX].
 */

static void wfi32_pmu_b0(void)
{
  uint32_t vreg;

  if ((getreg32(PIC32MZ_RCON) & (RCON_POR | RCON_BOR)) == 0)
    {
      TRACE('s');
      return;
    }

  TRACE('p');
  TRACEREG('1', PIC32MZ_OTP_BUCKCFG1);
  TRACEREG('2', PIC32MZ_OTP_BUCKCFG2);
  TRACEREG('3', PIC32MZ_OTP_BUCKCFG3);
  TRACEREG('4', PIC32MZ_OTP_MLDOCFG1);
  TRACEREG('5', PIC32MZ_OTP_MLDOCFG2);
  TRACEREG('6', PIC32MZ_OTP_VREGTRIM);
  putreg32(WFI32_PMUCLKCTRL, PIC32MZ_PMUCLKCTRL);

  wfi32_pmu_write(PMU_BUCKCFG1,
                  wfi32_trim(PIC32MZ_OTP_BUCKCFG1, WFI32_DEF_BUCKCFG1));
  wfi32_pmu_write(PMU_BUCKCFG2,
                  wfi32_trim(PIC32MZ_OTP_BUCKCFG2, WFI32_DEF_BUCKCFG2));
  wfi32_pmu_write(PMU_BUCKCFG3,
                  wfi32_trim(PIC32MZ_OTP_BUCKCFG3, WFI32_DEF_BUCKCFG3));
  wfi32_pmu_write(PMU_MLDOCFG1,
                  wfi32_trim(PIC32MZ_OTP_MLDOCFG1, WFI32_DEF_MLDOCFG1));
  wfi32_pmu_write(PMU_MLDOCFG2,
                  wfi32_trim(PIC32MZ_OTP_MLDOCFG2, WFI32_DEF_MLDOCFG2));
  wfi32_pmu_read(PMU_MLDOCFG2);  /* [EX] dummy read-back */
  TRACEPMU();

  vreg = wfi32_vreg_fields(wfi32_trim(PIC32MZ_OTP_VREGTRIM,
                                      WFI32_DEF_VREGTRIM));
  TRACE('q');

  /* Run and sleep profiles, then the override that applies the run
   * profile; clearing PHWC starts hardware control.
   */

  putreg32(PMUMODE_BUCKEN | PMUMODE_BUCKMODE | vreg, PIC32MZ_PMUMODECTRL1);
  putreg32(PMUMODE_BUCKEN | vreg, PIC32MZ_PMUMODECTRL2);
  putreg32(PMUMODE_BUCKEN | PMUMODE_BUCKMODE | PMUOVERCTRL_OVEREN | vreg,
           PIC32MZ_PMUOVERCTRL);
  modifyreg32(PIC32MZ_PMUOVERCTRL, PMUOVERCTRL_PHWC, 0);

  /* Wait until the regulator reports buck/PWM with MLDO off [DS 35-4] */

  TRACE('w');

  while ((getreg32(PIC32MZ_PMUCMODE) &
          (PMUMODE_BUCKEN | PMUMODE_BUCKMODE | PMUMODE_MLDOEN)) !=
         (PMUMODE_BUCKEN | PMUMODE_BUCKMODE))
    {
    }

  TRACE('x');

  /* Untrimmed parts: adjust the buck settings after the switch [EX] */

  if (!wfi32_trim_valid(getreg32(PIC32MZ_OTP_BUCKCFG1)))
    {
      wfi32_pmu_write(PMU_BUCKCFG1,
                      wfi32_pmu_read(PMU_BUCKCFG1) & 0xebff);
    }

  if (!wfi32_trim_valid(getreg32(PIC32MZ_OTP_BUCKCFG2)))
    {
      wfi32_pmu_setbits(PMU_BUCKCFG2, 0x0010);
    }

  TRACEPMU();
}

/* A1/G: stay in MLDO mode, updating its configuration and VREG trims, and
 * apply it through the software override flow of [DS Table 35-1].  The
 * internal register values are [EX].
 */

static void wfi32_pmu_mldo(enum wfi32_silicon_e silicon)
{
  uint32_t otp1 = getreg32(PIC32MZ_OTP_MLDOCFG1);
  uint32_t vreg;
  uint32_t mldo1;
  uint32_t regval;

  vreg  = wfi32_vreg_fields(wfi32_trim(PIC32MZ_OTP_VREGTRIM,
                                       WFI32_DEF_VREGTRIM));
  mldo1 = wfi32_pmu_read(PMU_MLDOCFG1);

  if (silicon == WFI32_SILICON_A1)
    {
      if (mldo1 == 0 && !wfi32_trim_valid(otp1))
        {
          mldo1 = 0x0180 | 0x0c07;
        }
      else
        {
          mldo1 = otp1 | 0x0c07;
        }

      wfi32_pmu_write(PMU_MLDOCFG1, mldo1);
      wfi32_pmu_write(PMU_MLDOCFG2, 0);
      wfi32_pmu_setbits(PMU_MLDOCFG2, 0x0a80);
      wfi32_pmu_setbits(PMU_BUCKCFG1, 0x0004);
    }
  else
    {
      uint32_t otp2 = getreg32(PIC32MZ_OTP_MLDOCFG2);
      uint32_t mldo2;

      mldo1 |= wfi32_trim_valid(otp1) ? (otp1 | 0x0c07) : 0x0d87;
      wfi32_pmu_write(PMU_MLDOCFG1, mldo1);

      wfi32_pmu_read(PMU_MLDOCFG2);  /* [EX] read, value unused */
      mldo2 = wfi32_trim_valid(otp2) ? (otp2 & 0x8000) : 0xca80;
      wfi32_pmu_write(PMU_MLDOCFG2, mldo2 | 0x0a80);

      if (wfi32_trim_valid(getreg32(PIC32MZ_OTP_BUCKCFG3)))
        {
          wfi32_pmu_setbits(PMU_BUCKCFG3, 0x00c8);
        }
      else
        {
          wfi32_pmu_read(PMU_BUCKCFG3);
          wfi32_pmu_write(PMU_BUCKCFG3, 0x00c8);
        }

      wfi32_pmu_setbits(PMU_BUCKCFG1, 0x0001);
      wfi32_pmu_setbits(PMU_BUCKCFG1, 0x0004);
    }

  regval  = getreg32(PIC32MZ_PMUOVERCTRL);
  regval &= ~(PMUMODE_BUCKEN | PMUMODE_VREGALL_MASK | PMUOVERCTRL_PHWC);
  regval |= PMUMODE_MLDOEN | PMUOVERCTRL_OVEREN | vreg;
  putreg32(regval, PIC32MZ_PMUOVERCTRL);
}

/* B0/G clock bring-up [EX: ordering and delays] */

static void wfi32_clk_b0g(enum wfi32_silicon_e silicon)
{
  bool cfg_a = (silicon == WFI32_SILICON_G || wfi32_silicon_ver() == 1);

  TRACE('b');
  TRACEREG('O', PIC32MZ_OSCCON);
  TRACEREG('P', PIC32MZ_SPLLCON);
  TRACEREG('K', PIC32MZ_CLKSTAT);
  TRACEREG('W', PIC32MZ_EWPLLCON);
  TRACEREG('2', PIC32MZ_CFGCON2);
  TRACEREG('3', PIC32MZ_CFGCON3);

  /* Bring up POSC and the SPLL unless SYSCLK already runs from the SPLL
   * (e.g. a bootloader did it) [DS Reg 11-1 COSC].  The example tests
   * SPLLCON == 0 instead, which never holds on B0 silicon: SPLLCON resets
   * to 0xC0000808 there (verified on hardware).
   */

  if ((getreg32(PIC32MZ_OSCCON) & OSCCON_COSC_MASK) != OSCCON_COSC_SPLL)
    {
      /* Park both PLLs and stop POSC while its analog front end is set
       * up over the RF bridge.
       */

      TRACE('S');
      putreg32(PLLCON_PWDN, PIC32MZ_EWPLLCON);
      putreg32(PLLCON_PWDN, PIC32MZ_SPLLCON);
      wfi32_udelay(300);

      wfi32_posc_enable(false);
      wfi32_udelay(300);

      putreg32(RFSPICTL_EN | RFSPICTL_RESET | RFSPICTL_CS, PIC32MZ_RFSPICTL);
      putreg32(RFSPICTL_EN | RFSPICTL_CS, PIC32MZ_RFSPICTL);

      if (cfg_a)
        {
          wfi32_rfspi_write_table(g_xosc_cfg_a, nitems(g_xosc_cfg_a));
        }
      else
        {
          wfi32_rfspi_write_table(g_xosc_cfg_b, nitems(g_xosc_cfg_b));
        }

      TRACE('r');
      wfi32_udelay(200);
      wfi32_posc_enable(true);
      wfi32_udelay(300);
      TRACE('o');

      wfi32_spll_start();
      TRACE('l');
      wfi32_sysclk_to_spll();
      TRACE('s');

      if ((getreg32(PIC32MZ_OSCCON) & OSCCON_NOSC_MASK) == OSCCON_NOSC_SPLL)
        {
          wfi32_wait_clkstat(CLKSTAT_POSCRDY);
        }

      TRACE('k');

      if (cfg_a)
        {
          wfi32_rfspi_write(&g_xosc_post_a);
        }
    }
  else
    {
      TRACE('N');
    }

  /* Ethernet/Wi-Fi PLL: load the configuration with the PLL powered down
   * and in reset, then power it up and release reset.  With
   * CFGCON0.ETHPLLHWMD = 0, reset is under software control [DS Reg 11-6,
   * 38-1]; the example only clears PWDN, which leaves the PLL in reset
   * (verified on hardware: ETHPLLRDY never set).
   */

  putreg32(WFI32_EWPLLCON, PIC32MZ_EWPLLCON);
  wfi32_udelay(200);
  modifyreg32(PIC32MZ_EWPLLCON, PLLCON_PWDN | PLLCON_RST, 0);
  TRACE('e');
  TRACEREG('W', PIC32MZ_EWPLLCON);
  TRACEREG('K', PIC32MZ_CLKSTAT);
  if (wfi32_wait_clkstat_timeout(CLKSTAT_ETHPLLRDY, 10000))
    {
      TRACE('E');
    }
  else
    {
      TRACE('!');
    }

  TRACEREG('K', PIC32MZ_CLKSTAT);

  wfi32_pll_powerdown(PIC32MZ_UPLLCON);
  wfi32_pll_powerdown(PIC32MZ_BTPLLCON);
}

/* A1 clock bring-up [EX: ordering and delays] */

static void wfi32_clk_a1(void)
{
  wfi32_posc_enable(false);
  wfi32_udelay(200);

  putreg32(RFSPICTL_EN | RFSPICTL_RESET | RFSPICTL_CS, PIC32MZ_RFSPICTL);
  wfi32_posc_enable(true);
  wfi32_udelay(200);
  putreg32(RFSPICTL_EN | RFSPICTL_CS, PIC32MZ_RFSPICTL);

  wfi32_rfspi_write_table(g_xosc_cfg_a1, nitems(g_xosc_cfg_a1));
  wfi32_wait_clkstat(CLKSTAT_POSCRDY);

  wfi32_spll_start();
  wfi32_pll_powerdown(PIC32MZ_UPLLCON);

  /* On A1 the Ethernet/Wi-Fi PLL is left to hardware power control */

  putreg32(WFI32_EWPLLCON, PIC32MZ_EWPLLCON);
  modifyreg32(PIC32MZ_CFGCON0, 0, CFGCON0_ETHPLLHWMD);
  wfi32_wait_plldbg(0x4);  /* [EX] */

  wfi32_pll_powerdown(PIC32MZ_BTPLLCON);
  putreg32(WFI32_CFGCON3, PIC32MZ_CFGCON3);

  wfi32_sysclk_to_spll();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
/****************************************************************************
 * Name: pic32mz_wfi32_trace_init / pic32mz_wfi32_trace /
 *       pic32mz_wfi32_trace_end
 *
 * Description:
 *   Bring-up aid: polled UART1 output clocked from the FRC, usable before
 *   and while the PLLs are reconfigured.  trace_end() hands UART1 back to
 *   the normal console driver (PBCLK3 baud clock).
 *
 ****************************************************************************/

void pic32mz_wfi32_trace_init(void)
{
  putreg32(0, PIC32MZ_UART1_MODE);
  putreg32(UART_MODE_CLKSEL_FRC | UART_MODE_BRGH, PIC32MZ_UART1_MODE);
  putreg32(WFI32_TRACE_BRG, PIC32MZ_UART1_BRG);
  putreg32(UART_STA_UTXEN, PIC32MZ_UART1_STASET);
  putreg32(UART_MODE_ON, PIC32MZ_UART1_MODESET);
}

void pic32mz_wfi32_trace(int ch)
{
  while ((getreg32(PIC32MZ_UART1_STA) & UART_STA_UTXBF) != 0)
    {
    }

  putreg32((uint32_t)ch, PIC32MZ_UART1_TXREG);

  /* Wait until it is fully on the wire, so a reset right after a marker
   * cannot swallow it.
   */

  while ((getreg32(PIC32MZ_UART1_STA) & UART_STA_UTRMT) == 0)
    {
    }
}

void pic32mz_wfi32_trace_hex(uint32_t value)
{
  int shift;

  for (shift = 28; shift >= 0; shift -= 4)
    {
      pic32mz_wfi32_trace("0123456789abcdef"[(value >> shift) & 0xf]);
    }
}

void pic32mz_wfi32_trace_end(void)
{
  while ((getreg32(PIC32MZ_UART1_STA) & UART_STA_UTRMT) == 0)
    {
    }

  putreg32(UART_MODE_ON, PIC32MZ_UART1_MODECLR);
  putreg32(UART_MODE_CLKSEL_MASK, PIC32MZ_UART1_MODECLR);
}
#endif

/****************************************************************************
 * Name: pic32mz_wfi32_pmu_initialize
 ****************************************************************************/

void pic32mz_wfi32_pmu_initialize(void)
{
  enum wfi32_silicon_e silicon = wfi32_silicon();

  TRACE('0' + silicon);

#ifdef CONFIG_PIC32MZ_W1_PMU_MLDO
  UNUSED(silicon);
  TRACE('M');
  return;
#endif

  /* The device powers up in MLDO mode using factory-calibrated settings
   * [DS 36.1], so an unrecognized revision is safely left as it is.
   */

  switch (silicon)
    {
      case WFI32_SILICON_B0:
        wfi32_pmu_b0();
        break;

      case WFI32_SILICON_A1:
      case WFI32_SILICON_G:
        wfi32_pmu_mldo(silicon);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: pic32mz_wfi32_clk_initialize
 ****************************************************************************/

void pic32mz_wfi32_clk_initialize(void)
{
  enum wfi32_silicon_e silicon = wfi32_silicon();

  TRACE('C');

  /* PLL and OSCCON writes need the system unlock sequence [DS Reg 38-9] */

  putreg32(0, PIC32MZ_SYSKEY);
  putreg32(UNLOCK_SYSKEY_0, PIC32MZ_SYSKEY);
  putreg32(UNLOCK_SYSKEY_1, PIC32MZ_SYSKEY);
  TRACE('u');

  switch (silicon)
    {
      case WFI32_SILICON_B0:
      case WFI32_SILICON_G:
        wfi32_clk_b0g(silicon);
        break;

      case WFI32_SILICON_A1:
        wfi32_clk_a1();
        break;

      default:
        break;
    }

  /* PBCLK4 (deep sleep controller, RTCC) = SYSCLK / 10 right away, the
   * divisor used by the example [EX] (PBxDIV layout: DS Reg 11-9).
   * pic32mz_pbclk() later applies board.h.
   */

  modifyreg32(PIC32MZ_PB4DIV, PBDIV_MASK, PBDIV(10));

  putreg32(LOCK_SYSKEY, PIC32MZ_SYSKEY);
  TRACE('Z');
}

#endif /* CONFIG_ARCH_CHIP_WFI32E01 */
