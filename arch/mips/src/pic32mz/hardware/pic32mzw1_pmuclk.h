/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzw1_pmuclk.h
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

/* PIC32MZ-W1 PMU (buck/MLDO regulator) and clock-generation registers that
 * have no PIC32MZ EC/EF equivalent.
 *
 * Every definition below is tagged with where it comes from:
 *
 *   [DS]  PIC32MZ W1 and WFI32E01 Family Data Sheet, DS70005425P.
 *   [DFP] Microchip PIC32MZ-W_DFP 1.12.356 (Apache-2.0),
 *         include/proc/pwfi32e01.h.
 *   [EX]  Not in [DS] or [DFP].  Observed in Microchip's
 *         WFI32_Ethernet_Wi-Fi_Bridge_OOB example firmware (pmu_init.c,
 *         plib_clk.c).  Only the hardware facts (addresses, bit positions,
 *         values) are used here; no code was copied.
 */

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_PMUCLK_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_PMUCLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMU controller registers [DS Table 35-2, DFP] ****************************/

#define PIC32MZ_PMUSPICTRL      (PIC32MZ_SFR_K1BASE + 0x00013e00) /* [DFP] */
#define PIC32MZ_PMUSPISTAT      (PIC32MZ_SFR_K1BASE + 0x00013e04) /* [DFP] */
#define PIC32MZ_PMUCLKCTRL      (PIC32MZ_SFR_K1BASE + 0x00013e08)
#define PIC32MZ_PMUMODECTRL1    (PIC32MZ_SFR_K1BASE + 0x00013e0c)
#define PIC32MZ_PMUMODECTRL2    (PIC32MZ_SFR_K1BASE + 0x00013e10)
#define PIC32MZ_PMUOVERCTRL     (PIC32MZ_SFR_K1BASE + 0x00013e1c)
#define PIC32MZ_PMUCMODE        (PIC32MZ_SFR_K1BASE + 0x00013e20)

/* PMUSPICTRL: memory-mapped access to the regulator's internal registers
 * [DFP _PMUSPICTRL_*].
 */

#define PMUSPICTRL_WDATA_SHIFT  (0)       /* Bits 0-15: SPIWDATA */
#define PMUSPICTRL_WDATA_MASK   (0xffff << PMUSPICTRL_WDATA_SHIFT)
#define PMUSPICTRL_ADDR_SHIFT   (16)      /* Bits 16-23: SPIADDR */
#define PMUSPICTRL_ADDR_MASK    (0xff << PMUSPICTRL_ADDR_SHIFT)
#define PMUSPICTRL_CMD          (1 << 24) /* Bit 24: CMD, 1 = read [EX] */

/* PMUSPISTAT [DFP _PMUSPISTAT_*] */

#define PMUSPISTAT_SPIERR       (1 << 0)  /* Bit 0: SPIERR */
#define PMUSPISTAT_SPIRDY       (1 << 7)  /* Bit 7: SPIRDY */
#define PMUSPISTAT_RDATA_SHIFT  (16)      /* Bits 16-31: SPIRDATA */
#define PMUSPISTAT_RDATA_MASK   (0xffffu << PMUSPISTAT_RDATA_SHIFT)

/* PMUCLKCTRL: field layout [DFP _PMUCLKCTRL_*].  The source-select
 * encodings are not documented; the example uses SPISRC=2 and BUCKSRC=1
 * [EX].
 */

#define PMUCLKCTRL_SPICLKDIV(n) ((uint32_t)(n) << 0)  /* Bits 0-5 */
#define PMUCLKCTRL_SPISRC(n)    ((uint32_t)(n) << 6)  /* Bits 6-7 */
#define PMUCLKCTRL_BUCKCLKDIV(n) ((uint32_t)(n) << 8) /* Bits 8-13 */
#define PMUCLKCTRL_BUCKSRC(n)   ((uint32_t)(n) << 14) /* Bits 14-15 */
#define PMUCLKCTRL_BACWD        (1 << 16)
#define PMUCLKCTRL_WLDOOFF      (1 << 30)             /* [DS Reg 35-1] */
#define PMUCLKCTRL_WCMRET       (1u << 31)            /* [DS Reg 35-1] */

/* PMUMODECTRL1/2, PMUOVERCTRL and PMUCMODE share this layout.  Enable and
 * mode bits: [DS Reg 35-2..35-4].  VREGn trim fields: [DFP
 * _PMUMODECTRL1_VREGnCTRL_*] only (the data sheet marks them as
 * unimplemented).
 */

#define PMUMODE_VREG4_SHIFT     (0)
#define PMUMODE_VREG3_SHIFT     (8)
#define PMUMODE_VREG2_SHIFT     (16)
#define PMUMODE_VREG1_SHIFT     (24)
#define PMUMODE_VREG_MASK       (0x1f)
#define PMUMODE_VREGALL_MASK    (0x1f1f1f1f)
#define PMUMODE_BUCKMODE        (1 << 29) /* 1 = PWM, 0 = PSM */
#define PMUMODE_MLDOEN          (1 << 30)
#define PMUMODE_BUCKEN          (1u << 31)

#define PMUOVERCTRL_PHWC        (1 << 22) /* [DS Reg 35-3] */
#define PMUOVERCTRL_OVEREN      (1 << 23) /* [DS Reg 35-3] */

/* Regulator-internal registers reached through PMUSPICTRL [EX] */

#define PMU_BUCKCFG1            0x14
#define PMU_BUCKCFG2            0x15
#define PMU_BUCKCFG3            0x16
#define PMU_MLDOCFG1            0x17
#define PMU_MLDOCFG2            0x18

/* Factory trim words in the boot-flash configuration space [EX].  The data
 * sheet only says that "calibrated values from the NVRFLASH area" must be
 * used (DS Table 35-2, note 2); it does not give their location.  An
 * erased (0xffffffff) or zero word means "not calibrated".
 */

#define PIC32MZ_OTP_BUCKCFG1    (PIC32MZ_BOOTFLASH_K1BASE + 0x56fe8)
#define PIC32MZ_OTP_BUCKCFG2    (PIC32MZ_BOOTFLASH_K1BASE + 0x56fec)
#define PIC32MZ_OTP_BUCKCFG3    (PIC32MZ_BOOTFLASH_K1BASE + 0x56ff0)
#define PIC32MZ_OTP_MLDOCFG1    (PIC32MZ_BOOTFLASH_K1BASE + 0x56ff4)
#define PIC32MZ_OTP_MLDOCFG2    (PIC32MZ_BOOTFLASH_K1BASE + 0x56ff8)
#define PIC32MZ_OTP_VREGTRIM    (PIC32MZ_BOOTFLASH_K1BASE + 0x56ffc)

/* Reset control [DFP RCON, pWFI32E01.S] */

#define PIC32MZ_RCON            (PIC32MZ_SFR_K1BASE + 0x00001260)
#define PIC32MZ_RCONCLR         (PIC32MZ_SFR_K1BASE + 0x00001264)

#define RCON_POR                (1 << 0)  /* Bit 0: power-on reset */
#define RCON_BOR                (1 << 1)  /* Bit 1: brown-out reset */

/* Additional PLLs and clock status (OSC block) *****************************/

#define PIC32MZ_UPLLCON         (PIC32MZ_OSC_K1BASE + 0x0030) /* [DS 11-4] */
#define PIC32MZ_BTPLLCON        (PIC32MZ_OSC_K1BASE + 0x0040) /* [DS 11-5] */
#define PIC32MZ_EWPLLCON        (PIC32MZ_OSC_K1BASE + 0x0050) /* [DS 11-6] */
#define PIC32MZ_CLKSTAT         (PIC32MZ_OSC_K1BASE + 0x0190) /* [DS 11-11] */

/* xPLLCON common field layout (SPLLCON, UPLLCON, BTPLLCON, EWPLLCON)
 * [DS Reg 11-3..11-6].
 */

#define PLLCON_BSWSEL(n)        ((uint32_t)(n) << 0)  /* Bits 0-2 */
#define PLLCON_PWDN             (1 << 3)
#define PLLCON_POSTDIV1(n)      ((uint32_t)(n) << 4)  /* Bits 4-9 */
#define PLLCON_FLOCK            (1 << 10)
#define PLLCON_RST              (1 << 11)
#define PLLCON_FBDIV(n)         ((uint32_t)(n) << 12) /* Bits 12-21 */
#define PLLCON_REFDIV(n)        ((uint32_t)(n) << 22) /* Bits 22-27 */
#define PLLCON_CLKOUTEN         (1 << 28)             /* BT/EWPLL only */
#define PLLCON_ICLK_FRC         (1 << 30)             /* 0 = POSC input */
#define PLLCON_BYP              (1u << 31)

/* CLKSTAT [DS Reg 11-11] */

#define CLKSTAT_FRCRDY          (1 << 0)
#define CLKSTAT_SPLLRDY         (1 << 1)
#define CLKSTAT_POSCRDY         (1 << 2)
#define CLKSTAT_UPLLRDY         (1 << 3)
#define CLKSTAT_ETHPLLRDY       (1 << 7)

/* CFGCON2: POSCMOD, bits 8-9 [DS Reg 38-3] */

#define CFGCON2_POSCMOD_SHIFT   (8)
#define CFGCON2_POSCMOD_MASK    (3 << CFGCON2_POSCMOD_SHIFT)
#define CFGCON2_POSCMOD_HS      (0 << CFGCON2_POSCMOD_SHIFT)
#define CFGCON2_POSCMOD_OFF     (3 << CFGCON2_POSCMOD_SHIFT)

/* CFGCON3: PLL second post-dividers [DS Table 38-1, DFP _CFGCON3_*] */

#define CFGCON3_ETHPLLPOSTDIV2(n) ((uint32_t)(n) << 0)  /* Bits 0-5 */
#define CFGCON3_SPLLPOSTDIV2(n)   ((uint32_t)(n) << 6)  /* Bits 6-11 */
#define CFGCON3_BTPLLPOSTDIV2(n)  ((uint32_t)(n) << 12) /* Bits 12-15 */

/* RF/analog serial bridge [EX].  Not in [DS] or [DFP].  Bit-banged with
 * 8-bit address + 16-bit data words, MSB first; its registers configure
 * the 40 MHz crystal oscillator's analog front end, which the example
 * programs before relying on POSC.
 */

#define PIC32MZ_RFSPICTL        (PIC32MZ_SFR_K1BASE + 0x000c8028)

#define RFSPICTL_SCK            (1 << 0)
#define RFSPICTL_CS             (1 << 1)  /* Idle (deasserted) level */
#define RFSPICTL_SDO_SHIFT      (2)
#define RFSPICTL_RESET          (1 << 5)
#define RFSPICTL_EN             (1u << 31)

/* PLL lock status [EX].  Not in [DS] or [DFP]; CLKSTAT is the documented
 * alternative but the example's exact polling is kept until verified on
 * hardware.
 */

#define PIC32MZ_PLLDBG          (PIC32MZ_SFR_K1BASE + 0x000000e0)

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_PMUCLK_H */
