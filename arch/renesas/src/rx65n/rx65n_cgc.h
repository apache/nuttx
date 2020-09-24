/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cgc.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_CGC_H
#define __ARCH_RENESAS_SRC_RX65N_CGC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Clock Control Register (SCKCR) */

/* Peripheral Module Clock D (PCLKD) */

#define _00000000_CGC_PCLKD_DIV_1           (0x00000000ul) /* x1 */
#define _00000001_CGC_PCLKD_DIV_2           (0x00000001ul) /* x1/2 */
#define _00000002_CGC_PCLKD_DIV_4           (0x00000002ul) /* x1/4 */
#define _00000003_CGC_PCLKD_DIV_8           (0x00000003ul) /* x1/8 */
#define _00000004_CGC_PCLKD_DIV_16          (0x00000004ul) /* x1/16 */
#define _00000005_CGC_PCLKD_DIV_32          (0x00000005ul) /* x1/32 */
#define _00000006_CGC_PCLKD_DIV_64          (0x00000006ul) /* x1/64 */

/* Peripheral Module Clock C (PCLKC) */

#define _00000000_CGC_PCLKC_DIV_1           (0x00000000ul) /* x1 */
#define _00000010_CGC_PCLKC_DIV_2           (0x00000010ul) /* x1/2 */
#define _00000020_CGC_PCLKC_DIV_4           (0x00000020ul) /* x1/4 */
#define _00000030_CGC_PCLKC_DIV_8           (0x00000030ul) /* x1/8 */
#define _00000040_CGC_PCLKC_DIV_16          (0x00000040ul) /* x1/16 */
#define _00000050_CGC_PCLKC_DIV_32          (0x00000050ul) /* x1/32 */
#define _00000060_CGC_PCLKC_DIV_64          (0x00000060ul) /* x1/64 */

/* Peripheral Module Clock B (PCLKB) */

#define _00000000_CGC_PCLKB_DIV_1           (0x00000000ul) /* x1 */
#define _00000100_CGC_PCLKB_DIV_2           (0x00000100ul) /* x1/2 */
#define _00000200_CGC_PCLKB_DIV_4           (0x00000200ul) /* x1/4 */
#define _00000300_CGC_PCLKB_DIV_8           (0x00000300ul) /* x1/8 */
#define _00000400_CGC_PCLKB_DIV_16          (0x00000400ul) /* x1/16 */
#define _00000500_CGC_PCLKB_DIV_32          (0x00000500ul) /* x1/32 */
#define _00000600_CGC_PCLKB_DIV_64          (0x00000600ul) /* x1/64 */

/* Peripheral Module Clock A (PCLKA) */

#define _00000000_CGC_PCLKA_DIV_1           (0x00000000ul) /* x1 */
#define _00001000_CGC_PCLKA_DIV_2           (0x00001000ul) /* x1/2 */
#define _00002000_CGC_PCLKA_DIV_4           (0x00002000ul) /* x1/4 */
#define _00003000_CGC_PCLKA_DIV_8           (0x00003000ul) /* x1/8 */
#define _00004000_CGC_PCLKA_DIV_16          (0x00004000ul) /* x1/16 */
#define _00005000_CGC_PCLKA_DIV_32          (0x00005000ul) /* x1/32 */
#define _00006000_CGC_PCLKA_DIV_64          (0x00006000ul) /* x1/64 */

/* External Bus Clock (BCLK) */

#define _00000000_CGC_BCLK_DIV_1            (0x00000000ul) /* x1 */
#define _00010000_CGC_BCLK_DIV_2            (0x00010000ul) /* x1/2 */
#define _00020000_CGC_BCLK_DIV_4            (0x00020000ul) /* x1/4 */
#define _00030000_CGC_BCLK_DIV_8            (0x00030000ul) /* x1/8 */
#define _00040000_CGC_BCLK_DIV_16           (0x00040000ul) /* x1/16 */
#define _00050000_CGC_BCLK_DIV_32           (0x00050000ul) /* x1/32 */
#define _00060000_CGC_BCLK_DIV_64           (0x00060000ul) /* x1/64 */

#define _00C00000_CGC_PSTOP0_PSTOP1         (0x00c00000)

/* System Clock (ICLK) */

#define _00000000_CGC_ICLK_DIV_1            (0x00000000ul) /* x1 */
#define _01000000_CGC_ICLK_DIV_2            (0x01000000ul) /* x1/2 */
#define _02000000_CGC_ICLK_DIV_4            (0x02000000ul) /* x1/4 */
#define _03000000_CGC_ICLK_DIV_8            (0x03000000ul) /* x1/8 */
#define _04000000_CGC_ICLK_DIV_16           (0x04000000ul) /* x1/16 */
#define _05000000_CGC_ICLK_DIV_32           (0x05000000ul) /* x1/32 */
#define _06000000_CGC_ICLK_DIV_64           (0x06000000ul) /* x1/64 */

/* System Clock (FCLK) */

#define _00000000_CGC_FCLK_DIV_1            (0x00000000ul) /* x1 */
#define _10000000_CGC_FCLK_DIV_2            (0x10000000ul) /* x1/2 */
#define _20000000_CGC_FCLK_DIV_4            (0x20000000ul) /* x1/4 */
#define _30000000_CGC_FCLK_DIV_8            (0x30000000ul) /* x1/8 */
#define _40000000_CGC_FCLK_DIV_16           (0x40000000ul) /* x1/16 */
#define _50000000_CGC_FCLK_DIV_32           (0x50000000ul) /* x1/32 */
#define _60000000_CGC_FCLK_DIV_64           (0x60000000ul) /* x1/64 */

/* ROM Wait Cycle Setting Register (ROMWT) */

/* ROM Wait Cycle Setting (ROMWT) */

#define _00_CGC_ROMWT_CYCLE_0               (0x00u) /* No wait */
#define _01_CGC_ROMWT_CYCLE_1               (0x01u) /* One wait cycle */
#define _02_CGC_ROMWT_CYCLE_2               (0x02u) /* Two wait cycles */

/* System Clock Control Register 2 (SCKCR2) */

#define _0010_CGC_UCLK_DIV_1                (0x0010u) /* x1/2 */
#define _0020_CGC_UCLK_DIV_3                (0x0020u) /* x1/3 */
#define _0030_CGC_UCLK_DIV_4                (0x0030u) /* x1/4 */
#define _0040_CGC_UCLK_DIV_5                (0x0040u) /* x1/5 */
#define _0001_SCKCR2_BIT0                   (0x0001u) /* RESERVE BIT0 */

/* System Clock Control Register 3 (SCKCR3) */

#define _0000_CGC_CLOCKSOURCE_LOCO          (0x0000u) /* LOCO */
#define _0100_CGC_CLOCKSOURCE_HOCO          (0x0100u) /* HOCO */
#define _0200_CGC_CLOCKSOURCE_MAINCLK       (0x0200u) /* Main clock oscillator */
#define _0300_CGC_CLOCKSOURCE_SUBCLK        (0x0300u) /* Sub-clock oscillator */
#define _0400_CGC_CLOCKSOURCE_PLL           (0x0400u) /* PLL circuit */

/* PLL Control Register (PLLCR) */

/* PLL Input Frequency Division Ratio Select (PLIDIV[1:0]) */

#define _0000_CGC_PLL_FREQ_DIV_1            (0x0000u) /* x1 */
#define _0001_CGC_PLL_FREQ_DIV_2            (0x0001u) /* x1/2 */
#define _0002_CGC_PLL_FREQ_DIV_3            (0x0002u) /* x1/3 */

/* PLL Clock Source Select (PLLSRCSEL) */

#define _0000_CGC_PLL_SOURCE_MAIN           (0x0000u) /* Main clock oscillator */
#define _0010_CGC_PLL_SOURCE_HOCO           (0x0010u) /* HOCO */

/* Frequency Multiplication Factor Select (STC[5:0]) */

#define _1300_CGC_PLL_FREQ_MUL_10_0         (0x1300u) /* x10.0 */
#define _1400_CGC_PLL_FREQ_MUL_10_5         (0x1400u) /* x10.5 */
#define _1500_CGC_PLL_FREQ_MUL_11_0         (0x1500u) /* x11.0 */
#define _1600_CGC_PLL_FREQ_MUL_11_5         (0x1600u) /* x11.5 */
#define _1700_CGC_PLL_FREQ_MUL_12_0         (0x1700u) /* x12.0 */
#define _1800_CGC_PLL_FREQ_MUL_12_5         (0x1800u) /* x12.5 */
#define _1900_CGC_PLL_FREQ_MUL_13_0         (0x1900u) /* x13.0 */
#define _1A00_CGC_PLL_FREQ_MUL_13_5         (0x1a00u) /* x13.5 */
#define _1B00_CGC_PLL_FREQ_MUL_14_0         (0x1b00u) /* x14.0 */
#define _1C00_CGC_PLL_FREQ_MUL_14_5         (0x1c00u) /* x14.5 */
#define _1D00_CGC_PLL_FREQ_MUL_15_0         (0x1d00u) /* x15.0 */
#define _1E00_CGC_PLL_FREQ_MUL_15_5         (0x1e00u) /* x15.5 */
#define _1F00_CGC_PLL_FREQ_MUL_16_0         (0x1f00u) /* x16.0 */
#define _2000_CGC_PLL_FREQ_MUL_16_5         (0x2000u) /* x16.5 */
#define _2100_CGC_PLL_FREQ_MUL_17_0         (0x2100u) /* x17.0 */
#define _2200_CGC_PLL_FREQ_MUL_17_5         (0x2200u) /* x17.5 */
#define _2300_CGC_PLL_FREQ_MUL_18_0         (0x2300u) /* x18.0 */
#define _2400_CGC_PLL_FREQ_MUL_18_5         (0x2400u) /* x18.5 */
#define _2500_CGC_PLL_FREQ_MUL_19_0         (0x2500u) /* x19.0 */
#define _2600_CGC_PLL_FREQ_MUL_19_5         (0x2600u) /* x19.5 */
#define _2700_CGC_PLL_FREQ_MUL_20_0         (0x2700u) /* x20.0 */
#define _2800_CGC_PLL_FREQ_MUL_20_5         (0x2800u) /* x20.5 */
#define _2900_CGC_PLL_FREQ_MUL_21_0         (0x2900u) /* x21.0 */
#define _2A00_CGC_PLL_FREQ_MUL_21_5         (0x2a00u) /* x21.5 */
#define _2B00_CGC_PLL_FREQ_MUL_22_0         (0x2b00u) /* x22.0 */
#define _2C00_CGC_PLL_FREQ_MUL_22_5         (0x2c00u) /* x22.5 */
#define _2D00_CGC_PLL_FREQ_MUL_23_0         (0x2d00u) /* x23.0 */
#define _2E00_CGC_PLL_FREQ_MUL_23_5         (0x2e00u) /* x23.5 */
#define _2F00_CGC_PLL_FREQ_MUL_24_0         (0x2f00u) /* x24.0 */
#define _3000_CGC_PLL_FREQ_MUL_24_5         (0x3000u) /* x24.5 */
#define _3100_CGC_PLL_FREQ_MUL_25_0         (0x3100u) /* x25.0 */
#define _3200_CGC_PLL_FREQ_MUL_25_5         (0x3200u) /* x25.5 */
#define _3300_CGC_PLL_FREQ_MUL_26_0         (0x3300u) /* x26.0 */
#define _3400_CGC_PLL_FREQ_MUL_26_5         (0x3400u) /* x26.5 */
#define _3500_CGC_PLL_FREQ_MUL_27_0         (0x3500u) /* x27.0 */
#define _3600_CGC_PLL_FREQ_MUL_27_5         (0x3600u) /* x27.5 */
#define _3700_CGC_PLL_FREQ_MUL_28_0         (0x3700u) /* x28.0 */
#define _3800_CGC_PLL_FREQ_MUL_28_5         (0x3800u) /* x28.5 */
#define _3900_CGC_PLL_FREQ_MUL_29_0         (0x3900u) /* x29.0 */
#define _3A00_CGC_PLL_FREQ_MUL_29_5         (0x3a00u) /* x29.5 */
#define _3B00_CGC_PLL_FREQ_MUL_30_0         (0x3b00u) /* x30.0 */

/* Oscillation Stop Detection Control Register (OSTDCR) */

/* Oscillation Stop Detection Interrupt Enable (OSTDIE) */

/* The oscillation stop detection interrupt is disabled */

#define _00_CGC_OSC_STOP_INT_DISABLE        (0x00u)

/* The oscillation stop detection interrupt is enabled */

#define _01_CGC_OSC_STOP_INT_ENABLE         (0x01u)

/* Oscillation Stop Detection Function Enable (OSTDE) */

/* Oscillation stop detection function is disabled */

#define _00_CGC_OSC_STOP_DISABLE            (0x00u)

/* Oscillation stop detection function is enabled */

#define _80_CGC_OSC_STOP_ENABLE             (0x80u)

/* High-Speed On-Chip Oscillator Control Register 2 (HOCOCR2) */

/* HOCO Frequency Setting (HCFRQ[1:0]) */

#define _00_CGC_HOCO_CLK_16                 (0x00u) /* 16 MHz */
#define _01_CGC_HOCO_CLK_18                 (0x01u) /* 18 MHz */
#define _02_CGC_HOCO_CLK_20                 (0x02u) /* 20 MHz */

/* Main Clock Oscillator Forced Oscillation Control Register (MOFCR) */

/* Main Clock Oscillator Forced Oscillation (MOFXIN) */

/* Oscillator is not controlled by this bit */

#define _00_CGC_MAINOSC_NOT_CONTROLLED      (0x00u)

/* The main clock oscillator is forcedly oscillated */

#define _01_CGC_MAINOSC_FORCE_OSCILLATED    (0x01u)

/* Main Oscillator Drive Capability 2 Switching (MODRV2[1:0]) */

#define _00_CGC_MAINOSC_UNDER24M            (0x00u) /* 20.1 to 24 MHz */
#define _10_CGC_MAINOSC_UNDER20M            (0x10u) /* 16.1 to 20 MHz */
#define _20_CGC_MAINOSC_UNDER16M            (0x20u) /* 8.1 to 16 MHz */
#define _30_CGC_MAINOSC_EQUATE8M            (0x30u) /* 8 MHz */

/* Main Clock Oscillator Switch (MOSEL) */

#define _00_CGC_MAINOSC_RESONATOR           (0x00u) /* Resonator */
#define _40_CGC_MAINOSC_EXTERNAL            (0x40u) /* External oscillator input */

/* RTC Control Register 4 (RCR4) */

/* Count source select (RCKSEL) */

#define _00_RTC_SOURCE_SELECT_SUB           (0x00u) /* Select sub-clock oscillator */
#define _01_RTC_SOURCE_SELECT_MAIN_FORCED   (0x01u) /* Select main clock oscillator */
#define _25_CGC_SOSCWTCR_VALUE              (0x25U)
/* Interrupt Source Priority Register n (IPRn) */

/* Interrupt Priority Level Select (IPR[3:0]) */

#define _00_CGC_PRIORITY_LEVEL0             (0x00u) /* Level 0 (interrupt disabled) */
#define _01_CGC_PRIORITY_LEVEL1             (0x01u) /* Level 1 */
#define _02_CGC_PRIORITY_LEVEL2             (0x02u) /* Level 2 */
#define _03_CGC_PRIORITY_LEVEL3             (0x03u) /* Level 3 */
#define _04_CGC_PRIORITY_LEVEL4             (0x04u) /* Level 4 */
#define _05_CGC_PRIORITY_LEVEL5             (0x05u) /* Level 5 */
#define _06_CGC_PRIORITY_LEVEL6             (0x06u) /* Level 6 */
#define _07_CGC_PRIORITY_LEVEL7             (0x07u) /* Level 7 */
#define _08_CGC_PRIORITY_LEVEL8             (0x08u) /* Level 8 */
#define _09_CGC_PRIORITY_LEVEL9             (0x09u) /* Level 9 */
#define _0A_CGC_PRIORITY_LEVEL10            (0x0au) /* Level 10 */
#define _0B_CGC_PRIORITY_LEVEL11            (0x0bu) /* Level 11 */
#define _0C_CGC_PRIORITY_LEVEL12            (0x0cu) /* Level 12 */
#define _0D_CGC_PRIORITY_LEVEL13            (0x0du) /* Level 13 */
#define _0E_CGC_PRIORITY_LEVEL14            (0x0eu) /* Level 14 */
#define _0F_CGC_PRIORITY_LEVEL15            (0x0fu) /* Level 15 (highest) */

#define _0001_CGC_LOCO_STOP                 (0x1)

/* Main clock oscillator wait time */

#define _5C_CGC_MOSCWTCR_VALUE              (0x5cu)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: r_cgc_create
 *
 * Description:
 * Clock Initialization
 ****************************************************************************/

void r_cgc_create(void);

#endif
