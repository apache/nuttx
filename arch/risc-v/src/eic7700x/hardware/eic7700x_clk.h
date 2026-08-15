/****************************************************************************
 * arch/risc-v/src/eic7700x/hardware/eic7700x_clk.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_CLK_H
#define __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definitions transcribed from the EIC7700X SoC Technical
 * Reference Manual v1.0.0, Part 1 section 3.2 "clock".  Page numbers in
 * the comments below are PDF page numbers of Part 1; the printed page
 * number is the PDF page number minus 16.
 *
 * The Clock and Reset Generator (CRG) occupies 0x51828000..0x5182ffff
 * (TRM p70, p80).  Only the clock half is described here; the reset
 * controller at offsets 0x300..0x4f0 is deliberately omitted.
 */

/* Clock and Reset Generator base address ***********************************/

#define EIC7700X_CLK_BASE          0x51828000ul

/* PLL registers ************************************************************/

/* Each PLL owns five consecutive words.  The offsets below locate the
 * first word of each block; add the EIC7700X_PLL_CFG* offsets to reach
 * the individual registers.  Note that the d2d PLL sits outside the
 * regular 0x14 stride (TRM p97).
 */

#define EIC7700X_SPLL0_BASE        (EIC7700X_CLK_BASE + 0x0000) /* p80 */
#define EIC7700X_SPLL1_BASE        (EIC7700X_CLK_BASE + 0x0014) /* p82 */
#define EIC7700X_SPLL2_BASE        (EIC7700X_CLK_BASE + 0x0028) /* p84 */
#define EIC7700X_VPLL_BASE         (EIC7700X_CLK_BASE + 0x003c) /* p86 */
#define EIC7700X_APLL_BASE         (EIC7700X_CLK_BASE + 0x0050) /* p89 */
#define EIC7700X_CPUPLL_BASE       (EIC7700X_CLK_BASE + 0x0064) /* p91 */
#define EIC7700X_DDRPLL_BASE       (EIC7700X_CLK_BASE + 0x0078) /* p93 */
#define EIC7700X_D2DPLL_BASE       (EIC7700X_CLK_BASE + 0x00b0) /* p97 */

#define EIC7700X_PLL_CFG0          0x00     /* Feedback and reference div  */
#define EIC7700X_PLL_CFG1          0x04     /* Fractional part, fout en    */
#define EIC7700X_PLL_CFG2          0x08     /* Per output post dividers    */
#define EIC7700X_PLL_DSKEWCAL      0x0c     /* Deskew calibration          */
#define EIC7700X_PLL_SSC           0x10     /* Spread spectrum modulator   */

/* PLL configuration word 0 (TRM p80) */

#define PLL_CFG0_EN                (1 << 0)  /* PLL enable                 */
#define PLL_CFG0_FREFCMLEN         (1 << 1)  /* Ref select, do not change  */
#define PLL_CFG0_BYP_SHIFT         (4)       /* One bypass bit per output  */
#define PLL_CFG0_BYP_MASK          (0xf << PLL_CFG0_BYP_SHIFT)
#define PLL_CFG0_DACEN             (1 << 8)  /* Fractional noise cancel    */
#define PLL_CFG0_DSMEN             (1 << 9)  /* 0: integer, 1: fractional  */
#define PLL_CFG0_REFDIV_SHIFT      (12)      /* Reference divider          */
#define PLL_CFG0_REFDIV_MASK       (0x3f << PLL_CFG0_REFDIV_SHIFT)
#define PLL_CFG0_FBDIV_SHIFT       (20)      /* Feedback divider           */
#define PLL_CFG0_FBDIV_MASK        (0xfff << PLL_CFG0_FBDIV_SHIFT)

/* PLL configuration word 1 (TRM p80) */

#define PLL_CFG1_FOUTEN_SHIFT      (0)       /* One enable bit per output  */
#define PLL_CFG1_FOUTEN_MASK       (0xf << PLL_CFG1_FOUTEN_SHIFT)
#define PLL_CFG1_FRAC_SHIFT        (4)       /* 24 bit fractional part     */
#define PLL_CFG1_FRAC_MASK         (0xfffffful << PLL_CFG1_FRAC_SHIFT)
#define PLL_CFG1_FRAC_BITS         (24)

/* PLL configuration word 2 (TRM p81).  Every output n has its own pair of
 * three bit post dividers, postdiv<n>_a and postdiv<n>_b.  The effective
 * divisor is (postdiv_a + 1) * (postdiv_b + 1); the TRM notes that
 * POSTDIV_A should be bigger than POSTDIV_B.
 *
 * The groups are not evenly spaced, and the exception is easy to miss.
 * Only group zero carries a pre bit, at bit 0, and only group zero is
 * therefore pushed up by one:
 *
 *   postdiv0_pre  0     postdiv0_a  3:1     postdiv0_b  6:4
 *                       postdiv1_a 10:8     postdiv1_b 14:12
 *                       postdiv2_a 18:16    postdiv2_b 22:20
 *                       postdiv3_a 26:24    postdiv3_b 30:28
 *
 * Reading groups one and above as though they were also offset by the pre
 * bit takes each field one bit too high, which quietly halves or doubles
 * an output rather than failing.  Linux agrees with the layout above:
 * drivers/clk/eswin/clk.c places postdiv1 at GENMASK(10, 8) and postdiv2
 * at GENMASK(18, 16).
 */

#define PLL_CFG2_POSTDIV_PRE0      (1 << 0)
#define PLL_CFG2_POSTDIV_WIDTH     (3)
#define PLL_CFG2_POSTDIV_A_SHIFT(n) ((n) == 0 ? 1 : 8 * (n))
#define PLL_CFG2_POSTDIV_B_SHIFT(n) ((n) == 0 ? 4 : 8 * (n) + 4)

/* PLL lock status, one bit per PLL, active high (TRM p96) */

#define EIC7700X_PLL_STATUS        (EIC7700X_CLK_BASE + 0x00a4)

#define PLL_STATUS_SPLL0_LOCK      (0)
#define PLL_STATUS_SPLL1_LOCK      (1)
#define PLL_STATUS_SPLL2_LOCK      (2)
#define PLL_STATUS_VPLL_LOCK       (3)
#define PLL_STATUS_APLL_LOCK       (4)
#define PLL_STATUS_CPUPLL_LOCK     (5)
#define PLL_STATUS_DDRPLL_LOCK     (6)
#define PLL_STATUS_D2DPLL_LOCK     (7)

/* Peripheral clock control registers ***************************************/

/* Offsets from EIC7700X_CLK_BASE.  The house layout puts gate enables at
 * the top of the word counting down from bit 31, the divisor in a field
 * starting at bit 4, and the mux selector at bit 0.
 */

#define EIC7700X_NOC_CLK_CTRL      0x0100   /* NOC clocks         TRM p99  */
#define EIC7700X_BOOTSPI_CLK_CTRL  0x0104   /* Boot SPI           TRM p100 */
#define EIC7700X_BOOTSPI_CFG_CTRL  0x0108   /* Boot SPI config    TRM p101 */
#define EIC7700X_SCPU_CORE_CTRL    0x010c   /* SCPU core          TRM p101 */
#define EIC7700X_SCPU_BUS_CTRL     0x0110   /* SCPU bus           TRM p101 */
#define EIC7700X_LPCPU_CORE_CTRL   0x0114   /* LPCPU core         TRM p102 */
#define EIC7700X_LPCPU_BUS_CTRL    0x0118   /* LPCPU bus          TRM p102 */
#define EIC7700X_TCU_ACLK_CTRL     0x011c   /* TCU AXI            TRM p103 */
#define EIC7700X_TCU_CFG_CTRL      0x0120   /* TCU config         TRM p103 */
#define EIC7700X_DDR0_CLK_CTRL     0x0124   /* DDR controller 0   TRM p103 */
#define EIC7700X_DDR1_CLK_CTRL     0x0128   /* DDR controller 1   TRM p104 */
#define EIC7700X_GPU_ACLK_CTRL     0x012c   /* GPU AXI            TRM p105 */
#define EIC7700X_GPU_CFG_CTRL      0x0130   /* GPU config         TRM p105 */
#define EIC7700X_GPU_GRAY_CTRL     0x0134   /* GPU gray           TRM p106 */
#define EIC7700X_DSP_ACLK_CTRL     0x0138   /* DSP AXI            TRM p106 */
#define EIC7700X_DSP_CFG_CTRL      0x013c   /* DSP config         TRM p106 */
#define EIC7700X_D2D_ACLK_CTRL     0x0140   /* Die to die AXI     TRM p108 */
#define EIC7700X_D2D_CFG_CTRL      0x0144   /* Die to die config  TRM p108 */
#define EIC7700X_NPU_ACLK_CTRL     0x0178   /* NPU AXI            TRM p115 */
#define EIC7700X_NPU_LLC_CTRL      0x017c   /* NPU last level $   TRM p115 */
#define EIC7700X_NPU_CORE_CTRL     0x0180   /* NPU core and E31   TRM p116 */
#define EIC7700X_VI_DW_CLK_CTRL    0x0184   /* VI dewarp          TRM p117 */
#define EIC7700X_VI_ACLK_CTRL      0x0188   /* VI AXI             TRM p118 */
#define EIC7700X_VI_DIG_ISP_CTRL   0x018c   /* VI image signal    TRM p118 */
#define EIC7700X_VI_DVP_CLK_CTRL   0x0190   /* VI digital video   TRM p119 */

/* Six camera shutter blocks on a four byte stride (TRM p120) */

#define EIC7700X_VI_SHUTTER_CTRL(n) (0x0194 + 4 * (n))

#define EIC7700X_VI_PHY_CLK_CTRL   0x01ac   /* VI phy             TRM p123 */
#define EIC7700X_VO_ACLK_CTRL      0x01b0   /* VO AXI             TRM p123 */
#define EIC7700X_VO_IESM_CLK_CTRL  0x01b4   /* VO HDMI iesm       TRM p124 */
#define EIC7700X_VO_PIXEL_CTRL     0x01b8   /* VO pixel           TRM p124 */
#define EIC7700X_VO_MCLK_CTRL      0x01bc   /* VO audio master    TRM p125 */
#define EIC7700X_VO_PHY_CLK_CTRL   0x01c0   /* VO phy and CEC     TRM p125 */
#define EIC7700X_VC_ACLK_CTRL      0x01c4   /* Video codec AXI    TRM p126 */
#define EIC7700X_VCDEC_ROOT_CTRL   0x01c8   /* Codec root select  TRM p127 */
#define EIC7700X_G2D_CTRL          0x01cc   /* 2D graphics        TRM p127 */
#define EIC7700X_VC_CLKEN_CTRL     0x01d0   /* Codec pclk gates   TRM p128 */
#define EIC7700X_JE_CLK_CTRL       0x01d4   /* JPEG encoder       TRM p128 */
#define EIC7700X_JD_CLK_CTRL       0x01d8   /* JPEG decoder       TRM p129 */
#define EIC7700X_VD_CLK_CTRL       0x01dc   /* Video decoder      TRM p129 */
#define EIC7700X_VE_CLK_CTRL       0x01e0   /* Video encoder      TRM p130 */
#define EIC7700X_HSP_ACLK_CTRL     0x0148   /* HSP AXI            TRM p109 */
#define EIC7700X_HSP_CFG_CTRL      0x014c   /* HSP config         TRM p109 */
#define EIC7700X_SATA_RBC_CTRL     0x0150   /* SATA rbc           TRM p110 */
#define EIC7700X_SATA_OOB_CTRL     0x0154   /* SATA oob and phy   TRM p110 */
#define EIC7700X_ETH0_CTRL         0x0158   /* Ethernet 0         TRM p110 */
#define EIC7700X_ETH1_CTRL         0x015c   /* Ethernet 1         TRM p111 */
#define EIC7700X_MSHC0_CORE_CTRL   0x0160   /* SD/eMMC 0          TRM p112 */
#define EIC7700X_MSHC1_CORE_CTRL   0x0164   /* SD/eMMC 1          TRM p112 */
#define EIC7700X_MSHC2_CORE_CTRL   0x0168   /* SD/eMMC 2          TRM p113 */
#define EIC7700X_PCIE_ACLK_CTRL    0x0170   /* PCIe AXI           TRM p114 */
#define EIC7700X_PCIE_CFG_CTRL     0x0174   /* PCIe config        TRM p114 */
#define EIC7700X_PKA_CLK_CTRL      0x01f0   /* Public key accel   TRM p133 */
#define EIC7700X_SPACC_CLK_CTRL    0x01f4   /* Crypto             TRM p133 */
#define EIC7700X_TRNG_CLK_CTRL     0x01f8   /* Random generator   TRM p133 */
#define EIC7700X_OTP_CLK_CTRL      0x01fc   /* One time program   TRM p134 */
#define EIC7700X_AON_DMA_CLK_CTRL  0x01e4   /* Always on DMA      TRM p130 */
#define EIC7700X_TIMER_CLK_CTRL    0x01e8   /* LSP timers         TRM p131 */
#define EIC7700X_RTC_CLK_CTRL      0x01ec   /* RTC and mtime      TRM p132 */
#define EIC7700X_LSP_CLK_EN0       0x0200   /* LSP gates bank 0   TRM p134 */
#define EIC7700X_LSP_CLK_EN1       0x0204   /* LSP gates bank 1   TRM p135 */
#define EIC7700X_U84_CLK_CTRL      0x0208   /* U84 cluster        TRM p135 */
#define EIC7700X_SYSCFG_CLK_CTRL   0x020c   /* APB config root    TRM p136 */

/* The two always on I2C controllers get a register each rather than a bit
 * in a bank, and the numbering inverts: the register called i2c0 gates the
 * controller the manual and the device tree call i2c10, and i2c1 gates
 * i2c11.  The inversion runs through the resets and the low power core's
 * interrupt list as well, so it is the naming to expect rather than a
 * transcription error here.
 */

#define EIC7700X_I2C0_CLK_CTRL     0x0210   /* i2c10 pclk         TRM p136 */
#define EIC7700X_I2C1_CLK_CTRL     0x0214   /* i2c11 pclk         TRM p136 */

/* NOC clock control (TRM p99).  Unlike most CRG divisors, the watchdog
 * reference divisor documents 0 and 1 as no divisor rather than as
 * divide by two.
 */

#define NOC_NSP_DIVSOR_SHIFT       (0)
#define NOC_NSP_DIVSOR_WIDTH       (3)
#define NOC_WDREF_DIVSOR_SHIFT     (4)
#define NOC_WDREF_DIVSOR_WIDTH     (16)
#define NOC_RNOC_NSP_CLKEN         (29)
#define NOC_WDREF_CLKEN            (30)
#define NOC_NSP_CLKEN              (31)

/* Boot SPI (TRM p100), SCPU (TRM p101) and LPCPU (TRM p102).  These three
 * share the CRG house layout: a divisor at [7:4], a selector at bit 0
 * choosing between the divided PLL output and the crystal, and a gate at
 * bit 31.  Every one of those selectors resets to the crystal.
 */

#define CLKCTRL_DIVSOR_SHIFT       (4)
#define CLKCTRL_DIVSOR_WIDTH       (4)
#define CLKCTRL_SRC_SEL_SHIFT      (0)
#define CLKCTRL_SRC_SEL_WIDTH      (1)
#define CLKCTRL_CLKEN              (31)

#define BOOTSPI_DIVSOR_SHIFT       (4)
#define BOOTSPI_DIVSOR_WIDTH       (6)

/* CPU bus ratio selectors sit at bit 16.  The SCPU one is a real choice
 * between half rate and full rate; the LPCPU one is documented as
 * reserved, so that bus is always half the core clock.
 */

#define CPU_BUSCLK_SEL_SHIFT       (16)
#define CPU_BUSCLK_SEL_WIDTH       (1)

/* DDR controller clock control (TRM p103).  Both controllers expose five
 * port clocks and a trace clock; only controller 0 carries the shared
 * source selector, divisor and configuration gate.
 */

#define DDR_ACLK_DIVSOR_SHIFT      (20)
#define DDR_ACLK_DIVSOR_WIDTH      (4)
#define DDR_ACLKSRC_SEL_SHIFT      (16)
#define DDR_ACLKSRC_SEL_WIDTH      (1)
#define DDR_CFG_CLKEN              (9)
#define DDR_PORT_ACLK_CLKEN(n)     (4 + (n))   /* port0..port4            */
#define DDR_TRACE_CLKEN            (0)

/* Video input, output and codec (TRM p117 to p130).  The shutter, pixel,
 * audio master and CEC dividers are all wider than the usual four bits.
 */

#define VI_CFG_CLKEN               (30)
#define VI_SHUTTER_DIVSOR_SHIFT    (4)
#define VI_SHUTTER_DIVSOR_WIDTH    (7)
#define VI_PHYCFG_CLKEN            (1)
#define VI_TXESC_CLKEN             (0)
#define VO_CFG_CLKEN               (30)
#define VO_PIXEL_DIVSOR_SHIFT      (4)
#define VO_PIXEL_DIVSOR_WIDTH      (6)
#define VO_MCLK_DIVSOR_SHIFT       (4)
#define VO_MCLK_DIVSOR_WIDTH       (8)
#define VO_CEC_DIVSOR_SHIFT        (16)
#define VO_CEC_DIVSOR_WIDTH        (16)
#define VO_CR_CLKEN                (1)
#define VO_TXESC_CLKEN             (0)
#define G2D_ACLKEN                 (31)
#define G2D_CLKEN                  (30)
#define G2D_PCLK_EN                (28)
#define VC_VE_PCLK_EN              (5)
#define VC_VD_PCLK_EN              (4)
#define VC_MON_PCLK_EN             (3)
#define VC_JE_PCLK_EN              (2)
#define VC_JD_PCLK_EN              (1)
#define VC_CFG_CLKEN               (0)

/* Die to die config gate sits at bit 0 rather than the usual bit 31
 * (TRM p108).
 */

#define D2D_CFG_CLKEN              (0)

/* Neural processor (TRM p115).  Both the cache and the core selectors
 * are two bits wide with only three valid encodings, which is why they
 * need the table driven selector rather than the generic one.
 */

#define NPU_CFG_CLKEN              (30)
#define NPU_LLC_SRC0_DIVSOR_SHIFT  (4)
#define NPU_LLC_SRC1_DIVSOR_SHIFT  (8)
#define NPU_LLC_DIVSOR_WIDTH       (4)
#define NPU_LLCLK_SEL_SHIFT        (0)
#define NPU_SEL_WIDTH              (2)
#define NPU_CORECLK_SEL_SHIFT      (0)
#define NPU_CORECLK_DIVSOR_SHIFT   (4)
#define NPU_E31CLK_SEL_SHIFT       (8)
#define NPU_E31CLK_DIVSOR_SHIFT    (12)
#define NPU_DIVSOR_WIDTH           (4)
#define NPU_E31_CORE_CLKEN         (30)

/* High speed peripherals (TRM p109 to p114).  The SATA, ethernet and
 * SD/eMMC blocks each carry a backup selector choosing between their
 * derived clock and the board supplied LPDDR reference.
 */

#define HSP_DMA0_CLKEN             (0)
#define HSP_PCLK_EN                (30)
#define SATA_RBC_CLKEN             (0)
#define SATA_OOB_CLKEN             (31)
#define SATA_PHY_BAK_SEL_SHIFT     (9)
#define SATA_PHY_REF_DIVSOR_SHIFT  (0)
#define SATA_PHY_REF_DIVSOR_WIDTH  (4)
#define ETH_RMII_REF_CLKEN         (31)
#define ETH_DIVSOR_SHIFT           (4)
#define ETH_DIVSOR_WIDTH           (7)
#define ETH_RMII_BAK_SEL_SHIFT     (2)
#define ETH_CORE_BAK_SEL_SHIFT     (1)
#define ETH_CLK_EN                 (0)
#define MSHC_CORE_CLKEN            (16)
#define MSHC_DIVSOR_SHIFT          (4)
#define MSHC_DIVSOR_WIDTH          (12)
#define MSHC_CORE_SEL_SHIFT        (0)
#define PCIE_AUX_CLKEN             (1)
#define PCIE_CR_CLKEN              (0)

/* Always on DMA (TRM p130).  Like the other root selectors this one
 * resets to the crystal rather than to the divided PLL output.
 */

#define AONDMA_AXI_CLKEN           (31)
#define AONDMA_CFG_CLKEN           (30)
#define AON_ACLK_CLKEN             (29)

/* Secure block clocks (TRM p133) */

#define SPACC_CRYPTO_CLKEN         (30)
#define CRYPTO_DIVSOR_SHIFT        (4)
#define CRYPTO_DIVSOR_WIDTH        (4)

/* Timer clock control (TRM p131).  The four low speed timers are fed
 * directly from the crystal with no divider.  Counter 7 of timer 3 has a
 * separate 49.5MHz feed used to derive a 90kHz AVC time stamp.
 */

#define TIMER_CLK_CLKEN(n)         (0 + (n))   /* timer_clk[0..3]         */
#define TIMER_PCLK_CLKEN(n)        (4 + (n))   /* timer_pclk[0..3]        */
#define TIMER3_CLK8_CLKEN          (8)

/* RTC clock control (TRM p132).  rtc_toggle is the signal the CLINT mtime
 * counter increments from (TRM p251), so rtc_toggle_divsor sets the NuttX
 * tick source frequency.  Note the two divisors do not share an encoding:
 * rtc_toggle_divsor documents 0 and 1 as bypass, aon_rtc_divsor uses the
 * usual "0, 1, 2 all mean divide by two".
 */

#define RTC_TOGGLE_DIVSOR_SHIFT    (16)
#define RTC_TOGGLE_DIVSOR_WIDTH    (5)
#define RTC_AON_DIVSOR_SHIFT       (21)
#define RTC_AON_DIVSOR_WIDTH       (11)
#define RTC_CLKEN                  (1)
#define RTC_CFG_CLKEN              (2)

/* U84 cluster clock control (TRM p135).  The core selector resets to the
 * crystal; the boot loader is what moves the cluster onto the CPU PLL.
 */

#define U84_CORE_CLK_SEL_SHIFT     (0)
#define U84_CORE_CLK_SEL_WIDTH     (2)
#define U84_BUSCLK_RATIO_SEL_SHIFT (20)
#define U84_BUSCLK_RATIO_SEL_WIDTH (1)
#define U84_TRACE_COM_CLKEN        (23)
#define U84_TRACE_CLKEN(n)         (24 + (n))  /* hart0..hart3            */
#define U84_CORE_CLKEN(n)          (28 + (n))  /* hart0..hart3            */

/* LSP gate bank 0 (TRM p134).  Bit set enables the clock.  The uart and
 * i2c fields are contiguous runs, one bit per instance, lowest instance
 * in the least significant bit.
 */

#define LSP_CLK_EN0_FAN_PCLK       (0)
#define LSP_CLK_EN0_PVT_PCLK       (1)
#define LSP_CLK_EN0_I2C_PCLK(n)    (7 + (n))   /* i2c0..i2c9              */
#define I2C_CLK_CTRL_PCLKEN        (31)        /* In I2Cn_CLK_CTRL        */
#define LSP_CLK_EN0_UART_PCLK(n)   (17 + (n))  /* uart0..uart4            */
#define LSP_CLK_EN0_TIMER_PCLK     (25)
#define LSP_CLK_EN0_SSI_PCLK(n)    (26 + (n))  /* ssi0..ssi1              */
#define LSP_CLK_EN0_WDT_PCLK(n)    (28 + (n))  /* wdt0..wdt3              */

/* LSP gate bank 1 (TRM p135) */

#define LSP_CLK_EN1_MBOX_PCLK(n)   (0 + (n))   /* mailbox0..mailbox15     */
#define LSP_CLK_EN1_PVT_CLK(n)     (16 + (n))  /* 0: lsp pvt, 1: ddr pvt  */

/* System configuration clock control (TRM p136).  This is the root of the
 * 200MHz APB domain that feeds every peripheral pclk.  Note that the
 * selector resets to 1, that is to the 24MHz crystal: the 200MHz figure
 * quoted in the TRM clock tree is the post bootloader configuration, so
 * the value must always be read rather than assumed.
 */

#define SYSCFG_CLK_SEL_SHIFT       (0)
#define SYSCFG_CLK_SEL_WIDTH       (1)
#define SYSCFG_DIVSOR_SHIFT        (4)
#define SYSCFG_DIVSOR_WIDTH        (3)

#endif /* __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_CLK_H */
