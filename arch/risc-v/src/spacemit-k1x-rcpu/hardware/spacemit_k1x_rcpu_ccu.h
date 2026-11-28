/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/hardware/spacemit_k1x_rcpu_ccu.h
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

#ifndef __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_CCU_H
#define __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_CCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock and reset control for the blocks the RCPU owns.
 *
 * The RCPU core clock and its always-on domain are brought up by the
 * application processor before this core is released, but the peripheral
 * clock gates are not: they come out of reset disabled, and a peripheral
 * whose functional clock is off cannot be reached over the bus at all.  It
 * reads back as a constant rather than as an error.
 *
 * The two blocks used here are:
 *
 *   AUD_MCUSYSCTRL  0xc0880000   per peripheral CLK/RST registers
 *   APBS            0xd4090000   audio PLL gate controls
 *   APMU            0xd4282800   RCPU core clock mux/divider
 *
 * Register offsets and field positions follow the K1 User Manual sections 9
 * (clock and reset) and 14.3 (audio subsystem registers).
 */

#define SPACEMIT_K1X_RCPU_SYSCTRL_BASE      0xc0880000u

/* AUD_MCUSYSCTRL register offsets */

#define SPACEMIT_K1X_RCPU_SYSCTRL_SSP0_CLK_RST   0x28u
#define SPACEMIT_K1X_RCPU_SYSCTRL_I2C0_CLK_RST   0x30u
#define SPACEMIT_K1X_RCPU_SYSCTRL_UART1_CLK_RST  0x3cu
#define SPACEMIT_K1X_RCPU_SYSCTRL_CAN_CLK_RST    0x48u
#define SPACEMIT_K1X_RCPU_SYSCTRL_IR_CLK_RST     0x4cu
#define SPACEMIT_K1X_RCPU_SYSCTRL_DDR_REMAP      0xc0u
#define SPACEMIT_K1X_RCPU_SYSCTRL_UART0_CLK_RST  0xd8u

/* Common CLK/RST register layout */

#define SPACEMIT_K1X_RCPU_CLK_SW_RSTN        (1u << 0)  /* Deassert reset  */
#define SPACEMIT_K1X_RCPU_CLK_FCLK_EN        (1u << 1)  /* Functional clk  */
#define SPACEMIT_K1X_RCPU_CLK_PCLK_EN        (1u << 2)  /* Bus clock       */
#define SPACEMIT_K1X_RCPU_CLK_FCLK_SEL_SHIFT 4
#define SPACEMIT_K1X_RCPU_CLK_FCLK_SEL_MASK  (3u << 4)
#define SPACEMIT_K1X_RCPU_CLK_FCLK_DIV_SHIFT 8
#define SPACEMIT_K1X_RCPU_CLK_FCLK_DIV_MASK  (0x7ffu << 8)

/* Functional clock sources for the SHUB peripherals */

#define SPACEMIT_K1X_RCPU_CLK_SEL_AUD24P5    0  /* pll1_aud_24p5 = 24.576 MHz */
#define SPACEMIT_K1X_RCPU_CLK_SEL_AUD245P7   1  /* pll1_aud_245p7 = 245.7 MHz */
#define SPACEMIT_K1X_RCPU_CLK_SEL_VCTCXO24   2  /* vctcxo_24 = 24 MHz         */
#define SPACEMIT_K1X_RCPU_CLK_SEL_3M         3  /* 3 MHz                      */

/* APBS (audio PLL gate) register block */

#define SPACEMIT_K1X_RCPU_APBS_BASE          0xd4090000u
#define SPACEMIT_K1X_RCPU_APBS_SPARE2        (SPACEMIT_K1X_RCPU_APBS_BASE + 0x104u)
#define SPACEMIT_K1X_RCPU_PLL1_AUD_24P5_GATE (1u << 11)

/* APMU: RCPU core clock source and divider */

#define SPACEMIT_K1X_RCPU_APMU_BASE          0xd4282800u
#define SPACEMIT_K1X_RCPU_CLK_RES_CTRL       (SPACEMIT_K1X_RCPU_APMU_BASE + 0x14cu)
#define SPACEMIT_K1X_RCPU_CORE_MUX_SHIFT     7
#define SPACEMIT_K1X_RCPU_CORE_MUX_MASK      (7u << 7)
#define SPACEMIT_K1X_RCPU_CORE_DIV_SHIFT     4
#define SPACEMIT_K1X_RCPU_CORE_DIV_MASK      (7u << 4)

#define SPACEMIT_K1X_RCPU_CORE_SRC_AUD245P7  0  /* N308 nominal 245.7 MHz */
#define SPACEMIT_K1X_RCPU_CORE_SRC_AUD307P2  1
#define SPACEMIT_K1X_RCPU_CORE_SRC_D5_491P52 2
#define SPACEMIT_K1X_RCPU_CORE_SRC_409P6     3

#define SPACEMIT_K1X_RCPU_CORE_SRC           SPACEMIT_K1X_RCPU_CORE_SRC_AUD245P7
#define SPACEMIT_K1X_RCPU_CORE_DIV           0  /* divide by 1 */

/* Console functional clock.
 *
 * The console UART is fed from vctcxo_24, the 24 MHz crystal oscillator.
 * Choosing the crystal over the audio PLL keeps the baud rate independent of
 * whether the PLL has locked, which matters during bring-up when little else
 * is known to be working.
 *
 * With this source the divisor for 115200 baud is 24000000 / (16 * 115200) =
 * 13.02, and the resulting rate is 115384 (0.16 percent fast), comfortably
 * inside the 2 percent a UART allows.
 */

#define SPACEMIT_K1X_RCPU_CONSOLE_FCK        24000000u
#define SPACEMIT_K1X_RCPU_CONSOLE_FCK_SEL    SPACEMIT_K1X_RCPU_CLK_SEL_VCTCXO24

#endif /* __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_CCU_H */
