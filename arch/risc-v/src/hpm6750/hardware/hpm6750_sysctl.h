/****************************************************************************
 * arch/risc-v/src/hpm6750/hardware/hpm6750_sysctl.h
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

#ifndef __ARCH_RISCV_SRC_HPM6750_HARDWARE_HPM6750_SYSCTL_H
#define __ARCH_RISCV_SRC_HPM6750_HARDWARE_HPM6750_SYSCTL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM6750_SYSCTL_GROUP0_0           (HPM6750_SYSCTL_BASE + 0x000800)
#define HPM6750_SYSCTL_GROUP0_1           (HPM6750_SYSCTL_BASE + 0x000810)
#define HPM6750_SYSCTL_GROUP0_2           (HPM6750_SYSCTL_BASE + 0x000820)
#define HPM6750_SYSCTL_CLOCK_NODE         (HPM6750_SYSCTL_BASE + 0x001800)

#define HPM6750_SYSCTL_GROUPx_0_AHB                   (1 << 0)
#define HPM6750_SYSCTL_GROUPx_0_AXI                   (1 << 1)
#define HPM6750_SYSCTL_GROUPx_0_CONN                  (1 << 2)
#define HPM6750_SYSCTL_GROUPx_0_VIS                   (1 << 3)
#define HPM6750_SYSCTL_GROUPx_0_FEMC                  (1 << 4)
#define HPM6750_SYSCTL_GROUPx_0_ROM                   (1 << 5)
#define HPM6750_SYSCTL_GROUPx_0_LMM0                  (1 << 6)
#define HPM6750_SYSCTL_GROUPx_0_LMM1                  (1 << 7)
#define HPM6750_SYSCTL_GROUPx_0_MCHTMR0               (1 << 8)
#define HPM6750_SYSCTL_GROUPx_0_MCHTMR1               (1 << 9)
#define HPM6750_SYSCTL_GROUPx_0_XSRAM0                (1 << 10)
#define HPM6750_SYSCTL_GROUPx_0_XSRAM1                (1 << 11)
#define HPM6750_SYSCTL_GROUPx_0_XIP0                  (1 << 12)
#define HPM6750_SYSCTL_GROUPx_0_XIP1                  (1 << 13)
#define HPM6750_SYSCTL_GROUPx_0_SDP                   (1 << 14)
#define HPM6750_SYSCTL_GROUPx_0_RNG                   (1 << 15)
#define HPM6750_SYSCTL_GROUPx_0_KEYM                  (1 << 16)
#define HPM6750_SYSCTL_GROUPx_0_HMDA                  (1 << 17)
#define HPM6750_SYSCTL_GROUPx_0_XDMA                  (1 << 18)
#define HPM6750_SYSCTL_GROUPx_0_GPIO                  (1 << 19)
#define HPM6750_SYSCTL_GROUPx_0_MBX0                  (1 << 20)
#define HPM6750_SYSCTL_GROUPx_0_MBX1                  (1 << 21)
#define HPM6750_SYSCTL_GROUPx_0_WDG0                  (1 << 22)
#define HPM6750_SYSCTL_GROUPx_0_WDG1                  (1 << 23)
#define HPM6750_SYSCTL_GROUPx_0_WDG2                  (1 << 24)
#define HPM6750_SYSCTL_GROUPx_0_WDG3                  (1 << 25)
#define HPM6750_SYSCTL_GROUPx_0_GPTMR0                (1 << 26)
#define HPM6750_SYSCTL_GROUPx_0_GPTMR1                (1 << 27)
#define HPM6750_SYSCTL_GROUPx_0_GPTMR2                (1 << 28)
#define HPM6750_SYSCTL_GROUPx_0_GPTMR3                (1 << 29)
#define HPM6750_SYSCTL_GROUPx_0_GPTMR4                (1 << 30)
#define HPM6750_SYSCTL_GROUPx_0_GPTMR5                (1 << 31)

#define HPM6750_SYSCTL_GROUPx_1_GPTMR6                (1 << 0)
#define HPM6750_SYSCTL_GROUPx_1_GPTMR7                (1 << 1)
#define HPM6750_SYSCTL_GROUPx_1_UART0                 (1 << 2)
#define HPM6750_SYSCTL_GROUPx_1_UART1                 (1 << 3)
#define HPM6750_SYSCTL_GROUPx_1_UART2                 (1 << 4)
#define HPM6750_SYSCTL_GROUPx_1_UART3                 (1 << 5)
#define HPM6750_SYSCTL_GROUPx_1_UART4                 (1 << 6)
#define HPM6750_SYSCTL_GROUPx_1_UART5                 (1 << 7)
#define HPM6750_SYSCTL_GROUPx_1_UART6                 (1 << 8)
#define HPM6750_SYSCTL_GROUPx_1_UART7                 (1 << 9)
#define HPM6750_SYSCTL_GROUPx_1_UART8                 (1 << 10)
#define HPM6750_SYSCTL_GROUPx_1_UART9                 (1 << 11)
#define HPM6750_SYSCTL_GROUPx_1_UART10                (1 << 12)
#define HPM6750_SYSCTL_GROUPx_1_UART11                (1 << 13)
#define HPM6750_SYSCTL_GROUPx_1_UART12                (1 << 14)
#define HPM6750_SYSCTL_GROUPx_1_UART13                (1 << 15)
#define HPM6750_SYSCTL_GROUPx_1_UART14                (1 << 16)
#define HPM6750_SYSCTL_GROUPx_1_UART15                (1 << 17)
#define HPM6750_SYSCTL_GROUPx_1_I2C0                  (1 << 18)
#define HPM6750_SYSCTL_GROUPx_1_I2C1                  (1 << 19)
#define HPM6750_SYSCTL_GROUPx_1_I2C2                  (1 << 20)
#define HPM6750_SYSCTL_GROUPx_1_I2C3                  (1 << 21)
#define HPM6750_SYSCTL_GROUPx_1_SPI0                  (1 << 22)
#define HPM6750_SYSCTL_GROUPx_1_SPI1                  (1 << 23)
#define HPM6750_SYSCTL_GROUPx_1_SPI2                  (1 << 24)
#define HPM6750_SYSCTL_GROUPx_1_SPI3                  (1 << 25)
#define HPM6750_SYSCTL_GROUPx_1_CAN0                  (1 << 26)
#define HPM6750_SYSCTL_GROUPx_1_CAN1                  (1 << 27)
#define HPM6750_SYSCTL_GROUPx_1_CAN2                  (1 << 28)
#define HPM6750_SYSCTL_GROUPx_1_CAN3                  (1 << 29)
#define HPM6750_SYSCTL_GROUPx_1_PTPC                  (1 << 30)
#define HPM6750_SYSCTL_GROUPx_1_ADC0                  (1 << 31)

#define HPM6750_SYSCTL_GROUPx_2_ADC1                  (1 << 0)
#define HPM6750_SYSCTL_GROUPx_2_ADC2                  (1 << 1)
#define HPM6750_SYSCTL_GROUPx_2_ADC3                  (1 << 2)
#define HPM6750_SYSCTL_GROUPx_2_ACMP                  (1 << 3)
#define HPM6750_SYSCTL_GROUPx_2_I2S0                  (1 << 4)
#define HPM6750_SYSCTL_GROUPx_2_I2S1                  (1 << 5)
#define HPM6750_SYSCTL_GROUPx_2_I2S2                  (1 << 6)
#define HPM6750_SYSCTL_GROUPx_2_I2S3                  (1 << 7)
#define HPM6750_SYSCTL_GROUPx_2_PDM                   (1 << 8)
#define HPM6750_SYSCTL_GROUPx_2_DAO                   (1 << 9)
#define HPM6750_SYSCTL_GROUPx_2_SYNT                  (1 << 10)
#define HPM6750_SYSCTL_GROUPx_2_MOT0                  (1 << 11)
#define HPM6750_SYSCTL_GROUPx_2_MOT1                  (1 << 12)
#define HPM6750_SYSCTL_GROUPx_2_MOT2                  (1 << 13)
#define HPM6750_SYSCTL_GROUPx_2_MOT3                  (1 << 14)
#define HPM6750_SYSCTL_GROUPx_2_LCDC                  (1 << 15)
#define HPM6750_SYSCTL_GROUPx_2_CAM0                  (1 << 16)
#define HPM6750_SYSCTL_GROUPx_2_CAM1                  (1 << 17)
#define HPM6750_SYSCTL_GROUPx_2_JPEG                  (1 << 18)
#define HPM6750_SYSCTL_GROUPx_2_PDMA                  (1 << 19)
#define HPM6750_SYSCTL_GROUPx_2_ENET0                 (1 << 20)
#define HPM6750_SYSCTL_GROUPx_2_ENET1                 (1 << 21)
#define HPM6750_SYSCTL_GROUPx_2_NTMR0                 (1 << 22)
#define HPM6750_SYSCTL_GROUPx_2_NTMR1                 (1 << 23)
#define HPM6750_SYSCTL_GROUPx_2_SDXC0                 (1 << 24)
#define HPM6750_SYSCTL_GROUPx_2_SDXC1                 (1 << 25)
#define HPM6750_SYSCTL_GROUPx_2_USB0                  (1 << 26)
#define HPM6750_SYSCTL_GROUPx_2_USB1                  (1 << 27)

/* LOC_BUSY (RO)
 *
 * local busy
 * 0: a change is pending for current node
 * 1: current node is changing status
 */
#define SYSCTL_CLOCK_LOC_BUSY_MASK (0x40000000UL)
#define SYSCTL_CLOCK_LOC_BUSY_SHIFT (30U)
#define SYSCTL_CLOCK_LOC_BUSY_GET(x) (((uint32_t)(x) & SYSCTL_CLOCK_LOC_BUSY_MASK) >> SYSCTL_CLOCK_LOC_BUSY_SHIFT)

/* MUX (RW)
 *
 * clock source selection
 * 0:osc0_clk0
 * 1:pll0_clk0
 * 2:pll1_clk0
 * 3:pll1_clk1
 * 4:pll2_clk0
 * 5:pll2_clk1
 * 6:pll3_clk0
 * 7:pll4_clk0
 */
#define SYSCTL_CLOCK_MUX_MASK (0xF00U)
#define SYSCTL_CLOCK_MUX_SHIFT (8U)
#define SYSCTL_CLOCK_MUX_SET(x) (((uint32_t)(x) << SYSCTL_CLOCK_MUX_SHIFT) & SYSCTL_CLOCK_MUX_MASK)
#define SYSCTL_CLOCK_MUX_GET(x) (((uint32_t)(x) & SYSCTL_CLOCK_MUX_MASK) >> SYSCTL_CLOCK_MUX_SHIFT)

/* DIV (RW)
 *
 * clock divider
 * 0: divider by1
 * 1: divider by 2
 * 2 divider by 3
 * . . .
 * 255: divider by 256
 */
#define SYSCTL_CLOCK_DIV_MASK (0xFFU)
#define SYSCTL_CLOCK_DIV_SHIFT (0U)
#define SYSCTL_CLOCK_DIV_SET(x) (((uint32_t)(x) << SYSCTL_CLOCK_DIV_SHIFT) & SYSCTL_CLOCK_DIV_MASK)
#define SYSCTL_CLOCK_DIV_GET(x) (((uint32_t)(x) & SYSCTL_CLOCK_DIV_MASK) >> SYSCTL_CLOCK_DIV_SHIFT)

#endif /* __ARCH_RISCV_SRC_HPM6750_HARDWARE_HPM6750_SYSCTL_H */
