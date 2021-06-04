/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_port.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_PORT_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_PORT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_PORT_PCR0_OFFSET      0x0000
#define RV32M1_PORT_PCR1_OFFSET      0x0004
#define RV32M1_PORT_PCR2_OFFSET      0x0008
#define RV32M1_PORT_PCR3_OFFSET      0x000c
#define RV32M1_PORT_PCR4_OFFSET      0x0010
#define RV32M1_PORT_PCR5_OFFSET      0x0014
#define RV32M1_PORT_PCR6_OFFSET      0x0018
#define RV32M1_PORT_PCR7_OFFSET      0x001c
#define RV32M1_PORT_PCR8_OFFSET      0x0020
#define RV32M1_PORT_PCR9_OFFSET      0x0024
#define RV32M1_PORT_PCR10_OFFSET     0x0028
#define RV32M1_PORT_PCR11_OFFSET     0x002c
#define RV32M1_PORT_PCR12_OFFSET     0x0030
#define RV32M1_PORT_PCR13_OFFSET     0x0034
#define RV32M1_PORT_PCR14_OFFSET     0x0038
#define RV32M1_PORT_PCR15_OFFSET     0x003c
#define RV32M1_PORT_PCR16_OFFSET     0x0040
#define RV32M1_PORT_PCR17_OFFSET     0x0044
#define RV32M1_PORT_PCR18_OFFSET     0x0048
#define RV32M1_PORT_PCR19_OFFSET     0x004c
#define RV32M1_PORT_PCR20_OFFSET     0x0050
#define RV32M1_PORT_PCR21_OFFSET     0x0054
#define RV32M1_PORT_PCR22_OFFSET     0x0058
#define RV32M1_PORT_PCR23_OFFSET     0x005c
#define RV32M1_PORT_PCR24_OFFSET     0x0060
#define RV32M1_PORT_PCR25_OFFSET     0x0064
#define RV32M1_PORT_PCR26_OFFSET     0x0068
#define RV32M1_PORT_PCR27_OFFSET     0x006c
#define RV32M1_PORT_PCR28_OFFSET     0x0070
#define RV32M1_PORT_PCR29_OFFSET     0x0074
#define RV32M1_PORT_PCR30_OFFSET     0x0078
#define RV32M1_PORT_PCR31_OFFSET     0x007c
#define RV32M1_PORT_GPCLR_OFFSET     0x0080 /* Global Pin Control Low Register */
#define RV32M1_PORT_GPCHR_OFFSET     0x0084 /* Global Pin Control High Register */
#define RV32M1_PORT_GICLR_OFFSET     0x0088 /* Global Interrupt Control Low Regisgter */
#define RV32M1_PORT_GICHR_OFFSET     0x008c /* Global Interrupt Control High Register */
#define RV32M1_PORT_ISFR_OFFSET      0x00a0 /* Interrupt Status Flag */

/* Register Bitfield Definitions ********************************************/

#define PORT_PCR_ISF                 (1 << 24) /* Interrupt Status Flag, W1C */

#define PORT_PCR_IRQC_SHIFT          (16) /* Bit[19:16]: Interrupt Configuration */
#define PORT_PCR_IRQC_MASK           (0xf << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_ISF_DISABLED   (0   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_DMA_RISE       (1   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_DMA_FALL       (2   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_DMA_EDGE       (3   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_4_RESERVED     (4   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_FLG_RISE       (5   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_FLG_FALL       (6   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_FLG_EDGE       (7   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_INT_LOW        (8   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_INT_RISE       (9   << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_INT_FALL       (10  << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_INT_EDGE       (11  << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_INT_HIGH       (12  << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_ACT_TRIH       (13  << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_ACT_TRIL       (14  << PORT_PCR_IRQC_SHIFT)
#define PORT_PCR_IRQC_15_RESERVED    (15  << PORT_PCR_IRQC_SHIFT)

#define PORT_PCR_LK                  (1 << 15)

#define PORT_PCR_MUX_SHIFT           (8)
#define PORT_PCR_MUX_MASK            (0x7 << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_MUX_ALT0            (0   << PORT_PCR_MUX_SHIFT)
#  define PORT_PCR_MUX_ANALOG        PORT_PCR_MUX_ALT0
#define PORT_PCR_MUX_ALT1            (1   << PORT_PCR_MUX_SHIFT)
#  define PORT_PCR_MUX_GPIO          PORT_PCR_MUX_ALT1
#define PORT_PCR_MUX_ALT2            (2   << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_MUX_ALT3            (3   << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_MUX_ALT4            (4   << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_MUX_ALT5            (5   << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_MUX_ALT6            (6   << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_MUX_ALT7            (7   << PORT_PCR_MUX_SHIFT)

#define PORT_PCR_ODE                 (1 << 5) /* Open Drain Enable */
#define PORT_PCR_PFE                 (1 << 4) /* Passive Filter Enable */
#define PORT_PCR_SRE                 (1 << 2) /* Slow Slew Rate Enable */
#define PORT_PCR_PE                  (1 << 1) /* Pull Enable */

/* if 'PE' bit  is enabled,
 * 'PS' bit: 0 -> internal pull down, 1: internal pull up
 */

#define PORT_PCR_PS                  (1 << 0) /* Pull Select */

#define PORT_GPC_WE_SHIFT            (16)
#define PORT_GPC_WE_MASK             (0xff << PORT_GPC_WE_SHIFT)

#define PORT_GPC_WD_SHIFT            (0)
#define PORT_GPC_WD_MASK             (0xff << PORT_GPC_WE_SHIFT)

#define PORT_GIC_WE_SHIFT            (16)
#define PORT_GIC_WE_MASK             (0xff << PORT_GPC_WE_SHIFT)

#define PORT_GIC_WD_SHIFT            (0)
#define PORT_GIC_WD_MASK             (0xff << PORT_GPC_WE_SHIFT)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_PORT_H */
