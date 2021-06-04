/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_intmux.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_INTMUX_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_INTMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_INTMUX_CH0_CSR_OFFSET        0x0000 /* Control Status */
#define RV32M1_INTMUX_CH0_VEC_OFFSET        0x0004 /* Vector Number */
#define RV32M1_INTMUX_CH0_IER_OFFSET        0x0010 /* Interrupt Enable */
#define RV32M1_INTMUX_CH0_IPR_OFFSET        0x0020 /* Interrupt Pending */
#define RV32M1_INTMUX_CH1_CSR_OFFSET        0x0040 /* Control Status */
#define RV32M1_INTMUX_CH1_VEC_OFFSET        0x0044 /* Vector Number */
#define RV32M1_INTMUX_CH1_IER_OFFSET        0x0050 /* Interrupt Enable */
#define RV32M1_INTMUX_CH1_IPR_OFFSET        0x0060 /* Interrupt Pending */
#define RV32M1_INTMUX_CH2_CSR_OFFSET        0x0080 /* Control Status */
#define RV32M1_INTMUX_CH2_VEC_OFFSET        0x0084 /* Vector Number */
#define RV32M1_INTMUX_CH2_IER_OFFSET        0x0090 /* Interrupt Enable */
#define RV32M1_INTMUX_CH2_IPR_OFFSET        0x00a0 /* Interrupt Pending */
#define RV32M1_INTMUX_CH3_CSR_OFFSET        0x00c0 /* Control Status */
#define RV32M1_INTMUX_CH3_VEC_OFFSET        0x00c4 /* Vector Number */
#define RV32M1_INTMUX_CH3_IER_OFFSET        0x00d0 /* Interrupt Enable */
#define RV32M1_INTMUX_CH3_IPR_OFFSET        0x00e0 /* Interrupt Pending */
#define RV32M1_INTMUX_CH4_CSR_OFFSET        0x0100 /* Control Status */
#define RV32M1_INTMUX_CH4_VEC_OFFSET        0x0104 /* Vector Number */
#define RV32M1_INTMUX_CH4_IER_OFFSET        0x0110 /* Interrupt Enable */
#define RV32M1_INTMUX_CH4_IPR_OFFSET        0x0120 /* Interrupt Pending */
#define RV32M1_INTMUX_CH5_CSR_OFFSET        0x0140 /* Control Status */
#define RV32M1_INTMUX_CH5_VEC_OFFSET        0x0144 /* Vector Number */
#define RV32M1_INTMUX_CH5_IER_OFFSET        0x0150 /* Interrupt Enable */
#define RV32M1_INTMUX_CH5_IPR_OFFSET        0x0160 /* Interrupt Pending */
#define RV32M1_INTMUX_CH6_CSR_OFFSET        0x0180 /* Control Status */
#define RV32M1_INTMUX_CH6_VEC_OFFSET        0x0184 /* Vector Number */
#define RV32M1_INTMUX_CH6_IER_OFFSET        0x0190 /* Interrupt Enable */
#define RV32M1_INTMUX_CH6_IPR_OFFSET        0x01a0 /* Interrupt Pending */
#define RV32M1_INTMUX_CH7_CSR_OFFSET        0x01c0 /* Control Status */
#define RV32M1_INTMUX_CH7_VEC_OFFSET        0x01c4 /* Vector Number */
#define RV32M1_INTMUX_CH7_IER_OFFSET        0x01d0 /* Interrupt Enable */
#define RV32M1_INTMUX_CH7_IPR_OFFSET        0x01e0 /* Interrupt Pending */

#define INTMUX_CH_CSR_OFFSET                0x0000 /* Control Status */
#define INTMUX_CH_VEC_OFFSET                0x0004 /* Vector Number */
#define INTMUX_CH_IER_OFFSET                0x0010 /* Interrupt Enable */
#define INTMUX_CH_IPR_OFFSET                0x0020 /* Interrupt Pending */

/* Register Address *********************************************************/

#define RV32M1_INTMUX0_CH_BASE(n)       (RV32M1_INTMUX0_BASE + n * 0x40)
#  define RV32M1_INTMUX_CH_BASE         RV32M1_INTMUX0_CH_BASE

/* Register Bitfield Definitions ********************************************/

#define INTMUX_CSR_IRQP            (1 << 31) /* Bit31: Interrupt Request Pending */
#define INTMUX_CSR_CHIN_SHIFT      (8)       /* Bit[11:8]: Channel Instance Number */
#define INTMUX_CSR_CHIN_MASK       (0xf << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN0           (0   << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN1           (1   << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN2           (2   << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN3           (3   << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN4           (4   << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN5           (5   << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN6           (6   << INTMUX_CSR_CHIN_SHIFT)
#define INTMUX_CSR_CHIN7           (7   << INTMUX_CSR_CHIN_SHIFT)

#define INTMUX_CSR_IRQN_SHIFT      (4)
#define INTMUX_CSR_IRQN_MASK       (3 << INTMUX_CSR_IRQN_SHIFT)

#define INTMUX_CSR_AND             (1 << 1)
#define INTMUX_CSR_RST             (1 << 0)

#define INTMUX_VEC_VECN_SHIFT      (2)
#define INTMUX_VEC_VECN_MASK       (0xfff << INTMUX_VEC_VECN_SHIFT)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_INTMUX_H */
