/****************************************************************************
 * arch/risc-v/src/hpm6000/hardware/hpm_uart.h
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

#ifndef ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_UART_H
#define ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_UART_MINIMUM_BAUDRATE        (200U)
#define HPM_UART_BAUDRATE_TOLERANCE      (3)
#define HPM_UART_OSC_MAX                 (32U)
#define HPM_UART_OSC_MIN                 (8U)
#define HPM_UART_BAUDRATE_DIV_MAX        (0xFFFFU)
#define HPM_UART_BAUDRATE_DIV_MIN        (1U)
#define HPM_UART_FREQ                    2400000

#define HPM_UART_OSCR_OFFSET             0x14
#define HPM_UART_RBR_OFFSET              0x20
#define HPM_UART_THR_OFFSET              0x20
#define HPM_UART_DLL_OFFSET              0x20
#define HPM_UART_IER_OFFSET              0x24
#define HPM_UART_DLM_OFFSET              0x24
#define HPM_UART_IIR_OFFSET              0x28
#define HPM_UART_FCR_OFFSET              0x28
#define HPM_UART_LCR_OFFSET              0x2C
#define HPM_UART_MCR_OFFSET              0x30
#define HPM_UART_LSR_OFFSET              0x34
#define HPM_UART_MSR_OFFSET              0x38

/* Register addresses *******************************************************/

#define HPM_UART0_OSC                    (HPM_UART0_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART0_RBR                    (HPM_UART0_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART0_THR                    (HPM_UART0_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART0_DLL                    (HPM_UART0_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART0_IER                    (HPM_UART0_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART0_DLM                    (HPM_UART0_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART0_IIR                    (HPM_UART0_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART0_FCR                    (HPM_UART0_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART0_LCR                    (HPM_UART0_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART0_MCR                    (HPM_UART0_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART0_LSR                    (HPM_UART0_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART0_MSR                    (HPM_UART0_BASE + HPM_UART_MSR_OFFSET)

#define HPM_UART1_OSC                    (HPM_UART1_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART1_RBR                    (HPM_UART1_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART1_THR                    (HPM_UART1_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART1_DLL                    (HPM_UART1_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART1_IER                    (HPM_UART1_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART1_DLM                    (HPM_UART1_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART1_IIR                    (HPM_UART1_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART1_FCR                    (HPM_UART1_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART1_LCR                    (HPM_UART1_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART1_MCR                    (HPM_UART1_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART1_LSR                    (HPM_UART1_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART1_MSR                    (HPM_UART1_BASE + HPM_UART_MSR_OFFSET)

#define HPM_UART2_OSC                    (HPM_UART2_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART2_RBR                    (HPM_UART2_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART2_THR                    (HPM_UART2_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART2_DLL                    (HPM_UART2_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART2_IER                    (HPM_UART2_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART2_DLM                    (HPM_UART2_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART2_IIR                    (HPM_UART2_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART2_FCR                    (HPM_UART2_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART2_LCR                    (HPM_UART2_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART2_MCR                    (HPM_UART2_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART2_LSR                    (HPM_UART2_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART2_MSR                    (HPM_UART2_BASE + HPM_UART_MSR_OFFSET)

#define HPM_UART3_OSC                    (HPM_UART3_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART3_RBR                    (HPM_UART3_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART3_THR                    (HPM_UART3_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART3_DLL                    (HPM_UART3_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART3_IER                    (HPM_UART3_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART3_DLM                    (HPM_UART3_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART3_IIR                    (HPM_UART3_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART3_FCR                    (HPM_UART3_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART3_LCR                    (HPM_UART3_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART3_MCR                    (HPM_UART3_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART3_LSR                    (HPM_UART3_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART3_MSR                    (HPM_UART3_BASE + HPM_UART_MSR_OFFSET)

#define HPM_UART4_OSC                    (HPM_UART4_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART4_RBR                    (HPM_UART4_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART4_THR                    (HPM_UART4_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART4_DLL                    (HPM_UART4_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART4_IER                    (HPM_UART4_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART4_DLM                    (HPM_UART4_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART4_IIR                    (HPM_UART4_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART4_FCR                    (HPM_UART4_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART4_LCR                    (HPM_UART4_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART4_MCR                    (HPM_UART4_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART4_LSR                    (HPM_UART4_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART4_MSR                    (HPM_UART4_BASE + HPM_UART_MSR_OFFSET)

#define HPM_UART5_OSC                    (HPM_UART5_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART5_RBR                    (HPM_UART5_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART5_THR                    (HPM_UART5_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART5_DLL                    (HPM_UART5_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART5_IER                    (HPM_UART5_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART5_DLM                    (HPM_UART5_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART5_IIR                    (HPM_UART5_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART5_FCR                    (HPM_UART5_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART5_LCR                    (HPM_UART5_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART5_MCR                    (HPM_UART5_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART5_LSR                    (HPM_UART5_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART5_MSR                    (HPM_UART5_BASE + HPM_UART_MSR_OFFSET)

#define HPM_UART6_OSC                    (HPM_UART6_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART6_RBR                    (HPM_UART6_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART6_THR                    (HPM_UART6_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART6_DLL                    (HPM_UART6_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART6_IER                    (HPM_UART6_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART6_DLM                    (HPM_UART6_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART6_IIR                    (HPM_UART6_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART6_FCR                    (HPM_UART6_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART6_LCR                    (HPM_UART6_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART6_MCR                    (HPM_UART6_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART6_LSR                    (HPM_UART6_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART6_MSR                    (HPM_UART6_BASE + HPM_UART_MSR_OFFSET)

#define HPM_UART7_OSC                    (HPM_UART7_BASE + HPM_UART_OSCR_OFFSET)
#define HPM_UART7_RBR                    (HPM_UART7_BASE + HPM_UART_RBR_OFFSET)
#define HPM_UART7_THR                    (HPM_UART7_BASE + HPM_UART_THR_OFFSET)
#define HPM_UART7_DLL                    (HPM_UART7_BASE + HPM_UART_DLL_OFFSET)
#define HPM_UART7_IER                    (HPM_UART7_BASE + HPM_UART_IER_OFFSET)
#define HPM_UART7_DLM                    (HPM_UART7_BASE + HPM_UART_DLM_OFFSET)
#define HPM_UART7_IIR                    (HPM_UART7_BASE + HPM_UART_IIR_OFFSET)
#define HPM_UART7_FCR                    (HPM_UART7_BASE + HPM_UART_FCR_OFFSET)
#define HPM_UART7_LCR                    (HPM_UART7_BASE + HPM_UART_LCR_OFFSET)
#define HPM_UART7_MCR                    (HPM_UART7_BASE + HPM_UART_MCR_OFFSET)
#define HPM_UART7_LSR                    (HPM_UART7_BASE + HPM_UART_LSR_OFFSET)
#define HPM_UART7_MSR                    (HPM_UART7_BASE + HPM_UART_MSR_OFFSET)

/* Register bit definitions *************************************************/

#define UART_RXIDLE_CFG_DETECT_COND      (1U << 9)
#define UART_RXIDLE_CFG_DETECT_EN        (1U << 8)
#define UART_RXIDLE_CFG_THR_SHIFT        (0U)
#define UART_RXIDLE_CFG_THR_MASK         (0xff << UART_RXIDLE_CFG_THR_SHIFT)
#define UART_RXIDLE_CFG_THR(n)           ((uint32_t)(n) << UART_RXIDLE_CFG_THR_SHIFT)

#define UART_CFG_FIFOSIZE_SHIFT          (0U)
#define UART_CFG_FIFOSIZE_MASK           (2 << UART_CFG_FIFOSIZE_SHIFT)
#  define UART_CFG_FIFOSIZE_16B          (0 << UART_CFG_FIFOSIZE_SHIFT)
#  define UART_CFG_FIFOSIZE_32B          (1 << UART_CFG_FIFOSIZE_SHIFT)
#  define UART_CFG_FIFOSIZE_64B          (2 << UART_CFG_FIFOSIZE_SHIFT)
#  define UART_CFG_FIFOSIZE_128B         (3 << UART_CFG_FIFOSIZE_SHIFT)

#define UART_OSCR_OSC_SHIFT              (0U)
#define UART_OSCR_OSC_MASK               (0x1f << UART_OSCR_OSC_SHIFT)
#  define UART_OSCR_OSC(n)               ((uint32_t)(n) << UART_OSCR_OSC_SHIFT)

#define UART_RBR_RBR_MASK                (0xffff)

#define UART_IER_ERBI                    (1 << 0)
#define UART_IER_ETHEI                   (1 << 1)
#define UART_IER_ELSI                    (1 << 2)
#define UART_IER_EMSI                    (1 << 3)
#define UART_IER_ERXIDLE                 (1 << 31)
#define UART_ALL_INTS                    (UART_IER_ERBI | UART_IER_ETHEI | UART_IER_ELSI | UART_IER_EMSI | UART_IER_ERXIDLE)

#define UART_IIR_INTRID_SHIFT            (0U)
#define UART_IIR_INTRID_MASK             (0xf << UART_IIR_INTRID_SHIFT)
#define UART_IIR_INTRID_MODEM_STATE      (0x1 << UART_IIR_INTRID_SHIFT)
#define UART_IIR_INTRID_TX_AVAILE        (0x2 << UART_IIR_INTRID_SHIFT)
#define UART_IIR_INTRID_RX_AVAILE        (0x4 << UART_IIR_INTRID_SHIFT)
#define UART_IIR_INTRID_RX_STATE         (0x6 << UART_IIR_INTRID_SHIFT)
#define UART_IIR_INTRID_RX_TIMEOUT       (0xc << UART_IIR_INTRID_SHIFT)
#define UART_IIR_FIFOED_SHIFT            (6U)
#define UART_IIR_FIFOED_MASK             (3 << UART_IIR_FIFOED_SHIFT)
#define UART_IIR_DATA_LOST               (1 << 27)
#define UART_IIR_ADDR_MATCH_IDLE         (1 << 28)
#define UART_IIR_ADDR_MATCH              (1 << 29)
#define UART_IIR_TXIDLE_FLAG             (1 << 30)
#define UART_IIR_RXIDLE_FLAG             (1 << 31)

#define UART_FCR_FIFOE                   (1 << 0)
#define UART_FCR_RXFIFORST               (1 << 1)
#define UART_FCR_TXFIFORST               (1 << 2)
#define UART_FCR_DMAE                    (1 << 3)
#define UART_FCR_TFIFOT_SHIFT            (4U)
#define UART_FCR_TFIFOT_MASK             (0x3U << UART_FCR_TFIFO_SHIFT)
#define UART_FCR_TFIFOT(n)               ((uint32_t)(n) << UART_FCR_TFIFO_SHIFT)
#define UART_FCR_RFIFOT_SHIFT            (6U)
#define UART_FCR_RFIFOT_MASK             (0x3U << UART_FCR_RFIFO_SHIFT)
#define UART_FCR_RFIFOT(n)               ((uint32_t)(n) << UART_FCR_RFIFO_SHIFT)

#define UART_LCR_WLS_SHIFT               (0U)
#define UART_LCR_WLS_MASK                (0x3U << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_5BITS             (0 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_6BITS             (1 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_7BITS             (2 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_8BITS             (3 << UART_LCR_WLS_SHIFT)
#define UART_LCR_STB                     (1 << 2)
#define UART_LCR_PEN                     (1 << 3)
#define UART_LCR_EPS                     (1 << 4)
#define UART_LCR_SPS                     (1 << 5)
#define UART_LCR_BC                      (1 << 6)
#define UART_LCR_DLAB                    (1 << 7)

#define UART_MCR_RTS                     (1 << 0)
#define UART_MCR_LOOP                    (1 << 1)
#define UART_MCR_AFE                     (1 << 5)

#define UART_LSR_DR                      (1 << 0)
#define UART_LSR_OE                      (1 << 1)
#define UART_LSR_PE                      (1 << 2)
#define UART_LSR_FE                      (1 << 3)
#define UART_LSR_LBREAK                  (1 << 4)
#define UART_LSR_THRE                    (1 << 5)
#define UART_LSR_TEMT                    (1 << 6)
#define UART_LSR_ERRF                    (1 << 7)

#define UART_MSR_DCTS                    (1 << 0)
#define UART_MSR_CTS                     (1 << 4)

#endif /* ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_UART_H */
