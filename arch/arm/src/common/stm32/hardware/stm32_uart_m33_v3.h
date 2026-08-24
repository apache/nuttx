/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_uart_m33_v3.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_UART_M33_V3_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_UART_M33_V3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_USART_CR1_OFFSET      0x0000 /* USART control register 1 */
#define STM32_USART_CR2_OFFSET      0x0004 /* USART control register 2 */
#define STM32_USART_CR3_OFFSET      0x0008 /* USART control register 3 */
#define STM32_USART_BRR_OFFSET      0x000c /* USART baud-rate register */
#define STM32_USART_GTPR_OFFSET     0x0010 /* USART guard-time and prescaler register */
#define STM32_USART_RTOR_OFFSET     0x0014 /* USART receiver-timeout register */
#define STM32_USART_RQR_OFFSET      0x0018 /* USART request register */
#define STM32_USART_ISR_OFFSET      0x001c /* USART interrupt and status register */
#define STM32_USART_ICR_OFFSET      0x0020 /* USART interrupt flag clear register */
#define STM32_USART_RDR_OFFSET      0x0024 /* USART receive-data register */
#define STM32_USART_TDR_OFFSET      0x0028 /* USART transmit-data register */
#define STM32_USART_PRESC_OFFSET    0x002c /* USART prescaler register */
#define STM32_USART_AUTOCR_OFFSET   0x0030 /* USART autonomous-mode control register */

/* Register Addresses *******************************************************/

#if STM32_NLPUART > 0
#  define STM32_LPUART1_CR1           (STM32_LPUART1_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_LPUART1_CR2           (STM32_LPUART1_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_LPUART1_CR3           (STM32_LPUART1_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_LPUART1_BRR           (STM32_LPUART1_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_LPUART1_GTPR          (STM32_LPUART1_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_LPUART1_RTOR          (STM32_LPUART1_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_LPUART1_RQR           (STM32_LPUART1_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_LPUART1_ISR           (STM32_LPUART1_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_LPUART1_ICR           (STM32_LPUART1_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_LPUART1_RDR           (STM32_LPUART1_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_LPUART1_TDR           (STM32_LPUART1_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_LPUART1_PRESC         (STM32_LPUART1_BASE + STM32_USART_PRESC_OFFSET)
#  define STM32_LPUART1_AUTOCR        (STM32_LPUART1_BASE + STM32_USART_AUTOCR_OFFSET)
#endif

#if STM32_NUSART > 0
#  define STM32_USART1_CR1            (STM32_USART1_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART1_CR2            (STM32_USART1_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART1_CR3            (STM32_USART1_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART1_BRR            (STM32_USART1_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART1_GTPR           (STM32_USART1_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART1_RTOR           (STM32_USART1_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART1_RQR            (STM32_USART1_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART1_ISR            (STM32_USART1_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART1_ICR            (STM32_USART1_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART1_RDR            (STM32_USART1_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART1_TDR            (STM32_USART1_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART1_PRESC          (STM32_USART1_BASE + STM32_USART_PRESC_OFFSET)
#  define STM32_USART1_AUTOCR         (STM32_USART1_BASE + STM32_USART_AUTOCR_OFFSET)
#endif

#if STM32_NUSART > 1
#  define STM32_USART2_CR1            (STM32_USART2_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART2_CR2            (STM32_USART2_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART2_CR3            (STM32_USART2_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART2_BRR            (STM32_USART2_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART2_GTPR           (STM32_USART2_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART2_RTOR           (STM32_USART2_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART2_RQR            (STM32_USART2_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART2_ISR            (STM32_USART2_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART2_ICR            (STM32_USART2_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART2_RDR            (STM32_USART2_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART2_TDR            (STM32_USART2_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART2_PRESC          (STM32_USART2_BASE + STM32_USART_PRESC_OFFSET)
#  define STM32_USART2_AUTOCR         (STM32_USART2_BASE + STM32_USART_AUTOCR_OFFSET)
#endif

#if STM32_NUSART > 2
#  define STM32_USART3_CR1            (STM32_USART3_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART3_CR2            (STM32_USART3_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART3_CR3            (STM32_USART3_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART3_BRR            (STM32_USART3_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART3_GTPR           (STM32_USART3_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART3_RTOR           (STM32_USART3_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART3_RQR            (STM32_USART3_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART3_ISR            (STM32_USART3_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART3_ICR            (STM32_USART3_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART3_RDR            (STM32_USART3_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART3_TDR            (STM32_USART3_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART3_PRESC          (STM32_USART3_BASE + STM32_USART_PRESC_OFFSET)
#  define STM32_USART3_AUTOCR         (STM32_USART3_BASE + STM32_USART_AUTOCR_OFFSET)
#endif

#if STM32_NUART > 0
#  define STM32_UART4_CR1             (STM32_UART4_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_UART4_CR2             (STM32_UART4_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_UART4_CR3             (STM32_UART4_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_UART4_BRR             (STM32_UART4_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_UART4_GTPR            (STM32_UART4_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_UART4_RTOR            (STM32_UART4_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_UART4_RQR             (STM32_UART4_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_UART4_ISR             (STM32_UART4_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_UART4_ICR             (STM32_UART4_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_UART4_RDR             (STM32_UART4_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_UART4_TDR             (STM32_UART4_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_UART4_PRESC           (STM32_UART4_BASE + STM32_USART_PRESC_OFFSET)
#  define STM32_UART4_AUTOCR          (STM32_UART4_BASE + STM32_USART_AUTOCR_OFFSET)
#endif

#if STM32_NUART > 1
#  define STM32_UART5_CR1             (STM32_UART5_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_UART5_CR2             (STM32_UART5_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_UART5_CR3             (STM32_UART5_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_UART5_BRR             (STM32_UART5_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_UART5_GTPR            (STM32_UART5_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_UART5_RTOR            (STM32_UART5_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_UART5_RQR             (STM32_UART5_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_UART5_ISR             (STM32_UART5_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_UART5_ICR             (STM32_UART5_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_UART5_RDR             (STM32_UART5_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_UART5_TDR             (STM32_UART5_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_UART5_PRESC           (STM32_UART5_BASE + STM32_USART_PRESC_OFFSET)
#  define STM32_UART5_AUTOCR          (STM32_UART5_BASE + STM32_USART_AUTOCR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* USART control register 1 */

#define USART_CR1_UE                               (1 << 0)
#define USART_CR1_UESM                             (1 << 1)
#define USART_CR1_RE                               (1 << 2)
#define USART_CR1_TE                               (1 << 3)
#define USART_CR1_IDLEIE                           (1 << 4)
#define USART_CR1_RXNEIE                           (1 << 5)
#define USART_CR1_TCIE                             (1 << 6)
#define USART_CR1_TXEIE                            (1 << 7)
#define USART_CR1_TXEIE_TXFNFIE                    (1 << 7)
#define USART_CR1_PEIE                             (1 << 8)
#define USART_CR1_PS                               (1 << 9)
#define USART_CR1_PCE                              (1 << 10)
#define USART_CR1_WAKE                             (1 << 11)
#define USART_CR1_M_MASK                           0x10001000
#define USART_CR1_M0                               (1 << 12)
#define USART_CR1_MME                              (1 << 13)
#define USART_CR1_CMIE                             (1 << 14)
#define USART_CR1_OVER8                            (1 << 15)
#define USART_CR1_DEDT_SHIFT                       (16)
#define USART_CR1_DEDT_MASK                        (0x1f << USART_CR1_DEDT_SHIFT)
#define USART_CR1_DEDT(n)                          ((n) << USART_CR1_DEDT_SHIFT)
#define USART_CR1_DEAT_SHIFT                       (21)
#define USART_CR1_DEAT_MASK                        (0x1f << USART_CR1_DEAT_SHIFT)
#define USART_CR1_DEAT(n)                          ((n) << USART_CR1_DEAT_SHIFT)
#define USART_CR1_RTOIE                            (1 << 26)
#define USART_CR1_EOBIE                            (1 << 27)
#define USART_CR1_M1                               (1 << 28)
#define USART_CR1_FIFOEN                           (1 << 29)
#define USART_CR1_TXFEIE                           (1 << 30)
#define USART_CR1_RXFFIE                           (1 << 31)
#define USART_CR1_RXNEIE_RXFNEIE                   USART_CR1_RXNEIE
#define USART_CR1_ALLINTS                          (USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE | \
   USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_CMIE | \
   USART_CR1_RTOIE | USART_CR1_EOBIE | USART_CR1_TXFEIE | \
   USART_CR1_RXFFIE)

/* USART control register 2 */

#define USART_CR2_SLVEN                            (1 << 0)
#define USART_CR2_DIS_NSS                          (1 << 3)
#define USART_CR2_ADDM7                            (1 << 4)
#define USART_CR2_LBDL                             (1 << 5)
#define USART_CR2_LBDIE                            (1 << 6)
#define USART_CR2_LBCL                             (1 << 8)
#define USART_CR2_CPHA                             (1 << 9)
#define USART_CR2_CPOL                             (1 << 10)
#define USART_CR2_CLKEN                            (1 << 11)
#define USART_CR2_STOP_SHIFT                       (12)
#define USART_CR2_STOP_MASK                        (0x3 << USART_CR2_STOP_SHIFT)
#define USART_CR2_STOP(n)                          ((n) << USART_CR2_STOP_SHIFT)
#define USART_CR2_LINEN                            (1 << 14)
#define USART_CR2_SWAP                             (1 << 15)
#define USART_CR2_RXINV                            (1 << 16)
#define USART_CR2_TXINV                            (1 << 17)
#define USART_CR2_DATAINV                          (1 << 18)
#define USART_CR2_MSBFIRST                         (1 << 19)
#define USART_CR2_ABREN                            (1 << 20)
#define USART_CR2_ABRMODE_SHIFT                    (21)
#define USART_CR2_ABRMODE_MASK                     (0x3 << USART_CR2_ABRMODE_SHIFT)
#define USART_CR2_ABRMODE(n)                       ((n) << USART_CR2_ABRMODE_SHIFT)
#define USART_CR2_RTOEN                            (1 << 23)
#define USART_CR2_ADD_SHIFT                        (24)
#define USART_CR2_ADD_MASK                         (0xff << USART_CR2_ADD_SHIFT)
#define USART_CR2_ADD(n)                           ((n) << USART_CR2_ADD_SHIFT)
#define USART_CR2_STOP1                            USART_CR2_STOP(0)
#define USART_CR2_STOP0p5                          USART_CR2_STOP(1)
#define USART_CR2_STOP2                            USART_CR2_STOP(2)
#define USART_CR2_STOP1p5                          USART_CR2_STOP(3)
#define USART_CR2_ABRMODE_START                    USART_CR2_ABRMODE(0)
#define USART_CR2_ABRMODE_EDGES                    USART_CR2_ABRMODE(1)
#define USART_CR2_ABRMODE_7F                       USART_CR2_ABRMODE(2)
#define USART_CR2_ABRMODE_55                       USART_CR2_ABRMODE(3)
#define USART_CR2_ABRMOD_SHIFT                     USART_CR2_ABRMODE_SHIFT
#define USART_CR2_ABRMOD_MASK                      USART_CR2_ABRMODE_MASK
#define USART_CR2_ABRMOD(n)                        USART_CR2_ABRMODE(n)
#define USART_CR2_ABRMOD_START                     USART_CR2_ABRMODE_START
#define USART_CR2_ABRMOD_EDGES                     USART_CR2_ABRMODE_EDGES
#define USART_CR2_ABRMOD_7F                        USART_CR2_ABRMODE_7F
#define USART_CR2_ABRMOD_55                        USART_CR2_ABRMODE_55

/* USART control register 3 */

#define USART_CR3_EIE                              (1 << 0)
#define USART_CR3_IREN                             (1 << 1)
#define USART_CR3_IRLP                             (1 << 2)
#define USART_CR3_HDSEL                            (1 << 3)
#define USART_CR3_NACK                             (1 << 4)
#define USART_CR3_SCEN                             (1 << 5)
#define USART_CR3_DMAR                             (1 << 6)
#define USART_CR3_DMAT                             (1 << 7)
#define USART_CR3_RTSE                             (1 << 8)
#define USART_CR3_CTSE                             (1 << 9)
#define USART_CR3_CTSIE                            (1 << 10)
#define USART_CR3_ONEBIT                           (1 << 11)
#define USART_CR3_OVRDIS                           (1 << 12)
#define USART_CR3_DDRE                             (1 << 13)
#define USART_CR3_DEM                              (1 << 14)
#define USART_CR3_DEP                              (1 << 15)
#define USART_CR3_SCARCNT_SHIFT                    (17)
#define USART_CR3_SCARCNT_MASK                     (0x7 << USART_CR3_SCARCNT_SHIFT)
#define USART_CR3_SCARCNT(n)                       ((n) << USART_CR3_SCARCNT_SHIFT)
#define USART_CR3_TXFTIE                           (1 << 23)
#define USART_CR3_TCBGTIE                          (1 << 24)
#define USART_CR3_RXFTCFG_SHIFT                    (25)
#define USART_CR3_RXFTCFG_MASK                     (0x7 << USART_CR3_RXFTCFG_SHIFT)
#define USART_CR3_RXFTCFG(n)                       ((n) << USART_CR3_RXFTCFG_SHIFT)
#define USART_CR3_RXFTIE                           (1 << 28)
#define USART_CR3_TXFTCFG_SHIFT                    (29)
#define USART_CR3_TXFTCFG_MASK                     (0x7 << USART_CR3_TXFTCFG_SHIFT)
#define USART_CR3_TXFTCFG(n)                       ((n) << USART_CR3_TXFTCFG_SHIFT)
#define USART_CR3_SCARCNT2_SHIFT                   USART_CR3_SCARCNT_SHIFT
#define USART_CR3_SCARCNT2_MASK                    USART_CR3_SCARCNT_MASK
#define USART_CR3_RXFTCFG_1_8                      USART_CR3_RXFTCFG(0)
#define USART_CR3_RXFTCFG_1_4                      USART_CR3_RXFTCFG(1)
#define USART_CR3_RXFTCFG_1_2                      USART_CR3_RXFTCFG(2)
#define USART_CR3_RXFTCFG_3_4                      USART_CR3_RXFTCFG(3)
#define USART_CR3_RXFTCFG_7_8                      USART_CR3_RXFTCFG(4)
#define USART_CR3_RXFTCFG_FULL                     USART_CR3_RXFTCFG(5)
#define USART_CR3_TXFTCFG_1_8                      USART_CR3_TXFTCFG(0)
#define USART_CR3_TXFTCFG_1_4                      USART_CR3_TXFTCFG(1)
#define USART_CR3_TXFTCFG_1_2                      USART_CR3_TXFTCFG(2)
#define USART_CR3_TXFTCFG_3_4                      USART_CR3_TXFTCFG(3)
#define USART_CR3_TXFTCFG_7_8                      USART_CR3_TXFTCFG(4)
#define USART_CR3_TXFTCFG_FULL                     USART_CR3_TXFTCFG(5)

/* USART baud-rate register */

#define USART_BRR_LPUART_SHIFT                     (0)
#define USART_BRR_LPUART_MASK                      (0xfffff << USART_BRR_LPUART_SHIFT)
#define USART_BRR_LPUART(n)                        ((n) << USART_BRR_LPUART_SHIFT)
#define USART_BRR_BRR_SHIFT                        (0)
#define USART_BRR_BRR_MASK                         (0xffff << USART_BRR_BRR_SHIFT)
#define USART_BRR_BRR(n)                           ((n) << USART_BRR_BRR_SHIFT)
#define USART_BRR_FRAC_SHIFT                       (0)
#define USART_BRR_FRAC_MASK                        (0xf << USART_BRR_FRAC_SHIFT)
#define USART_BRR_MANT_SHIFT                       (4)
#define USART_BRR_MANT_MASK                        (0xfff << USART_BRR_MANT_SHIFT)

/* USART guard-time and prescaler register */

#define USART_GTPR_PSC_SHIFT                       (0)
#define USART_GTPR_PSC_MASK                        (0xff << USART_GTPR_PSC_SHIFT)
#define USART_GTPR_PSC(n)                          ((n) << USART_GTPR_PSC_SHIFT)
#define USART_GTPR_GT_SHIFT                        (8)
#define USART_GTPR_GT_MASK                         (0xff << USART_GTPR_GT_SHIFT)
#define USART_GTPR_GT(n)                           ((n) << USART_GTPR_GT_SHIFT)

/* USART receiver-timeout register */

#define USART_RTOR_RTO_SHIFT                       (0)
#define USART_RTOR_RTO_MASK                        (0xffffff << USART_RTOR_RTO_SHIFT)
#define USART_RTOR_RTO(n)                          ((n) << USART_RTOR_RTO_SHIFT)
#define USART_RTOR_BLEN_SHIFT                      (24)
#define USART_RTOR_BLEN_MASK                       (0xff << USART_RTOR_BLEN_SHIFT)
#define USART_RTOR_BLEN(n)                         ((n) << USART_RTOR_BLEN_SHIFT)

/* USART request register */

#define USART_RQR_ABRRQ                            (1 << 0)
#define USART_RQR_SBKRQ                            (1 << 1)
#define USART_RQR_MMRQ                             (1 << 2)
#define USART_RQR_RXFRQ                            (1 << 3)
#define USART_RQR_TXFRQ                            (1 << 4)

/* USART interrupt and status register */

#define USART_ISR_PE                               (1 << 0)
#define USART_ISR_FE                               (1 << 1)
#define USART_ISR_NE                               (1 << 2)
#define USART_ISR_ORE                              (1 << 3)
#define USART_ISR_IDLE                             (1 << 4)
#define USART_ISR_RXNE                             (1 << 5)
#define USART_ISR_TC                               (1 << 6)
#define USART_ISR_TXE                              (1 << 7)
#define USART_ISR_LBDF                             (1 << 8)
#define USART_ISR_CTSIF                            (1 << 9)
#define USART_ISR_CTS                              (1 << 10)
#define USART_ISR_RTOF                             (1 << 11)
#define USART_ISR_EOBF                             (1 << 12)
#define USART_ISR_UDR                              (1 << 13)
#define USART_ISR_ABRE                             (1 << 14)
#define USART_ISR_ABRF                             (1 << 15)
#define USART_ISR_BUSY                             (1 << 16)
#define USART_ISR_CMF                              (1 << 17)
#define USART_ISR_SBKF                             (1 << 18)
#define USART_ISR_RWU                              (1 << 19)
#define USART_ISR_TEACK                            (1 << 21)
#define USART_ISR_REACK                            (1 << 22)
#define USART_ISR_TXFE                             (1 << 23)
#define USART_ISR_RXFF                             (1 << 24)
#define USART_ISR_TCBGT                            (1 << 25)
#define USART_ISR_RXFT                             (1 << 26)
#define USART_ISR_TXFT                             (1 << 27)
#define USART_ISR_RXNE_RXFNE                       USART_ISR_RXNE
#define USART_ISR_TXE_TXFNF                        USART_ISR_TXE
#define USART_ISR_NF                               USART_ISR_NE
#define USART_ISR_LBD                              USART_ISR_LBDF

/* USART interrupt flag clear register */

#define USART_ICR_PECF                             (1 << 0)
#define USART_ICR_FECF                             (1 << 1)
#define USART_ICR_NECF                             (1 << 2)
#define USART_ICR_ORECF                            (1 << 3)
#define USART_ICR_IDLECF                           (1 << 4)
#define USART_ICR_TXFECF                           (1 << 5)
#define USART_ICR_TCCF                             (1 << 6)
#define USART_ICR_TCBGTCF                          (1 << 7)
#define USART_ICR_LBDCF                            (1 << 8)
#define USART_ICR_CTSCF                            (1 << 9)
#define USART_ICR_RTOCF                            (1 << 11)
#define USART_ICR_EOBCF                            (1 << 12)
#define USART_ICR_UDRCF                            (1 << 13)
#define USART_ICR_CMCF                             (1 << 17)
#define USART_ICR_NCF                              USART_ICR_NECF

/* USART receive-data register */

#define USART_RDR_RDR_SHIFT                        (0)
#define USART_RDR_RDR_MASK                         (0x1ff << USART_RDR_RDR_SHIFT)
#define USART_RDR_RDR(n)                           ((n) << USART_RDR_RDR_SHIFT)
#define USART_RDR_SHIFT                            USART_RDR_RDR_SHIFT
#define USART_RDR_MASK                             USART_RDR_RDR_MASK

/* USART transmit-data register */

#define USART_TDR_TDR_SHIFT                        (0)
#define USART_TDR_TDR_MASK                         (0x1ff << USART_TDR_TDR_SHIFT)
#define USART_TDR_TDR(n)                           ((n) << USART_TDR_TDR_SHIFT)
#define USART_TDR_SHIFT                            USART_TDR_TDR_SHIFT
#define USART_TDR_MASK                             USART_TDR_TDR_MASK

/* USART prescaler register */

#define USART_PRESC_PRESCALER_SHIFT                (0)
#define USART_PRESC_PRESCALER_MASK                 (0xf << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER(n)                   ((n) << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_DIV1                           USART_PRESC_PRESCALER(0)
#define USART_PRESC_DIV2                           USART_PRESC_PRESCALER(1)
#define USART_PRESC_DIV4                           USART_PRESC_PRESCALER(2)
#define USART_PRESC_DIV6                           USART_PRESC_PRESCALER(3)
#define USART_PRESC_DIV8                           USART_PRESC_PRESCALER(4)
#define USART_PRESC_DIV10                          USART_PRESC_PRESCALER(5)
#define USART_PRESC_DIV12                          USART_PRESC_PRESCALER(6)
#define USART_PRESC_DIV16                          USART_PRESC_PRESCALER(7)
#define USART_PRESC_DIV32                          USART_PRESC_PRESCALER(8)
#define USART_PRESC_DIV64                          USART_PRESC_PRESCALER(9)
#define USART_PRESC_DIV128                         USART_PRESC_PRESCALER(10)
#define USART_PRESC_DIV256                         USART_PRESC_PRESCALER(11)

/* USART autonomous-mode control register */

#define USART_AUTOCR_TDN_SHIFT                     (0)
#define USART_AUTOCR_TDN_MASK                      (0xffff << USART_AUTOCR_TDN_SHIFT)
#define USART_AUTOCR_TDN(n)                        ((n) << USART_AUTOCR_TDN_SHIFT)
#define USART_AUTOCR_TRIGPOL                       (1 << 16)
#define USART_AUTOCR_TRIGEN                        (1 << 17)
#define USART_AUTOCR_IDLEDIS                       (1 << 18)
#define USART_AUTOCR_TRIGSEL_SHIFT                 (19)
#define USART_AUTOCR_TRIGSEL_MASK                  (0xf << USART_AUTOCR_TRIGSEL_SHIFT)
#define USART_AUTOCR_TRIGSEL(n)                    ((n) << USART_AUTOCR_TRIGSEL_SHIFT)

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_UART_M33_V3_H */
