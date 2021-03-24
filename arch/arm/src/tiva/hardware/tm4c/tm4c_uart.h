/****************************************************************************
 * arch/arm/src/tiva/hardware/tm4c/tm4c_uart.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C_UART_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART register offsets ****************************************************/

#define TIVA_UART_DR_OFFSET        0x0000 /* UART Data */
#define TIVA_UART_RSR_OFFSET       0x0004 /* UART Receive Status */
#define TIVA_UART_ECR_OFFSET       0x0004 /* UART Error Clear */
#define TIVA_UART_FR_OFFSET        0x0018 /* UART Flag */
#define TIVA_UART_ILPR_OFFSET      0x0020 /* UART IrDA Low-Power Register */
#define TIVA_UART_IBRD_OFFSET      0x0024 /* UART Integer Baud-Rate Divisor*/
#define TIVA_UART_FBRD_OFFSET      0x0028 /* UART Fractional Baud-Rate Divisor */
#define TIVA_UART_LCRH_OFFSET      0x002c /* UART Line Control */
#define TIVA_UART_CTL_OFFSET       0x0030 /* UART Control */
#define TIVA_UART_IFLS_OFFSET      0x0034 /* UART Interrupt FIFO Level Select */
#define TIVA_UART_IM_OFFSET        0x0038 /* UART Interrupt Mask */
#define TIVA_UART_RIS_OFFSET       0x003c /* UART Raw Interrupt Status */
#define TIVA_UART_MIS_OFFSET       0x0040 /* UART Masked Interrupt Status */
#define TIVA_UART_ICR_OFFSET       0x0044 /* UART Interrupt Clear */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART_DMACTL_OFFSET    0x0048  /* UART DMA Control */
#  define TIVA_UART_9BITADDR_OFFSET  0x00a4  /* UART 9-Bit Self Address */
#  define TIVA_UART_9BITAMASK_OFFSET 0x00a8  /* UART 9-Bit Self Address Mask */
#  define TIVA_UART_PP_OFFSET        0x0fc0  /* UART Peripheral Properties */
#  define TIVA_UART_CC_OFFSET        0x0fc8  /* UART Clock Configuration */
#endif

#define TIVA_UART_PERIPHID4_OFFSET 0x0fd0 /* UART Peripheral Identification 4 */
#define TIVA_UART_PERIPHID5_OFFSET 0x0fd4 /* UART Peripheral Identification 5 */
#define TIVA_UART_PERIPHID6_OFFSET 0x0fd8 /* UART Peripheral Identification 6 */
#define TIVA_UART_PERIPHID7_OFFSET 0x0fdc /* UART Peripheral Identification 7 */
#define TIVA_UART_PERIPHID0_OFFSET 0x0fe0 /* UART Peripheral Identification 0 */
#define TIVA_UART_PERIPHID1_OFFSET 0x0fe4 /* UART Peripheral Identification 1 */
#define TIVA_UART_PERIPHID2_OFFSET 0x0fe8 /* UART Peripheral Identification 2 */
#define TIVA_UART_PERIPHID3_OFFSET 0x0fec /* UART Peripheral Identification 3 */
#define TIVA_UART_PCELLID0_OFFSET  0x0ff0 /* UART PrimeCell Identification 0 */
#define TIVA_UART_PCELLID1_OFFSET  0x0ff4 /* UART PrimeCell Identification 1 */
#define TIVA_UART_PCELLID2_OFFSET  0x0ff8 /* UART PrimeCell Identification 2 */
#define TIVA_UART_PCELLID3_OFFSET  0x0ffc /* UART PrimeCell Identification 3 */

/* UART register addresses **************************************************/

#define TIVA_UART_BASE(n)          (TIVA_UART0_BASE + (n)*0x01000)

#define TIVA_UART_DR(n)            (TIVA_UART_BASE(n) + TIVA_UART_DR_OFFSET)
#define TIVA_UART_RSR(n)           (TIVA_UART_BASE(n) + TIVA_UART_RSR_OFFSET)
#define TIVA_UART_ECR(n)           (TIVA_UART_BASE(n) + TIVA_UART_ECR_OFFSET)
#define TIVA_UART_FR(n)            (TIVA_UART_BASE(n) + TIVA_UART_FR_OFFSET)
#define TIVA_UART_ILPR(n)          (TIVA_UART_BASE(n) + TIVA_UART_ILPR_OFFSET)
#define TIVA_UART_IBRD(n)          (TIVA_UART_BASE(n) + TIVA_UART_IBRD_OFFSET)
#define TIVA_UART_FBRD(n)          (TIVA_UART_BASE(n) + TIVA_UART_FBRD_OFFSET)
#define TIVA_UART_LCRH(n)          (TIVA_UART_BASE(n) + TIVA_UART_LCRH_OFFSET)
#define TIVA_UART_CTL(n)           (TIVA_UART_BASE(n) + TIVA_UART_CTL_OFFSET)
#define TIVA_UART_IFLS(n)          (TIVA_UART_BASE(n) + TIVA_UART_IFLS_OFFSET)
#define TIVA_UART_IM(n)            (TIVA_UART_BASE(n) + TIVA_UART_IM_OFFSET)
#define TIVA_UART_RIS(n)           (TIVA_UART_BASE(n) + TIVA_UART_RIS_OFFSET)
#define TIVA_UART_MIS(n)           (TIVA_UART_BASE(n) + TIVA_UART_MIS_OFFSET)
#define TIVA_UART_ICR(n)           (TIVA_UART_BASE(n) + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART_DMACTL(n)      (TIVA_UART_BASE(n) + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART_9BITADDR(n)    (TIVA_UART_BASE(n) + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART_9BITAMASK(n)   (TIVA_UART_BASE(n) + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART_PP(n)          (TIVA_UART_BASE(n) + TIVA_UART_PP_OFFSET)
#  define TIVA_UART_CC(n)          (TIVA_UART_BASE(n) + TIVA_UART_CC_OFFSET)
#endif

#define TIVA_UART_PERIPHID4(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID4_OFFSET)
#define TIVA_UART_PERIPHID5(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID5_OFFSET)
#define TIVA_UART_PERIPHID6(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID6_OFFSET)
#define TIVA_UART_PERIPHID7(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID7_OFFSET)
#define TIVA_UART_PERIPHID0(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID0_OFFSET)
#define TIVA_UART_PERIPHID1(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID1_OFFSET)
#define TIVA_UART_PERIPHID2(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID2_OFFSET)
#define TIVA_UART_PERIPHID3(n)     (TIVA_UART_BASE(n) + TIVA_UART_PERIPHID3_OFFSET)
#define TIVA_UART_PCELLID0(n)      (TIVA_UART_BASE(n) + TIVA_UART_PCELLID0_OFFSET)
#define TIVA_UART_PCELLID1(n)      (TIVA_UART_BASE(n) + TIVA_UART_PCELLID1_OFFSET)
#define TIVA_UART_PCELLID2(n)      (TIVA_UART_BASE(n) + TIVA_UART_PCELLID2_OFFSET)
#define TIVA_UART_PCELLID3(n)      (TIVA_UART_BASE(n) + TIVA_UART_PCELLID3_OFFSET)

#if TIVA_NUARTS > 0
#  define TIVA_UART0_DR            (TIVA_UART0_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART0_RSR           (TIVA_UART0_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART0_ECR           (TIVA_UART0_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART0_FR            (TIVA_UART0_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART0_ILPR          (TIVA_UART0_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART0_IBRD          (TIVA_UART0_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART0_FBRD          (TIVA_UART0_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART0_LCRH          (TIVA_UART0_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART0_CTL           (TIVA_UART0_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART0_IFLS          (TIVA_UART0_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART0_IM            (TIVA_UART0_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART0_RIS           (TIVA_UART0_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART0_MIS           (TIVA_UART0_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART0_ICR           (TIVA_UART0_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART0_DMACTL        (TIVA_UART0_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART0_9BITADDR      (TIVA_UART0_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART0_9BITAMASK     (TIVA_UART0_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART0_PP            (TIVA_UART0_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART0_CC            (TIVA_UART0_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART0_PERIPHID4     (TIVA_UART0_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART0_PERIPHID5     (TIVA_UART0_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART0_PERIPHID6     (TIVA_UART0_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART0_PERIPHID7     (TIVA_UART0_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART0_PERIPHID0     (TIVA_UART0_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART0_PERIPHID1     (TIVA_UART0_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART0_PERIPHID2     (TIVA_UART0_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART0_PERIPHID3     (TIVA_UART0_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART0_PCELLID0      (TIVA_UART0_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART0_PCELLID1      (TIVA_UART0_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART0_PCELLID2      (TIVA_UART0_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART0_PCELLID3      (TIVA_UART0_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

#if TIVA_NUARTS > 1
#  define TIVA_UART1_DR            (TIVA_UART1_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART1_RSR           (TIVA_UART1_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART1_ECR           (TIVA_UART1_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART1_FR            (TIVA_UART1_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART1_ILPR          (TIVA_UART1_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART1_IBRD          (TIVA_UART1_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART1_FBRD          (TIVA_UART1_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART1_LCRH          (TIVA_UART1_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART1_CTL           (TIVA_UART1_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART1_IFLS          (TIVA_UART1_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART1_IM            (TIVA_UART1_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART1_RIS           (TIVA_UART1_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART1_MIS           (TIVA_UART1_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART1_ICR           (TIVA_UART1_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART1_DMACTL        (TIVA_UART1_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART1_9BITADDR      (TIVA_UART1_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART1_9BITAMASK     (TIVA_UART1_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART1_PP            (TIVA_UART1_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART1_CC            (TIVA_UART1_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART1_PERIPHID4     (TIVA_UART1_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART1_PERIPHID5     (TIVA_UART1_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART1_PERIPHID6     (TIVA_UART1_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART1_PERIPHID7     (TIVA_UART1_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART1_PERIPHID0     (TIVA_UART1_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART1_PERIPHID1     (TIVA_UART1_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART1_PERIPHID2     (TIVA_UART1_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART1_PERIPHID3     (TIVA_UART1_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART1_PCELLID0      (TIVA_UART1_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART1_PCELLID1      (TIVA_UART1_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART1_PCELLID2      (TIVA_UART1_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART1_PCELLID3      (TIVA_UART1_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

#if TIVA_NUARTS > 2
#  define TIVA_UART2_DR            (TIVA_UART2_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART2_RSR           (TIVA_UART2_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART2_ECR           (TIVA_UART2_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART2_FR            (TIVA_UART2_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART2_ILPR          (TIVA_UART2_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART2_IBRD          (TIVA_UART2_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART2_FBRD          (TIVA_UART2_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART2_LCRH          (TIVA_UART2_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART2_CTL           (TIVA_UART2_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART2_IFLS          (TIVA_UART2_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART2_IM            (TIVA_UART2_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART2_RIS           (TIVA_UART2_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART2_MIS           (TIVA_UART2_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART2_ICR           (TIVA_UART2_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART2_DMACTL        (TIVA_UART2_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART2_9BITADDR      (TIVA_UART2_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART2_9BITAMASK     (TIVA_UART2_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART2_PP            (TIVA_UART2_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART2_CC            (TIVA_UART2_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART2_PERIPHID4     (TIVA_UART2_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART2_PERIPHID5     (TIVA_UART2_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART2_PERIPHID6     (TIVA_UART2_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART2_PERIPHID7     (TIVA_UART2_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART2_PERIPHID0     (TIVA_UART2_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART2_PERIPHID1     (TIVA_UART2_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART2_PERIPHID2     (TIVA_UART2_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART2_PERIPHID3     (TIVA_UART2_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART2_PCELLID0      (TIVA_UART2_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART2_PCELLID1      (TIVA_UART2_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART2_PCELLID2      (TIVA_UART2_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART2_PCELLID3      (TIVA_UART2_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

#if TIVA_NUARTS > 3
#  define TIVA_UART3_DR            (TIVA_UART3_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART3_RSR           (TIVA_UART3_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART3_ECR           (TIVA_UART3_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART3_FR            (TIVA_UART3_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART3_ILPR          (TIVA_UART3_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART3_IBRD          (TIVA_UART3_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART3_FBRD          (TIVA_UART3_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART3_LCRH          (TIVA_UART3_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART3_CTL           (TIVA_UART3_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART3_IFLS          (TIVA_UART3_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART3_IM            (TIVA_UART3_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART3_RIS           (TIVA_UART3_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART3_MIS           (TIVA_UART3_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART3_ICR           (TIVA_UART3_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART3_DMACTL        (TIVA_UART3_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART3_9BITADDR      (TIVA_UART3_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART3_9BITAMASK     (TIVA_UART3_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART3_PP            (TIVA_UART3_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART3_CC            (TIVA_UART3_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART3_PERIPHID4     (TIVA_UART3_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART3_PERIPHID5     (TIVA_UART3_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART3_PERIPHID6     (TIVA_UART3_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART3_PERIPHID7     (TIVA_UART3_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART3_PERIPHID0     (TIVA_UART3_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART3_PERIPHID1     (TIVA_UART3_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART3_PERIPHID2     (TIVA_UART3_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART3_PERIPHID3     (TIVA_UART3_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART3_PCELLID0      (TIVA_UART3_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART3_PCELLID1      (TIVA_UART3_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART3_PCELLID2      (TIVA_UART3_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART3_PCELLID3      (TIVA_UART3_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

#if TIVA_NUARTS > 4
#  define TIVA_UART4_DR            (TIVA_UART4_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART4_RSR           (TIVA_UART4_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART4_ECR           (TIVA_UART4_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART4_FR            (TIVA_UART4_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART4_ILPR          (TIVA_UART4_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART4_IBRD          (TIVA_UART4_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART4_FBRD          (TIVA_UART4_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART4_LCRH          (TIVA_UART4_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART4_CTL           (TIVA_UART4_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART4_IFLS          (TIVA_UART4_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART4_IM            (TIVA_UART4_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART4_RIS           (TIVA_UART4_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART4_MIS           (TIVA_UART4_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART4_ICR           (TIVA_UART4_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART4_DMACTL        (TIVA_UART4_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART4_9BITADDR      (TIVA_UART4_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART4_9BITAMASK     (TIVA_UART4_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART4_PP            (TIVA_UART4_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART4_CC            (TIVA_UART4_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART4_PERIPHID4     (TIVA_UART4_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART4_PERIPHID5     (TIVA_UART4_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART4_PERIPHID6     (TIVA_UART4_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART4_PERIPHID7     (TIVA_UART4_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART4_PERIPHID0     (TIVA_UART4_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART4_PERIPHID1     (TIVA_UART4_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART4_PERIPHID2     (TIVA_UART4_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART4_PERIPHID3     (TIVA_UART4_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART4_PCELLID0      (TIVA_UART4_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART4_PCELLID1      (TIVA_UART4_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART4_PCELLID2      (TIVA_UART4_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART4_PCELLID3      (TIVA_UART4_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

#if TIVA_NUARTS > 5
#  define TIVA_UART5_DR            (TIVA_UART5_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART5_RSR           (TIVA_UART5_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART5_ECR           (TIVA_UART5_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART5_FR            (TIVA_UART5_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART5_ILPR          (TIVA_UART5_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART5_IBRD          (TIVA_UART5_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART5_FBRD          (TIVA_UART5_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART5_LCRH          (TIVA_UART5_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART5_CTL           (TIVA_UART5_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART5_IFLS          (TIVA_UART5_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART5_IM            (TIVA_UART5_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART5_RIS           (TIVA_UART5_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART5_MIS           (TIVA_UART5_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART5_ICR           (TIVA_UART5_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART5_DMACTL        (TIVA_UART5_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART5_9BITADDR      (TIVA_UART5_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART5_9BITAMASK     (TIVA_UART5_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART5_PP            (TIVA_UART5_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART5_CC            (TIVA_UART5_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART5_PERIPHID4     (TIVA_UART5_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART5_PERIPHID5     (TIVA_UART5_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART5_PERIPHID6     (TIVA_UART5_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART5_PERIPHID7     (TIVA_UART5_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART5_PERIPHID0     (TIVA_UART5_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART5_PERIPHID1     (TIVA_UART5_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART5_PERIPHID2     (TIVA_UART5_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART5_PERIPHID3     (TIVA_UART5_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART5_PCELLID0      (TIVA_UART5_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART5_PCELLID1      (TIVA_UART5_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART5_PCELLID2      (TIVA_UART5_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART5_PCELLID3      (TIVA_UART5_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

#if TIVA_NUARTS > 6
#  define TIVA_UART6_DR            (TIVA_UART6_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART6_RSR           (TIVA_UART6_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART6_ECR           (TIVA_UART6_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART6_FR            (TIVA_UART6_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART6_ILPR          (TIVA_UART6_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART6_IBRD          (TIVA_UART6_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART6_FBRD          (TIVA_UART6_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART6_LCRH          (TIVA_UART6_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART6_CTL           (TIVA_UART6_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART6_IFLS          (TIVA_UART6_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART6_IM            (TIVA_UART6_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART6_RIS           (TIVA_UART6_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART6_MIS           (TIVA_UART6_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART6_ICR           (TIVA_UART6_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART6_DMACTL        (TIVA_UART6_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART6_9BITADDR      (TIVA_UART6_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART6_9BITAMASK     (TIVA_UART6_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART6_PP            (TIVA_UART6_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART6_CC            (TIVA_UART6_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART6_PERIPHID4     (TIVA_UART6_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART6_PERIPHID5     (TIVA_UART6_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART6_PERIPHID6     (TIVA_UART6_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART6_PERIPHID7     (TIVA_UART6_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART6_PERIPHID0     (TIVA_UART6_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART6_PERIPHID1     (TIVA_UART6_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART6_PERIPHID2     (TIVA_UART6_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART6_PERIPHID3     (TIVA_UART6_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART6_PCELLID0      (TIVA_UART6_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART6_PCELLID1      (TIVA_UART6_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART6_PCELLID2      (TIVA_UART6_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART6_PCELLID3      (TIVA_UART6_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

#if TIVA_NUARTS > 7
#  define TIVA_UART7_DR            (TIVA_UART7_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART7_RSR           (TIVA_UART7_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART7_ECR           (TIVA_UART7_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART7_FR            (TIVA_UART7_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART7_ILPR          (TIVA_UART7_BASE + TIVA_UART_ILPR_OFFSET)
#  define TIVA_UART7_IBRD          (TIVA_UART7_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART7_FBRD          (TIVA_UART7_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART7_LCRH          (TIVA_UART7_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART7_CTL           (TIVA_UART7_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART7_IFLS          (TIVA_UART7_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART7_IM            (TIVA_UART7_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART7_RIS           (TIVA_UART7_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART7_MIS           (TIVA_UART7_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART7_ICR           (TIVA_UART7_BASE + TIVA_UART_ICR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_UART7_DMACTL        (TIVA_UART7_BASE + TIVA_UART_DMACTL_OFFSET)
#  define TIVA_UART7_9BITADDR      (TIVA_UART7_BASE + TIVA_UART_9BITADDR_OFFSET)
#  define TIVA_UART7_9BITAMASK     (TIVA_UART7_BASE + TIVA_UART_9BITAMASK_OFFSET)
#  define TIVA_UART7_PP            (TIVA_UART7_BASE + TIVA_UART_PP_OFFSET)
#  define TIVA_UART7_CC            (TIVA_UART7_BASE + TIVA_UART_CC_OFFSET)
#endif

#  define TIVA_UART7_PERIPHID4     (TIVA_UART7_BASE + TIVA_UART_PERIPHID4_OFFSET)
#  define TIVA_UART7_PERIPHID5     (TIVA_UART7_BASE + TIVA_UART_PERIPHID5_OFFSET)
#  define TIVA_UART7_PERIPHID6     (TIVA_UART7_BASE + TIVA_UART_PERIPHID6_OFFSET)
#  define TIVA_UART7_PERIPHID7     (TIVA_UART7_BASE + TIVA_UART_PERIPHID7_OFFSET)
#  define TIVA_UART7_PERIPHID0     (TIVA_UART7_BASE + TIVA_UART_PERIPHID0_OFFSET)
#  define TIVA_UART7_PERIPHID1     (TIVA_UART7_BASE + TIVA_UART_PERIPHID1_OFFSET)
#  define TIVA_UART7_PERIPHID2     (TIVA_UART7_BASE + TIVA_UART_PERIPHID2_OFFSET)
#  define TIVA_UART7_PERIPHID3     (TIVA_UART7_BASE + TIVA_UART_PERIPHID3_OFFSET)
#  define TIVA_UART7_PCELLID0      (TIVA_UART7_BASE + TIVA_UART_PCELLID0_OFFSET)
#  define TIVA_UART7_PCELLID1      (TIVA_UART7_BASE + TIVA_UART_PCELLID1_OFFSET)
#  define TIVA_UART7_PCELLID2      (TIVA_UART7_BASE + TIVA_UART_PCELLID2_OFFSET)
#  define TIVA_UART7_PCELLID3      (TIVA_UART7_BASE + TIVA_UART_PCELLID3_OFFSET)
#endif

/* UART register bit settings ***********************************************/

/* UART Data (DR) */

#define UART_DR_DATA_SHIFT         0         /* Bits 7-0: Data Transmitted or Received */
#define UART_DR_DATA_MASK          (0xff << UART_DR_DATA_SHIFT)
#define UART_DR_FE                 (1 << 8)  /* Bit 8:  UART Framing Error */
#define UART_DR_PE                 (1 << 9)  /* Bit 9:  UART Parity Error */
#define UART_DR_BE                 (1 << 10) /* Bit 10: UART Break Error */
#define UART_DR_OE                 (1 << 11) /* Bit 11: UART Overrun Error */

/* UART Receive Status (RSR) */

#define UART_RSR_FE                (1 << 0)  /* Bit 0:  UART Framing Error */
#define UART_RSR_PE                (1 << 1)  /* Bit 1:  UART Parity Error */
#define UART_RSR_BE                (1 << 2)  /* Bit 2:  UART Break Error */
#define UART_RSR_OE                (1 << 3)  /* Bit 3:  UART Overrun Error */

/* UART Error Clear (ECR) */

#define UART_ECR_FE                (1 << 0)  /* Bit 0:  UART Clear Framing Error */
#define UART_ECR_PE                (1 << 1)  /* Bit 1:  UART Clear Parity Error */
#define UART_ECR_BE                (1 << 2)  /* Bit 2:  UART Clear Break Error */
#define UART_ECR_OE                (1 << 3)  /* Bit 3:  UART Clear Overrun Error */

/* UART Flag (FR) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_FR_CTS              (1 << 0)  /* Bit 0:  Data Set Ready */
#  define UART_FR_DSR              (1 << 1)  /* Bit 1:  Data Set Ready */
#  define UART_FR_DCD              (1 << 2)  /* Bit 2:  Data Carrier Detect */
#endif

#define UART_FR_BUSY               (1 << 3)  /* Bit 3:  UART Busy */
#define UART_FR_RXFE               (1 << 4)  /* Bit 4:  UART Receive FIFO Empty */
#define UART_FR_TXFF               (1 << 5)  /* Bit 5:  UART Transmit FIFO Full */
#define UART_FR_RXFF               (1 << 6)  /* Bit 6:  UART Receive FIFO Full */
#define UART_FR_TXFE               (1 << 7)  /* Bit 7:  UART Transmit FIFO Empty */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_FR_RI               (1 << 8)  /* Bit 8:  Ring Indicator */
#endif

/* UART IrDA Low-Power Register (ILPR) */

#define UART_ILPR_DVSR_MASK        (0xff)    /* Bits 7-0: IrDA Low-Power Divisor */

/* UART Integer Baud-Rate Divisor (IBRD) */

#define UART_IBRD_DIVINT_MASK      (0xffff)  /* Bits 15-0: Integer Baud-Rate Divisor */

/* UART Fractional Baud-Rate Divisor (UARTFBRD) */

#define UART_FBRD_DIVFRAC_MASK     (0x3f)    /* Bits 5-0: Fractional Baud-Rate Divisor */

/* Register 7: UART Line Control (LCRH) */

#define UART_LCRH_BRK              (1 << 0)  /* Bit 0:  UART Send Break */
#define UART_LCRH_PEN              (1 << 1)  /* Bit 1:  UART Parity Enable */
#define UART_LCRH_EPS              (1 << 2)  /* Bit 2:  UART Even Parity Select */
#define UART_LCRH_STP2             (1 << 3)  /* Bit 3:  UART Two Stop Bits Select */
#define UART_LCRH_FEN              (1 << 4)  /* Bit 4:  UART Enable FIFOs */
#define UART_LCRH_WLEN_SHIFT       5         /* Bits 6-5: UART Word Length */

#define UART_LCRH_WLEN_MASK        (3 << UART_LCRH_WLEN_SHIFT)
#  define UART_LCRH_WLEN_5BITS     (0 << UART_LCRH_WLEN_SHIFT) /* 5-bits (reset) */
#  define UART_LCRH_WLEN_6BITS     (1 << UART_LCRH_WLEN_SHIFT) /* 6-bits */
#  define UART_LCRH_WLEN_7BITS     (2 << UART_LCRH_WLEN_SHIFT) /* 7-bits */
#  define UART_LCRH_WLEN_8BITS     (3 << UART_LCRH_WLEN_SHIFT) /* 8-bits */

#define UART_LCRH_SPS              (1 << 7)  /* Bit 7:  UART Stick Parity Select */

/* UART Control (CTL) */

#define UART_CTL_UARTEN            (1 << 0)  /* Bit 0:  UART Enable */
#define UART_CTL_SIREN             (1 << 1)  /* Bit 1:  UART SIR Enable */
#define UART_CTL_SIRLP             (1 << 2)  /* Bit 2:  UART SIR Low Power Mode */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_CTL_SMART           (1 << 3)  /* Bit 3:  ISO 7816 Smart Card Support */
#  define UART_CTL_EOT             (1 << 4)  /* Bit 4:  End of Transmission */
#  define UART_CTL_HSE             (1 << 5)  /* Bit 5:  High-Speed Enable */
#endif

#define UART_CTL_LBE               (1 << 7)  /* Bit 7:  UART Loop Back Enable */
#define UART_CTL_TXE               (1 << 8)  /* Bit 8:  UART Transmit Enable */
#define UART_CTL_RXE               (1 << 9)  /* Bit 9:  UART Receive Enable */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_CTL_DTR             (1 << 10) /* Bit 10: Data Terminal Ready */
#  define UART_CTL_RTS             (1 << 11) /* Bit 11: Request to Send */
#  define UART_CTL_RTSEN           (1 << 14) /* Bit 14: Enable Request to Send */
#  define UART_CTL_CTSEN           (1 << 15) /* Bit 15: Enable Clear To Send */
#endif

/* UART Interrupt FIFO Level Select (IFLS) */

#define UART_IFLS_TXIFLSEL_SHIFT   0         /* Bits 2-0: UART Transmit Interrupt FIFO Level Select */
#define UART_IFLS_TXIFLSEL_MASK    (7 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TXIFLSEL_18TH  (0 << UART_IFLS_TXIFLSEL_SHIFT) /* 1/8th full */
#  define UART_IFLS_TXIFLSEL_14TH  (1 << UART_IFLS_TXIFLSEL_SHIFT) /* 1/4th full */
#  define UART_IFLS_TXIFLSEL_HALF  (2 << UART_IFLS_TXIFLSEL_SHIFT) /* half full */
#  define UART_IFLS_TXIFLSEL_34TH  (3 << UART_IFLS_TXIFLSEL_SHIFT) /* 3/4th full */
#  define UART_IFLS_TXIFLSEL_78TH  (4 << UART_IFLS_TXIFLSEL_SHIFT) /* 7/8th full */

#define UART_IFLS_RXIFLSEL_SHIFT   3         /* Bits 5-3: UART Receive Interrupt FIFO Level Select */
#define UART_IFLS_RXIFLSEL_MASK    (7 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RXIFLSEL_18TH  (0 << UART_IFLS_RXIFLSEL_SHIFT) /* 1/8th full */
#  define UART_IFLS_RXIFLSEL_14TH  (1 << UART_IFLS_RXIFLSEL_SHIFT) /* 1/4th full */
#  define UART_IFLS_RXIFLSEL_HALF  (2 << UART_IFLS_RXIFLSEL_SHIFT) /* half full */
#  define UART_IFLS_RXIFLSEL_34TH  (3 << UART_IFLS_RXIFLSEL_SHIFT) /* 3/4th full */
#  define UART_IFLS_RXIFLSEL_78TH  (4 << UART_IFLS_RXIFLSEL_SHIFT) /* 7/8th full */

/* UART Interrupt Mask (IM) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_IM_RIIM             (1 << 0)  /* Bit 0:  UART Ring Indicator Modem Interrupt Mask */
#  define UART_IM_CTSIM            (1 << 1)  /* Bit 1:  UART Clear to Send Modem Interrupt Mask */
#  define UART_IM_DCDIM            (1 << 2)  /* Bit 2:  UART Data Carrier Detect Modem Interrupt Mask */
#  define UART_IM_DSRIM            (1 << 3)  /* Bit 3:  UART Data Set Ready Modem Interrupt Mask */
#endif

#define UART_IM_RXIM               (1 << 4)  /* Bit 4:  UART Receive Interrupt Mask */
#define UART_IM_TXIM               (1 << 5)  /* Bit 5:  UART Transmit Interrupt Mask */
#define UART_IM_RTIM               (1 << 6)  /* Bit 6:  UART Receive Time-Out Interrupt Mask */
#define UART_IM_FEIM               (1 << 7)  /* Bit 7:  UART Framing Error Interrupt Mask */
#define UART_IM_PEIM               (1 << 8)  /* Bit 8:  UART Parity Error Interrupt Mask */
#define UART_IM_BEIM               (1 << 9)  /* Bit 9:  UART Break Error Interrupt Mask */
#define UART_IM_OEIM               (1 << 10) /* Bit 10: UART Overrun Error Interrupt Mask */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_IM_EOTIM            (1 << 11) /* Bit 11: End of Transmission Interrupt Mask */
#  define UART_IM_9BITIM           (1 << 12) /* Bit 12: 9-Bit Mode Interrupt Mask */
#  define UART_IM_DMARXIM          (1 << 16) /* Bit 16: Receive DMA Interrupt Mask */
#  define UART_IM_DMATXIM          (1 << 17) /* Bit 17: Transmit DMA Interrupt Mask */
#endif

/* UART Raw Interrupt Status (RIS) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_RIS_RIIS            (1 << 0)  /* Bit 0:  UART Ring Indicator Modem Raw Interrupt Status */
#  define UART_RIS_CTSIS           (1 << 1)  /* Bit 1:  UART Clear to Send Modem Raw Interrupt Status */
#  define UART_RIS_DCDIS           (1 << 2)  /* Bit 2:  UART Data Carrier Detect Modem Raw Interrupt Status */
#  define UART_RIS_DSRIS           (1 << 3)  /* Bit 3:  UART Data Set Ready Modem Raw Interrupt Status */
#endif

#define UART_RIS_RXRIS             (1 << 4)  /* Bit 4:  UART Receive Raw Interrupt Status */
#define UART_RIS_TXRIS             (1 << 5)  /* Bit 5:  UART Transmit Raw Interrupt Status */
#define UART_RIS_RTRIS             (1 << 6)  /* Bit 6:  UART Receive Time-Out Raw Interrupt Status */
#define UART_RIS_FERIS             (1 << 7)  /* Bit 7:  UART Framing Error Raw Interrupt Status */
#define UART_RIS_PERIS             (1 << 8)  /* Bit 8:  UART Parity Error Raw Interrupt Status */
#define UART_RIS_BERIS             (1 << 9)  /* Bit 9:  UART Break Error Raw Interrupt Status */
#define UART_RIS_OERIS             (1 << 10) /* Bit 10: UART Overrun Error Raw Interrupt Status */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_RIS_EOTIS           (1 << 11) /* Bit 11: End of Transmission Raw Interrupt Status */
#  define UART_RIS_9BITIS          (1 << 12) /* Bit 12: 9-Bit Mode Raw Interrupt Status */
#  define UART_RIS_DMARXIS         (1 << 16) /* Bit 16: Receive DMA Raw Interrupt Status */
#  define UART_RIS_DMATXIS         (1 << 17) /* Bit 17: Transmit DMA Raw Interrupt Status */
#endif

/* UART Masked Interrupt Status (MIS) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_MIS_RIIS            (1 << 0)  /* Bit 0:  UART Ring Indicator Modem Masked Interrupt Status */
#  define UART_MIS_CTSIS           (1 << 1)  /* Bit 1:  UART Clear to Send Modem Masked Interrupt Status */
#  define UART_MIS_DCDIS           (1 << 2)  /* Bit 2:  UART Data Carrier Detect Modem Masked Interrupt Status */
#  define UART_MIS_DSRIS           (1 << 3)  /* Bit 3:  UART Data Set Ready Modem Masked Interrupt Status */
#endif

#define UART_MIS_RXMIS             (1 << 4)  /* Bit 4:  UART Receive Masked Interrupt Status */
#define UART_MIS_TXMIS             (1 << 5)  /* Bit 5:  UART Transmit Masked Interrupt Status */
#define UART_MIS_RTMIS             (1 << 6)  /* Bit 6:  UART Receive Time-Out Masked Interrupt Status */
#define UART_MIS_FEMIS             (1 << 7)  /* Bit 7:  UART Framing Error Masked Interrupt Status */
#define UART_MIS_PEMIS             (1 << 8)  /* Bit 8:  UART Parity Error Masked Interrupt Status */
#define UART_MIS_BEMIS             (1 << 9)  /* Bit 9:  UART Break Error Masked Interrupt Status */
#define UART_MIS_OEMIS             (1 << 10) /* Bit 10: UART Overrun Error Masked Interrupt Status */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_MIS_EOTIS           (1 << 11) /* Bit 11: End of Transmission Masked Interrupt Status */
#  define UART_MIS_9BITIS          (1 << 12) /* Bit 12: 9-Bit Mode Masked Interrupt Status */
#  define UART_MIS_DMARXIS         (1 << 16) /* Bit 16: Receive DMA Masked Interrupt Status */
#  define UART_MIS_DMATXIS         (1 << 17) /* Bit 17: Transmit DMA Masked Interrupt Status */
#endif

/* UART Interrupt Clear (ICR) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_ICR_RIIC            (1 << 0)  /* Bit 0:  UART Ring Indicator Modem Interrupt Clear */
#  define UART_ICR_CTSIC           (1 << 1)  /* Bit 1:  UART Clear to Send Modem Interrupt Clear */
#  define UART_ICR_DCDIC           (1 << 2)  /* Bit 2:  UART Data Carrier Detect Modem Interrupt Clear */
#  define UART_ICR_DSRIC           (1 << 3)  /* Bit 3:  UART Data Set Ready Modem Interrupt Clear */
#endif

#define UART_ICR_RXIC              (1 << 4)  /* Bit 4:  Receive Interrupt Clear */
#define UART_ICR_TXIC              (1 << 5)  /* Bit 5:  Transmit Interrupt Clear */
#define UART_ICR_RTIC              (1 << 6)  /* Bit 6:  Receive Time-Out Interrupt Clear */
#define UART_ICR_FEIC              (1 << 7)  /* Bit 7:  Framing Error Interrupt Clear */
#define UART_ICR_PEIC              (1 << 8)  /* Bit 8:  Parity Error Interrupt Clear */
#define UART_ICR_BEIC              (1 << 9)  /* Bit 9:  Break Error Interrupt Clear */
#define UART_ICR_OEIC              (1 << 10) /* Bit 10: Overrun Error Interrupt Clear */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_ICR_EOTIC           (1 << 11) /* Bit 11: End of Transmission Interrupt Clear */
#  define UART_ICR_9BITIC          (1 << 12) /* Bit 12: 9-Bit Mode Interrupt Clear */
#  define UART_ICR_DMARXIC         (1 << 16) /* Bit 16: Receive DMA Interrupt Clear */
#  define UART_ICR_DMATXIC         (1 << 17) /* Bit 17: Transmit DMA Interrupt Clear */
#endif

/* UART DMA Control (DMACTL) */

#define UART_DMACTL_RXDMAE         (1 << 0)  /* Bit 0:  Receive DMA Enable */
#define UART_DMACTL_TXDMAE         (1 << 1)  /* Bit 1:  Transmit DMA Enable */
#define UART_DMACTL_DMAERR         (1 << 2)  /* Bit 2:  Stop DMA receive if UART error */

/* UART 9-Bit Self Address (9BITADDR) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_9BITADDR_ADDR_SHIFT (0)       /* Bits 0-7: Self Address for 9-Bit Mode */
#  define UART_9BITADDR_ADDR_MASK  (0xff << UART_9BITADDR_ADDR_SHIFT)
#    define UART_9BITADDR_ADDR(n)  ((uint32_t)(n) << UART_9BITADDR_ADDR_SHIFT)
#  define UART_9BITADDR_9BITEN     (1 << 15) /* Bit 15: Enable 9-Bit Mode */
#endif

/* UART 9-Bit Self Address Mask (9BITAMASK) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_9BITAMASK_MASK_SHIFT (0)       /* Bits 0-7: Self Address Mask for 9-Bit Mode */
#  define UART_9BITAMASK_MASK_MASK  (0xff << UART_9BITAMASK_MASK_SHIFT)
#endif

/* UART Peripheral Properties (PP) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_PP_SC               (1 << 0)  /* Bit 0:  Smart Card Support */
#  define UART_PP_NB               (1 << 1)  /* Bit 1:  9-Bit Support */
#  define UART_PP_MS               (1 << 2)  /* Bit 2:  Modem Support */
#  define UART_PP_MSE              (1 << 3)  /* Bit 3:  Modem Support Extended */
#endif

/* UART Clock Configuration (CC) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define UART_CC_CS_SHIFT         (0)       /* Bit 0-3: UART Baud Clock Source */
#  define UART_CC_CS_MASK          (15 << UART_CC_CS_SHIFT)
#    define UART_CC_CS_SYSCLK      (0 << UART_CC_CS_SHIFT) /* System clock in RSCLKCFG */
#    define UART_CC_CS_ALTSRC      (5 << UART_CC_CS_SHIFT) /* Alternate source in ALTCLKCFG */
#endif

/* UART Peripheral Identification 4 (PERIPHID4) */

#define UART_PERIPHID4_MASK        (0xff)    /* UART Peripheral ID Register[7:0] */

/* UART Peripheral Identification 5 (UARTPERIPHID5 */

#define UART_PERIPHID5_MASK        (0xff)    /* UART Peripheral ID Register[15:8] */

/* UART Peripheral Identification 6 (UARTPERIPHID6) */

#define UART_PERIPHID6_MASK        (0xff)    /* UART Peripheral ID Register[23:16] */

/* UART Peripheral Identification 7 (UARTPERIPHID7) */

#define UART_PERIPHID7_MASK        (0xff)    /* UART Peripheral ID Register[31:24] */

/* UART Peripheral Identification 0 (UARTPERIPHID0) */

#define UART_PERIPHID0_MASK        (0xff)    /* UART Peripheral ID Register[7:0] */

/* UART Peripheral Identification 1 (UARTPERIPHID1) */

#define UART_PERIPHID1_MASK        (0xff)    /* UART Peripheral ID Register[15:8] */

/* UART Peripheral Identification 2 (UARTPERIPHID2) */

#define UART_PERIPHID2_MASK        (0xff)    /* UART Peripheral ID Register[23:16] */

/* UART Peripheral Identification 3 (UARTPERIPHID3) */

#define UART_PERIPHID3_MASK        (0xff)    /* UART Peripheral ID Register[31:24] */

/* UART PrimeCell Identification 0 (CELLID0) */

#define UART_CELLID0_MASK          (0xff)    /* UART PrimeCell ID Register[7:0] */

/* UART PrimeCell Identification 1 (UARTPCELLID1) */

#define UART_CELLID1_MASK          (0xff)    /* UART PrimeCell ID Register[15:8] */

/* UART PrimeCell Identification 2 (UARTPCELLID2) */

#define UART_CELLID02MASK          (0xff)    /* UART PrimeCell ID Register[23:16] */

/* UART PrimeCell Identification 3 (UARTPCELLID3) */

#define UART_CELLID3_MASK          (0xff)    /* UART PrimeCell ID Register[31:24] */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C_UART_H */
