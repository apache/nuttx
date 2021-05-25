/****************************************************************************
 * arch/risc-v/include/k210/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_K210_IRQ_H
#define __ARCH_RISCV_INCLUDE_K210_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

/* IRQ 0-15 : (exception:interrupt=0) */

#define K210_IRQ_IAMISALIGNED  (0) /* Instruction Address Misaligned */
#define K210_IRQ_IAFAULT       (1) /* Instruction Address Fault */
#define K210_IRQ_IINSTRUCTION  (2) /* Illegal Instruction */
#define K210_IRQ_BPOINT        (3) /* Break Point */
#define K210_IRQ_LAMISALIGNED  (4) /* Load Address Misaligned */
#define K210_IRQ_LAFAULT       (5) /* Load Access Fault */
#define K210_IRQ_SAMISALIGNED  (6) /* Store/AMO Address Misaligned */
#define K210_IRQ_SAFAULT       (7) /* Store/AMO Access Fault */
#define K210_IRQ_ECALLU        (8) /* Environment Call from U-mode */
                                   /* 9-10: Reserved */

#define K210_IRQ_ECALLM       (11) /* Environment Call from M-mode */
                                   /* 12-15: Reserved */

/* IRQ 16- : (async event:interrupt=1) */

#define K210_IRQ_ASYNC        (16)
#define K210_IRQ_MSOFT    (K210_IRQ_ASYNC + 3)  /* Machine Software Int */
#define K210_IRQ_MTIMER   (K210_IRQ_ASYNC + 7)  /* Machine Timer Int */
#define K210_IRQ_MEXT     (K210_IRQ_ASYNC + 11) /* Machine External Int */

/* Machine Global External Interrupt */

#ifdef CONFIG_K210_WITH_QEMU
#define K210_IRQ_UART0    (K210_IRQ_MEXT + 4)
#else

#define K210_IRQ_NO        (K210_IRQ_MEXT + 0)
#define K210_IRQ_SPI0      (K210_IRQ_MEXT + 1)
#define K210_IRQ_SPI1      (K210_IRQ_MEXT + 2)
#define K210_IRQ_SPI_SLAVE (K210_IRQ_MEXT + 3)
#define K210_IRQ_SPI3      (K210_IRQ_MEXT + 4)
#define K210_IRQ_I2S0      (K210_IRQ_MEXT + 5)
#define K210_IRQ_I2S1      (K210_IRQ_MEXT + 6)
#define K210_IRQ_I2S2      (K210_IRQ_MEXT + 7)
#define K210_IRQ_I2C0      (K210_IRQ_MEXT + 8)
#define K210_IRQ_I2C1      (K210_IRQ_MEXT + 9)
#define K210_IRQ_I2C2      (K210_IRQ_MEXT + 10)
#define K210_IRQ_UART1     (K210_IRQ_MEXT + 11)
#define K210_IRQ_UART2     (K210_IRQ_MEXT + 12)
#define K210_IRQ_UART3     (K210_IRQ_MEXT + 13)
#define K210_IRQ_TIMER0A   (K210_IRQ_MEXT + 14)
#define K210_IRQ_TIMER0B   (K210_IRQ_MEXT + 15)
#define K210_IRQ_TIMER1A   (K210_IRQ_MEXT + 16)
#define K210_IRQ_TIMER1B   (K210_IRQ_MEXT + 17)
#define K210_IRQ_TIMER2A   (K210_IRQ_MEXT + 18)
#define K210_IRQ_TIMER2B   (K210_IRQ_MEXT + 19)
#define K210_IRQ_RTC       (K210_IRQ_MEXT + 20)
#define K210_IRQ_WDT0      (K210_IRQ_MEXT + 21)
#define K210_IRQ_WDT1      (K210_IRQ_MEXT + 22)
#define K210_IRQ_APB_GPIO  (K210_IRQ_MEXT + 23)
#define K210_IRQ_DVP       (K210_IRQ_MEXT + 24)
#define K210_IRQ_AI        (K210_IRQ_MEXT + 25)
#define K210_IRQ_FFT       (K210_IRQ_MEXT + 26)
#define K210_IRQ_DMA0      (K210_IRQ_MEXT + 27)
#define K210_IRQ_DMA1      (K210_IRQ_MEXT + 28)
#define K210_IRQ_DMA2      (K210_IRQ_MEXT + 29)
#define K210_IRQ_DMA3      (K210_IRQ_MEXT + 30)
#define K210_IRQ_DMA4      (K210_IRQ_MEXT + 31)
#define K210_IRQ_DMA5      (K210_IRQ_MEXT + 32)
#define K210_IRQ_UART0     (K210_IRQ_MEXT + 33)
#define K210_IRQ_GPIOHS0   (K210_IRQ_MEXT + 34)
#define K210_IRQ_GPIOHS1   (K210_IRQ_MEXT + 35)
#define K210_IRQ_GPIOHS2   (K210_IRQ_MEXT + 36)
#define K210_IRQ_GPIOHS3   (K210_IRQ_MEXT + 37)
#define K210_IRQ_GPIOHS4   (K210_IRQ_MEXT + 38)
#define K210_IRQ_GPIOHS5   (K210_IRQ_MEXT + 39)
#define K210_IRQ_GPIOHS6   (K210_IRQ_MEXT + 40)
#define K210_IRQ_GPIOHS7   (K210_IRQ_MEXT + 41)
#define K210_IRQ_GPIOHS8   (K210_IRQ_MEXT + 42)
#define K210_IRQ_GPIOHS9   (K210_IRQ_MEXT + 43)
#define K210_IRQ_GPIOHS10  (K210_IRQ_MEXT + 44)
#define K210_IRQ_GPIOHS11  (K210_IRQ_MEXT + 45)
#define K210_IRQ_GPIOHS12  (K210_IRQ_MEXT + 46)
#define K210_IRQ_GPIOHS13  (K210_IRQ_MEXT + 47)
#define K210_IRQ_GPIOHS14  (K210_IRQ_MEXT + 48)
#define K210_IRQ_GPIOHS15  (K210_IRQ_MEXT + 49)
#define K210_IRQ_GPIOHS16  (K210_IRQ_MEXT + 50)
#define K210_IRQ_GPIOHS17  (K210_IRQ_MEXT + 51)
#define K210_IRQ_GPIOHS18  (K210_IRQ_MEXT + 52)
#define K210_IRQ_GPIOHS19  (K210_IRQ_MEXT + 53)
#define K210_IRQ_GPIOHS20  (K210_IRQ_MEXT + 54)
#define K210_IRQ_GPIOHS21  (K210_IRQ_MEXT + 55)
#define K210_IRQ_GPIOHS22  (K210_IRQ_MEXT + 56)
#define K210_IRQ_GPIOHS23  (K210_IRQ_MEXT + 57)
#define K210_IRQ_GPIOHS24  (K210_IRQ_MEXT + 58)
#define K210_IRQ_GPIOHS25  (K210_IRQ_MEXT + 59)
#define K210_IRQ_GPIOHS26  (K210_IRQ_MEXT + 60)
#define K210_IRQ_GPIOHS27  (K210_IRQ_MEXT + 61)
#define K210_IRQ_GPIOHS28  (K210_IRQ_MEXT + 62)
#define K210_IRQ_GPIOHS29  (K210_IRQ_MEXT + 63)
#define K210_IRQ_GPIOHS30  (K210_IRQ_MEXT + 64)
#define K210_IRQ_GPIOHS31  (K210_IRQ_MEXT + 65)

/* Total number of IRQs */
#define NR_IRQS            (K210_IRQ_GPIOHS31 + 1)
#endif

#endif /* __ARCH_RISCV_INCLUDE_K210_IRQ_H */
