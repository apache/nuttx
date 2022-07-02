/****************************************************************************
 * arch/risc-v/include/fe310/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_FE310_IRQ_H
#define __ARCH_RISCV_INCLUDE_FE310_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

/* Machine Global External Interrupt */

#define FE310_IRQ_UART0    (RISCV_IRQ_MEXT + 3)
#define FE310_IRQ_UART1    (RISCV_IRQ_MEXT + 4)

#define FE310_IRQ_GPIO0    (RISCV_IRQ_MEXT + 8)
#define FE310_IRQ_GPIO1    (RISCV_IRQ_MEXT + 9)
#define FE310_IRQ_GPIO2    (RISCV_IRQ_MEXT + 10)
#define FE310_IRQ_GPIO3    (RISCV_IRQ_MEXT + 11)
#define FE310_IRQ_GPIO4    (RISCV_IRQ_MEXT + 12)
#define FE310_IRQ_GPIO5    (RISCV_IRQ_MEXT + 13)
#define FE310_IRQ_GPIO6    (RISCV_IRQ_MEXT + 14)
#define FE310_IRQ_GPIO7    (RISCV_IRQ_MEXT + 15)
#define FE310_IRQ_GPIO8    (RISCV_IRQ_MEXT + 16)
#define FE310_IRQ_GPIO9    (RISCV_IRQ_MEXT + 17)
#define FE310_IRQ_GPIO10   (RISCV_IRQ_MEXT + 18)
#define FE310_IRQ_GPIO11   (RISCV_IRQ_MEXT + 19)
#define FE310_IRQ_GPIO12   (RISCV_IRQ_MEXT + 20)
#define FE310_IRQ_GPIO13   (RISCV_IRQ_MEXT + 21)
#define FE310_IRQ_GPIO14   (RISCV_IRQ_MEXT + 22)
#define FE310_IRQ_GPIO15   (RISCV_IRQ_MEXT + 23)
#define FE310_IRQ_GPIO16   (RISCV_IRQ_MEXT + 24)
#define FE310_IRQ_GPIO17   (RISCV_IRQ_MEXT + 25)
#define FE310_IRQ_GPIO18   (RISCV_IRQ_MEXT + 26)
#define FE310_IRQ_GPIO19   (RISCV_IRQ_MEXT + 27)
#define FE310_IRQ_GPIO20   (RISCV_IRQ_MEXT + 28)
#define FE310_IRQ_GPIO21   (RISCV_IRQ_MEXT + 29)
#define FE310_IRQ_GPIO22   (RISCV_IRQ_MEXT + 30)
#define FE310_IRQ_GPIO23   (RISCV_IRQ_MEXT + 31)
#define FE310_IRQ_GPIO24   (RISCV_IRQ_MEXT + 32)
#define FE310_IRQ_GPIO25   (RISCV_IRQ_MEXT + 33)
#define FE310_IRQ_GPIO26   (RISCV_IRQ_MEXT + 34)
#define FE310_IRQ_GPIO27   (RISCV_IRQ_MEXT + 35)
#define FE310_IRQ_GPIO28   (RISCV_IRQ_MEXT + 36)
#define FE310_IRQ_GPIO29   (RISCV_IRQ_MEXT + 37)
#define FE310_IRQ_GPIO30   (RISCV_IRQ_MEXT + 38)
#define FE310_IRQ_GPIO31   (RISCV_IRQ_MEXT + 39)

/* Total number of IRQs */

#define NR_IRQS            (FE310_IRQ_GPIO31 + 1)

#endif /* __ARCH_RISCV_INCLUDE_FE310_IRQ_H */
