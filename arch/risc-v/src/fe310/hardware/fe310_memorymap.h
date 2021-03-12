/****************************************************************************
 * arch/risc-v/src/fe310/hardware/fe310_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_MEMORYMAP_H
#define __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define FE310_CLINT_BASE   0x02000000
#define FE310_PLIC_BASE    0x0c000000

#define FE310_PRCI_BASE    0x10008000  /* 0x10008000 - 0x10008fff: PRCI  */

#define FE310_GPIO_BASE    0x10012000  /* 0x10012000 - 0x10012fff: GPIO  */
#define FE310_UART0_BASE   0x10013000  /* 0x10013000 - 0x10013fff: UART0 */
#define FE310_QSPI0_BASE   0x10014000  /* 0x10014000 - 0x10014fff: QSPI0 */
#define FE310_UART1_BASE   0x10023000  /* 0x10023000 - 0x10023fff: UART1 */

#endif /* __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_MEMORYMAP_H */
