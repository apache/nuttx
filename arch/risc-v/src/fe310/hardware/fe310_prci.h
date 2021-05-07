/****************************************************************************
 * arch/risc-v/src/fe310/hardware/fe310_prci.h
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

#ifndef __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PRCI_H
#define __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PRCI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FE310_HFROSCCFG (FE310_PRCI_BASE + 0x00)
#define FE310_HFXOSCCFG (FE310_PRCI_BASE + 0x04)
#define FE310_PLLCFG    (FE310_PRCI_BASE + 0x08)
#define FE310_PLLOUTDIV (FE310_PRCI_BASE + 0x0c)

#define HFXOSCCFG_HFXOSCEN  (0x1 << 30)
#define HFXOSCCFG_HFXOSCRDY (0x1 << 31)

#define PLLCFG_PLLSEL       (0x1 << 16)
#define PLLCFG_PLLREFSEL    (0x1 << 17)
#define PLLCFG_PLLBYPASS    (0x1 << 18)
#define PLLCFG_PLLLOCK      (0x1 << 31)

#endif /* __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PRCI_H */
