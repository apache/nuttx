/****************************************************************************
 * arch/risc-v/src/k230/k230_hart.h
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

#ifndef __ARCH_RISCV_SRC_K230_K230_HART_H
#define __ARCH_RISCV_SRC_K230_K230_HART_H

/****************************************************************************
 * Preprocessor Macros
 ****************************************************************************/

#define CSR_MSECCFG     0x747
#define CSR_MSECCFGH    0x757

#define MSECCFG_MML     (1 << 0)
#define MSECCFG_RLB     (1 << 2)

/****************************************************************************
 * Public functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)

void k230_hart_init(void);                /* M-mode initialization */
bool k230_hart_is_big(void);              /* true if on big core */
void k230_hart_big_boot(uintptr_t addr);  /* turn on big core w/ boot addr */
void k230_hart_big_stop(void);            /* turn off big core */

#endif /* !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI) */
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_K230_K230_HART_H */
