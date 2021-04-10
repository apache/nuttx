/****************************************************************************
 * arch/risc-v/include/arch.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_RISCV_INCLUDE_ARCH_H
#define __ARCH_RISCV_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include <arch/csr.h>

#ifdef CONFIG_ARCH_RV32IM
#  include <arch/rv32im/arch.h>
#endif

#ifdef CONFIG_ARCH_RV64GC
#  include <arch/rv64gc/arch.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to get the core and vendor ID, HART, arch and ISA codes, etc.
 */

#ifdef CONFIG_RV32IM_SYSTEM_CSRRS_SUPPORT

uint32_t up_getmisa(void);
uint32_t up_getarchid(void);
uint32_t up_getimpid(void);
uint32_t up_getvendorid(void);
uint32_t up_gethartid(void);

#else

#define up_getmisa() 0
#define up_getarchid() 0
#define up_getimpid() 0
#define up_getvendorid() 0
#define up_gethartid() 0

#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RV32IM_HW_MULDIV
uint32_t up_hard_mul(uint32_t a, uint32_t b);
uint32_t up_hard_mulh(uint32_t a, uint32_t b);
uint32_t up_hard_mulhsu(uint32_t a, uint32_t b);
uint32_t up_hard_mulhu(uint32_t a, uint32_t b);
uint32_t up_hard_div(uint32_t a, uint32_t b);
uint32_t up_hard_rem(uint32_t a, uint32_t b);
uint32_t up_hard_divu(uint32_t a, uint32_t b);
uint32_t up_hard_remu(uint32_t a, uint32_t b);
uint32_t time_hard_mul(uint32_t a, uint32_t b, uint32_t *t);
#endif

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_INCLUDE_ARCH_H */
