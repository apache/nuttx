/****************************************************************************
 * arch/risc-v/include/arch.h
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather,
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

#ifdef CONFIG_ARCH_RV32IM
#  include "rv32im/csr.h"
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
 * Public Variables
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
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_INCLUDE_ARCH_H */

