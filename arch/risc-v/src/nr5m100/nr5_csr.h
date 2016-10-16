/************************************************************************************
 * arch/riscv/src/nr5/nr5_csr.h
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ************************************************************************************/

/* CSR Definitions */

#ifndef __ARCH_RISCV_SRC_NR5_NR5_CSR_H
#define __ARCH_RISCV_SRC_NR5_NR5_CSR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#include <arch/arch.h>
#endif

#if defined(CONFIG_NR5_NR5M1XX)
#  include "chip/nr5m1xx_epic.h"
#endif

#include <arch/rv32im/csr.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define NR5_EPIC_IRQ_MASK        0x7E0
#define NR5_EPIC_PRI1            0x7E1
#define NR5_EPIC_PRI2            0x7E2
#define NR5_EPIC_PRI3            0x7E3
#define NR5_EPIC_PRIMASK         0x7E4
#define NR5_MSYSTICK_REG         0x7E5

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
void up_setsystick(uint32_t);

void up_setpri1bit(uint32_t);
void up_setpri2bit(uint32_t);
void up_setpri3bit(uint32_t);

void up_clearpri1bit(uint32_t);
void up_clearpri2bit(uint32_t);
void up_clearpri3bit(uint32_t);

void up_setirqmaskbit(uint32_t);
void up_clearirqmaskbit(uint32_t);

void up_disableints(void);
void up_enableints(void);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_NR5_NR5_CSR_H */

