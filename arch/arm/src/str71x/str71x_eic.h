/************************************************************************************
 * arch/arm/src/str71x/str71x_eic.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_EIC_H
#define __ARCH_ARM_SRC_STR71X_STR71X_EIC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "<arch/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enhanced Interupt Controller (EIC) registers *************************************/

#define STR71X_EIC_ICR             (STR71X_EIC_BASE + 0x0000)  /* 32-bits wide */
#define STR71X_EIC_CICR            (STR71X_EIC_BASE + 0x0004)  /* 32-bits wide */
#define STR71X_EIC_CIPR            (STR71X_EIC_BASE + 0x0008)  /* 32-bits wide */
#define STR71X_EIC_IVR             (STR71X_EIC_BASE + 0x0018)  /* 32-bits wide */
#define STR71X_EIC_FIR             (STR71X_EIC_BASE + 0x001c)  /* 32-bits wide */
#define STR71X_EIC_IER0            (STR71X_EIC_BASE + 0x0020)  /* 32-bits wide */
#define STR71X_EIC_IPR0            (STR71X_EIC_BASE + 0x0040)  /* 32-bits wide */

#define STR71X_EIC_SIR(n)          (STR71X_EIC_BASE + 0x0060 + ((n) << 2))

#define STR71X_EIC_SIR0            (STR71X_EIC_BASE + 0x0060)  /* 32-bits wide */
#define STR71X_EIC_SIR1            (STR71X_EIC_BASE + 0x0064)  /* 32-bits wide */
#define STR71X_EIC_SIR2            (STR71X_EIC_BASE + 0x0068)  /* 32-bits wide */
#define STR71X_EIC_SIR3            (STR71X_EIC_BASE + 0x006c)  /* 32-bits wide */
#define STR71X_EIC_SIR4            (STR71X_EIC_BASE + 0x0070)  /* 32-bits wide */
#define STR71X_EIC_SIR5            (STR71X_EIC_BASE + 0x0074)  /* 32-bits wide */
#define STR71X_EIC_SIR6            (STR71X_EIC_BASE + 0x0078)  /* 32-bits wide */
#define STR71X_EIC_SIR7            (STR71X_EIC_BASE + 0x007c)  /* 32-bits wide */
#define STR71X_EIC_SIR8            (STR71X_EIC_BASE + 0x0080)  /* 32-bits wide */
#define STR71X_EIC_SIR9            (STR71X_EIC_BASE + 0x0084)  /* 32-bits wide */
#define STR71X_EIC_SIR10           (STR71X_EIC_BASE + 0x0088)  /* 32-bits wide */
#define STR71X_EIC_SIR11           (STR71X_EIC_BASE + 0x008c)  /* 32-bits wide */
#define STR71X_EIC_SIR12           (STR71X_EIC_BASE + 0x0090)  /* 32-bits wide */
#define STR71X_EIC_SIR13           (STR71X_EIC_BASE + 0x0094)  /* 32-bits wide */
#define STR71X_EIC_SIR14           (STR71X_EIC_BASE + 0x0098)  /* 32-bits wide */
#define STR71X_EIC_SIR15           (STR71X_EIC_BASE + 0x009c)  /* 32-bits wide */
#define STR71X_EIC_SIR16           (STR71X_EIC_BASE + 0x00a0)  /* 32-bits wide */
#define STR71X_EIC_SIR17           (STR71X_EIC_BASE + 0x00a4)  /* 32-bits wide */
#define STR71X_EIC_SIR18           (STR71X_EIC_BASE + 0x00a8)  /* 32-bits wide */
#define STR71X_EIC_SIR19           (STR71X_EIC_BASE + 0x00ac)  /* 32-bits wide */
#define STR71X_EIC_SIR20           (STR71X_EIC_BASE + 0x00b0)  /* 32-bits wide */
#define STR71X_EIC_SIR21           (STR71X_EIC_BASE + 0x00b4)  /* 32-bits wide */
#define STR71X_EIC_SIR22           (STR71X_EIC_BASE + 0x00b8)  /* 32-bits wide */
#define STR71X_EIC_SIR23           (STR71X_EIC_BASE + 0x00bc)  /* 32-bits wide */
#define STR71X_EIC_SIR24           (STR71X_EIC_BASE + 0x00c0)  /* 32-bits wide */
#define STR71X_EIC_SIR25           (STR71X_EIC_BASE + 0x00c4)  /* 32-bits wide */
#define STR71X_EIC_SIR26           (STR71X_EIC_BASE + 0x00c8)  /* 32-bits wide */
#define STR71X_EIC_SIR27           (STR71X_EIC_BASE + 0x00cc)  /* 32-bits wide */
#define STR71X_EIC_SIR28           (STR71X_EIC_BASE + 0x00d0)  /* 32-bits wide */
#define STR71X_EIC_SIR29           (STR71X_EIC_BASE + 0x00d4)  /* 32-bits wide */
#define STR71X_EIC_SIR30           (STR71X_EIC_BASE + 0x00d8)  /* 32-bits wide */
#define STR71X_EIC_SIR31           (STR71X_EIC_BASE + 0x00dc)  /* 32-bits wide */

/* Register bit settings ************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_EIC_H */
