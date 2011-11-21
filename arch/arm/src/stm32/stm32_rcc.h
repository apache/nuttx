/************************************************************************************
 * arch/arm/src/stm32/stm32_rcc.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_RRC_H
#define __ARCH_ARM_SRC_STM32_STM32_RRC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32F10XX)
#  include "chip/stm32f10xxx_rcc.h"
#elif defined(CONFIG_STM32_STM32F40XX)
#  include "chip/stm32f40xxx_rcc.h"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* This symbol references the Cortex-M3 vector table (as positioned by the the linker
 * script, ld.script or ld.script.dfu.  The standard location for the vector table is
 * at the beginning of FLASH at address 0x0800:0000.  If we are using the STMicro DFU
 * bootloader, then the vector table will be offset to a different location in FLASH
 * and we will need to set the NVIC vector location to this alternative location.
 */

extern uint32_t stm32_vectors[];	/* See stm32_vectors.S */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/* Called to change to new clock based on settings in board.h
 * 
 * NOTE:  This logic needs to be extended so that we can selected low-power
 * clocking modes as well!
 */

EXTERN void stm32_clockconfig(void);

/* Enable LSE Clock */

EXTERN void stm32_rcc_enablelse(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_RRC_H */
