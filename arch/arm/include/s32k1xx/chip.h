/************************************************************************************
 * arch/arm/include/x32k1xx/chip.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_INCLUDE_S32K1XX_CHIP_H
#define __ARCH_ARM_INCLUDE_S32K1XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* NVIC priority levels *************************************************************/

/* Each priority field holds a priority value. The lower the value, the greater the
 * priority of the corresponding interrupt.
 */

#if defined(CONFIG_ARCH_CORTEXM4)
/* The Cortex-M4F core supports 16 programmable interrupt priority levels. */

#  define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is minimum priority */
#  define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#  define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#  define NVIC_SYSH_PRIORITY_STEP       0x10 /* Steps between priorities */

#elif defined(CONFIG_ARCH_CORTEXM0)
/* The Cortex-M0+ core supports 4 programmable interrupt priority levels. */

#  define NVIC_SYSH_PRIORITY_MIN        0xc0 /* All bits[7:4] set is minimum priority */
#  define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#  define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#  define NVIC_SYSH_PRIORITY_STEP       0x40 /* Steps between priorities */

#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_S32K1XX_CHIP_H */
