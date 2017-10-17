/************************************************************************************
 * arch/arm/src/bcm2708/chip/bcm2708_systimr.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_BCM2708_CHIP_BCM2780_SYSTIMR_H
#define __ARCH_ARM_SRC_BCM2708_CHIP_BCM2780_SYSTIMR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip/bcm2708_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* System Tmer Register Offsets *****************************************************/

#define BCM_SYSTIMR_C_OFFSET    0x0000  /* System Timer Control/Status */
#define BCM_SYSTIMR_CLO_OFFSET  0x0004  /* System Timer Counter Lower 32 bits */
#define BCM_SYSTIMR_CHI_OFFSET  0x0008  /* System Timer Counter Higher 32 bits */
#define BCM_SYSTIMR_C0_OFFSET   0x000c  /* System Timer Compare 0 */
#define BCM_SYSTIMR_C1_OFFSET   0x0010  /* System Timer Compare 1 */
#define BCM_SYSTIMR_C2_OFFSET   0x0014  /* System Timer Compare 2 */
#define BCM_SYSTIMR_C3_OFFSET   0x0018  /* System Timer Compare 3 */

/* System Tmer Register Addresses ***************************************************/

#define BCM_SYSTIMR_C           (BCM_SYSTMR_VBASE+BCM_SYSTIMR_C_OFFSET)
#define BCM_SYSTIMR_CLO         (BCM_SYSTMR_VBASE+BCM_SYSTIMR_CLO_OFFSET)
#define BCM_SYSTIMR_CHI         (BCM_SYSTMR_VBASE+BCM_SYSTIMR_CHI_OFFSET)
#define BCM_SYSTIMR_C0          (BCM_SYSTMR_VBASE+BCM_SYSTIMR_C0_OFFSET)
#define BCM_SYSTIMR_C1          (BCM_SYSTMR_VBASE+BCM_SYSTIMR_C1_OFFSET)
#define BCM_SYSTIMR_C2          (BCM_SYSTMR_VBASE+BCM_SYSTIMR_C2_OFFSET)
#define BCM_SYSTIMR_C3          (BCM_SYSTMR_VBASE+BCM_SYSTIMR_C3_OFFSET)

/* System Tmer Register Bit Definitions ****************************************************/

/* System Timer Control/Status */

#define SYSTIMR_C_M0            (1 << 0)  /* Bit nn: System Timer Match 0 */
#define SYSTIMR_C_M1            (1 << 1)  /* Bit nn: System Timer Match 1 */
#define SYSTIMR_C_M2            (1 << 2)  /* Bit nn: System Timer Match 2 */
#define SYSTIMR_C_M3            (1 << 3)  /* Bit nn: System Timer Match 3 */

/* System Timer Counter Lower 32 bits (32 bit counter value) */
/* System Timer Counter Higher 32 bits (32 bit counter value) */

/* System Timer Compare 0 (32 bit compare value) */
/* System Timer Compare 1 (32 bit compare value) */
/* System Timer Compare 2 (32 bit compare value) */
/* System Timer Compare 3 (32 bit compare value) */

#endif /* __ARCH_ARM_SRC_BCM2708_CHIP_BCM2780_SYSTIMR_H */
