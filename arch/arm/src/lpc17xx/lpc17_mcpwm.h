/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_mcpwm.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_MCPWM_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_MCPWM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "lp17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC17_MCPWM_CON_OFFSET       0x0000 /* PWM Control read address */
#define LPC17_MCPWM_CONSET_OFFSET    0x0004 /* PWM Control set address */
#define LPC17_MCPWM_CONCLR_OFFSET    0x0008 /* PWM Control clear address */
#define LPC17_MCPWM_CAPCON_OFFSET    0x000c /* Capture Control read address */
#define LPC17_MCPWM_CAPCONSET_OFFSET 0x0010 /* Capture Control set address */
#define LPC17_MCPWM_CAPCONCLR_OFFSET 0x0014 /* Event Control clear address */
#define LPC17_MCPWM_TC0_OFFSET       0x0018 /* Timer Counter register, channel 0 */
#define LPC17_MCPWM_TC1_OFFSET       0x001c /* Timer Counter register, channel 1 */
#define LPC17_MCPWM_TC2_OFFSET       0x0020 /* Timer Counter register, channel 2 */
#define LPC17_MCPWM_LIM0_OFFSET      0x0024 /* Limit register, channel 0 */
#define LPC17_MCPWM_LIM1_OFFSET      0x0028 /* Limit register, channel 1 */
#define LPC17_MCPWM_LIM2_OFFSET      0x002c /* Limit register, channel 2 */
#define LPC17_MCPWM_MAT0_OFFSET      0x0030 /* Match register, channel 0 */
#define LPC17_MCPWM_MAT1_OFFSET      0x0034 /* Match register, channel 1 */
#define LPC17_MCPWM_MAT2_OFFSET      0x0038 /* Match register, channel 2 */
#define LPC17_MCPWM_DT_OFFSET        0x003c /* Dead time register */
#define LPC17_MCPWM_CP_OFFSET        0x0040 /* Commutation Pattern register */
#define LPC17_MCPWM_CAP0_OFFSET      0x0044 /* Capture register, channel 0 */
#define LPC17_MCPWM_CAP1_OFFSET      0x0048 /* Capture register, channel 1 */
#define LPC17_MCPWM_CAP2_OFFSET      0x004c /* Capture register, channel 2 */
#define LPC17_MCPWM_INTEN_OFFSET     0x0050 /* Interrupt Enable read address */
#define LPC17_MCPWM_INTENSET_OFFSET  0x0054 /* Interrupt Enable set address */
#define LPC17_MCPWM_INTENCLR_OFFSET  0x0058 /* Interrupt Enable clear address */
#define LPC17_MCPWM_CNTCON_OFFSET    0x005c /* Count Control read address */
#define LPC17_MCPWM_CNTCONSET_OFFSET 0x0060 /* Count Control set address */
#define LPC17_MCPWM_CNTCONCLR_OFFSET 0x0064 /* Count Control clear address */
#define LPC17_MCPWM_INTF_OFFSET      0x0068 /* Interrupt flags read address */
#define LPC17_MCPWM_INTFSET_OFFSET   0x006c /* Interrupt flags set address */
#define LPC17_MCPWM_INTFCLR_OFFSET   0x0070 /* Interrupt flags clear address */
#define LPC17_MCPWM_CAPCLR_OFFSET    0x0074 /* Capture clear address */

/* Register addresses ***************************************************************/

#define LPC17_MCPWM_CON              (LPC17_MCPWM_BASE+LPC17_MCPWM_CON_OFFSET)
#define LPC17_MCPWM_CONSET           (LPC17_MCPWM_BASE+LPC17_MCPWM_CONSET_OFFSET)
#define LPC17_MCPWM_CONCLR           (LPC17_MCPWM_BASE+LPC17_MCPWM_CONCLR_OFFSET)
#define LPC17_MCPWM_CAPCON           (LPC17_MCPWM_BASE+LPC17_MCPWM_CAPCON_OFFSET)
#define LPC17_MCPWM_CAPCONSET        (LPC17_MCPWM_BASE+LPC17_MCPWM_CAPCONSET_OFFSET)
#define LPC17_MCPWM_CAPCONCLR        (LPC17_MCPWM_BASE+LPC17_MCPWM_CAPCONCLR_OFFSET)
#define LPC17_MCPWM_TC0              (LPC17_MCPWM_BASE+LPC17_MCPWM_TC0_OFFSET)
#define LPC17_MCPWM_TC1              (LPC17_MCPWM_BASE+LPC17_MCPWM_TC1_OFFSET)
#define LPC17_MCPWM_TC2              (LPC17_MCPWM_BASE+LPC17_MCPWM_TC2_OFFSET)
#define LPC17_MCPWM_LIM0             (LPC17_MCPWM_BASE+LPC17_MCPWM_LIM0_OFFSET)
#define LPC17_MCPWM_LIM1             (LPC17_MCPWM_BASE+LPC17_MCPWM_LIM1_OFFSET)
#define LPC17_MCPWM_LIM2             (LPC17_MCPWM_BASE+LPC17_MCPWM_LIM2_OFFSET)
#define LPC17_MCPWM_MAT0             (LPC17_MCPWM_BASE+LPC17_MCPWM_MAT0_OFFSET)
#define LPC17_MCPWM_MAT1             (LPC17_MCPWM_BASE+LPC17_MCPWM_MAT1_OFFSET)
#define LPC17_MCPWM_MAT2             (LPC17_MCPWM_BASE+LPC17_MCPWM_MAT2_OFFSET)
#define LPC17_MCPWM_DT               (LPC17_MCPWM_BASE+LPC17_MCPWM_DT_OFFSET)
#define LPC17_MCPWM_CP               (LPC17_MCPWM_BASE+LPC17_MCPWM_CP_OFFSET)
#define LPC17_MCPWM_CAP0             (LPC17_MCPWM_BASE+LPC17_MCPWM_CAP0_OFFSET)
#define LPC17_MCPWM_CAP1             (LPC17_MCPWM_BASE+LPC17_MCPWM_CAP1_OFFSET)
#define LPC17_MCPWM_CAP2             (LPC17_MCPWM_BASE+LPC17_MCPWM_CAP2_OFFSET)
#define LPC17_MCPWM_INTEN            (LPC17_MCPWM_BASE+LPC17_MCPWM_INTEN_OFFSET)
#define LPC17_MCPWM_INTENSET         (LPC17_MCPWM_BASE+LPC17_MCPWM_INTENSET_OFFSET)
#define LPC17_MCPWM_INTENCLR         (LPC17_MCPWM_BASE+LPC17_MCPWM_INTENCLR_OFFSET)
#define LPC17_MCPWM_CNTCON           (LPC17_MCPWM_BASE+LPC17_MCPWM_CNTCON_OFFSET)
#define LPC17_MCPWM_CNTCONSET        (LPC17_MCPWM_BASE+LPC17_MCPWM_CNTCONSET_OFFSET)
#define LPC17_MCPWM_CNTCONCLR        (LPC17_MCPWM_BASE+LPC17_MCPWM_CNTCONCLR_OFFSET)
#define LPC17_MCPWM_INTF             (LPC17_MCPWM_BASE+LPC17_MCPWM_INTF_OFFSET)
#define LPC17_MCPWM_INTFSET          (LPC17_MCPWM_BASE+LPC17_MCPWM_INTFSET_OFFSET)
#define LPC17_MCPWM_INTFCLR          (LPC17_MCPWM_BASE+LPC17_MCPWM_INTFCLR_OFFSET)
#define LPC17_MCPWM_CAPCLR           (LPC17_MCPWM_BASE+LPC17_MCPWM_CAPCLR_OFFSET)

/* Register bit definitions *********************************************************/
/* PWM Control read address */

#define MCPWM_CON_

/* PWM Control set address */

#define MCPWM_CONSET_

/* PWM Control clear address */

#define MCPWM_CONCLR_

/* Capture Control read address */

#define MCPWM_CAPCON_

/* Capture Control set address */

#define MCPWM_CAPCONSET_

/* Event Control clear address */

#define MCPWM_CAPCONCLR_

/* Timer Counter register, channel 0 */

#define MCPWM_TC0_

/* Timer Counter register, channel 1 */

#define MCPWM_TC1_

/* Timer Counter register, channel 2 */

#define MCPWM_TC2_

/* Limit register, channel 0 */

#define MCPWM_LIM0_

/* Limit register, channel 1 */

#define MCPWM_LIM1_

/* Limit register, channel 2 */

#define MCPWM_LIM2_

/* Match register, channel 0 */

#define MCPWM_MAT0_

/* Match register, channel 1 */

#define MCPWM_MAT1_

/* Match register, channel 2 */

#define MCPWM_MAT2_

/* Dead time register */

#define MCPWM_DT_

/* Commutation Pattern register */

#define MCPWM_CP_

/* Capture register, channel 0 */

#define MCPWM_CAP0_

/* Capture register, channel 1 */

#define MCPWM_CAP1_

/* Capture register, channel 2 */

#define MCPWM_CAP2_

/* Interrupt Enable read address */

#define MCPWM_INTEN_

/* Interrupt Enable set address */

#define MCPWM_INTENSET_

/* Interrupt Enable clear address */

#define MCPWM_INTENCLR_

/* Count Control read address */

#define MCPWM_CNTCON_

/* Count Control set address */

#define MCPWM_CNTCONSET_

/* Count Control clear address */

#define MCPWM_CNTCONCLR_

/* Interrupt flags read address */

#define MCPWM_INTF_

/* Interrupt flags set address */

#define MCPWM_INTFSET_

/* Interrupt flags clear address */

#define MCPWM_INTFCLR_

/* Capture clear address */

#define MCPWM_CAPCLR_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_MCPWM_H */
