/************************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_tmr.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_TMR_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_TMR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define MAX326_TMR_CNT_OFFSET       0x0000  /* Timer Counter Register */
#define MAX326_TMR_CMP_OFFSET       0x0004  /* Timer Compare Register */
#define MAX326_TMR_PWM_OFFSET       0x0008  /* Timer PWM Register */
#define MAX326_TMR_INT_OFFSET       0x000c  /* Timer Interrupt Register */
#define MAX326_TMR_CN_OFFSET        0x0010  /* Timer Control Register */

/* Register Addresses ***************************************************************/

#define MAX326_TMR0_CNT             (MAX326_TMR0_BASE + MAX326_TMR_CNT_OFFSET)
#define MAX326_TMR0_CMP             (MAX326_TMR0_BASE + MAX326_TMR_CMP_OFFSET)
#define MAX326_TMR0_PWM             (MAX326_TMR0_BASE + MAX326_TMR_PWM_OFFSET)
#define MAX326_TMR0_INT             (MAX326_TMR0_BASE + MAX326_TMR_INT_OFFSET)
#define MAX326_TMR0_CN              (MAX326_TMR0_BASE + MAX326_TMR_CN_OFFSET)

#define MAX326_TMR1_CNT             (MAX326_TMR1_BASE + MAX326_TMR_CNT_OFFSET)
#define MAX326_TMR1_CMP             (MAX326_TMR1_BASE + MAX326_TMR_CMP_OFFSET)
#define MAX326_TMR1_PWM             (MAX326_TMR1_BASE + MAX326_TMR_PWM_OFFSET)
#define MAX326_TMR1_INT             (MAX326_TMR1_BASE + MAX326_TMR_INT_OFFSET)
#define MAX326_TMR1_CN              (MAX326_TMR1_BASE + MAX326_TMR_CN_OFFSET)

#define MAX326_TMR2_CNT             (MAX326_TMR2_BASE + MAX326_TMR_CNT_OFFSET)
#define MAX326_TMR2_CMP             (MAX326_TMR2_BASE + MAX326_TMR_CMP_OFFSET)
#define MAX326_TMR2_PWM             (MAX326_TMR2_BASE + MAX326_TMR_PWM_OFFSET)
#define MAX326_TMR2_INT             (MAX326_TMR2_BASE + MAX326_TMR_INT_OFFSET)
#define MAX326_TMR2_CN              (MAX326_TMR2_BASE + MAX326_TMR_CN_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Timer Counter Register (32-bit count value) */
/* Timer Compare Register (32-bit compare value) */
/* Timer PWM Register (32-bit match or timer capture value) */

/* Timer Interrupt Register */

#define TMR_INT_IRQ                 (1 << 0)  /* Bit 0:  Timer interrupt */

/* Timer Control Register */

/* Full, four bit prescaler values: */

#define TMR_DIV1                    (0)       /* PRESM=0 PRESL=0: Fpclk / 1 */
#define TMR_DIV2                    (1)       /* PRESM=0 PRESL=1: Fpclk / 2 */
#define TMR_DIV4                    (2)       /* PRESM=0 PRESL=2: Fpclk / 4 */
#define TMR_DIV8                    (3)       /* PRESM=0 PRESL=3: Fpclk / 8 */
#define TMR_DIV16                   (4)       /* PRESM=0 PRESL=4: Fpclk / 16 */
#define TMR_DIV32                   (5)       /* PRESM=0 PRESL=5: Fpclk / 32 */
#define TMR_DIV64                   (6)       /* PRESM=0 PRESL=6: Fpclk / 64 */
#define TMR_DIV128                  (7)       /* PRESM=0 PRESL=7: Fpclk / 128 */
#define TMR_DIV256                  (8)       /* PRESM=1 PRESL=0: Fpclk / 256 */
#define TMR_DIV512                  (9)       /* PRESM=1 PRESL=1: Fpclk / 512 */
#define TMR_DIV1024                 (10)      /* PRESM=1 PRESL=2: Fpclk / 1024 */
#define TMR_DIV2048                 (11)      /* PRESM=1 PRESL=3: Fpclk / 2048 */
#define TMR_DIV4096                 (12)      /* PRESM=1 PRESL=4: Fpclk / 4096 */
#define TMR_DIV8192                 (13)      /* PRESM=1 PRESL=5: Fpclk / 8192 */

#define TMR_CN_TMODE_SHIFT          (0)       /* Bits 0-2: Timer Mode Select */
#define TMR_CN_TMODE_MASK           (7 << TMR_CN_TMODE_SHIFT)
#  define TMR_CN_TMODE_ONESHOT      (0 << TMR_CN_TMODE_SHIFT) /* One-Shot */
#  define TMR_CN_TMODE_CONTINUOUS   (1 << TMR_CN_TMODE_SHIFT) /* Continuous */
#  define TMR_CN_TMODE_COUNTER      (2 << TMR_CN_TMODE_SHIFT) /* Counter */
#  define TMR_CN_TMODE_PWM          (3 << TMR_CN_TMODE_SHIFT) /* PWM */
#  define TMR_CN_TMODE_CAPTURE      (4 << TMR_CN_TMODE_SHIFT) /* Capture */
#  define TMR_CN_TMODE_COMPARE      (5 << TMR_CN_TMODE_SHIFT) /* Compare */
#  define TMR_CN_TMODE_GATED        (6 << TMR_CN_TMODE_SHIFT) /* Gated */
#  define TMR_CN_TMODE_CAPCOMP      (7 << TMR_CN_TMODE_SHIFT) /* Capture/Compare */
#define TMR_CN_PRESL_SHIFT          (3)       /* Bits 3-5: Timer Prescaler Select LSBs */
#define TMR_CN_PRESL_MASK           (7 << TMR_CN_PRESL_SHIFT)
#  define TMR_CN_PRESL(n)           ((uint32_t)((n) & 7) << TMR_CN_PRESL_SHIFT)
#define TMR_CN_TPOL                 (1 << 6)  /* Bit 6:  Timer Polarity */
#define TMR_CN_TEN                  (1 << 7)  /* Bit 7:  Timer Enable */
#define TMR_CN_PRESM_SHIFT          (8)       /* Bit 8:  Timer Prescale Select MSB */
#define TMR_CN_PRESM_MASK           (1 << TMR_CN_PRESM_SHIFT)
#  define TMR_CN_PRESM(n)           ((uint32_t)((n) >> 3) << TMR_CN_PRESM_SHIFT)
#define TMR_CN_PWMSYNC              (1 << 9)  /* Bit 9:  PWM Synchronization Mode */
#define TMR_CN_NOLHPOL              (1 << 10) /* Bit 10: PWM Output phase A Polarity */
#define TMR_CN_NOLLPOL              (1 << 11) /* Bit 11: PWM Output phase A' Polarity */
#define TMR_CN_PWMCKBD              (1 << 12) /* Bit 12: PWM Output phase A' Disable */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_TMR_H */
