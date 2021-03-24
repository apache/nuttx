/****************************************************************************
 * arch/arm/src/tiva/hardware/tiva_pwm.h
 *
 *   Copyright (C) 2016 Young Mu. All rights reserved.
 *   Author: Young Mu <young.mu@aliyun.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_PWM_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIVA_PWM_CTL_OFFSET             (0x0)   /* PWM Master Control */
#define TIVA_PWM_SYNC_OFFSET            (0x4)   /* PWM Time Base Sync */
#define TIVA_PWM_ENABLE_OFFSET          (0x8)   /* PWM Output Enable */
#define TIVA_PWM_INVERT_OFFSET          (0xc)   /* PWM Output Inversion */
#define TIVA_PWM_FAULT_OFFSET           (0x10)  /* PWM Output Fault */
#define TIVA_PWM_INTEN_OFFSET           (0x14)  /* PWM Interrupt Enable */
#define TIVA_PWM_RIS_OFFSET             (0x18)  /* PWM Raw Interrupt Status */
#define TIVA_PWM_ISC_OFFSET             (0x1c)  /* PWM Interrupt Status and Clear */
#define TIVA_PWM_STATUS_OFFSET          (0x20)  /* PWM Status */
#define TIVA_PWM_FAULTVAL_OFFSET        (0x24)  /* PWM Fault Condition Value */
#define TIVA_PWM_ENUPD_OFFSET           (0x28)  /* PWM Enable Update */

#define TIVA_PWMN_BASE                  (0x40)  /* PWMn Base */
#define TIVA_PWMN_INTERVAL              (0x40)  /* PWMn Interval */

#define TIVA_PWMN_CTL_OFFSET            (0x0)   /* PWMn Control */
#define TIVA_PWMN_INTEN_OFFSET          (0x4)   /* PWMn Interrupt and Trigger Enable */
#define TIVA_PWMN_RIS_OFFSET            (0x8)   /* PWMn Raw Interrupt Status */
#define TIVA_PWMN_ISC_OFFSET            (0xc)   /* PWMn Interrupt Status and Clear */
#define TIVA_PWMN_LOAD_OFFSET           (0x10)  /* PWMn Load */
#define TIVA_PWMN_COUNT_OFFSET          (0x14)  /* PWMn Counter */
#define TIVA_PWMN_CMPA_OFFSET           (0x18)  /* PWMn Compare A */
#define TIVA_PWMN_CMPB_OFFSET           (0x1c)  /* PWMn Compare B */
#define TIVA_PWMN_GENA_OFFSET           (0x20)  /* PWMn Generator A Control */
#define TIVA_PWMN_GENB_OFFSET           (0x24)  /* PWMn Generator B Control */
#define TIVA_PWMN_DBCTL_OFFSET          (0x28)  /* PWMn Dead-Band Control */
#define TIVA_PWMN_DBRISE_OFFSET         (0x2c)  /* PWMn Dead-Band Rising-Edge-Delay */
#define TIVA_PWMN_DBFALL_OFFSET         (0x30)  /* PWMn Dead-Band Falling-Edge-Delay */
#define TIVA_PWMN_FLTSRC0_OFFSET        (0x34)  /* PWMn Fault Source 0 */
#define TIVA_PWMN_FLTSRC1_OFFSET        (0x38)  /* PWMn Fault Source 1 */
#define TIVA_PWMN_MINFLTPER_OFFSET      (0x3c)  /* PWMn Minimum Fault Period */

#define TIVA_PWMN_FAULT_BASE            (0x800) /* PWMn Fault Base */
#define TIVA_PWMN_FAULT_INTERVAL        (0x80)  /* PWMn Fault Interval */

#define TIVA_PWMN_FAULT_SEN_OFFSET      (0x0)   /* PWMn Fault Pin Logic Sense */
#define TIVA_PWMN_FAULT_STAT0_OFFSET    (0x4)   /* PWMn Fault Status 0 */
#define TIVA_PWMN_FAULT_STAT1_OFFSET    (0x8)   /* PWMn Fault Status 1 */

#define TIVA_PWM_PP                     (0xfc0) /* PWM Peripheral Properties */
#define TIVA_PWM_CC                     (0xfc8) /* PWM Clock Configuration */

#define TIVA_PWMN_GENX_ACTCMPBD         (10)    /* (Bit) Action for Comparator B Down */
#define TIVA_PWMN_GENX_ACTCMPBU         (8)     /* (Bit) Action for Comparator B Up */
#define TIVA_PWMN_GENX_ACTCMPAD         (6)     /* (Bit) Action for Comparator A Down */
#define TIVA_PWMN_GENX_ACTCMPAU         (4)     /* (Bit) Action for Comparator A Up */
#define TIVA_PWMN_GENX_ACTLOAD          (2)     /* (Bit) Action for Counter equals LOAD */
#define TIVA_PWMN_GENX_ACTZERO          (0)     /* (Bit) Action for Counter equals ZERO */
#define GENX_INVERT                     (0x1)   /* (Value) Invert */
#define GENX_LOW                        (0x2)   /* (Value) Drive Low */
#define GENX_HIGH                       (0x3)   /* (Value) Drive High */

#define TIVA_PWM_CC_USEPWM              (8)     /* (Bit) Use PWM Clock Divisor */
#define TIVA_PWM_CC_PWMDIV              (0)     /* (Bit) PWM Clock Divider */
#define CC_USEPWM                       (0x1)   /* (Value) Use PWM divider as clock source */
#define CC_PWMDIV_2                     (0x0)   /* (Value) Divided by 2 */
#define CC_PWMDIV_4                     (0x1)   /* (Value) Divided by 4 */
#define CC_PWMDIV_8                     (0x2)   /* (Value) Divided by 8 */
#define CC_PWMDIV_16                    (0x3)   /* (Value) Divided by 16 */
#define CC_PWMDIV_32                    (0x4)   /* (Value) Divided by 32 */
#define CC_PWMDIV_64                    (0x5)   /* (Value) Divided by 64 */

#define TIVA_PWMN_CTL_ENABLE            (0)     /* (Bit) PWM Block Enable */
#define CTL_DISABLE                     (0)     /* (Value) Disable */
#define CTL_ENABLE                      (1)     /* (Value) Enable */

#define INTEN_GEN3                      (3)     /* (Bit) Enable PWM GEN3 Interrupt */
#define INTEN_GEN2                      (2)     /* (Bit) Enable PWM GEN2 Interrupt */
#define INTEN_GEN1                      (1)     /* (Bit) Enable PWM GEN1 Interrupt */
#define INTEN_GEN0                      (0)     /* (Bit) Enable PWM GEN0 Interrupt */
#define INT_DISABLE                     (0)     /* (Value) Disable */
#define INT_ENABLE                      (1)     /* (Value) Enable */

#define INTCMPBD                        (5)     /* (Bit) Interrupt for Counter=PWMnCMPB Down */
#define INTCMPBU                        (4)     /* (Bit) Interrupt for Counter=PWMnCMPB Up */
#define INTCMPAD                        (3)     /* (Bit) Interrupt for Counter=PWMnCMPA Down */
#define INTCMPAU                        (2)     /* (Bit) Interrupt for Counter=PWMnCMPA Up */
#define INTCNTLOAD                      (1)     /* (Bit) Interrupt for Counter=PWMnLOAD */
#define INTCNTZERO                      (0)     /* (Bit) Interrupt for Counter=0 */
#define INT_CLR                         (0)     /* (Value) Bit Clear */
#define INT_SET                         (1)     /* (Value) Bit Set */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_PWM_H */
