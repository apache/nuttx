/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_pwm.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PWM_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_PWM_CSR_OFFSET(n)  (0x000000 + (n) * 14) /* PWM control and status register */
#define RP23XX_PWM_DIV_OFFSET(n)  (0x000004 + (n) * 14) /* PWM clock divisor register */
#define RP23XX_PWM_CTR_OFFSET(n)  (0x000008 + (n) * 14) /* PWM counter register */
#define RP23XX_PWM_CC_OFFSET(n)   (0x00000c + (n) * 14) /* PWM compare register */
#define RP23XX_PWM_TOP_OFFSET(n)  (0x000010 + (n) * 14) /* PWM wrap value register */
#define RP23XX_PWM_EN_OFFSET     0x0000f0               /* PWM enable register */
#define RP23XX_PWM_INTR_OFFSET    0x0000f4              /* PWM raw interrupt register */
#define RP23XX_PWM_IRQ0_INTE_OFFSET    0x0000f8         /* PWM interrupt enable register */
#define RP23XX_PWM_IRQ0_INTF_OFFSET    0x0000fC         /* PWM interrupt force register */
#define RP23XX_PWM_IRQ0_INTS_OFFSET    0x000100         /* PWM interrupt status register */
#define RP23XX_PWM_IRQ1_INTE_OFFSET    0x000104
#define RP23XX_PWM_IRQ1_INTF_OFFSET    0x000108
#define RP23XX_PWM_IRQ1_INTS_OFFSET    0x00010c

/* Register definitions *****************************************************/

#define RP23XX_PWM_CSR(n)     (RP23XX_PWM_BASE + RP23XX_PWM_CSR_OFFSET(n))
#define RP23XX_PWM_DIV(n)     (RP23XX_PWM_BASE + RP23XX_PWM_DIV_OFFSET(n))
#define RP23XX_PWM_CTR(n)     (RP23XX_PWM_BASE + RP23XX_PWM_CTR_OFFSET(n))
#define RP23XX_PWM_CC(n)      (RP23XX_PWM_BASE + RP23XX_PWM_CC_OFFSET(n))
#define RP23XX_PWM_TOP(n)     (RP23XX_PWM_BASE + RP23XX_PWM_TOP_OFFSET(n))
#define RP23XX_PWM_EN         (RP23XX_PWM_BASE + RP23XX_PWM_EN_OFFSET)
#define RP23XX_PWM_INTR       (RP23XX_PWM_BASE + RP23XX_PWM_INTR_OFFSET)
#define RP23XX_PWM_IRQ0_INTE  (RP23XX_PWM_BASE + RP23XX_PWM_IRQ0_INTE_OFFSET)
#define RP23XX_PWM_IRQ0_INTF  (RP23XX_PWM_BASE + RP23XX_PWM_IRQ0_INTF_OFFSET)
#define RP23XX_PWM_IRQ0_INTS  (RP23XX_PWM_BASE + RP23XX_PWM_IRQ0_INTS_OFFSET)
#define RP23XX_PWM_IRQ1_INTE  (RP23XX_PWM_BASE + RP23XX_PWM_IRQ1_INTE_OFFSET)
#define RP23XX_PWM_IRQ1_INTF  (RP23XX_PWM_BASE + RP23XX_PWM_IRQ1_INTF_OFFSET)
#define RP23XX_PWM_IRQ1_INTS  (RP23XX_PWM_BASE + RP23XX_PWM_IRQ1_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_PWM_CSR_PH_ADV        (1 << 7) /* advance phase of counter by one */
#define RP23XX_PWM_CSR_PH_RET        (1 << 6) /* retard phase of counter by one */
#define RP23XX_PWM_CSR_DIVMODE_SHIFT (4)      /* divisor mode */
#define RP23XX_PWM_CSR_DIVMODE_MASK  (0x03 << RP23XX_PWM_CSR_DIVMODE_SHIFT)
#define RP23XX_PWM_CSR_B_INV         (1 << 3) /* invert output B */
#define RP23XX_PWM_CSR_A_INV         (1 << 2) /* invert output A */
#define RP23XX_PWM_CSR_PH_CORRECT    (1 << 1) /* enable phase correct modulation */
#define RP23XX_PWM_CSR_EN            (1 << 0) /* enable the PWM channel */

#define RP23XX_PWN_CSR_DIVMODE_DIV    0x00
#define RP23XX_PWN_CSR_DIVMODE_LEVEL  0x01
#define RP23XX_PWN_CSR_DIVMODE_RISE   0x02
#define RP23XX_PWN_CSR_DIVMODE_FALL   0x03
#define RP23XX_PWM_DIV_INT_SHIFT     (4)      /* divisor integer part */
#define RP23XX_PWM_DIV_INT_MASK      (0xff << RP23XX_PWM_DIV_INT_SHIFT)
#define RP23XX_PWM_DIV_FRAC_SHIFT    (0)      /* divisor fraction part */
#define RP23XX_PWM_DIV_FRAC_MASK     (0x0f << RP23XX_PWM_DIV_FRAC_SHIFT)

#define RP23XX_PWM_CC_B_SHIFT        (16)      /* channel B compare register */
#define RP23XX_PWM_CC_B_MASK         (0xffff << RP23XX_PWM_CC_B_SHIFT)
#define RP23XX_PWM_CC_A_SHIFT        (0)       /* channel A compare register */
#define RP23XX_PWM_CC_A_MASK         (0xffff << RP23XX_PWM_CC_A_SHIFT)

#define RP23XX_PWM_TOP_SHIFT         (0)       /* channel A compare register */
#define RP23XX_PWM_TOP_MASK          (0xffff << RP23XX_PWM_TOP_SHIFT)

/*  Bit mask for ENA, INTR, INTE, INTF, and INTS registers */

#define RP23XX_PWM_CH11             (1 << 11) /* PWM channel 11 */
#define RP23XX_PWM_CH10             (1 << 10) /* PWM channel 10 */
#define RP23XX_PWM_CH9              (1 << 9)  /* PWM channel 9 */
#define RP23XX_PWM_CH8              (1 << 8)  /* PWM channel 8 */
#define RP23XX_PWM_CH7              (1 << 7)  /* PWM channel 7 */
#define RP23XX_PWM_CH6              (1 << 6)  /* PWM channel 6 */
#define RP23XX_PWM_CH5              (1 << 5)  /* PWM channel 5 */
#define RP23XX_PWM_CH4              (1 << 4)  /* PWM channel 4 */
#define RP23XX_PWM_CH3              (1 << 3)  /* PWM channel 3 */
#define RP23XX_PWM_CH2              (1 << 2)  /* PWM channel 2 */
#define RP23XX_PWM_CH1              (1 << 1)  /* PWM channel 1 */
#define RP23XX_PWM_CH0              (1 << 0)  /* PWM channel 0 */

/****************************************************************************
 * The following IOCTL values set additional flags in the RP23XX PWM
 * device.
 ****************************************************************************/

/****************************************************************************
 * PWMIOC_RP23XX_SETINVERTPULSE sets the pulse invert flag.
 *
 * The argument is an integer where:
 *   bit zero is set to invert channel A
 *   bit one  is set to invert channel B
 ****************************************************************************/

#define PWMIOC_RP23XX_SETINVERTPULSE  _PWMIOC(0x80)

#define PWMIOC_RP23XX_GETINVERTPULSE  _PWMIOC(0x81)

/****************************************************************************
 * PWMIOC_RP23XX_SETPHASECORRECT sets phase correct flags.
 *
 * The argument is an integer which if non-zero sets the phase correct flag.
 ****************************************************************************/

#define PWMIOC_RP23XX_SETPHASECORRECT _PWMIOC(0x82)

#define PWMIOC_RP23XX_GETPHASECORRECT _PWMIOC(0x83)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PWM_H */
