/************************************************************************************
 * arch/arm/src/am335x/hardware/am335x_intc.h
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_INTC_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_INTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/am335x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AM335X_INTC_REVISION_OFFSET     0x0000 /* IP revision code */
#define AM335X_INTC_SYSCONFIG_OFFSET    0x0010 /* Various parameters of the OCP interface */
#define AM335X_INTC_SYSSTATUS_OFFSET    0x0014 /* Status information about the module */
#define AM335X_INTC_SIR_IRQ_OFFSET      0x0040 /* Currently active IRQ interrupt number */
#define AM335X_INTC_SIR_FIQ_OFFSET      0x0044 /* Currently active FIQ interrupt number */
#define AM335X_INTC_CONTROL_OFFSET      0x0048 /* New interrupt agreement bits */
#define AM335X_INTC_PROTECTION_OFFSET   0x004C /* Controls protection of the other registers */
#define AM335X_INTC_IDLE_OFFSET         0x0050 /* Controls the clock auto-idle for the functional clock and the input synchronizers */
#define AM335X_INTC_IRQ_PRIO_OFFSET     0x0060 /* Currently active IRQ priority level */
#define AM335X_INTC_FIQ_PRIO_OFFSET     0x0064 /* Currently active FIQ priority level */
#define AM335X_INTC_THRESHOLD_OFFSET    0x0068 /* Sets the priority threshold */

#define AM335X_INTC_ITR_OFFSET(n)       (0x0080 + (((n) >> 5) * 0x20))
#define AM335X_INTC_ITR0_OFFSET         0x0080 /* Raw interrupt input status 0 before masking */
#define AM335X_INTC_ITR1_OFFSET         0x00A0 /* Raw interrupt input status 1 before masking */
#define AM335X_INTC_ITR2_OFFSET         0x00C0 /* Raw interrupt input status 2 before masking */
#define AM335X_INTC_ITR3_OFFSET         0x00E0 /* Raw interrupt input status 3 before masking */

#define AM335X_INTC_MIR_OFFSET(n)       (0x0084 + (((n) >> 5) * 0x20))
#define AM335X_INTC_MIR0_OFFSET         0x0084 /* Interrupt mask 0 */
#define AM335X_INTC_MIR1_OFFSET         0x00A4 /* Interrupt mask 1 */
#define AM335X_INTC_MIR2_OFFSET         0x00C4 /* Interrupt mask 2 */
#define AM335X_INTC_MIR3_OFFSET         0x00E4 /* Interrupt mask 3 */

#define AM335X_INTC_MIR_CLEAR_OFFSET(n) (0x0088 + (((n) >> 5) * 0x20))
#define AM335X_INTC_MIR_CLEAR0_OFFSET   0x0088 /* Clear the interrupt mask 0 bits */
#define AM335X_INTC_MIR_CLEAR1_OFFSET   0x00A8 /* Clear the interrupt mask 1 bits */
#define AM335X_INTC_MIR_CLEAR2_OFFSET   0x00C8 /* Clear the interrupt mask 2 bits */
#define AM335X_INTC_MIR_CLEAR3_OFFSET   0x00E8 /* Clear the interrupt mask 3 bits */

#define AM335X_INTC_MIR_SET_OFFSET(n)   (0x008C + (((n) >> 5) * 0x20))
#define AM335X_INTC_MIR_SET0_OFFSET     0x008C /* Set the interrupt mask 0 bits */
#define AM335X_INTC_MIR_SET1_OFFSET     0x00AC /* Set the interrupt mask 1 bits */
#define AM335X_INTC_MIR_SET2_OFFSET     0x00CC /* Set the interrupt mask 2 bits */
#define AM335X_INTC_MIR_SET3_OFFSET     0x00EC /* Set the interrupt mask 3 bits */

#define AM335X_INTC_ISR_SET_OFFSET(n)   (0x0090 + (((n) >> 5) * 0x20))
#define AM335X_INTC_ISR_SET0_OFFSET     0x0090 /* Set/read the software interrupt 0 bits */
#define AM335X_INTC_ISR_SET1_OFFSET     0x00B0 /* Set/read the software interrupt 1 bits */
#define AM335X_INTC_ISR_SET2_OFFSET     0x00D0 /* Set/read the software interrupt 2 bits */
#define AM335X_INTC_ISR_SET3_OFFSET     0x00F0 /* Set/read the software interrupt 3 bits */

#define AM335X_INTC_ISR_CLEAR_OFFSET(n) (0x0094 + (((n) >> 5) * 0x20))
#define AM335X_INTC_ISR_CLEAR0_OFFSET   0x0094 /* Clear the software interrupt 0 bits */
#define AM335X_INTC_ISR_CLEAR1_OFFSET   0x00B4 /* Clear the software interrupt 1 bits */
#define AM335X_INTC_ISR_CLEAR2_OFFSET   0x00D4 /* Clear the software interrupt 2 bits */
#define AM335X_INTC_ISR_CLEAR3_OFFSET   0x00F4 /* Clear the software interrupt 3 bits */

#define AM335X_INTC_PEND_IRQ_OFFSET(n)  (0x0098 + (((n) >> 5) * 0x20))
#define AM335X_INTC_PEND_IRQ0_OFFSET    0x0098 /* IRQ status 0 after masking */
#define AM335X_INTC_PEND_IRQ1_OFFSET    0x00B8 /* IRQ status 1 after masking */
#define AM335X_INTC_PEND_IRQ2_OFFSET    0x00D8 /* IRQ status 2 after masking */
#define AM335X_INTC_PEND_IRQ3_OFFSET    0x00F8 /* IRQ status 3 after masking */

#define AM335X_INTC_PEND_FIQ_OFFSET(n)  (0x009C + (((n) >> 5) * 0x20))
#define AM335X_INTC_PEND_FIQ0_OFFSET    0x009C /* FIQ status 0 after masking */
#define AM335X_INTC_PEND_FIQ1_OFFSET    0x00BC /* FIQ status 1 after masking */
#define AM335X_INTC_PEND_FIQ2_OFFSET    0x00DC /* FIQ status 2 after masking */
#define AM335X_INTC_PEND_FIQ3_OFFSET    0x00FC /* FIQ status 3 after masking */

#define AM335X_INTC_ILR_OFFSET(n)       (0x0100 + ((n) * 0x04)) /* Priority for the interrupts and the FIQ/IRQ steering */

/* Register virtual addresses *******************************************************/

#define AM335X_INTC_REVISION            (AM335X_INTC_VADDR + AM335X_INTC_REVISION_OFFSET)
#define AM335X_INTC_SYSCONFIG           (AM335X_INTC_VADDR + AM335X_INTC_SYSCONFIG_OFFSET)
#define AM335X_INTC_SYSSTATUS           (AM335X_INTC_VADDR + AM335X_INTC_SYSSTATUS_OFFSET)
#define AM335X_INTC_SIR_IRQ             (AM335X_INTC_VADDR + AM335X_INTC_SIR_IRQ_OFFSET)
#define AM335X_INTC_SIR_FIQ             (AM335X_INTC_VADDR + AM335X_INTC_SIR_FIQ_OFFSET)
#define AM335X_INTC_CONTROL             (AM335X_INTC_VADDR + AM335X_INTC_CONTROL_OFFSET)
#define AM335X_INTC_PROTECTION          (AM335X_INTC_VADDR + AM335X_INTC_PROTECTION_OFFSET)
#define AM335X_INTC_IDLE                (AM335X_INTC_VADDR + AM335X_INTC_IDLE_OFFSET)
#define AM335X_INTC_IRQ_PRIO            (AM335X_INTC_VADDR + AM335X_INTC_IRQ_PRIO_OFFSET)
#define AM335X_INTC_FIQ_PRIO            (AM335X_INTC_VADDR + AM335X_INTC_FIQ_PRIO_OFFSET)
#define AM335X_INTC_THRESHOLD           (AM335X_INTC_VADDR + AM335X_INTC_THRESHOLD_OFFSET)

#define AM335X_INTC_ITR(n)              (AM335X_INTC_VADDR + AM335X_INTC_ITR_OFFSET(n))
#define AM335X_INTC_ITR0                (AM335X_INTC_VADDR + AM335X_INTC_ITR0_OFFSET)
#define AM335X_INTC_ITR1                (AM335X_INTC_VADDR + AM335X_INTC_ITR1_OFFSET)
#define AM335X_INTC_ITR2                (AM335X_INTC_VADDR + AM335X_INTC_ITR2_OFFSET)
#define AM335X_INTC_ITR3                (AM335X_INTC_VADDR + AM335X_INTC_ITR3_OFFSET)

#define AM335X_INTC_MIR(n)              (AM335X_INTC_VADDR + AM335X_INTC_MIR_OFFSET(n))
#define AM335X_INTC_MIR0                (AM335X_INTC_VADDR + AM335X_INTC_MIR0_OFFSET)
#define AM335X_INTC_MIR1                (AM335X_INTC_VADDR + AM335X_INTC_MIR1_OFFSET)
#define AM335X_INTC_MIR2                (AM335X_INTC_VADDR + AM335X_INTC_MIR2_OFFSET)
#define AM335X_INTC_MIR3                (AM335X_INTC_VADDR + AM335X_INTC_MIR3_OFFSET)

#define AM335X_INTC_MIR_CLEAR(n)        (AM335X_INTC_VADDR + AM335X_INTC_MIR_CLEAR_OFFSET(n))
#define AM335X_INTC_MIR_CLEAR0          (AM335X_INTC_VADDR + AM335X_INTC_MIR_CLEAR0_OFFSET)
#define AM335X_INTC_MIR_CLEAR1          (AM335X_INTC_VADDR + AM335X_INTC_MIR_CLEAR1_OFFSET)
#define AM335X_INTC_MIR_CLEAR2          (AM335X_INTC_VADDR + AM335X_INTC_MIR_CLEAR2_OFFSET)
#define AM335X_INTC_MIR_CLEAR3          (AM335X_INTC_VADDR + AM335X_INTC_MIR_CLEAR3_OFFSET)

#define AM335X_INTC_MIR_SET(n)          (AM335X_INTC_VADDR + AM335X_INTC_MIR_SET_OFFSET(n))
#define AM335X_INTC_MIR_SET0            (AM335X_INTC_VADDR + AM335X_INTC_MIR_SET0_OFFSET)
#define AM335X_INTC_MIR_SET1            (AM335X_INTC_VADDR + AM335X_INTC_MIR_SET1_OFFSET)
#define AM335X_INTC_MIR_SET2            (AM335X_INTC_VADDR + AM335X_INTC_MIR_SET2_OFFSET)
#define AM335X_INTC_MIR_SET3            (AM335X_INTC_VADDR + AM335X_INTC_MIR_SET3_OFFSET)

#define AM335X_INTC_ISR_SET(n)          (AM335X_INTC_VADDR + AM335X_INTC_ISR_SET_OFFSET(n))
#define AM335X_INTC_ISR_SET0            (AM335X_INTC_VADDR + AM335X_INTC_ISR_SET0_OFFSET)
#define AM335X_INTC_ISR_SET1            (AM335X_INTC_VADDR + AM335X_INTC_ISR_SET1_OFFSET)
#define AM335X_INTC_ISR_SET2            (AM335X_INTC_VADDR + AM335X_INTC_ISR_SET2_OFFSET)
#define AM335X_INTC_ISR_SET3            (AM335X_INTC_VADDR + AM335X_INTC_ISR_SET3_OFFSET)

#define AM335X_INTC_ISR_CLEAR(n)        (AM335X_INTC_VADDR + AM335X_INTC_ISR_CLEAR_OFFSET(n))
#define AM335X_INTC_ISR_CLEAR0          (AM335X_INTC_VADDR + AM335X_INTC_ISR_CLEAR0_OFFSET)
#define AM335X_INTC_ISR_CLEAR1          (AM335X_INTC_VADDR + AM335X_INTC_ISR_CLEAR1_OFFSET)
#define AM335X_INTC_ISR_CLEAR2          (AM335X_INTC_VADDR + AM335X_INTC_ISR_CLEAR2_OFFSET)
#define AM335X_INTC_ISR_CLEAR3          (AM335X_INTC_VADDR + AM335X_INTC_ISR_CLEAR3_OFFSET)

#define AM335X_INTC_PEND_IRQ(n)         (AM335X_INTC_VADDR + AM335X_INTC_PEND_IRQ_OFFSET(n))
#define AM335X_INTC_PEND_IRQ0           (AM335X_INTC_VADDR + AM335X_INTC_PEND_IRQ0_OFFSET)
#define AM335X_INTC_PEND_IRQ1           (AM335X_INTC_VADDR + AM335X_INTC_PEND_IRQ1_OFFSET)
#define AM335X_INTC_PEND_IRQ2           (AM335X_INTC_VADDR + AM335X_INTC_PEND_IRQ2_OFFSET)
#define AM335X_INTC_PEND_IRQ3           (AM335X_INTC_VADDR + AM335X_INTC_PEND_IRQ3_OFFSET)

#define AM335X_INTC_PEND_FIQ(n)         (AM335X_INTC_VADDR + AM335X_INTC_PEND_FIQ_OFFSET(n))
#define AM335X_INTC_PEND_FIQ0           (AM335X_INTC_VADDR + AM335X_INTC_PEND_FIQ0_OFFSET)
#define AM335X_INTC_PEND_FIQ1           (AM335X_INTC_VADDR + AM335X_INTC_PEND_FIQ1_OFFSET)
#define AM335X_INTC_PEND_FIQ2           (AM335X_INTC_VADDR + AM335X_INTC_PEND_FIQ2_OFFSET)
#define AM335X_INTC_PEND_FIQ3           (AM335X_INTC_VADDR + AM335X_INTC_PEND_FIQ3_OFFSET)

#define AM335X_INTC_ILR(n)              (AM335X_INTC_VADDR + AM335X_INTC_ILR_OFFSET(n))

/* Register bit field definitions ***************************************************/

/* System Configuration */

#define INTC_SYSCONFIG_AUTOIDLE         (1 << 0)  /* Bit 0:  Internal OCP clock gating strategy */
#define INTC_SYSCONFIG_SOFTRESET        (1 << 1)  /* Bit 1:  Software reset */

/* System Status */

#define INTC_SYSSTATUS_RESETDONE        (1 << 0)  /* Bit 0:  Internal reset monitoring */

/* System Interrupt IRQ */

#define INTC_SIR_IRQ_ACTIVE_MASK        (0x0000007f)  /* Bits 0..6: Active IRQ number */
#define INTC_SIR_IRQ_SPURIOUS_MASK      (0xffffff80)  /* Bits 7..31:  Spurious IRQ flag */

/* System Interrupt FIQ */

#define INTC_SIR_FIQ_ACTIVE_MASK        (0x0000007f)  /* Bits 0..6:  Active FIQ number */
#define INTC_SIR_FIQ_SPURIOUS_MASK      (0xffffff80)  /* Bits 7..31:  Spurious FIQ flag */

/* Interrupt Control */

#define INTC_CONTROL_NEWIRQAGR          (1 << 0)  /* Bit 0:  New IRQ generation */
#define INTC_CONTROL_NEWFIQAGR          (1 << 1)  /* Bit 1:  Reset FIQ output and enable new FIQ generation */

/* Interrupt Protection */

#define INTC_PROTECTION_ENABLE          (1 << 0)  /* Bit 0:  Enabled protected register access */

/* Interrupt Idle */

#define INTC_IDLE_FUNCIDLE              (1 << 0)  /* Bit 0:  Functional clock auto-idle mode */
#define INTC_IDLE_TURBO                 (1 << 1)  /* Bit 1:  Input synchronizer clock auto-gating */

/* Interrupt IRQ Priority */

#define INTC_IRQ_PRIO_IRQ               (0x0000003f)  /* Bits 0..6:  Current IRQ priority */
#define INTC_IRQ_PRIO_SPURIOUS_FLAG     (0xffffffc0)  /* Bits 7..31:  Spurious IRQ flag  */

/* Interrupt FIQ Priority */

#define INTC_FIQ_PRIORITY_FIQ           (0x0000003f)  /* Bits 0..6:  Current FIQ priority */
#define INTC_FIQ_PRIO_SPURIOUS_FLAG     (0xffffffc0)  /* Bits 7..31:  Spurious FIQ flag  */

/* Interrupt Priority Threshold */

#define INTC_THRESHOLD_MASK             (0x0000003f)  /* Bits 0..6:  Priority threshold */
#define INTC_THRESHOLD_DISABLE          (0x000000ff)  /* Value FFh disables the threshold */

/* Interrupt Input 0-3 Status Before Masking */

#define INTC_ITR(n)                     (1 << ((n) & 0x1f)) /* n=0-127:  Interrupt status */

/* Interrupt Mask 0-3 */

#define INTC_MIR(n)                     (1 << ((n) & 0x1f)) /* n=0-127:  Interrupt mask */

/* Clear Interrupt Mask 0-3 */

#define INTC_MIR_CLEAR(n)               (1 << ((n) & 0x1f)) /* n=0-127:  Interrupt clear mask */

/* Set Interrupt Mask 0-3 */

#define INTC_MIR_SET(n)                 (1 << ((n) & 0x1f)) /* n=0-127:  Interrupt set mask */

/* Set Software Interrupt 0-3 / Currently Active Software Interrupts */

#define INTC_ISR_SET(n)                 (1 << ((n) & 0x1f)) /* n=0-127:  Set software interrupt */

/* Clear Software Interrupt 0-3 */

#define INTC_ISR_CLEAR(n)               (1 << ((n) & 0x1f)) /* n=0-127:  Clear software Interrupt */

/* IRQ Status After Masking 0-3 */

#define INTC_PEND_IRQ(n)                (1 << ((n) & 0x1f)) /* n=0-127:  Interrupt pending */

/* FIQ Status After Masking 0-3 */

#define INTC_PEND_FIQ(n)                (1 << ((n) & 0x1f)) /* n=0-127:  Interrupt pending */

/* Priority for the Interrupt 0-127 and the FIQ/IRQ Steering */

#define INTC_PRIO_MIN                   0
#define INTC_PRIO_MAX                   127

#define INTC_ILR_MAP_IRQ                (0 << 0)  /* Bit 0:  Interrupt is routed to IRQ */
#define INTC_ILR_MAP_FIQ                (1 << 0)  /* Bit 0:  Interrupt is routed to FIQ */
#define INTC_ILR_PRIO_SHIFT             (2)
#define INTC_ILR_PRIO_MASK              (127)  /* Bits 2..7:  Interrupt Priority */
#  define INTC_ILR_PRIO(p)              (((p) & INTC_ILR_PRIO_MASK) << INTC_ILR_PRIO_SHIFT)

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_INTC_H */
