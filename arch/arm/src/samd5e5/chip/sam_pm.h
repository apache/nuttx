/****************************************************************************************************
 * arch/arm/src/samd5e5/chip/sam_pm.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_PM_H
#define __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_PM_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip/sam_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* PM register offsets ******************************************************************************/

#define SAM_PM_CTRLA_OFFSET              0x0000  /* Control A */
#define SAM_PM_SLEEPCFG_OFFSET           0x0001  /* Sleep configuration */
#define SAM_PM_INTENCLR_OFFSET           0x0004  /* Interrupt enable clear register */
#define SAM_PM_INTENSET_OFFSET           0x0005  /* Interrupt enable set register */
#define SAM_PM_INTFLAG_OFFSET            0x0006  /* Interrupt flag status and clear register */
#define SAM_PM_STDBYCFG_OFFSET           0x0008  /* Standy configuration */
#define SAM_PM_HIBCFG_OFFSET             0x0009  /* Hibernate configuration */
#define SAM_PM_BKUPCFG_OFFSET            0x000a  /* Backup configuration */

/* PM register addresses ****************************************************************************/

#define SAM_PM_CTRLA                     (SAM_PM_BASE + SAM_PM_CTRLA_OFFSET)
#define SAM_PM_SLEEPCFG                  (SAM_PM_BASE + SAM_PM_SLEEPCFG_OFFSET)
#define SAM_PM_INTENCLR                  (SAM_PM_BASE + SAM_PM_INTENCLR_OFFSET)
#define SAM_PM_INTENSET                  (SAM_PM_BASE + SAM_PM_INTENSET_OFFSET)
#define SAM_PM_INTFLAG                   (SAM_PM_BASE + SAM_PM_INTFLAG_OFFSET)
#define SAM_PM_STDBYCFG                  (SAM_PM_BASE + SAM_PM_STDBYCFG_OFFSET)
#define SAM_PM_HIBCFG                    (SAM_PM_BASE + SAM_PM_HIBCFG_OFFSET)
#define SAM_PM_BKUPCFG                   (SAM_PM_BASE + SAM_PM_BKUPCFG_OFFSET)

/* PM register bit definitions **********************************************************************/

/* Control A register */

#define PM_CTRLA_IORET                   (1 << 2)  /* Bit 2: I/O retention */

/* Sleep configuration register */

#define PM_SLEEPCFG_MODE_SHIFT           (0)        /* Bits 0-2: Idle Mode Configuration */
#define PM_SLEEPCFG_MODE_MASK            (7 << PM_SLEEPCFG_MODE_SHIFT)
#  define PM_SLEEPCFG_MODE_IDLE          (2 << PM_SLEEPCFG_MODE_SHIFT) /* CPU, AHBx, APBx clocks OFF */
#  define PM_SLEEPCFG_MODE_STANDBY       (4 << PM_SLEEPCFG_MODE_SHIFT) /* All clocks OFF */
#  define PM_SLEEPCFG_MODE_HIBERNATE     (5 << PM_SLEEPCFG_MODE_SHIFT) /* Backup domain is ON as well as
                                                                        * some PDRAMs */
#  define PM_SLEEPCFG_MODE_BACKUP        (6 << PM_SLEEPCFG_MODE_SHIFT) /* Only backup domain is
                                                                        * powered ON */
#  define PM_SLEEPCFG_MODE_OFF           (7 << PM_SLEEPCFG_MODE_SHIFT) /* All power domains are
                                                                        * powered OFF */

/* Interrupt enable clear, Interrupt enable set, and Interrupt flag status and clear registers */

#define PM_INT_SLEEPRDY                  (1 << 0)  /* Bit 0: Sleep mode entry ready interrupt */

/* Standy configuration */

#define PM_STDBYCFG_RAMCFG_SHIFT         (0)       /* Bits 0-1: RAM Configuration */
#define PM_STDBYCFG_RAMCFG_MASK          (3 << PM_STDBYCFG_RAMCFG_SHIFT)
#  define PM_STDBYCFG_RAMCFG_RET         (0 << PM_STDBYCFG_RAMCFG_SHIFT) /* System RAM is retained */
#  define PM_STDBYCFG_RAMCFG_PARTIAL     (1 << PM_STDBYCFG_RAMCFG_SHIFT) /* PD0 ACTIVE; PD1/2 handled by HW */
#  define PM_STDBYCFG_RAMCFG_OFF         (2 << PM_STDBYCFG_RAMCFG_SHIFT) /* Only the first 32Kb of system
                                                                          * RAM is retained */
#define PM_STDBYCFG_FASTWKUP_SHIFT       (4)       /* Bits 4-5: Fast Wakeup */
#define PM_STDBYCFG_FASTWKUP_MASK        (3 << PM_STDBYCFG_FASTWKUP_SHIFT)
#  define PM_STDBYCFG_FASTWKUP_NO        (0 << PM_STDBYCFG_FASTWKUP_SHIFT) /* Disabled */
#  define PM_STDBYCFG_FASTWKUP_NVM       (1 << PM_STDBYCFG_FASTWKUP_SHIFT) /* Enabled on NVM */
#  define PM_STDBYCFG_FASTWKUP_MAINVREG  (2 << PM_STDBYCFG_FASTWKUP_SHIFT) /* Enabled on MAINVREG */
#  define PM_STDBYCFG_FASTWKUP_BOTH      (3 << PM_STDBYCFG_FASTWKUP_SHIFT) /* Enabled on both */

/* Hibernate configuration */

#define PM_HIBCFG_RAMCFG_SHIFT           (0)       /* Bits 0-1: RAM Configuration */
#define PM_HIBCFG_RAMCFG_MASK            (3 << PM_HIBCFG_RAMCFG_SHIFT)
#  define PM_HIBCFG_RAMCFG_RET           (0 << PM_HIBCFG_RAMCFG_SHIFT) /* System RAM is retained */
#  define PM_HIBCFG_RAMCFG_PARTIAL       (1 << PM_HIBCFG_RAMCFG_SHIFT) /* PD0 ACTIVE; PD1/2 handled by HW */
#  define PM_HIBCFG_RAMCFG_OFF           (2 << PM_HIBCFG_RAMCFG_SHIFT) /* Only the first 32Kb of system
                                                                        * RAM is retained */
#define PM_HIBCFG_BRAMCFG_SHIFT          (0)       /* Bits 0-1: Backup RAM Configuration */
#define PM_HIBCFG_BRAMCFG_MASK           (3 << PM_HIBCFG_BRAMCFG_SHIFT)
#  define PM_HIBCFG_BRAMCFG_RET          (0 << PM_HIBCFG_BRAMCFG_SHIFT) /* System RAM is retained */
#  define PM_HIBCFG_BRAMCFG_PARTIAL      (1 << PM_HIBCFG_BRAMCFG_SHIFT) /* PD0 ACTIVE; PD1/2 handled by HW */
#  define PM_HIBCFG_BRAMCFG_OFF          (2 << PM_HIBCFG_BRAMCFG_SHIFT) /* Only the first 32Kb of system
                                                                           * RAM is retained */

/* Backup configuration */

#define PM_BKUPCFG_BRAMCFG_SHIFT         (0)       /* Bits 0-1: RAM Configuration */
#define PM_BKUPCFG_BRAMCFG_MASK          (3 << PM_BKUPCFG_BRAMCFG_SHIFT)
#  define PM_BKUPCFG_BRAMCFG_RET         (0 << PM_BKUPCFG_BRAMCFG_SHIFT) /* System RAM is retained */
#  define PM_BKUPCFG_BRAMCFG_PARTIAL     (1 << PM_BKUPCFG_BRAMCFG_SHIFT) /* PD0 ACTIVE; PD1/2 handled by HW */
#  define PM_BKUPCFG_BRAMCFG_OFF         (2 << PM_BKUPCFG_BRAMCFG_SHIFT) /* Only the first 32Kb of system
                                                                          * RAM is retained */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_PM_H */
