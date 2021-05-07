/****************************************************************************
 * arch/arm/src/am335x/hardware/am335x_timer.h
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

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_TIMER_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/am335x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define AM335X_TMR_TIDR_OFFSET          0x0000 /* Identification Register */
#define AM335X_TMR_TIOCP_CFG_OFFSET     0x0010 /* Timer OCP Configuration Register */
#define AM335X_TMR_IRQ_EOI_OFFSET       0x0020 /* Timer IRQ End-of-Interrupt Register */
#define AM335X_TMR_IRQ_STAT_RAW_OFFSET  0x0024 /* Timer IRQ Status Raw Register */
#define AM335X_TMR_IRQ_STAT_OFFSET      0x0028 /* Timer IRQ Status Register */
#define AM335X_TMR_IRQ_EN_SET_OFFSET    0x002C /* Timer IRQ Enable Set Register */
#define AM335X_TMR_IRQ_EN_CLR_OFFSET    0x0030 /* Timer IRQ Enable Clear Register */
#define AM335X_TMR_IRQ_WAKE_EN_OFFSET   0x0034 /* Timer IRQ Wakeup Enable Register */
#define AM335X_TMR_TCLR_OFFSET          0x0038 /* Timer Control Register */
#define AM335X_TMR_TCRR_OFFSET          0x003C /* Timer Counter Register */
#define AM335X_TMR_TLDR_OFFSET          0x0040 /* Timer Load Register */
#define AM335X_TMR_TTGR_OFFSET          0x0044 /* Timer Trigger Register */
#define AM335X_TMR_TWPS_OFFSET          0x0048 /* Timer Write Posting Bits Register */
#define AM335X_TMR_TMAR_OFFSET          0x004C /* Timer Match Register */
#define AM335X_TMR_TCAR1_OFFSET         0x0050 /* Timer Capture 1 Register */
#define AM335X_TMR_TSICR_OFFSET         0x0054 /* Timer Synchronous Interface Control Register */
#define AM335X_TMR_TCAR2_OFFSET         0x0058 /* Timer Capture 2 Register */

#define AM335X_TMR1MS_TIDR_OFFSET       0x0000 /* Identification Register Section */
#define AM335X_TMR1MS_TIOCP_CFG_OFFSET  0x0010 /* 1ms Timer OCP Configuration Register Section */
#define AM335X_TMR1MS_TISTAT_OFFSET     0x0014 /* 1ms Timer Status Information Register */
#define AM335X_TMR1MS_TISR_OFFSET       0x0018 /* 1ms Timer IRQ Status Register */
#define AM335X_TMR1MS_TIER_OFFSET       0x001C /* 1ms Timer IRQ Enable Register */
#define AM335X_TMR1MS_TWER_OFFSET       0x0020 /* 1ms Timer IRQ Wakeup Enable Register */
#define AM335X_TMR1MS_TCLR_OFFSET       0x0024 /* 1ms Timer Control Register */
#define AM335X_TMR1MS_TCRR_OFFSET       0x0028 /* 1ms Timer Counter Register */
#define AM335X_TMR1MS_TLDR_OFFSET       0x002C /* 1ms Timer Load Register */
#define AM335X_TMR1MS_TTGR_OFFSET       0x0030 /* 1ms Timer Trigger Register */
#define AM335X_TMR1MS_TWPS_OFFSET       0x0034 /* 1ms Timer Write Posting Bits Register */
#define AM335X_TMR1MS_TMAR_OFFSET       0x0038 /* 1ms Timer Match Register */
#define AM335X_TMR1MS_TCAR1_OFFSET      0x003C /* 1ms Timer Capture 1 Register */
#define AM335X_TMR1MS_TSICR_OFFSET      0x0040 /* 1ms Timer Synchronous Interface Control Register */
#define AM335X_TMR1MS_TCAR2_OFFSET      0x0044 /* 1ms Timer Capture 2 Register */
#define AM335X_TMR1MS_TPIR_OFFSET       0x0048 /* 1ms Timer Positive Increment Register */
#define AM335X_TMR1MS_TNIR_OFFSET       0x004C /* 1ms Timer Negative Increment Register */
#define AM335X_TMR1MS_TCVR_OFFSET       0x0050 /* 1ms Timer Counter Value Register */
#define AM335X_TMR1MS_TOCR_OFFSET       0x0054 /* 1ms Timer Overflow Counter Register */
#define AM335X_TMR1MS_TOWR_OFFSET       0x0058 /* 1ms Timer Overflow Interrupts Register */

/* Register virtual addresses ***********************************************/

#define AM335X_TMR1MS_TIOCP_CFG         (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TIOCP_CFG_OFFSET)
#define AM335X_TMR1MS_TISTAT            (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TISTAT_OFFSET)
#define AM335X_TMR1MS_TISR              (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TISR_OFFSET)
#define AM335X_TMR1MS_TIER              (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TIER_OFFSET)
#define AM335X_TMR1MS_TCLR              (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TCLR_OFFSET)
#define AM335X_TMR1MS_TCRR              (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TCRR_OFFSET)
#define AM335X_TMR1MS_TLDR              (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TLDR_OFFSET)
#define AM335X_TMR1MS_TPIR              (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TPIR_OFFSET)
#define AM335X_TMR1MS_TNIR              (AM335X_DMTIMER1_1MS_VADDR + AM335X_TMR1MS_TNIR_OFFSET)

#define AM335X_TMR2_TIOCP_CFG           (AM335X_DMTIMER2_VADDR + AM335X_TMR_TIOCP_CFG_OFFSET)
#define AM335X_TMR2_IRQ_STAT            (AM335X_DMTIMER2_VADDR + AM335X_TMR_IRQ_STAT_OFFSET)
#define AM335X_TMR2_IRQ_EN_SET          (AM335X_DMTIMER2_VADDR + AM335X_TMR_IRQ_EN_SET_OFFSET)
#define AM335X_TMR2_IRQ_EN_CLR          (AM335X_DMTIMER2_VADDR + AM335X_TMR_IRQ_EN_CLR_OFFSET)
#define AM335X_TMR2_TCLR                (AM335X_DMTIMER2_VADDR + AM335X_TMR_TCLR_OFFSET)
#define AM335X_TMR2_TCRR                (AM335X_DMTIMER2_VADDR + AM335X_TMR_TCRR_OFFSET)
#define AM335X_TMR2_TLDR                (AM335X_DMTIMER2_VADDR + AM335X_TMR_TLDR_OFFSET)

/* Register bit field definitions *******************************************/

#define TMR_TIOCP_SOFT_RESET            (1 << 0) /* Bit 0: Software reset */
#define TMR_TIOCP_EMU_FREE              (1 << 1) /* Bit 1: Sensitivity to emulation (debug) suspend event from Debug Subsystem. */
#define TMR_TIOCP_IDLE_MODE_SHIFT       (2)      /* Bits 2-3: Power management, req/ack control */
#define TMR_TIOCP_IDLE_MODE_MASK        (3 << TMR_TIOCP_IDLE_MODE_SHIFT)
#  define TMR_TIOCP_IDLE_MODE_FORCE     (0 << TMR_TIOCP_IDLE_MODE_SHIFT) /* Force-idle mode */
#  define TMR_TIOCP_IDLE_MODE_NOIDLE    (1 << TMR_TIOCP_IDLE_MODE_SHIFT) /* No-idle mode */
#  define TMR_TIOCP_IDLE_MODE_SMART     (2 << TMR_TIOCP_IDLE_MODE_SHIFT) /* Smart-idle mode */
#  define TMR_TIOCP_IDLE_MODE_WAKEUP    (3 << TMR_TIOCP_IDLE_MODE_SHIFT) /* Smart-idle wakeup-capable mode */

#define TMR_IRQ_EOI_DMA_ACK             (1 << 0) /* Bit 0: DMA event acknowledge bit */

#define TMR_IRQ_FlAG_MAT                (1 << 0) /* Bit 0: IRQ flag for Match */
#define TMR_IRQ_FLAG_OVF                (1 << 1) /* Bit 1: IRQ flag for Overflow */
#define TMR_IRQ_FLAG_TCAR               (1 << 2) /* Bit 2: IRQ flag for Capture */

#define TMR_TCLR_ST                     (1 << 0) /* Bit 0: Start/stop timer */
#define TMR_TCLR_AR                     (1 << 1) /* Bit 1: Auto-reload */
#define TMR_TCLR_PTV_SHIFT              (2)      /* Bits 2-4: Pre-scale clock Timer value */
#define TMR_TCLR_PTV_MASK               (7 << TMR_TCLR_PTV_SHIFT)
#define TMR_TCLR_PRE                    (1 << 5) /* Bit 5: Prescaler enable */
#define TMR_TCLR_CE                     (1 << 6) /* Bit 6: Compare mode */
#define TMR_TCLR_SCPWM                  (1 << 7) /* Bit 7: Set/clear PORTIMERPWM output pin */
#define TMR_TCLR_TCM_SHIFT              (8)      /* Bits 8-9: Transition Capture Mode */
#define TMR_TCLR_TCM_MASK               (3 << TMR_TCLR_TCM_SHIFT)
#  define TMR_TCLR_TCM_NONE             (0 << TMR_TCLR_TCM_SHIFT) /* No capture */
#  define TMR_TCLR_TCM_RISING           (1 << TMR_TCLR_TCM_SHIFT) /* Capture on rising edges */
#  define TMR_TCLR_TCM_FALLING          (2 << TMR_TCLR_TCM_SHIFT) /* Capture on falling edges */
#  define TMR_TCLR_TCM_BOTH             (3 << TMR_TCLR_TCM_SHIFT) /* Capture on both edges */

#define TMR_TCLR_TRG_SHIFT              (10)      /* Bits 10-11: Trigger Output Mode */
#define TMR_TCLR_TRG_MASK               (3 << TMR_TCLR_TRG_SHIFT)
#  define TMR_TCLR_TRG_NONE             (0 << TMR_TCLR_TRG_SHIFT) /* No trigger */
#  define TMR_TCLR_TRG_OFLOW            (1 << TMR_TCLR_TRG_SHIFT) /* Trigger on overflow */
#  define TMR_TCLR_TRG_OFLOWMATCH       (2 << TMR_TCLR_TRG_SHIFT) /* Trigger on overflow and match */

#define TMR_TCLR_PT                     (1 << 12) /* Bit 12: Pulse or toggle mode */
#define TMR_TCLR_CAPT                   (1 << 13) /* Bit 13: Capture mode. */
#define TMR_TCLR_GPO_CFG                (1 << 14) /* Bit 14: General purpose output configuration */

#define TMR_TWPS_W_PEND_TCLR            (1 << 0) /* Bit 0: Write pending for TCLR register */
#define TMR_TWPS_W_PEND_TCRR            (1 << 1) /* Bit 1: Write pending for TCRR register */
#define TMR_TWPS_W_PEND_TLDR            (1 << 2) /* Bit 2: Write pending for TLDR register */
#define TMR_TWPS_W_PEND_TTGR            (1 << 3) /* Bit 3: Write pending for TTGR register */
#define TMR_TWPS_W_PEND_TMAR            (1 << 4) /* Bit 4: Write pending for TMAR register */

#define TMR_TSICR_SFT                   (1 << 1) /* Bit 1: Software reset */
#define TMR_TSICR_POSTED                (1 << 2) /* Bit 2: Posted mode */

#define TMR1MS_TIOCP_AUTO_IDLE          (1 << 0) /* Bit 0: Internal OCP clock gating strategy */
#define TMR1MS_TIOCP_SOFT_RESET         (1 << 1) /* Bit 1: Software reset */
#define TMR1MS_TIOCP_ENA_WAKEUP         (1 << 2) /* Bit 2: Wake-up feature global control */
#define TMR1MS_TIOCP_IDLE_MODE_SHIFT    (3)      /* Bits 3-4: Power management, req/ack control */
#define TMR1MS_TIOCP_IDLE_MODE_MASK     (3 << TMR1MS_TIOCP_IDLE_MODE_SHIFT)
#  define TMR1MS_TIOCP_IDLE_MODE_FORCE  (0 << TMR1MS_TIOCP_IDLE_MODE_SHIFT) /* Force-idle mode */
#  define TMR1MS_TIOCP_IDLE_MODE_NOIDLE (1 << TMR1MS_TIOCP_IDLE_MODE_SHIFT) /* No-idle mode */
#  define TMR1MS_TIOCP_IDLE_MODE_SMART  (2 << TMR1MS_TIOCP_IDLE_MODE_SHIFT) /* Smart-idle mode */
#  define TMR1MS_TIOCP_IDLE_MODE_WAKEUP (3 << TMR1MS_TIOCP_IDLE_MODE_SHIFT) /* Smart-idle wakeup-capable mode */

#define TMR1MS_TIOCP_EMU_FREE           (1 << 5) /* Bit 5: Sensitivity to emulation (debug) suspend event from Debug Subsystem. */

#define TMR1MS_TISTAT                   (1 << 0) /* Bit 0: Internal reset monitoring */

#define TMR1MS_IRQ_FLAG_MAT             (1 << 0) /* Bit 0: IRQ flag for Match */
#define TMR1MS_IRQ_FLAG_OVF             (1 << 1) /* Bit 1: IRQ flag for Overflow */
#define TMR1MS_IRQ_FLAG_TCAR            (1 << 2) /* Bit 2: IRQ flag for Capture */

#define TMR1MS_TCLR_ST                  (1 << 0) /* Bit 0: Start/stop timer */
#define TMR1MS_TCLR_AR                  (1 << 1) /* Bit 1: Auto-reload */
#define TMR1MS_TCLR_PTV_SHIFT           (2)      /* Bits 2-4: Pre-scale clock Timer value */
#define TMR1MS_TCLR_PTV_MASK            (7 << TMR1MS_TCLR_PTV_SHIFT)
#define TMR1MS_TCLR_PRE                 (1 << 5) /* Bit 5: Prescaler enable */
#define TMR1MS_TCLR_CE                  (1 << 6) /* Bit 6: Compare mode */
#define TMR1MS_TCLR_SCPWM               (1 << 7) /* Bit 7: Pulse Width Modulation output pin default value */
#define TMR1MS_TCLR_TCM_SHIFT           (8)      /* Bits 8-9: Transition Capture Mode */
#define TMR1MS_TCLR_TCM_MASK            (3 << TMR1MS_TCLR_TCM_SHIFT)
#  define TMR1MS_TCLR_TCM_NONE          (0 << TMR1MS_TCLR_TCM_SHIFT) /* No capture */
#  define TMR1MS_TCLR_TCM_RISING        (1 << TMR1MS_TCLR_TCM_SHIFT) /* Capture on rising edges */
#  define TMR1MS_TCLR_TCM_FALLING       (2 << TMR1MS_TCLR_TCM_SHIFT) /* Capture on falling edges */
#  define TMR1MS_TCLR_TCM_BOTH          (3 << TMR1MS_TCLR_TCM_SHIFT) /* Capture on both edges */

#define TMR1MS_TCLR_TRG_SHIFT           (10)      /* Bits 10-11: Trigger Output Mode */
#define TMR1MS_TCLR_TRG_MASK            (3 << TMR1MS_TCLR_TRG_SHIFT)
#  define TMR1MS_TCLR_TRG_NONE          (0 << TMR1MS_TCLR_TRG_SHIFT) /* No trigger */
#  define TMR1MS_TCLR_TRG_OFLOW         (1 << TMR1MS_TCLR_TRG_SHIFT) /* Trigger on overflow */
#  define TMR1MS_TCLR_TRG_OFLOWMATCH    (2 << TMR1MS_TCLR_TRG_SHIFT) /* Trigger on overflow and match */

#define TMR1MS_TCLR_PT                  (1 << 12) /* Bit 12: Pulse or toggle mode */
#define TMR1MS_TCLR_CAPT                (1 << 13) /* Bit 13: Capture mode. */
#define TMR1MS_TCLR_GPO_CFG             (1 << 14) /* Bit 14: General purpose output configuration */

#define TMR1MS_TWPS_W_PEND_TCLR         (1 << 0) /* Bit 0: Write pending for TCLR register */
#define TMR1MS_TWPS_W_PEND_TCRR         (1 << 1) /* Bit 1: Write pending for TCRR register */
#define TMR1MS_TWPS_W_PEND_TLDR         (1 << 2) /* Bit 2: Write pending for TLDR register */
#define TMR1MS_TWPS_W_PEND_TTGR         (1 << 3) /* Bit 3: Write pending for TTGR register */
#define TMR1MS_TWPS_W_PEND_TMAR         (1 << 4) /* Bit 4: Write pending for TMAR register */
#define TMR1MS_TWPS_W_PEND_TPIR         (1 << 5) /* Bit 5: Write pending for TPIR register */
#define TMR1MS_TWPS_W_PEND_TNIR         (1 << 6) /* Bit 6: Write pending for TNIR register */
#define TMR1MS_TWPS_W_PEND_TCVR         (1 << 7) /* Bit 7: Write pending for TCVR register */
#define TMR1MS_TWPS_W_PEND_TOCR         (1 << 8) /* Bit 8: Write pending for TOCR register */
#define TMR1MS_TWPS_W_PEND_TOWR         (1 << 9) /* Bit 9: Write pending for TOWR register */

#define TMR1MS_TSICR_SFT                (1 << 1) /* Bit 1: Software reset */
#define TMR1MS_TSICR_POSTED             (1 << 2) /* Bit 2: Posted mode */

#define TMR1MS_TOCR_MASK                (0xffffff)

#define TMR1MS_TOWR_MASK                (0xffffff)

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_TIMER_H */
