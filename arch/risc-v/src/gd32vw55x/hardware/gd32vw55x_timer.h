/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_timer.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_TIMER_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x has six timers:
 *
 *   TIMER0  Advanced timer, 16-bit counter, 4 channels (CH0..CH3),
 *           complementary outputs on CH0..CH2, dead-time, break,
 *           repetition counter.  APB2 domain.
 *   TIMER1  General purpose L0, 32-bit counter, 4 channels (CH0..CH3),
 *           no complementary output, no break.  APB1 domain.
 *   TIMER2  General purpose L0, 32-bit counter, 4 channels (CH0..CH3),
 *           no complementary output, no break.  APB1 domain.
 *   TIMER5  Basic timer, 16-bit counter, no channel at all.  APB1 domain.
 *   TIMER15 General purpose L2, 16-bit counter, 1 channel (CH0) with
 *           complementary output, dead-time, break, repetition counter.
 *           APB2 domain.
 *   TIMER16 Same as TIMER15.  APB2 domain.
 *
 * Only TIMER0, TIMER1 and TIMER2 have the slave mode controller.  Note
 * that, unlike the ST parts this IP is derived from, the slave mode
 * selection (SMC) and the trigger selection (TRGS) are *not* located in
 * TIMER_SMCFG.  They live in the SYSCFG_TIMERxCFG registers instead (see
 * the end of this file).
 */

/* Register offsets *********************************************************/

#define GD32VW55X_TIMER_CTL0_OFFSET      0x0000  /* Control register 0 */
#define GD32VW55X_TIMER_CTL1_OFFSET      0x0004  /* Control register 1 */
#define GD32VW55X_TIMER_SMCFG_OFFSET     0x0008  /* Slave mode config */
#define GD32VW55X_TIMER_DMAINTEN_OFFSET  0x000c  /* DMA and interrupt en. */
#define GD32VW55X_TIMER_INTF_OFFSET      0x0010  /* Interrupt flag */
#define GD32VW55X_TIMER_SWEVG_OFFSET     0x0014  /* Software event gen. */
#define GD32VW55X_TIMER_CHCTL0_OFFSET    0x0018  /* Channel control 0 */
#define GD32VW55X_TIMER_CHCTL1_OFFSET    0x001c  /* Channel control 1 */
#define GD32VW55X_TIMER_CHCTL2_OFFSET    0x0020  /* Channel control 2 */
#define GD32VW55X_TIMER_CNT_OFFSET       0x0024  /* Counter */
#define GD32VW55X_TIMER_PSC_OFFSET       0x0028  /* Prescaler */
#define GD32VW55X_TIMER_CAR_OFFSET       0x002c  /* Counter auto reload */
#define GD32VW55X_TIMER_CREP_OFFSET      0x0030  /* Counter repetition */
#define GD32VW55X_TIMER_CH0CV_OFFSET     0x0034  /* Ch0 capture/compare */
#define GD32VW55X_TIMER_CH1CV_OFFSET     0x0038  /* Ch1 capture/compare */
#define GD32VW55X_TIMER_CH2CV_OFFSET     0x003c  /* Ch2 capture/compare */
#define GD32VW55X_TIMER_CH3CV_OFFSET     0x0040  /* Ch3 capture/compare */
#define GD32VW55X_TIMER_CCHP_OFFSET      0x0044  /* Complementary protect */
#define GD32VW55X_TIMER_DMACFG_OFFSET    0x0048  /* DMA configuration */
#define GD32VW55X_TIMER_DMATB_OFFSET     0x004c  /* DMA transfer buffer */
#define GD32VW55X_TIMER_CFG_OFFSET       0x00fc  /* Configuration */

/* Channel n (n=0..3) capture/compare value register offset */

#define GD32VW55X_TIMER_CHCV_OFFSET(n) \
  (GD32VW55X_TIMER_CH0CV_OFFSET + ((n) << 2))

/* Register bit definitions *************************************************/

/* Control register 0 (CTL0) */

#define TIMER_CTL0_CEN               (1 << 0)  /* Counter enable */
#define TIMER_CTL0_UPDIS             (1 << 1)  /* Update disable */
#define TIMER_CTL0_UPS               (1 << 2)  /* Update source */
#define TIMER_CTL0_SPM               (1 << 3)  /* Single pulse mode */
#define TIMER_CTL0_DIR               (1 << 4)  /* Counter direction: down */
#define TIMER_CTL0_CAM_SHIFT         (5)       /* Counter align mode */
#define TIMER_CTL0_CAM_MASK          (3 << TIMER_CTL0_CAM_SHIFT)
#  define TIMER_CTL0_CAM_EDGE        (0 << TIMER_CTL0_CAM_SHIFT)
#  define TIMER_CTL0_CAM_CENTER_DOWN (1 << TIMER_CTL0_CAM_SHIFT)
#  define TIMER_CTL0_CAM_CENTER_UP   (2 << TIMER_CTL0_CAM_SHIFT)
#  define TIMER_CTL0_CAM_CENTER_BOTH (3 << TIMER_CTL0_CAM_SHIFT)
#define TIMER_CTL0_ARSE              (1 << 7)  /* Auto-reload shadow en. */
#define TIMER_CTL0_CKDIV_SHIFT       (8)       /* Clock division (fDTS) */
#define TIMER_CTL0_CKDIV_MASK        (3 << TIMER_CTL0_CKDIV_SHIFT)
#  define TIMER_CTL0_CKDIV_DIV1      (0 << TIMER_CTL0_CKDIV_SHIFT)
#  define TIMER_CTL0_CKDIV_DIV2      (1 << TIMER_CTL0_CKDIV_SHIFT)
#  define TIMER_CTL0_CKDIV_DIV4      (2 << TIMER_CTL0_CKDIV_SHIFT)

/* Control register 1 (CTL1) */

#define TIMER_CTL1_CCSE              (1 << 0)  /* Commutation shadow en. */
#define TIMER_CTL1_CCUC              (1 << 2)  /* Commutation update ctl */
#define TIMER_CTL1_DMAS              (1 << 3)  /* DMA request source */
#define TIMER_CTL1_MMC_SHIFT         (4)       /* Master mode control */
#define TIMER_CTL1_MMC_MASK          (7 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_RESET       (0 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_ENABLE      (1 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_UPDATE      (2 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_CH0         (3 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_O0CPRE      (4 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_O1CPRE      (5 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_O2CPRE      (6 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_O3CPRE      (7 << TIMER_CTL1_MMC_SHIFT)
#define TIMER_CTL1_TI0S              (1 << 7)  /* Ch0 trigger input sel */
#define TIMER_CTL1_ISO0              (1 << 8)  /* Idle state of CH0_O */
#define TIMER_CTL1_ISO0N             (1 << 9)  /* Idle state of CH0_ON */
#define TIMER_CTL1_ISO1              (1 << 10) /* Idle state of CH1_O */
#define TIMER_CTL1_ISO1N             (1 << 11) /* Idle state of CH1_ON */
#define TIMER_CTL1_ISO2              (1 << 12) /* Idle state of CH2_O */
#define TIMER_CTL1_ISO2N             (1 << 13) /* Idle state of CH2_ON */
#define TIMER_CTL1_ISO3              (1 << 14) /* Idle state of CH3_O */

/* Slave mode configuration register (SMCFG).
 *
 * Beware: this register holds only the external trigger (ETI) setup and
 * the master-slave and external-clock-mode-1 bits.  The slave mode and the
 * trigger source selection are configured through SYSCFG_TIMERxCFG.
 */

#define TIMER_SMCFG_MSM              (1 << 7)  /* Master-slave mode */
#define TIMER_SMCFG_ETFC_SHIFT       (8)       /* Ext. trigger filter */
#define TIMER_SMCFG_ETFC_MASK        (15 << TIMER_SMCFG_ETFC_SHIFT)
#define TIMER_SMCFG_ETPSC_SHIFT      (12)      /* Ext. trigger prescaler */
#define TIMER_SMCFG_ETPSC_MASK       (3 << TIMER_SMCFG_ETPSC_SHIFT)
#  define TIMER_SMCFG_ETPSC_OFF      (0 << TIMER_SMCFG_ETPSC_SHIFT)
#  define TIMER_SMCFG_ETPSC_DIV2     (1 << TIMER_SMCFG_ETPSC_SHIFT)
#  define TIMER_SMCFG_ETPSC_DIV4     (2 << TIMER_SMCFG_ETPSC_SHIFT)
#  define TIMER_SMCFG_ETPSC_DIV8     (3 << TIMER_SMCFG_ETPSC_SHIFT)
#define TIMER_SMCFG_SMC1             (1 << 14) /* External clock mode 1 */
#define TIMER_SMCFG_ETP              (1 << 15) /* Ext. trigger polarity */

/* DMA and interrupt enable register (DMAINTEN) */

#define TIMER_DMAINTEN_UPIE          (1 << 0)  /* Update interrupt */
#define TIMER_DMAINTEN_CH0IE         (1 << 1)  /* Ch0 capture/compare int */
#define TIMER_DMAINTEN_CH1IE         (1 << 2)  /* Ch1 capture/compare int */
#define TIMER_DMAINTEN_CH2IE         (1 << 3)  /* Ch2 capture/compare int */
#define TIMER_DMAINTEN_CH3IE         (1 << 4)  /* Ch3 capture/compare int */
#define TIMER_DMAINTEN_CMTIE         (1 << 5)  /* Commutation interrupt */
#define TIMER_DMAINTEN_TRGIE         (1 << 6)  /* Trigger interrupt */
#define TIMER_DMAINTEN_BRKIE         (1 << 7)  /* Break interrupt */
#define TIMER_DMAINTEN_UPDEN         (1 << 8)  /* Update DMA request */
#define TIMER_DMAINTEN_CH0DEN        (1 << 9)  /* Ch0 DMA request */
#define TIMER_DMAINTEN_CH1DEN        (1 << 10) /* Ch1 DMA request */
#define TIMER_DMAINTEN_CH2DEN        (1 << 11) /* Ch2 DMA request */
#define TIMER_DMAINTEN_CH3DEN        (1 << 12) /* Ch3 DMA request */
#define TIMER_DMAINTEN_CMTDEN        (1 << 13) /* Commutation DMA request */
#define TIMER_DMAINTEN_TRGDEN        (1 << 14) /* Trigger DMA request */

/* Channel n (n=0..3) capture/compare interrupt enable */

#define TIMER_DMAINTEN_CHIE(n)       (1 << ((n) + 1))

/* Interrupt flag register (INTF) */

#define TIMER_INTF_UPIF              (1 << 0)  /* Update interrupt flag */
#define TIMER_INTF_CH0IF             (1 << 1)  /* Ch0 cap/cmp int flag */
#define TIMER_INTF_CH1IF             (1 << 2)  /* Ch1 cap/cmp int flag */
#define TIMER_INTF_CH2IF             (1 << 3)  /* Ch2 cap/cmp int flag */
#define TIMER_INTF_CH3IF             (1 << 4)  /* Ch3 cap/cmp int flag */
#define TIMER_INTF_CMTIF             (1 << 5)  /* Commutation int flag */
#define TIMER_INTF_TRGIF             (1 << 6)  /* Trigger interrupt flag */
#define TIMER_INTF_BRKIF             (1 << 7)  /* Break interrupt flag */
#define TIMER_INTF_CH0OF             (1 << 9)  /* Ch0 overcapture flag */
#define TIMER_INTF_CH1OF             (1 << 10) /* Ch1 overcapture flag */
#define TIMER_INTF_CH2OF             (1 << 11) /* Ch2 overcapture flag */
#define TIMER_INTF_CH3OF             (1 << 12) /* Ch3 overcapture flag */

/* Channel n (n=0..3) capture/compare and overcapture flags */

#define TIMER_INTF_CHIF(n)           (1 << ((n) + 1))
#define TIMER_INTF_CHOF(n)           (1 << ((n) + 9))

/* Software event generation register (SWEVG) */

#define TIMER_SWEVG_UPG              (1 << 0)  /* Update event generation */
#define TIMER_SWEVG_CH0G             (1 << 1)  /* Ch0 cap/cmp event gen. */
#define TIMER_SWEVG_CH1G             (1 << 2)  /* Ch1 cap/cmp event gen. */
#define TIMER_SWEVG_CH2G             (1 << 3)  /* Ch2 cap/cmp event gen. */
#define TIMER_SWEVG_CH3G             (1 << 4)  /* Ch3 cap/cmp event gen. */
#define TIMER_SWEVG_CMTG             (1 << 5)  /* Commutation event gen. */
#define TIMER_SWEVG_TRGG             (1 << 6)  /* Trigger event gen. */
#define TIMER_SWEVG_BRKG             (1 << 7)  /* Break event generation */

/* Channel control register 0 (CHCTL0) and channel control register 1
 * (CHCTL1).  CHCTL0 holds channels 0 and 1, CHCTL1 holds channels 2 and 3.
 * Each channel occupies 8 bits: the even channel of the pair at bits 0-7
 * and the odd channel of the pair at bits 8-15.  The shift for channel n
 * within its register is therefore ((n & 1) << 3).
 */

#define TIMER_CHCTL_SHIFT(n)         (((n) & 1) << 3)
#define TIMER_CHCTL_MASK(n)          (0xff << TIMER_CHCTL_SHIFT(n))

/* Output compare mode (channel n, before shifting) */

#define TIMER_CHCTL_CHMS_SHIFT       (0)       /* Channel mode selection */
#define TIMER_CHCTL_CHMS_MASK        (3 << TIMER_CHCTL_CHMS_SHIFT)
#  define TIMER_CHCTL_CHMS_OUTPUT    (0 << TIMER_CHCTL_CHMS_SHIFT)
#  define TIMER_CHCTL_CHMS_DIRECTTI  (1 << TIMER_CHCTL_CHMS_SHIFT)
#  define TIMER_CHCTL_CHMS_INDIRECTT (2 << TIMER_CHCTL_CHMS_SHIFT)
#  define TIMER_CHCTL_CHMS_ITS       (3 << TIMER_CHCTL_CHMS_SHIFT)
#define TIMER_CHCTL_CHCOMFEN         (1 << 2)  /* Output compare fast en. */
#define TIMER_CHCTL_CHCOMSEN         (1 << 3)  /* Output compare shadow */
#define TIMER_CHCTL_CHCOMCTL_SHIFT   (4)       /* Output compare control */
#define TIMER_CHCTL_CHCOMCTL_MASK    (7 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_TIMING   (0 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_ACTIVE   (1 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_INACTIVE (2 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_TOGGLE   (3 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_LOW      (4 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_HIGH     (5 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_PWM0     (6 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#  define TIMER_CHCTL_CHCOMCTL_PWM1     (7 << TIMER_CHCTL_CHCOMCTL_SHIFT)
#define TIMER_CHCTL_CHCOMCEN         (1 << 7)  /* Output compare clear en. */

/* Input capture mode (channel n, before shifting) */

#define TIMER_CHCTL_CHCAPPSC_SHIFT   (2)       /* Input capture prescaler */
#define TIMER_CHCTL_CHCAPPSC_MASK    (3 << TIMER_CHCTL_CHCAPPSC_SHIFT)
#  define TIMER_CHCTL_CHCAPPSC_DIV1  (0 << TIMER_CHCTL_CHCAPPSC_SHIFT)
#  define TIMER_CHCTL_CHCAPPSC_DIV2  (1 << TIMER_CHCTL_CHCAPPSC_SHIFT)
#  define TIMER_CHCTL_CHCAPPSC_DIV4  (2 << TIMER_CHCTL_CHCAPPSC_SHIFT)
#  define TIMER_CHCTL_CHCAPPSC_DIV8  (3 << TIMER_CHCTL_CHCAPPSC_SHIFT)
#define TIMER_CHCTL_CHCAPFLT_SHIFT   (4)       /* Input capture filter */
#define TIMER_CHCTL_CHCAPFLT_MASK    (15 << TIMER_CHCTL_CHCAPFLT_SHIFT)

/* Channel control register 2 (CHCTL2).  Each channel occupies 4 bits. */

#define TIMER_CHCTL2_SHIFT(n)        ((n) << 2)
#define TIMER_CHCTL2_MASK(n)         (15 << TIMER_CHCTL2_SHIFT(n))

#define TIMER_CHCTL2_CHEN(n)         (1 << (((n) << 2) + 0)) /* Enable */
#define TIMER_CHCTL2_CHP(n)          (1 << (((n) << 2) + 1)) /* Polarity */
#define TIMER_CHCTL2_CHNEN(n)        (1 << (((n) << 2) + 2)) /* Compl. en. */
#define TIMER_CHCTL2_CHNP(n)         (1 << (((n) << 2) + 3)) /* Compl. pol */

/* Complementary channel protection register (CCHP) */

#define TIMER_CCHP_DTCFG_SHIFT       (0)       /* Dead time configuration */
#define TIMER_CCHP_DTCFG_MASK        (0xff << TIMER_CCHP_DTCFG_SHIFT)
#define TIMER_CCHP_PROT_SHIFT        (8)       /* Register protection */
#define TIMER_CCHP_PROT_MASK         (3 << TIMER_CCHP_PROT_SHIFT)
#  define TIMER_CCHP_PROT_OFF        (0 << TIMER_CCHP_PROT_SHIFT)
#  define TIMER_CCHP_PROT_0          (1 << TIMER_CCHP_PROT_SHIFT)
#  define TIMER_CCHP_PROT_1          (2 << TIMER_CCHP_PROT_SHIFT)
#  define TIMER_CCHP_PROT_2          (3 << TIMER_CCHP_PROT_SHIFT)
#define TIMER_CCHP_IOS               (1 << 10) /* Idle mode off-state */
#define TIMER_CCHP_ROS               (1 << 11) /* Run mode off-state */
#define TIMER_CCHP_BRKEN             (1 << 12) /* Break enable */
#define TIMER_CCHP_BRKP              (1 << 13) /* Break polarity */
#define TIMER_CCHP_OAEN              (1 << 14) /* Output automatic enable */
#define TIMER_CCHP_POEN              (1 << 15) /* Primary output enable */

/* DMA configuration register (DMACFG) */

#define TIMER_DMACFG_DMATA_SHIFT     (0)       /* Transfer start address */
#define TIMER_DMACFG_DMATA_MASK      (31 << TIMER_DMACFG_DMATA_SHIFT)
#define TIMER_DMACFG_DMATC_SHIFT     (8)       /* Transfer count */
#define TIMER_DMACFG_DMATC_MASK      (31 << TIMER_DMACFG_DMATC_SHIFT)

/* Configuration register (CFG) */

#define TIMER_CFG_OUTSEL             (1 << 0)  /* Output value selection */
#define TIMER_CFG_CHVSEL             (1 << 1)  /* Write CHxVAL selection */

/* Counter, prescaler and auto-reload limits */

#define TIMER_PSC_MAX                0xffff
#define TIMER_CAR_MAX16              0xffff
#define TIMER_CAR_MAX32              0xffffffff

/* SYSCFG timer slave mode configuration ************************************
 *
 * On this family the slave mode controller of TIMER0, TIMER1 and TIMER2 is
 * configured from the SYSCFG block.  SYSCFG_TIMERxCFG is split into eight
 * 4-bit fields, one per slave mode.  The field of the wanted slave mode
 * holds the trigger source; all other fields must be zero:
 *
 *   SYSCFG_TIMERxCFG = TRGSEL << (4 * SLAVE_MODE)
 */

#define GD32VW55X_SYSCFG_TIMER0CFG   (GD32VW55X_SYSCFG_BASE + 0x0100)
#define GD32VW55X_SYSCFG_TIMER1CFG   (GD32VW55X_SYSCFG_BASE + 0x0104)
#define GD32VW55X_SYSCFG_TIMER2CFG   (GD32VW55X_SYSCFG_BASE + 0x0108)

/* Slave modes (the index of the 4-bit field) */

#define TIMER_SLAVE_MODE_QUAD0       0  /* Quadrature decoder mode 0 */
#define TIMER_SLAVE_MODE_QUAD1       1  /* Quadrature decoder mode 1 */
#define TIMER_SLAVE_MODE_QUAD2       2  /* Quadrature decoder mode 2 */
#define TIMER_SLAVE_MODE_RESTART     3  /* Restart mode */
#define TIMER_SLAVE_MODE_PAUSE       4  /* Pause mode */
#define TIMER_SLAVE_MODE_EVENT       5  /* Event mode */
#define TIMER_SLAVE_MODE_EXTERNAL0   6  /* External clock mode 0 */
#define TIMER_SLAVE_MODE_DISABLE     7  /* Slave mode disabled */

/* Trigger sources (the value of the 4-bit field) */

#define TIMER_TRGSEL_NONE            0  /* No trigger */
#define TIMER_TRGSEL_ITI0            1  /* Internal trigger 0 */
#define TIMER_TRGSEL_ITI1            2  /* Internal trigger 1 */
#define TIMER_TRGSEL_ITI2            3  /* Internal trigger 2 */
#define TIMER_TRGSEL_ITI3            4  /* Internal trigger 3 */
#define TIMER_TRGSEL_CI0F_ED         5  /* CI0 edge detector */
#define TIMER_TRGSEL_CI0FE0          6  /* Filtered timer input 0 */
#define TIMER_TRGSEL_CI1FE1          7  /* Filtered timer input 1 */
#define TIMER_TRGSEL_ETIFP           8  /* External trigger */

#define SYSCFG_TIMERCFG(mode, trgsel) ((uint32_t)(trgsel) << ((mode) << 2))

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_TIMER_H */
